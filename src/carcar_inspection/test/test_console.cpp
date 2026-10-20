// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
// 隔离ROS域227、临时数据及替身硬件/导航服务；不打开真实串口。
#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <carcar_interfaces/srv/set_motion_hold.hpp>
#include <carcar_interfaces/srv/set_camera_servo.hpp>
#include <carcar_interfaces/msg/indicator_command.hpp>
#include <nlohmann/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <fstream>
#include <filesystem>
#include <thread>
#include <atomic>
extern char ** environ;
namespace {
using json=nlohmann::json;namespace asio=boost::asio;namespace beast=boost::beast;namespace http=beast::http;namespace ws=beast::websocket;using tcp=asio::ip::tcp;
using Hold=carcar_interfaces::srv::SetMotionHold;using Servo=carcar_interfaces::srv::SetCameraServo;
using Nav=nav2_msgs::action::NavigateToPose;using Goal=rclcpp_action::ServerGoalHandle<Nav>;using Clear=nav2_msgs::srv::ClearEntireCostmap;
double clock_now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Child {
  pid_t pid=-1;
  Child(const std::filesystem::path & root,bool hardware,unsigned port) {
    std::vector<std::string> args{CONSOLE_EXE,"--ros-args","-p","bind_address:=127.0.0.1","-p","port:="+std::to_string(port),
      "-p","data_root:="+(root/"data").string(),"-p","map_root:="+(root/"maps").string(),"-p","web_root:=" WEB_ROOT,
      "-p",std::string("enable_hardware:=")+(hardware?"true":"false"),"-p",std::string("enable_navigation:=")+(hardware?"true":"false"),
      "-p","settle_seconds:=0.2","-p","stop_seconds:=0.2","-p","mapping_status_url:=\"\""};
    std::vector<char*> raw;for(auto & arg:args)raw.push_back(arg.data());raw.push_back(nullptr);
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions,STDOUT_FILENO,(root/"console.log").c_str(),O_CREAT|O_WRONLY|O_TRUNC,0600);
    posix_spawn_file_actions_adddup2(&actions,STDOUT_FILENO,STDERR_FILENO);
    const auto rc=posix_spawn(&pid,raw[0],&actions,nullptr,raw.data(),environ);posix_spawn_file_actions_destroy(&actions);
    if(rc)throw std::runtime_error("测试子进程启动失败");
  }
  ~Child(){if(pid>0){kill(pid,SIGINT);for(int i=0;i<50;++i){int status;if(waitpid(pid,&status,WNOHANG)==pid)return;std::this_thread::sleep_for(std::chrono::milliseconds(20));}kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
};
struct Temporary {
  std::filesystem::path root;
  Temporary(){char name[]="/tmp/carcar-inspection-console-XXXXXX";root=mkdtemp(name);std::filesystem::create_directories(root/"maps");
    std::ofstream yaml(root/"maps/testmap.yaml");yaml<<"image: testmap.pgm\nmode: trinary\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";yaml.close();
    std::ofstream pgm(root/"maps/testmap.pgm",std::ios::binary);pgm<<"P5\n20 20\n255\n";std::string pixels(400,static_cast<char>(255));pgm.write(pixels.data(),pixels.size());}
  ~Temporary(){if(!::testing::Test::HasFailure())std::filesystem::remove_all(root);else std::cerr<<"测试证据保留："<<root<<'\n';}
};
class Fixture : public rclcpp::Node {
public:
  Fixture():Node("inspection_test_fixture"),tf(*this) {
    odom=create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",rclcpp::SensorDataQoS());
    scan=create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());rgb=create_publisher<sensor_msgs::msg::Image>("/camera/camera/color/image_raw",rclcpp::SensorDataQoS());
    raw=create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/points",rclcpp::SensorDataQoS());
    obstacles=create_publisher<sensor_msgs::msg::PointCloud2>("/camera/navigation/obstacles",rclcpp::SensorDataQoS());
    clearing=create_publisher<sensor_msgs::msg::PointCloud2>("/camera/navigation/clearing",rclcpp::SensorDataQoS());
    filter_diag=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/camera/navigation/diagnostics",1);
    local_diag=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/local_costmap/depth_stvl/diagnostics",1);
    global_diag=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/global_costmap/depth_stvl/diagnostics",1);
    map=create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
    local=create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap",rclcpp::QoS(1).reliable().transient_local());
    global=create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",rclcpp::QoS(1).reliable().transient_local());
    ready=create_publisher<std_msgs::msg::Bool>("/localization_monitor/ready",rclcpp::QoS(1).reliable().transient_local());
    fault=create_publisher<std_msgs::msg::Bool>("/localization_monitor/fault",rclcpp::QoS(1).reliable().transient_local());
    perception=create_publisher<std_msgs::msg::Bool>("/navigation/perception_motion_allowed",rclcpp::QoS(1).reliable().transient_local());
    nav_velocity=create_publisher<geometry_msgs::msg::Twist>("/inspection/navigation/cmd_vel",1);
    indicator=create_subscription<carcar_interfaces::msg::IndicatorCommand>("/hardware/indicator/command",10,
      [this](carcar_interfaces::msg::IndicatorCommand::ConstSharedPtr value){
        if(value->r>0&&value->g==0)++red_samples;
        if(value->r>0&&value->g>0)++yellow_samples;
        if(value->beep_trigger&&value->beep_duration_ms==3000)++beep_windows;
        last_red=value->r;last_green=value->g;
      });
    hold_service=create_service<Hold>("/hardware/inspection/motion_hold",[this](const Hold::Request::SharedPtr req,Hold::Response::SharedPtr res){
      ++holds;res->accepted=true;res->held=req->hold;res->driver_epoch="offline_driver_epoch";res->board_command=board;held=req->hold;
    });
    servo_service=create_service<Servo>("/hardware/inspection/camera_servo",[this](const Servo::Request::SharedPtr req,Servo::Response::SharedPtr res){
      ++commands;res->sent=held&&req->expected_board_command==board&&std::abs(req->board_command-board)<=1;
      if(res->sent)board=req->board_command;
      res->board_command=board;res->driver_epoch="offline_driver_epoch";
    });
    local_service=create_service<Clear>("/local_costmap/clear_entirely_local_costmap",[](Clear::Request::SharedPtr,Clear::Response::SharedPtr){});
    global_service=create_service<Clear>("/global_costmap/clear_entirely_global_costmap",[](Clear::Request::SharedPtr,Clear::Response::SharedPtr){});
    navigation=rclcpp_action::create_server<Nav>(this,"navigate_to_pose",
      [](const rclcpp_action::GoalUUID &,std::shared_ptr<const Nav::Goal>){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](const std::shared_ptr<Goal>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Goal> goal){active=goal;goal_at=clock_now();++goals;});
    timer=create_wall_timer(std::chrono::milliseconds(25),[this]{publish();});
  }
  void publish(){const auto stamp=now();
    if(active){if(active->is_canceling()){active->canceled(std::make_shared<Nav::Result>());active.reset();}
      else if(clock_now()-goal_at>.25){if(fail_navigation)active->abort(std::make_shared<Nav::Result>());else {position=active->get_goal()->pose.pose;active->succeed(std::make_shared<Nav::Result>());}active.reset();}}
    geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";t.header.stamp=stamp;t.transform.translation.x=position.position.x;t.transform.translation.y=position.position.y;t.transform.rotation=position.orientation;tf.sendTransform(t);
    nav_msgs::msg::Odometry o;o.header.stamp=stamp;o.header.frame_id="odom";o.child_frame_id="base_footprint";odom->publish(o);
    sensor_msgs::msg::LaserScan s;s.header.stamp=stamp;s.header.frame_id="laser_frame";s.range_min=.1;s.range_max=10;s.angle_increment=.1;s.ranges.assign(30,2);scan->publish(s);
    sensor_msgs::msg::Image image;image.header.stamp=stamp;image.header.frame_id="camera_color_optical_frame";image.width=32;image.height=24;image.step=96;image.encoding="rgb8";image.data.assign(32*24*3,120);rgb->publish(image);
    sensor_msgs::msg::PointCloud2 cloud;cloud.header.stamp=stamp;cloud.header.frame_id="camera_depth_optical_frame";cloud.width=1;cloud.height=1;cloud.point_step=16;cloud.row_step=16;cloud.data.resize(16);raw->publish(cloud);obstacles->publish(cloud);clearing->publish(cloud);
    diagnostic_msgs::msg::DiagnosticArray diagnostic;diagnostic.header.stamp=stamp;diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="D435DepthFilter";diagnostic_msgs::msg::KeyValue count;count.key="valid_measurements";count.value="1";status.values.push_back(count);diagnostic.status.push_back(status);filter_diag->publish(diagnostic);
    status.name="depth_stvl";status.values.clear();count.key="source_current";count.value="true";status.values.push_back(count);count.key="update_seq";count.value=std::to_string(++layer_sequence);status.values.push_back(count);diagnostic.status={status};if(publish_layers){local_diag->publish(diagnostic);global_diag->publish(diagnostic);}
    nav_msgs::msg::OccupancyGrid grid;grid.header.stamp=stamp;grid.header.frame_id="map";grid.info.width=grid.info.height=20;grid.info.resolution=.1;grid.info.origin.orientation.w=1;grid.data.assign(400,0);map->publish(grid);local->publish(grid);global->publish(grid);
    std_msgs::msg::Bool flag;flag.data=true;ready->publish(flag);perception->publish(flag);flag.data=false;fault->publish(flag);
    geometry_msgs::msg::Twist velocity;if(active)velocity.linear.x=.05;nav_velocity->publish(velocity);
  }
  std::atomic<int> commands{0},holds{0},goals{0};int board=94;bool held=false;double goal_at=0;
  std::atomic<bool> publish_layers{true},fail_navigation{false};
  std::atomic<int> red_samples{0},yellow_samples{0},beep_windows{0},last_red{-1},last_green{-1};
  geometry_msgs::msg::Pose position=[](){geometry_msgs::msg::Pose p;p.position.x=p.position.y=1;p.orientation.w=1;return p;}();
  std::shared_ptr<Goal> active;tf2_ros::TransformBroadcaster tf;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles,clearing;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr filter_diag,local_diag,global_diag;uint64_t layer_sequence=0;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map,local,global;rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready,fault,perception;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_velocity;
  rclcpp::Service<Hold>::SharedPtr hold_service;rclcpp::Service<Servo>::SharedPtr servo_service;rclcpp::Service<Clear>::SharedPtr local_service,global_service;
  rclcpp_action::Server<Nav>::SharedPtr navigation;rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<carcar_interfaces::msg::IndicatorCommand>::SharedPtr indicator;
};
struct Browser {
  asio::io_context io;unsigned short port;ws::stream<tcp::socket> socket{io};std::string session;
  explicit Browser(unsigned p):port(p){socket.next_layer().connect({asio::ip::make_address("127.0.0.1"),port});socket.handshake("127.0.0.1:"+std::to_string(port),"/ws/control");beast::flat_buffer b;socket.read(b);session=json::parse(beast::buffers_to_string(b.data())).at("session");}
  void drain(){pollfd f{socket.next_layer().native_handle(),POLLIN,0};while(poll(&f,1,0)>0){beast::flat_buffer b;socket.read(b);}}
  http::response<http::string_body> request(const std::string & target,json body=nullptr){tcp::socket connection(io);connection.connect({asio::ip::make_address("127.0.0.1"),port});http::request<http::string_body> req(body.is_null()?http::verb::get:http::verb::post,target,11);req.set(http::field::host,"127.0.0.1:"+std::to_string(port));
    if(!body.is_null()){body["session"]=session;req.set("X-Mapping-Console","1");req.body()=body.dump();req.prepare_payload();}http::write(connection,req);beast::flat_buffer b;http::response<http::string_body> res;http::read(connection,b,res);return res;}
  json state(){drain();const auto res=request("/api/state");return json::parse(res.body());}
  json post(const std::string & target,json body=json::object()){const auto res=request(target,std::move(body));if(res.result_int()!=200)throw std::runtime_error(res.body());return json::parse(res.body());}
  bool wait(const std::function<bool(const json &)> & predicate,double timeout=4){const auto until=clock_now()+timeout;while(clock_now()<until){if(predicate(state()))return true;std::this_thread::sleep_for(std::chrono::milliseconds(25));}return false;}
  void begin(){post("/api/teach/begin");if(!wait([](const json & s){return s["teaching"]["phase"]=="READY";}))throw std::runtime_error("未进入READY");}
  void step(double value){const auto s=state();post("/api/teach/step",{{"revision",s["teaching"]["revision"]},{"target_degrees",value}});if(!wait([](const json & s){return s["teaching"]["phase"]=="AWAITING_CONFIRMATION";}))throw std::runtime_error("未取得新画面");}
  json confirm(const std::string & purpose,const std::string & name="测试观察点") {
    const auto s=state();const auto image=request("/api/video.jpg");const auto sequence=std::stoull(std::string(image["X-Frame-Sequence"]));const auto revision=s["teaching"]["revision"];
    post("/api/teach/viewed",{{"revision",revision},{"frame_sequence",sequence}});
    return post("/api/teach/confirm",{{"purpose",purpose},{"name",name},{"revision",revision},{"frame_sequence",sequence}});
  }
  json task_one_point(){
    post("/api/map/select",{{"map_id","testmap"}});
    if(!wait([](const json & s){return s["health"]["map_matches"]==true&&s["health"]["localization"]==true&&s["health"]["still"]==true;}))throw std::runtime_error("隔离数据未就绪");
    begin();step(140.5);const auto point=confirm("point");begin();step(140.5);confirm("navigation");
    return post("/api/tasks/save",{{"task",{{"name","隔离单点巡检"},{"point_ids",{point["id"]}},{"final_parking_pose",{{"x",1.2},{"y",1.0},{"yaw",0}}}}}});
  }
};
bool listening(unsigned port){asio::io_context io;tcp::socket socket(io);beast::error_code error;socket.connect({asio::ip::make_address("127.0.0.1"),static_cast<unsigned short>(port)},error);return !error;}
bool startup(unsigned port){const auto until=clock_now()+8;while(clock_now()<until){if(listening(port))return true;std::this_thread::sleep_for(std::chrono::milliseconds(50));}return false;}
class ConsoleTest : public ::testing::Test {
protected:
  void SetUp()override{rclcpp::init(0,nullptr);fixture=std::make_shared<Fixture>();executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();executor->add_node(fixture);thread=std::thread([this]{executor->spin();});}
  void TearDown()override{executor->cancel();thread.join();executor.reset();fixture.reset();rclcpp::shutdown();}
  std::shared_ptr<Fixture> fixture;std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;std::thread thread;
};
TEST_F(ConsoleTest, ConfigurationModeNeverRequestsActuatorCommands) {
  Temporary temp;Child child(temp.root,false,18187);ASSERT_TRUE(startup(18187));Browser b(18187);
  ASSERT_TRUE(b.wait([](const json & s){return s["health"]["hardware_interface"]==true;}));
  b.post("/api/map/select",{{"map_id","testmap"}});
  EXPECT_EQ(b.request("/api/teach/begin",json::object()).result_int(),409U);
  EXPECT_EQ(fixture->commands,0);EXPECT_EQ(fixture->holds,0);EXPECT_FALSE(b.state()["hardware_enabled"]);
}
TEST_F(ConsoleTest, TeachingThenAutonomousMissionSurvivesBrowserDisconnectAndParksLast) {
  Temporary temp;Child child(temp.root,true,18188);ASSERT_TRUE(startup(18188));Browser b(18188);
  b.post("/api/map/select",{{"map_id","testmap"}});
  ASSERT_TRUE(b.wait([](const json & s){return s["health"]["map_matches"]==true&&s["health"]["localization"]==true&&s["health"]["still"]==true;}));
  b.begin();b.step(141.5);b.step(142.5);const auto p1=b.confirm("point","第一观察点");
  b.begin();b.step(141.5);b.step(140.5);const auto p2=b.confirm("point","第二观察点");
  b.begin();b.step(140.5);b.confirm("navigation");
  const auto task=b.post("/api/tasks/save",{{"task",{{"name","隔离两点巡检"},{"point_ids",{p1["id"],p2["id"]}},{"final_parking_pose",{{"x",1.2},{"y",1.0},{"yaw",0}}}}}});
  const auto started=b.post("/api/run/start",{{"task_id",task["id"]},{"request_id","isolated_mission_001"}});
  EXPECT_EQ(b.post("/api/run/start",{{"task_id",task["id"]},{"request_id","isolated_mission_001"}})["id"],started["id"]);
  b.socket.next_layer().close();
  asio::io_context io;const auto until=clock_now()+15;json finished;
  while(clock_now()<until){std::this_thread::sleep_for(std::chrono::milliseconds(100));const auto res=b.request("/api/state");finished=json::parse(res.body());if(finished["run"]["status"]!="RUNNING")break;}
  ASSERT_EQ(finished["run"]["status"],"SUCCEEDED")<<finished.dump();
  EXPECT_EQ(finished["run"]["captures"].size(),2U);EXPECT_EQ(finished["run"]["parking_status"],"SUCCEEDED");EXPECT_EQ(fixture->goals,3);EXPECT_TRUE(finished["motion_held"]);
  for(const auto & photo:finished["run"]["captures"])EXPECT_TRUE(std::filesystem::is_regular_file(temp.root/"data/media"/(photo.at("id").get<std::string>()+".jpg")));
}
TEST_F(ConsoleTest, RealBrowserCompletesStepwiseTeachingOnTabletAndPhone) {
  if(!std::getenv("INSPECTION_BROWSER_RUNTIME"))GTEST_SKIP()<<"未提供真实浏览器运行入口";
  Temporary temp;Child child(temp.root,true,18189);ASSERT_TRUE(startup(18189));
  const std::string address="http://127.0.0.1:18189",directory=(temp.root/"browser").string();
  std::vector<std::string> args{NODE_EXE,BROWSER_TEST,address,directory};std::vector<char*> raw;for(auto & arg:args)raw.push_back(arg.data());raw.push_back(nullptr);
  pid_t browser=-1;ASSERT_EQ(posix_spawn(&browser,raw[0],nullptr,nullptr,raw.data(),environ),0);
  int status=0;bool done=false;for(int i=0;i<1200;++i){if(waitpid(browser,&status,WNOHANG)==browser){done=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(25));}
  if(!done){kill(browser,SIGKILL);waitpid(browser,&status,0);}ASSERT_TRUE(done);ASSERT_TRUE(WIFEXITED(status));ASSERT_EQ(WEXITSTATUS(status),0);
  const auto evidence=std::filesystem::path("/tmp/carcar_app001_browser");std::filesystem::create_directories(evidence);
  for(const auto & name:{"tablet.png","phone.png"})std::filesystem::copy_file(temp.root/"browser"/name,evidence/name,std::filesystem::copy_options::overwrite_existing);
}
TEST_F(ConsoleTest, StaleLayerEvidenceCannotReleaseCameraHoldOrStartNavigation) {
  Temporary temp;Child child(temp.root,true,18190);ASSERT_TRUE(startup(18190));Browser b(18190);
  const auto task=b.task_one_point();fixture->publish_layers=false;
  b.post("/api/run/start",{{"task_id",task["id"]},{"request_id","stale_layers_001"}});
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  EXPECT_EQ(fixture->goals,0);EXPECT_TRUE(b.state()["motion_held"]);
  b.post("/api/run/cancel");ASSERT_TRUE(b.wait([](const json & s){return s["run"]["status"]=="CANCELED";}));
}
TEST_F(ConsoleTest, NavigationFailureStopsWithoutParkingAndUsesOneRedAlarmWindow) {
  Temporary temp;Child child(temp.root,true,18191);ASSERT_TRUE(startup(18191));Browser b(18191);
  const auto task=b.task_one_point();fixture->fail_navigation=true;
  b.post("/api/run/start",{{"task_id",task["id"]},{"request_id","navigation_failure_001"}});
  ASSERT_TRUE(b.wait([](const json & s){return s["run"]["status"]=="FAILED";},12));
  EXPECT_EQ(fixture->goals,2);EXPECT_TRUE(b.state()["run"]["captures"].empty());
  EXPECT_EQ(b.state()["run"]["missed"].size(),1U);EXPECT_EQ(b.state()["run"]["parking_status"],"FAILED");
  EXPECT_GT(fixture->red_samples,0);EXPECT_EQ(fixture->yellow_samples,0);EXPECT_EQ(fixture->beep_windows,1);
  EXPECT_EQ(fixture->last_red,0);EXPECT_EQ(fixture->last_green,0);
}
TEST_F(ConsoleTest, PlanningSketchDoesNotSeedPointsOrStartNavigation) {
  Temporary temp;Child child(temp.root,false,18193);ASSERT_TRUE(startup(18193));Browser b(18193);
  b.post("/api/plan/floor");
  ASSERT_TRUE(b.wait([](const json & s){return s["map"].value("planning",false)==true;}));
  EXPECT_TRUE(b.state()["points"].empty());
  const auto saved=b.post("/api/points/save",{{"point",{{"name","自定义点"},{"pose",{{"x",4.0},{"y",3.0},{"yaw",0.2}}},{"target_degrees",140.5}}}});
  EXPECT_EQ(saved["name"],"自定义点");
  b.post("/api/points/rename",{{"id",saved["id"]},{"name","门口"}});
  EXPECT_EQ(b.state()["points"][0]["name"],"门口");
  const auto denied=b.request("/api/run/start",{{"task_id","missing"},{"request_id","plan_blocked"}});
  EXPECT_EQ(denied.result_int(),409);
  EXPECT_NE(denied.body().find("规划草图"),std::string::npos);
  b.post("/api/points/remove",{{"id",saved["id"]}});
  EXPECT_TRUE(b.state()["points"].empty());
}
TEST_F(ConsoleTest, PhotoWriteFailureStopsWithoutParkingAndUsesOneYellowAlarmWindow) {
  Temporary temp;Child child(temp.root,true,18192);ASSERT_TRUE(startup(18192));Browser b(18192);
  const auto task=b.task_one_point();
  struct ReadOnlyMedia {
    std::filesystem::path path;
    explicit ReadOnlyMedia(std::filesystem::path value):path(std::move(value)){std::filesystem::permissions(path,std::filesystem::perms::owner_read|std::filesystem::perms::owner_exec);}
    ~ReadOnlyMedia(){std::error_code error;std::filesystem::permissions(path,std::filesystem::perms::owner_all,error);}
  } media(temp.root/"data/media");
  b.post("/api/run/start",{{"task_id",task["id"]},{"request_id","photo_failure_001"}});
  ASSERT_TRUE(b.wait([](const json & s){return s["run"]["status"]=="FAILED";},8));
  EXPECT_EQ(fixture->goals,1);EXPECT_TRUE(b.state()["run"]["captures"].empty());
  EXPECT_EQ(fixture->red_samples,0);EXPECT_GT(fixture->yellow_samples,0);EXPECT_EQ(fixture->beep_windows,1);
  EXPECT_EQ(fixture->last_red,0);EXPECT_EQ(fixture->last_green,0);
}
}  // namespace
