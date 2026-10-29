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
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <carcar_interfaces/srv/set_motion_hold.hpp>
#include <carcar_interfaces/srv/set_camera_servo.hpp>
#include <carcar_interfaces/msg/indicator_command.hpp>
#include <nlohmann/json.hpp>
#include "carcar_mapping_console/http_server.hpp"
#include "carcar_mapping_console/task_process.hpp"
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
  Child(const std::filesystem::path & root,bool hardware,unsigned port,bool goal_navigation=true,
      const std::string & mapping_status_url="",double navigation_timeout=180.0,bool require_depth=true) {
    std::vector<std::string> args{CONSOLE_EXE,"--ros-args","-p","bind_address:=127.0.0.1","-p","port:="+std::to_string(port),
      "-p","data_root:="+(root/"data").string(),"-p","map_root:="+(root/"maps").string(),"-p","web_root:=" WEB_ROOT,
      "-p",std::string("enable_hardware:=")+(hardware?"true":"false"),"-p",std::string("enable_navigation:=")+(hardware?"true":"false"),
      "-p",std::string("enable_goal_navigation:=")+(hardware&&goal_navigation?"true":"false"),
      "-p",std::string("require_depth:=")+(require_depth?"true":"false"),
      "-p","settle_seconds:=0.2","-p","stop_seconds:=0.2","-p","navigation_timeout_seconds:="+std::to_string(navigation_timeout),"-p",
      "mapping_status_url:="+(mapping_status_url.empty()?std::string("\"\""):mapping_status_url)};
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
    controller=std::make_shared<rclcpp::Node>("controller_server");
    controller->declare_parameter("FollowPath.max_vel_x",.18);
    controller->declare_parameter("FollowPath.max_speed_xy",.18);
    controller->declare_parameter("FollowPath.min_speed_theta",0.0);
    speed_callback=controller->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &){
      rcl_interfaces::msg::SetParametersResult result;result.successful=!reject_speed;result.reason="隔离控制器拒绝切档";return result;});
    commanded=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr value){command_vx=value->linear.x;});
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
    global_path=create_publisher<nav_msgs::msg::Path>("/plan",1);
    local_path=create_publisher<nav_msgs::msg::Path>("/local_plan",1);
    ready=create_publisher<std_msgs::msg::Bool>("/localization_monitor/ready",rclcpp::QoS(1).reliable().transient_local());
    amcl_pose=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",1);
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
    load_service=create_service<nav2_msgs::srv::LoadMap>("/map_server/load_map",[this](const nav2_msgs::srv::LoadMap::Request::SharedPtr req,nav2_msgs::srv::LoadMap::Response::SharedPtr res){
      loaded_path=req->map_url;++loads;res->result=nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
    });
    navigation=rclcpp_action::create_server<Nav>(this,"navigate_to_pose",
      [](const rclcpp_action::GoalUUID &,std::shared_ptr<const Nav::Goal>){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](const std::shared_ptr<Goal>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Goal> goal){active=goal;goal_at=clock_now();++goals;});
    timer=create_wall_timer(std::chrono::milliseconds(25),[this]{publish();});
  }
  void publish(){const auto stamp=now();
    if(active){if(active->is_canceling()){active->canceled(std::make_shared<Nav::Result>());active.reset();}
      else if(clock_now()-goal_at>goal_duration){if(fail_navigation)active->abort(std::make_shared<Nav::Result>());else {position=active->get_goal()->pose.pose;position.position.x+=arrival_offset;
        const auto q=position.orientation;const double heading=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+arrival_yaw_offset;
        position.orientation.z=std::sin(heading/2);position.orientation.w=std::cos(heading/2);
        active->succeed(std::make_shared<Nav::Result>());}active.reset();}}
    geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";t.header.stamp=stamp;t.transform.translation.x=position.position.x;t.transform.translation.y=position.position.y;t.transform.rotation=position.orientation;tf.sendTransform(t);
    nav_msgs::msg::Odometry o;o.header.stamp=stamp;o.header.frame_id="odom";o.child_frame_id="base_footprint";odom->publish(o);
    sensor_msgs::msg::LaserScan s;s.header.stamp=stamp;s.header.frame_id="laser_frame";s.range_min=.1;s.range_max=10;s.angle_increment=.1;s.ranges.assign(30,valid_scan?2.f:NAN);scan->publish(s);
    sensor_msgs::msg::Image image;image.header.stamp=stamp;image.header.frame_id="camera_color_optical_frame";image.width=32;image.height=24;image.step=96;image.encoding="rgb8";image.data.assign(32*24*3,120);rgb->publish(image);
    if(publish_depth){
    sensor_msgs::msg::PointCloud2 cloud;cloud.header.stamp=stamp;cloud.header.frame_id="camera_depth_optical_frame";cloud.width=1;cloud.height=1;cloud.point_step=16;cloud.row_step=16;cloud.data.resize(16);raw->publish(cloud);obstacles->publish(cloud);clearing->publish(cloud);
    diagnostic_msgs::msg::DiagnosticArray diagnostic;diagnostic.header.stamp=stamp;diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="D435DepthFilter";diagnostic_msgs::msg::KeyValue count;count.key="valid_measurements";count.value="1";status.values.push_back(count);diagnostic.status.push_back(status);filter_diag->publish(diagnostic);
    status.name="depth_stvl";status.values.clear();count.key="source_current";count.value="true";status.values.push_back(count);count.key="update_seq";count.value=std::to_string(++layer_sequence);status.values.push_back(count);diagnostic.status={status};if(publish_layers){local_diag->publish(diagnostic);global_diag->publish(diagnostic);}
    }
    nav_msgs::msg::OccupancyGrid grid;grid.header.stamp=stamp;grid.header.frame_id="map";grid.info.width=grid.info.height=20;grid.info.resolution=.1;grid.info.origin.orientation.w=1;grid.data.assign(400,0);map->publish(grid);
    if(display_navigation_layers)grid.data[2*20+2]=50;
    global->publish(grid);grid.data.assign(400,0);
    if(display_navigation_layers)grid.data[2*20+4]=50;
    local->publish(grid);
    if(display_navigation_layers){
      nav_msgs::msg::Path path;path.header=grid.header;
      for(double x:{.2,.8}){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=x;p.pose.position.y=1.5;p.pose.orientation.w=1;path.poses.push_back(p);}
      global_path->publish(path);
      for(auto & p:path.poses)p.pose.position.y=1.7;
      local_path->publish(path);
    }
    std_msgs::msg::Bool flag;flag.data=true;ready->publish(flag);if(publish_depth)perception->publish(flag);flag.data=false;fault->publish(flag);
    geometry_msgs::msg::PoseWithCovarianceStamped localized;localized.header.frame_id="map";localized.header.stamp=stamp;localized.pose.pose=position;amcl_pose->publish(localized);
    geometry_msgs::msg::Twist velocity;if(active)velocity.linear.x=nav_command.load();nav_velocity->publish(velocity);
  }
  std::atomic<bool> display_navigation_layers{false};
  rclcpp::Node::SharedPtr controller;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr speed_callback;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commanded;
  std::atomic<bool> reject_speed{false};
  std::atomic<double> nav_command{.05},command_vx{0};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path,local_path;
  std::atomic<int> commands{0},holds{0},goals{0};int board=94;bool held=false;double goal_at=0;
  std::atomic<bool> publish_layers{true},publish_depth{true},valid_scan{true},fail_navigation{false};
  std::atomic<int> loads{0};std::string loaded_path;
  std::atomic<double> goal_duration{.25},arrival_offset{0},arrival_yaw_offset{0};
  std::atomic<int> red_samples{0},yellow_samples{0},beep_windows{0},last_red{-1},last_green{-1};
  geometry_msgs::msg::Pose position=[](){geometry_msgs::msg::Pose p;p.position.x=p.position.y=1;p.orientation.w=1;return p;}();
  std::shared_ptr<Goal> active;tf2_ros::TransformBroadcaster tf;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles,clearing;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr filter_diag,local_diag,global_diag;uint64_t layer_sequence=0;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map,local,global;rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready,fault,perception;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_velocity;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose;
  rclcpp::Service<Hold>::SharedPtr hold_service;rclcpp::Service<Servo>::SharedPtr servo_service;rclcpp::Service<Clear>::SharedPtr local_service,global_service;
  rclcpp_action::Server<Nav>::SharedPtr navigation;rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_service;
  rclcpp::Subscription<carcar_interfaces::msg::IndicatorCommand>::SharedPtr indicator;
};
struct Browser {
  asio::io_context io;unsigned short port;ws::stream<tcp::socket> socket{io};std::string session,prefix;
  explicit Browser(unsigned p,std::string path_prefix=""):port(p),prefix(std::move(path_prefix)){socket.next_layer().connect({asio::ip::make_address("127.0.0.1"),port});socket.handshake("127.0.0.1:"+std::to_string(port),prefix+"/ws/control");beast::flat_buffer b;socket.read(b);session=json::parse(beast::buffers_to_string(b.data())).at("session");}
  void drain(){pollfd f{socket.next_layer().native_handle(),POLLIN,0};while(poll(&f,1,0)>0){beast::flat_buffer b;socket.read(b);}}
  http::response<http::string_body> request(const std::string & target,json body=nullptr){tcp::socket connection(io);connection.connect({asio::ip::make_address("127.0.0.1"),port});http::request<http::string_body> req(body.is_null()?http::verb::get:http::verb::post,prefix+target,11);req.set(http::field::host,"127.0.0.1:"+std::to_string(port));
    if(!body.is_null()){body["session"]=session;req.set("X-Mapping-Console","1");req.body()=body.dump();req.prepare_payload();}http::write(connection,req);beast::flat_buffer b;http::response<http::string_body> res;http::read(connection,b,res);return res;}
  json state(){drain();const auto res=request("/api/state");return json::parse(res.body());}
  json post(const std::string & target,json body=json::object()){const auto res=request(target,std::move(body));if(res.result_int()!=200)throw std::runtime_error(res.body());return json::parse(res.body());}
  bool wait(const std::function<bool(const json &)> & predicate,double timeout=4){const auto until=clock_now()+timeout;while(clock_now()<until){if(predicate(state()))return true;std::this_thread::sleep_for(std::chrono::milliseconds(25));}return false;}
  void begin(){post("/api/teach/begin");if(!wait([](const json & s){return s["teaching"]["phase"]=="READY";}))throw std::runtime_error("未进入READY");}
  void step(double value){const auto s=state();post("/api/teach/step",{{"revision",s["teaching"]["revision"]},{"target_degrees",value}});if(!wait([](const json & s){return s["teaching"]["phase"]=="AWAITING_CONFIRMATION";}))throw std::runtime_error("未取得新画面");}
  json confirm(const std::string & purpose,const std::string & name="测试观察点",const std::string & trial_id="") {
    const auto s=state();const auto image=request("/api/video.jpg");const auto sequence=std::stoull(std::string(image["X-Frame-Sequence"]));const auto revision=s["teaching"]["revision"];
    post("/api/teach/viewed",{{"revision",revision},{"frame_sequence",sequence}});
    json body{{"purpose",purpose},{"name",name},{"revision",revision},{"frame_sequence",sequence}};
    if(!trial_id.empty())body["trial_id"]=trial_id;
    return post("/api/teach/confirm",body);
  }
  void prepare_navigation_view(){
    post("/api/map/select",{{"map_id","testmap"}});
    if(!wait([](const json & s){return s["health"]["map_matches"]==true&&s["health"]["localization"]==true&&s["health"]["still"]==true;}))throw std::runtime_error("隔离定位未就绪");
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
  void SetUp()override{rclcpp::init(0,nullptr);fixture=std::make_shared<Fixture>();executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();executor->add_node(fixture);executor->add_node(fixture->controller);thread=std::thread([this]{executor->spin();});}
  void TearDown()override{executor->cancel();thread.join();executor.reset();fixture.reset();rclcpp::shutdown();}
  std::shared_ptr<Fixture> fixture;std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;std::thread thread;
};
TEST_F(ConsoleTest, ConfigurationModeNeverRequestsActuatorCommands) {
  Temporary temp;Child child(temp.root,false,18187);ASSERT_TRUE(startup(18187));Browser b(18187);
  ASSERT_TRUE(b.wait([](const json & s){return s["health"]["hardware_interface"]==true;}));
  b.post("/api/map/select",{{"map_id","testmap"}});
  EXPECT_EQ(b.request("/api/teach/begin",json::object()).result_int(),409U);
  EXPECT_EQ(fixture->commands,0);EXPECT_EQ(fixture->holds,0);EXPECT_FALSE(b.state()["hardware_enabled"]);
  EXPECT_EQ(b.request("/api/navigation/speed",{{"max_linear_mps",.30}}).result_int(),409U);
}
TEST_F(ConsoleTest, PureLaserGoalDoesNotRequireDepthAndActivationHoldsWithoutMoving) {
  fixture->publish_depth=false;
  Temporary temp;Child child(temp.root,true,18228,true,"",180,false);ASSERT_TRUE(startup(18228));Browser b(18228);
  b.prepare_navigation_view();EXPECT_FALSE(b.state()["require_depth"]);
  const json pose{{"x",1.3},{"y",1.0},{"yaw",0}};
  b.post("/api/teach/navigate",{{"request_id","laser-only-goal"},{"pose",pose}});
  ASSERT_TRUE(b.wait([](const json&s){return s["navigation_trial"]["phase"]=="ARRIVED";},8));
  EXPECT_EQ(fixture->goals,1);
  ASSERT_TRUE(b.wait([](const json&s){return s["health"]["still"]==true;}));
  b.post("/api/maps/activate",{{"map_id","testmap"}});
  ASSERT_TRUE(b.wait([](const json&s){return s["map_activation"]["phase"]=="WAITING_LOCALIZATION";}));
  EXPECT_EQ(fixture->loads,1);EXPECT_TRUE(b.state()["motion_held"]);
  EXPECT_EQ(fixture->goals,1);EXPECT_DOUBLE_EQ(fixture->command_vx,0);
  EXPECT_FALSE(b.state()["health"]["localization"]);
  EXPECT_EQ(b.request("/api/teach/navigate",{{"request_id","needs-new-localization"},{"pose",pose}}).result_int(),409U);
  b.post("/api/localize",{{"pose",pose}});ASSERT_TRUE(b.wait([](const json&s){return s["map_activation"]["phase"]=="READY";}));
  fixture->valid_scan=false;ASSERT_TRUE(b.wait([](const json&s){return s["health"]["scan"]==false;}));
  EXPECT_EQ(b.request("/api/teach/navigate",{{"request_id","invalid-laser"},{"pose",pose}}).result_int(),409U);EXPECT_EQ(fixture->goals,1);
}
TEST_F(ConsoleTest, RealMappingBrowserReviewsAndActivatesThroughIndependentInspectionSession) {
  if(!std::getenv("INSPECTION_BROWSER_RUNTIME"))GTEST_SKIP()<<"未提供真实浏览器运行入口";
  fixture->publish_depth=false;Temporary temp;
  const auto sessions=temp.root/"maps/mapping_sessions",candidate=sessions/"candidate";std::filesystem::create_directories(candidate);
  std::filesystem::copy_file(temp.root/"maps/testmap.pgm",candidate/"map.pgm");
  std::ofstream(candidate/"map.yaml")<<"image: map.pgm\nmode: trinary\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  std::ofstream(candidate/"graph.data")<<"isolated graph data";std::ofstream(candidate/"graph.posegraph")<<"isolated graph";
  json manifest{{"complete",true},{"slam_version","2.6.10"},{"base_frame","base_footprint"},{"review_required",true},{"range_m",3.0},{"resolution",.1}};
  for(const auto&name:{"map.yaml","map.pgm","graph.data","graph.posegraph"})manifest["sha256"][name]=carcar_mapping_console::sha256(candidate/name);
  std::ofstream(candidate/"manifest.json")<<manifest.dump();std::ofstream(candidate/"publication.json")<<"{\"status\":\"candidate\"}";
  Child child(temp.root,true,18230,true,"",180,false);ASSERT_TRUE(startup(18230));
  const auto build_root=std::filesystem::path(CONSOLE_EXE).parent_path().parent_path();
  const auto mapping_web=std::filesystem::path(WEB_ROOT).parent_path().parent_path().parent_path()/"carcar_mapping_console/web";
  carcar_mapping_console::TaskProcess gateway;
  gateway.start({(build_root/"carcar_mapping_console/mapping_web_gateway").string(),"--ros-args","-p","bind_address:=127.0.0.1","-p","port:=18229","-p","inspection_port:=18230","-p","mode:=standby","-p","autonomy_enabled:=true","-p","backend_hardware:=false","-p","map_root:="+sessions.string(),"-p","web_root:="+mapping_web.string()},temp.root/"gateway.log");
  ASSERT_TRUE(startup(18229));
  auto directory=temp.root/"publication-browser";
  if(const auto evidence=std::getenv("NAV018_BROWSER_EVIDENCE"))directory=std::filesystem::path(evidence)/temp.root.filename();
  std::vector<std::string> args{NODE_EXE,(std::filesystem::path(BROWSER_TEST).parent_path()/"mapping-autonomy.mjs").string(),"http://127.0.0.1:18229",directory.string(),(build_root/"carcar_mapping_console/mapping_test_chromium").string()};
  std::vector<char*> raw;for(auto&arg:args)raw.push_back(arg.data());raw.push_back(nullptr);pid_t browser=-1;ASSERT_EQ(posix_spawn(&browser,raw[0],nullptr,nullptr,raw.data(),environ),0);
  int status=0;bool done=false;for(int i=0;i<1000;++i){if(waitpid(browser,&status,WNOHANG)==browser){done=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(25));}
  if(!done){kill(browser,SIGKILL);waitpid(browser,&status,0);}ASSERT_TRUE(done);ASSERT_TRUE(WIFEXITED(status));ASSERT_EQ(WEXITSTATUS(status),0);
  EXPECT_EQ(fixture->loads,1);EXPECT_EQ(fixture->goals,0);EXPECT_EQ(fixture->commands,0);EXPECT_DOUBLE_EQ(fixture->command_vx,0);
}
TEST_F(ConsoleTest, NavigationSpeedChangesEveryTierAndRejectsUnknownLimitsWithoutMoving) {
  Temporary temp;Child child(temp.root,true,18220);ASSERT_TRUE(startup(18220));Browser b(18220);
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_speed"]["ready"]==true&&s["health"]["still"]==true;}));
  EXPECT_EQ(b.state()["navigation_speed"]["applied_mps"],.18);
  for(const double limit:{.30,.10,.25,.18}) {
    b.post("/api/navigation/speed",{{"max_linear_mps",limit}});
    ASSERT_TRUE(b.wait([limit](const json & s){return s["navigation_speed"]["ready"]==true&&s["navigation_speed"]["applied_mps"]==limit;}));
    EXPECT_DOUBLE_EQ(fixture->controller->get_parameter("FollowPath.max_vel_x").as_double(),limit);
    EXPECT_DOUBLE_EQ(fixture->controller->get_parameter("FollowPath.max_speed_xy").as_double(),limit);
  }
  for(const double limit:{0.0,.19,.31,1.0})EXPECT_EQ(b.request("/api/navigation/speed",{{"max_linear_mps",limit}}).result_int(),409U);
  EXPECT_EQ(fixture->goals,0);EXPECT_EQ(fixture->commands,0);EXPECT_DOUBLE_EQ(fixture->command_vx,0);
}
TEST_F(ConsoleTest, RejectedSpeedCannotStartGoalAndRetryConfirmsControllerParameters) {
  Temporary temp;Child child(temp.root,true,18221);ASSERT_TRUE(startup(18221));Browser b(18221);b.prepare_navigation_view();
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_speed"]["ready"]==true;}));
  fixture->reject_speed=true;b.post("/api/navigation/speed",{{"max_linear_mps",.25}});
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_speed"]["phase"]=="FAILED";}));
  EXPECT_EQ(b.request("/api/teach/navigate",{{"request_id","speed_rejected_001"},{"pose",{{"x",1.2},{"y",1.0},{"yaw",0.0}}}}).result_int(),409U);
  EXPECT_EQ(fixture->goals,0);EXPECT_DOUBLE_EQ(fixture->command_vx,0);
  fixture->reject_speed=false;b.post("/api/navigation/speed",{{"max_linear_mps",.25}});
  EXPECT_TRUE(b.wait([](const json & s){return s["navigation_speed"]["applied_mps"]==.25;}));
}
TEST_F(ConsoleTest, SpeedParameterTimeoutDoesNotBlockWebStatusOrSendGoal) {
  executor->remove_node(fixture->controller);
  Temporary temp;Child child(temp.root,true,18223);ASSERT_TRUE(startup(18223));Browser b(18223);
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_speed"]["phase"]=="FAILED";},5));
  const auto started=clock_now();const auto s=b.state();
  EXPECT_LT(clock_now()-started,.5);EXPECT_FALSE(s["navigation_speed"]["ready"].get<bool>());
  EXPECT_NE(s["navigation_speed"]["message"].get<std::string>().find("1.5秒"),std::string::npos);
  EXPECT_EQ(fixture->goals,0);EXPECT_DOUBLE_EQ(fixture->command_vx,0);
}
TEST_F(ConsoleTest, SpeedCannotChangeInFlightAndFinalCommandHonorsSelectedTier) {
  Temporary temp;Child child(temp.root,true,18222);ASSERT_TRUE(startup(18222));Browser b(18222);b.prepare_navigation_view();
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_speed"]["ready"]==true;}));
  fixture->goal_duration=10;fixture->nav_command=.8;
  for(const double limit:{.10,.30}) {
    b.post("/api/navigation/speed",{{"max_linear_mps",limit}});
    ASSERT_TRUE(b.wait([limit](const json & s){return s["navigation_speed"]["applied_mps"]==limit;}));
    b.post("/api/teach/navigate",{{"request_id",limit==.10?"speed_low_001":"speed_high_001"},{"pose",{{"x",1.2},{"y",1.0},{"yaw",0.0}}}});
    ASSERT_TRUE(b.wait([this,limit](const json &){return std::abs(fixture->command_vx.load()-limit)<1e-9;}));
    EXPECT_EQ(b.request("/api/navigation/speed",{{"max_linear_mps",.25}}).result_int(),409U);
    b.post("/api/teach/cancel-navigation");
    ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="CANCELED";}));
    EXPECT_DOUBLE_EQ(fixture->command_vx,0);
  }
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
  fixture->display_navigation_layers=true;
  fixture->arrival_offset=.12;
  fixture->arrival_yaw_offset=.16;
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

TEST_F(ConsoleTest, SingleGoalArrivesBeforeTeachingAndSavesMeasuredPose) {
  Temporary temp;Child child(temp.root,true,18195);ASSERT_TRUE(startup(18195));Browser b(18195);
  b.prepare_navigation_view();fixture->arrival_offset=.03;
  EXPECT_TRUE(b.state()["navigation_view"].is_null());
  EXPECT_EQ(b.state()["goal_navigation_block_reason"],"");
  const json target{{"x",1.3},{"y",1.0},{"yaw",.2}};
  const json request{{"request_id","single_goal_measure_001"},{"pose",target}};
  b.post("/api/teach/navigate",request);
  EXPECT_EQ(b.post("/api/teach/navigate",request)["request_id"],request["request_id"]);
  EXPECT_EQ(b.request("/api/teach/begin",json::object()).result_int(),409U);
  EXPECT_EQ(b.request("/api/map/select",{{"map_id","testmap"}}).result_int(),409U);
  EXPECT_EQ(b.request("/api/localize",{{"pose",target}}).result_int(),409U);
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="ARRIVED";},8));
  EXPECT_EQ(fixture->goals,1);EXPECT_TRUE(b.state()["motion_held"]);EXPECT_TRUE(b.state()["run"].is_null());
  EXPECT_DOUBLE_EQ(b.state()["navigation_trial"]["target"]["x"].get<double>(),1.3);
  EXPECT_NEAR(b.state()["navigation_trial"]["reached_pose"]["x"].get<double>(),1.33,1e-6);
  b.begin();b.step(141.5);
  const auto saved=b.confirm("point","导航到点后确认","single_goal_measure_001");
  EXPECT_NEAR(saved["pose"]["x"].get<double>(),1.33,1e-6);
  ASSERT_TRUE(saved.contains("reference"))<<saved.dump();
  EXPECT_DOUBLE_EQ(saved["reference"]["navigation_target"]["x"].get<double>(),1.3);
  EXPECT_EQ(saved["reference"]["navigation_request_id"],"single_goal_measure_001");
  EXPECT_EQ(b.request("/api/media/"+saved["reference"]["id"].get<std::string>()).result_int(),200U);
}
TEST_F(ConsoleTest, Nav2SuccessNearGoalCanTeachAndSaveActualParkingPose) {
  Temporary temp;Child child(temp.root,true,18224);ASSERT_TRUE(startup(18224));Browser b(18224);
  b.prepare_navigation_view();fixture->arrival_offset=.12;fixture->arrival_yaw_offset=.16;
  const json target{{"x",1.3},{"y",1.0},{"yaw",.2}};
  b.post("/api/teach/navigate",{{"request_id","nav2_success_offset_001"},{"pose",target}});
  ASSERT_TRUE(b.wait([](const json & s){const auto phase=s["navigation_trial"]["phase"];return phase=="ARRIVED"||phase=="FAILED";},8));
  const auto trial=b.state()["navigation_trial"];
  ASSERT_EQ(trial["phase"],"ARRIVED")<<trial.dump();
  EXPECT_EQ(trial["nav2_result"],"SUCCEEDED");
  EXPECT_NEAR(trial["arrival_error"]["distance_m"].get<double>(),.12,1e-6);
  EXPECT_NEAR(trial["arrival_error"]["yaw_rad"].get<double>(),.16,1e-6);
  EXPECT_TRUE(b.state()["motion_held"]);EXPECT_EQ(fixture->goals,1);
  b.begin();b.step(141.5);
  const auto saved=b.confirm("point","接受导航后实际停车位置","nav2_success_offset_001");
  EXPECT_NEAR(saved["pose"]["x"].get<double>(),1.42,1e-6);
  EXPECT_DOUBLE_EQ(saved["reference"]["navigation_target"]["x"].get<double>(),1.3);
  EXPECT_TRUE(saved["verified"]);
  EXPECT_EQ(saved["reference"]["navigation_result"],"SUCCEEDED");
}
TEST_F(ConsoleTest, Nav2AbortedIsReportedSeparatelyAndCannotBeConfirmedAsArrival) {
  Temporary temp;Child child(temp.root,true,18225);ASSERT_TRUE(startup(18225));Browser b(18225);b.prepare_navigation_view();
  fixture->fail_navigation=true;
  b.post("/api/teach/navigate",{{"request_id","nav2_aborted_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0.0}}}});
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="FAILED";},8));
  const auto trial=b.state()["navigation_trial"];
  EXPECT_EQ(trial["nav2_result"],"ABORTED");EXPECT_EQ(trial["failure_code"],"NAV2_ABORTED");
  EXPECT_NE(trial["error"].get<std::string>().find("ABORTED"),std::string::npos);
  EXPECT_EQ(trial["error"].get<std::string>().find("超时或"),std::string::npos);
  b.begin();b.step(141.5);
  EXPECT_THROW(b.confirm("point","不能伪造导航成功","nav2_aborted_001"),std::runtime_error);
  EXPECT_TRUE(b.state()["points"].empty());EXPECT_TRUE(b.state()["motion_held"]);
}
TEST_F(ConsoleTest, NavigationTimeoutHasItsOwnReasonAndCancelsBeforeTeaching) {
  Temporary temp;Child child(temp.root,true,18226,true,"",10.0);ASSERT_TRUE(startup(18226));Browser b(18226);b.prepare_navigation_view();
  fixture->goal_duration=20;
  b.post("/api/teach/navigate",{{"request_id","nav2_timeout_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0.0}}}});
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="FAILED";},14));
  const auto trial=b.state()["navigation_trial"];
  EXPECT_EQ(trial["failure_code"],"NAVIGATION_TIMEOUT");
  EXPECT_EQ(trial["nav2_result"],"CANCELED");
  EXPECT_NE(trial["error"].get<std::string>().find("超过10秒"),std::string::npos);
  EXPECT_EQ(trial["error"].get<std::string>().find("到达误差"),std::string::npos);
  EXPECT_TRUE(b.state()["motion_held"]);EXPECT_FALSE(trial["active"]);
}
TEST_F(ConsoleTest, SingleGoalCancelThenRefineAndNavigateAgain) {
  Temporary temp;Child child(temp.root,true,18196);ASSERT_TRUE(startup(18196));Browser b(18196);
  b.prepare_navigation_view();fixture->goal_duration=20;
  const json request{{"request_id","single_goal_cancel_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0}}}};
  b.post("/api/teach/navigate",request);
  ASSERT_TRUE(b.wait([this](const json & s){return s["navigation_trial"]["phase"]=="NAVIGATING"&&fixture->goals==1;}));
  b.post("/api/teach/cancel-navigation");
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="CANCELED";}));
  EXPECT_TRUE(b.state()["motion_held"]);EXPECT_FALSE(b.state()["navigation_trial"]["active"]);
  EXPECT_EQ(b.post("/api/teach/navigate",request)["phase"],"CANCELED");EXPECT_EQ(fixture->goals,1);
  fixture->goal_duration=.25;
  b.post("/api/teach/navigate",{{"request_id","single_goal_refined_002"},{"pose",{{"x",1.4},{"y",1.0},{"yaw",.1}}}});
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="ARRIVED";},8));
  EXPECT_EQ(fixture->goals,2);EXPECT_TRUE(b.state()["motion_held"]);
}
TEST_F(ConsoleTest, SingleGoalDisconnectCancelsAndHolds) {
  Temporary temp;Child child(temp.root,true,18197);ASSERT_TRUE(startup(18197));Browser b(18197);
  b.prepare_navigation_view();fixture->goal_duration=20;
  b.post("/api/teach/navigate",{{"request_id","single_goal_disconnect_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0}}}});
  ASSERT_TRUE(b.wait([this](const json & s){return s["navigation_trial"]["phase"]=="NAVIGATING"&&fixture->goals==1;}));
  b.socket.next_layer().close();
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="CANCELED";}));
  EXPECT_TRUE(b.state()["motion_held"]);EXPECT_TRUE(b.state()["run"].is_null());
}
TEST_F(ConsoleTest, SingleGoalDisabledAndStaleLayersDoNotDispatchNavigation) {
  Temporary temp;Child child(temp.root,true,18198,false);ASSERT_TRUE(startup(18198));Browser b(18198);
  b.prepare_navigation_view();
  EXPECT_EQ(b.request("/api/teach/navigate",{{"request_id","single_goal_disabled_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0}}}}).result_int(),409U);
  EXPECT_EQ(fixture->goals,0);
}
TEST_F(ConsoleTest, SingleGoalRequiresFreshRebuiltLayersBeforeDriving) {
  Temporary temp;Child child(temp.root,true,18199);ASSERT_TRUE(startup(18199));Browser b(18199);
  b.prepare_navigation_view();fixture->publish_layers=false;
  b.post("/api/teach/navigate",{{"request_id","single_goal_stale_layers_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0}}}});
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  EXPECT_EQ(fixture->goals,0);EXPECT_TRUE(b.state()["motion_held"]);
  b.post("/api/teach/cancel-navigation");
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="CANCELED";}));
}

TEST_F(ConsoleTest, SingleGoalThroughGatewayWithStatusBackqueryDoesNotTimeout) {
  // 使用同一个真实HTTP工作线程转发巡检请求和处理建图状态，覆盖昨晚的互等链路。
  // ROS仍在隔离域；底盘、相机和导航均为Fixture替身，不连接实车。
  Temporary temp;std::atomic<int> status_reads{0};
  carcar_mapping_console::HttpServer gateway("127.0.0.1",18200,
    [&](const carcar_mapping_console::HttpRequest & request){
      carcar_mapping_console::HttpResponse response;
      const std::string body=request.target=="/api/status"?"{\"task_state\":\"STANDBY\"}":"{}";
      if(request.target=="/api/status")++status_reads;
      response.body.assign(body.begin(),body.end());return response;
    },[](const std::string &,const std::string &){});
  gateway.set_inspect_upstream("127.0.0.1",18201);
  Child child(temp.root,true,18201,true,"http://127.0.0.1:18200/api/status");
  ASSERT_TRUE(startup(18201));Browser b(18200,"/inspect");b.prepare_navigation_view();
  ASSERT_GT(status_reads.load(),0);fixture->goal_duration=20;
  const json request{{"request_id","gateway_backquery_cancel_001"},{"pose",{{"x",1.3},{"y",1.0},{"yaw",0}}}};
  const auto reads_before=status_reads.load();
  auto submit=[&](const std::string & target,const json & body){
    const auto start=clock_now();const auto response=b.request(target,body);
    const auto elapsed=clock_now()-start;
    EXPECT_EQ(response.result_int(),200U)<<response.body();
    EXPECT_LT(elapsed,1.0)<<"上位机请求等待超过1秒，检查转发与状态回查互等";
    std::cout<<"APP-001-R10 isolated target="<<target<<" status="<<response.result_int()<<" elapsed_s="<<elapsed<<'\n';
  };
  submit("/api/teach/navigate",request);
  ASSERT_TRUE(b.wait([this](const json & s){return s["navigation_trial"]["phase"]=="NAVIGATING"&&fixture->goals==1;}));
  for(int i=0;i<10;++i){submit("/api/teach/navigate",request);b.state();std::this_thread::sleep_for(std::chrono::milliseconds(60));}
  EXPECT_EQ(fixture->goals,1)<<"重复请求不得重复下发目标";
  submit("/api/teach/cancel-navigation",json::object());
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="CANCELED";}));
  EXPECT_TRUE(b.state()["motion_held"]);
  fixture->goal_duration=.25;
  submit("/api/teach/navigate",{{"request_id","gateway_backquery_arrive_002"},{"pose",{{"x",1.4},{"y",1.0},{"yaw",.1}}}});
  ASSERT_TRUE(b.wait([](const json & s){return s["navigation_trial"]["phase"]=="ARRIVED";},8));
  EXPECT_EQ(fixture->goals,2);EXPECT_TRUE(b.state()["motion_held"]);
  EXPECT_GT(status_reads.load(),reads_before);
  const auto health=b.state()["health"];EXPECT_TRUE(health["odom"]);EXPECT_TRUE(health["scan"]);EXPECT_TRUE(health["localization"]);
  std::ifstream log(temp.root/"console.log");const std::string contents((std::istreambuf_iterator<char>(log)),{});
  EXPECT_EQ(contents.find("reason=http_wait"),std::string::npos)<<contents;
}
