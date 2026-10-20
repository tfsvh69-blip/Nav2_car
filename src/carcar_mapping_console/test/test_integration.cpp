#include <gtest/gtest.h>
#include "carcar_mapping_console/common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <slam_toolbox/srv/deserialize_pose_graph.hpp>
#include <nav2_map_server/map_io.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fstream>
#include <atomic>
#include <mutex>
#include <array>
extern char ** environ;
using namespace carcar_mapping_console;
namespace asio=boost::asio;namespace beast=boost::beast;namespace http=beast::http;namespace websocket=beast::websocket;using tcp=asio::ip::tcp;
class Integration:public ::testing::Test {
protected:
  asio::io_context io; std::unique_ptr<websocket::stream<tcp::socket>> ws;
  std::shared_ptr<rclcpp::Node> node;std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;std::thread thread;
  pid_t guard=-1,gateway=-1,slam=-1;std::filesystem::path root;int port=18089;
  virtual bool real_slam() const {return false;}
  virtual bool managed_backend() const {return false;}
  virtual bool strict_video() const {return true;}
  std::atomic<double> output{0};std::atomic<bool> video{true}, save_fails{false}, emit_pose{true},emit_scan{true};std::atomic<int> nonzero{0},relay_count{0};std::string session;
  std::atomic<int> accepted_poses{0};std::mutex truth_mutex;std::array<double,6> truth{};
  void set_truth(std::array<double,6> value){std::lock_guard<std::mutex> lock(truth_mutex);truth=value;}
  rclcpp::TimerBase::SharedPtr timer;std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr observed_poses;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr relayed;
  rclcpp::Service<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serializer;
  rclcpp::Service<slam_toolbox::srv::DeserializePoseGraph>::SharedPtr deserializer;
  pid_t spawn(std::vector<std::string> args) {
    std::vector<char*> raw;for(auto&arg:args)raw.push_back(arg.data());raw.push_back(nullptr);pid_t pid;
    if(posix_spawn(&pid,raw[0],nullptr,nullptr,raw.data(),environ))throw std::runtime_error("spawn失败");
    return pid;
  }
  void kill_child(pid_t &pid){if(pid>0){kill(pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;return;}usleep(20000);}kill(pid,SIGKILL);waitpid(pid,nullptr,0);pid=-1;}}
  json request(std::string target,json body=nullptr){
    // 浏览器自动处理WebSocket ping/pong；同步测试客户端用ping维持同等存活语义。
    if(ws&&ws->is_open())ws->ping({});
    beast::tcp_stream socket(io);socket.connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),port));
    http::request<http::string_body> req(body.is_null()?http::verb::get:http::verb::post,target,11);req.set(http::field::host,"127.0.0.1:"+std::to_string(port));
    if(!body.is_null()){body["session"]=session;req.set("X-Mapping-Console","1");req.body()=body.dump();req.prepare_payload();}
    http::write(socket,req);beast::flat_buffer buffer;http::response<http::string_body> res;http::read(socket,buffer,res);
    auto value=res.body().empty()?json::object():json::parse(res.body());value["http_status"]=res.result_int();return value;
  }
  bool wait(std::function<bool()> test,double seconds=5){double end=steady()+seconds;while(steady()<end){if(test())return true;usleep(20000);}return false;}
  std::string get_video(){
    beast::tcp_stream socket(io);socket.connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),port));http::request<http::empty_body> req(http::verb::get,"/api/video.jpg",11);req.set(http::field::host,"localhost");http::write(socket,req);
    beast::flat_buffer buffer;http::response<http::vector_body<unsigned char>> res;http::read(socket,buffer,res);return std::string(res["X-Frame-Id"]);
  }
  json read_ws(){beast::flat_buffer buffer;ws->read(buffer);return json::parse(beast::buffers_to_string(buffer.data()));}
  void send(json value){ws->write(asio::buffer(value.dump()));}
  void connect_control(){ws=std::make_unique<websocket::stream<tcp::socket>>(io);ws->next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),port));ws->handshake("127.0.0.1:"+std::to_string(port),"/ws/control");auto welcome=read_ws();session=welcome.at("session");}
  void start_gateway(std::string mode="new",std::string bundle=""){
    std::vector<std::string> args{GATEWAY_EXE,"--ros-args","-p","bind_address:=127.0.0.1","-p","port:="+std::to_string(port),"-p","web_root:=" WEB_ROOT,"-p","map_root:="+root.string(),"-p","mode:="+mode};
    if(!bundle.empty()){args.push_back("-p");args.push_back("bundle_id:="+bundle);}
    if(mode=="standby") {args.insert(args.end(),{"-p","backend_hardware:=false","-p","log_root:="+(root/"logs").string()});}
    gateway=spawn(args);
    ASSERT_TRUE(wait([&]{try{return request("/api/state").contains("phase");}catch(...){return false;}},8));
  }
  void SetUp()override{
    rclcpp::init(0,nullptr);root=std::filesystem::temp_directory_path()/nonce();std::filesystem::create_directories(root);
    node=std::make_shared<rclcpp::Node>(real_slam()?"offline_mapping_driver":"slam_toolbox");
    for(auto name:{"resolution","max_laser_range","min_laser_range","map_update_interval"})node->declare_parameter(name,.05);
    for(auto name:{"scan_topic","map_frame","base_frame","odom_frame"})node->declare_parameter(name,std::string("test"));
    scan=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());image=node->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/debug/rgb_reference/compressed",rclcpp::SensorDataQoS());
    odom=node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
    if(!real_slam()) {map=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());pose=node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",1);}
    commands=node->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,[this](const geometry_msgs::msg::Twist&v){output=v.linear.x;if(std::abs(v.linear.x)>.001||std::abs(v.angular.z)>.001)nonzero++;});
    observed_poses=node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",1,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr){++accepted_poses;});
    relayed=node->create_subscription<sensor_msgs::msg::LaserScan>("/mapping/slam_scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr s){relay_count++;if(real_slam()||!emit_pose)return;geometry_msgs::msg::PoseWithCovarianceStamped p;p.header=s->header;p.header.frame_id="map";p.pose.pose.orientation.w=1;pose->publish(p);});
    if (!real_slam()) {
    serializer=node->create_service<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map",[this](std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> res){usleep(300000);if(save_fails){res->result=255;return;}std::ofstream(req->filename+".posegraph")<<"mock graph";std::ofstream(req->filename+".data")<<"mock data";res->result=0;});
    deserializer=node->create_service<slam_toolbox::srv::DeserializePoseGraph>("/slam_toolbox/deserialize_map",[](std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request>,std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response>){});
    }
    broadcaster=std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    dynamic_broadcaster=std::make_unique<tf2_ros::TransformBroadcaster>(node);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;for(auto pair:{std::make_pair("map","odom"),std::make_pair("odom","base_footprint"),std::make_pair("base_footprint","laser_frame")}){if(real_slam()&&(std::string(pair.first)=="map"||std::string(pair.first)=="odom"))continue;geometry_msgs::msg::TransformStamped t;t.header.stamp=node->now();t.header.frame_id=pair.first;t.child_frame_id=pair.second;t.transform.rotation.w=1;transforms.push_back(t);}broadcaster->sendTransform(transforms);
    timer=node->create_wall_timer(std::chrono::milliseconds(50),[this]{
      auto stamp=node->now();sensor_msgs::msg::LaserScan s;s.header.stamp=stamp;s.header.frame_id="laser_frame";s.range_min=.1;s.range_max=5;s.angle_min=-M_PI;s.angle_increment=2*M_PI/360;s.angle_max=M_PI-s.angle_increment;s.scan_time=.05;s.ranges.assign(360,2);
      std::array<double,6> t;{std::lock_guard<std::mutex> lock(truth_mutex);t=truth;}
      if(real_slam())for(size_t i=0;i<s.ranges.size();++i){double a=t[2]+s.angle_min+i*s.angle_increment,dx=std::cos(a),dy=std::sin(a);s.ranges[i]=std::min(std::abs(dx)>1e-8?((dx>0?1.5:-1.5)-t[0])/dx:1e9,std::abs(dy)>1e-8?((dy>0?2.0:-2.0)-t[1])/dy:1e9);}
      if(emit_scan)scan->publish(s);
      nav_msgs::msg::Odometry o;o.header.stamp=stamp;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.pose.pose.position.x=t[3];o.pose.pose.position.y=t[4];o.pose.pose.orientation.w=std::cos(t[5]/2);o.pose.pose.orientation.z=std::sin(t[5]/2);odom->publish(o);
      if(real_slam()){geometry_msgs::msg::TransformStamped transform;transform.header=o.header;transform.child_frame_id=o.child_frame_id;transform.transform.translation.x=t[3];transform.transform.translation.y=t[4];transform.transform.rotation=o.pose.pose.orientation;dynamic_broadcaster->sendTransform(transform);}
      if(!real_slam()){nav_msgs::msg::OccupancyGrid m;m.header.stamp=stamp;m.header.frame_id="map";m.info.resolution=.05;m.info.width=m.info.height=80;m.info.origin.position.x=m.info.origin.position.y=-2;m.info.origin.orientation.w=1;m.data.assign(6400,0);map->publish(m);}
      if(video){sensor_msgs::msg::CompressedImage im;im.header.stamp=stamp;im.format="jpeg";cv::imencode(".jpg",cv::Mat(8,8,CV_8UC3,cv::Scalar(20,60,100)),im.data);image->publish(im);}
    });
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();executor->add_node(node);thread=std::thread([this]{executor->spin();});
    if(!managed_backend()) guard=spawn({GUARD_EXE,"--ros-args","-p",strict_video()?"require_video:=true":"require_video:=false"});
    if(real_slam()&&!managed_backend())slam=spawn({"/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node","--ros-args","-p","scan_topic:=/mapping/slam_scan","-p","use_map_saver:=false","-p","map_update_interval:=0.5","-p","max_laser_range:=3.0","-p","minimum_time_interval:=0.0","-p","minimum_travel_distance:=0.1","-p","minimum_travel_heading:=0.1"});
  }
  void TearDown()override{
    ws.reset();kill_child(gateway);kill_child(guard);kill_child(slam);executor->cancel();if(thread.joinable())thread.join();executor.reset();node.reset();rclcpp::shutdown();
    if(const auto evidence=getenv("MAPPING_TEST_EVIDENCE_ROOT")) {const auto destination=std::filesystem::path(evidence)/root.filename();std::filesystem::create_directories(destination);std::filesystem::copy(root,destination,std::filesystem::copy_options::recursive);}
    std::filesystem::remove_all(root);
  }
};
class BrowserIntegration:public Integration {bool strict_video()const override{return false;}};
TEST_F(BrowserIntegration, LightweightStatusAndVersionedScanPreserveLegacyAndFreshness) {
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));
  // 任务CURRENT先于下一次投影/TF发现；等真实预览就绪后比较同种负载。
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return s.contains("scan")&&s["scan"].size()>100;}));
  const auto full=request("/api/state"),status=request("/api/status");
  ASSERT_EQ(status["http_status"],200);EXPECT_FALSE(status.contains("scan"));
  EXPECT_TRUE(full.contains("scan"));EXPECT_LT(status.dump().size(),full.dump().size()/2);
  std::cout<<"DATAFLOW_BYTES legacy_state="<<full.dump().size()<<" light_status="<<status.dump().size()<<'\n';
  const auto scan=request("/api/scan");ASSERT_EQ(scan["http_status"],200);
  ASSERT_EQ(scan.value("frame_id",""),"map");ASSERT_TRUE(scan.value("valid",false));
  ASSERT_GT(scan["points"].size(),100u);ASSERT_TRUE(scan.contains("version"));
  std::cout<<"DATAFLOW_BYTES scan="<<scan.dump().size()<<" points="<<scan["points"].size()<<'\n';
  // 同一份投影不重复传输；若恰好更新，允许收到新版本。
  const auto unchanged=request("/api/scan?version="+scan["version"].get<std::string>());
  EXPECT_TRUE(unchanged["http_status"]==204||
    (unchanged["http_status"]==200&&unchanged["version"]!=scan["version"]));
  emit_scan=false;
  ASSERT_TRUE(wait([&]{const auto stale=request("/api/scan");return !stale.value("valid",true)&&stale["points"].empty();},2));
  EXPECT_EQ(nonzero.load(),0);
}
TEST_F(BrowserIntegration, ActualBrowserEnableJoystickAndBoundedStallWithTabletLatency) {
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));
  for(const auto latency:{0,80,100}) {
    if(const auto only=getenv("MAPPING_TEST_LATENCY_MS")){
      const auto selected=std::string(only);ASSERT_TRUE(selected=="0"||selected=="80"||selected=="100");
      if(selected!=std::to_string(latency))continue;
    }
    nonzero=0;
    const auto script=getenv("MAPPING_TEST_CHROME")?BROWSER_TOUCH:BROWSER_PROTOCOL;
    auto browser=spawn({NODE_EXE,script,APP_JS,std::to_string(port),std::to_string(latency)});
    int status=0;const bool done=wait([&]{return waitpid(browser,&status,WNOHANG)==browser;},15);
    if(!done)kill_child(browser);
    ASSERT_TRUE(done)<<"网页协议测试未结束";ASSERT_TRUE(WIFEXITED(status));ASSERT_EQ(WEXITSTATUS(status),0)<<"单程时延 "<<latency<<"ms";
    EXPECT_GT(nonzero.load(),0);
    // 浏览器退出后，断开事件经ROS门控和HTTP状态缓存异步传播；仍须撤使能并保持零速。
    EXPECT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);},2));
    EXPECT_EQ(output.load(),0);
    usleep(150000);
  }
}
TEST_F(BrowserIntegration, FifoBurstDelayAndReconnectCannotReplayMotion) {
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));
  auto browser=spawn({NODE_EXE,BROWSER_PROTOCOL,APP_JS,std::to_string(port),"0","burst"});
  int status=0;const bool done=wait([&]{return waitpid(browser,&status,WNOHANG)==browser;},20);
  if(!done)kill_child(browser);
  ASSERT_TRUE(done);ASSERT_TRUE(WIFEXITED(status));ASSERT_EQ(WEXITSTATUS(status),0);
  EXPECT_GT(nonzero.load(),0);
  EXPECT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);},2));
  EXPECT_EQ(output.load(),0);
}
TEST_F(Integration, ActualWebSocketMotionTimeoutVideoAndReadOnlyInterfaces){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(700000);
  auto frame=get_video();ASSERT_FALSE(frame.empty());send({{"action","enable"},{"frame",frame}});
  ASSERT_TRUE(wait([&]{return request("/api/state")["control"].value("armed",false);}));
  uint64_t sequence=0;bool moved=false;
  for(int i=0;i<60;++i){auto status=read_ws();if(!status.contains("challenge")||!status.value("armed",false))continue;frame=get_video();send({{"action","intent"},{"sequence",++sequence},{"challenge",status["challenge"]},{"direction","forward"},{"frame",frame}});usleep(20000);if(nonzero>0){moved=true;break;}}
  ASSERT_TRUE(moved);ASSERT_TRUE(wait([&]{return output.load()==0;},1));
  ASSERT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);},1));
  EXPECT_EQ(request("/api/not-a-shell",json::object())["http_status"],404);
  frame=get_video();send({{"action","enable"},{"frame",frame}});ASSERT_TRUE(wait([&]{return request("/api/state")["control"].value("armed",false);}));
  video=false;ASSERT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);},2));
  EXPECT_EQ(output.load(),0);
}
TEST_F(Integration, SaveFreezesMotionAndProducesVerifiedBundle){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);
  EXPECT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&s.value("result","").find("保存完整")!=std::string::npos;},12));
  EXPECT_EQ(nonzero.load(),0);bool complete=false;
  for(const auto&item:std::filesystem::directory_iterator(root)){EXPECT_NO_THROW(validate_bundle(root,item.path().filename().string()));complete=true;}
  EXPECT_TRUE(complete);
}
TEST_F(Integration, SustainedFastJoystickAndStopDiagnosticsRemainObservable) {
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);
  send({{"action","gear"},{"gear","fast"}});
  ASSERT_TRUE(wait([&]{auto c=request("/api/state")["control"];return c.value("gear","")=="fast"&&c.value("health","not_ready").empty();}));
  send({{"action","enable"},{"frame",get_video()}});
  ASSERT_TRUE(wait([&]{return request("/api/state")["control"].value("armed",false);}));
  uint64_t sequence=0;const double until=steady()+3;
  while(steady()<until){auto c=request("/api/state")["control"];ASSERT_TRUE(c.value("armed",false))<<c.dump();
    send({{"action","drive"},{"sequence",++sequence},{"challenge",c["challenge"]},{"x",1.0},{"yaw",0.0},{"frame",get_video()}});usleep(30000);}
  EXPECT_NEAR(output.load(),.18,1e-9);
  send({{"action","client_event"},{"event","pointer_cancel"},{"metrics",{{"control_gap_ms",30},{"http_gap_ms",1500},{"session","do-not-log"}}}});
  send({{"action","stop"},{"cause","pointer_cancel"}});
  ASSERT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);}));
  const auto d=request("/api/diagnostics");ASSERT_TRUE(d.contains("events"));
  EXPECT_FALSE(d["control"].contains("challenge"));EXPECT_EQ(d.value("diagnostics_version",0),2);
  EXPECT_NE(d["control"]["last_stop"]["reason"].get<std::string>().find("pointer_cancel"),std::string::npos);
  EXPECT_TRUE(d["control"]["last_stop"]["timing"].contains("request_gap_ms"));
  EXPECT_EQ(d.dump().find("do-not-log"),std::string::npos);
  bool client_event=false;for(const auto&event:d["events"])if(event.value("event","")=="client_pointer_cancel")client_event=true;
  EXPECT_TRUE(client_event);
}
TEST_F(Integration, ResumeDoesNotForwardBeforeHintAndOnlyOneScanBeforeConfirmation){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);
  ASSERT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&s.value("result","").find("保存完整")!=std::string::npos;},12));
  std::string bundle;for(auto&item:std::filesystem::directory_iterator(root))bundle=item.path().filename();ASSERT_FALSE(bundle.empty());
  ws.reset();kill_child(gateway);relay_count=0;start_gateway("resume",bundle);usleep(400000);EXPECT_EQ(relay_count.load(),0);connect_control();usleep(500000);
  EXPECT_EQ(request("/api/resume",{{"pose",{0,0,0}}})["http_status"],202);
  ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CANDIDATE";}));
  usleep(400000);EXPECT_EQ(relay_count.load(),1);EXPECT_EQ(nonzero.load(),0);
  EXPECT_EQ(request("/api/save",json::object())["http_status"],409);
  EXPECT_EQ(request("/api/confirm",json::object())["http_status"],200);
  ASSERT_TRUE(wait([&]{return relay_count.load()>2;}));
}
class RealSlamIntegration : public Integration { bool real_slam() const override {return true;} };
TEST_F(RealSlamIntegration, InstalledSlamToolboxGraphSaveAndArbitraryPoseResume){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";},10));connect_control();usleep(500000);
  ASSERT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&s.value("result","").find("保存完整")!=std::string::npos;},15));
  std::string bundle;for(auto&item:std::filesystem::directory_iterator(root))bundle=item.path().filename();ASSERT_FALSE(bundle.empty());
  EXPECT_NO_THROW(validate_bundle(root,bundle));const auto before=sha256(root/bundle/"graph.posegraph");
  ws.reset();kill_child(gateway);relay_count=0;start_gateway("resume",bundle);connect_control();usleep(500000);
  set_truth({.4,.25,M_PI/2,0,0,0});usleep(200000);
  EXPECT_EQ(relay_count.load(),0);ASSERT_EQ(request("/api/resume",{{"pose",{.4,.25,M_PI/2}}})["http_status"],202);
  ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CANDIDATE";},10));
  EXPECT_EQ(relay_count.load(),1);EXPECT_EQ(nonzero.load(),0);EXPECT_EQ(sha256(root/bundle/"graph.posegraph"),before);
  const auto candidate=request("/api/state").at("pose").get<std::vector<double>>();
  EXPECT_NEAR(candidate[0],.4,.15);EXPECT_NEAR(candidate[1],.25,.15);EXPECT_NEAR(std::remainder(candidate[2]-M_PI/2,2*M_PI),0,.15);
  ASSERT_EQ(request("/api/confirm",json::object())["http_status"],200);ASSERT_TRUE(wait([&]{return relay_count.load()>3;}));
  usleep(200000);const int previous_count=accepted_poses.load();set_truth({.4,.50,M_PI/2,.25,0,0});
  ASSERT_TRUE(wait([&]{return accepted_poses.load()>previous_count;},5));
  const auto save_reply=request("/api/save",json::object());ASSERT_EQ(save_reply["http_status"],202);
  const auto operation=save_reply.at("operation").get<uint64_t>();
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return s.value("completed_operation",uint64_t{0})==operation&&!s.value("busy",true);},15));
  EXPECT_TRUE(request("/api/state").value("error","").empty());size_t bundles=0;
  for(auto&item:std::filesystem::directory_iterator(root)){EXPECT_NO_THROW(validate_bundle(root,item.path().filename()));++bundles;}EXPECT_EQ(bundles,2u);
}
TEST_F(Integration, PartialSaveRemainsIncompleteAndLocked){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);save_fails=true;
  ASSERT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&!s.value("error","").empty();},12));
  bool partial=false;for(auto&item:std::filesystem::directory_iterator(root)){EXPECT_THROW(validate_bundle(root,item.path().filename()),std::exception);partial=true;}
  EXPECT_TRUE(partial);EXPECT_EQ(nonzero.load(),0);EXPECT_FALSE(request("/api/state")["control"].value("armed",true));
  send({{"action","enable"},{"frame",get_video()}});usleep(300000);
  EXPECT_FALSE(request("/api/state")["control"].value("armed",true));
  save_fails=false;ASSERT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&!s.value("save_failed",true);},12));
}
class ManagedIntegration : public Integration {
  bool real_slam() const override {return true;}
  bool managed_backend() const override {return true;}
  bool strict_video() const override {return false;}
};
TEST_F(ManagedIntegration, TabletLifecycleWithoutHardwareOrVideoSavesLoadsAndResumes) {
  video=false; start_gateway("standby"); connect_control();
  EXPECT_EQ(request("/api/state")["phase"],"STANDBY");
  EXPECT_TRUE(node->get_publishers_info_by_topic("/mapping/cmd_vel").empty());
  EXPECT_EQ(request("/api/start",{{"mode","invalid"}})["http_status"],400);
  ASSERT_EQ(request("/api/start",{{"mode","new"}})["http_status"],202);
  EXPECT_EQ(request("/api/start",{{"mode","new"}})["http_status"],409);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return s.value("task_state","")=="RUNNING"&&s.value("phase","")=="CURRENT";},25));
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return s.contains("pose")&&!s.value("scan",json::array()).empty();},5));
  EXPECT_TRUE(request("/api/state")["control"]["require_video"]==false);
  EXPECT_FALSE(request("/api/state")["control"].contains("controller_session"));
  // RUNNING先于许可消息到达门控；确认其已连接且健康后才发送一次使能。
  ASSERT_TRUE(wait([&]{const auto s=request("/api/state");const auto c=s.value("control",json::object());
    return s.value("control_age",1.0)<.3&&c.value("health","未就绪").empty()&&c.value("reason","")=="已连接，请使能";
  })) << request("/api/diagnostics").dump();
  send({{"action","enable"}});
  ASSERT_TRUE(wait([&]{return request("/api/state")["control"].value("armed",false);})) << request("/api/diagnostics").dump();
  uint64_t seq=0;
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");send({{"action","drive"},{"sequence",++seq},{"challenge",s["control"]["challenge"]},{"x",.7},{"yaw",.5}});return nonzero.load()>0;},2));
  send({{"action","release"},{"sequence",++seq}});ASSERT_TRUE(wait([&]{return output.load()==0;},1));
  EXPECT_TRUE(request("/api/state")["control"].value("armed",false));
  const auto previous=nonzero.load();
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");send({{"action","drive"},{"sequence",++seq},{"challenge",s["control"]["challenge"]},{"x",1.0},{"yaw",0.0}});return nonzero.load()>previous;},2));
  send({{"action","stop"}});
  ASSERT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&!s.value("last_saved_bundle","").empty();},16));
  const auto saved=request("/api/state").at("last_saved_bundle").get<std::string>();
  nav_msgs::msg::OccupancyGrid loaded;
  EXPECT_EQ(nav2_map_server::loadMapFromYaml((root/saved/"map.yaml").string(),loaded),nav2_map_server::LOAD_MAP_SUCCESS);
  EXPECT_GT(loaded.info.width,0u); const auto original=sha256(root/saved/"graph.posegraph");
  ASSERT_EQ(request("/api/restart",{{"acknowledge_loss",true}})["http_status"],202);
  ASSERT_TRUE(wait([&]{auto s=request("/api/state");return s.value("task_state","")=="RUNNING"&&s.value("phase","")=="REFERENCE";},25));
  EXPECT_EQ(request("/api/state")["bundle"],saved);
  set_truth({.4,.25,M_PI/2,0,0,0});usleep(300000);
  const auto reply=request("/api/resume",{{"pose",{.4,.25,M_PI/2}}});
  ASSERT_EQ(reply["http_status"],202) << reply.dump() << request("/api/diagnostics").dump();
  ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CANDIDATE";},10));
  ASSERT_TRUE(wait([&]{return !request("/api/state").value("scan",json::array()).empty();},5));
  ASSERT_EQ(request("/api/confirm",json::object())["http_status"],200);
  ASSERT_EQ(request("/api/finish",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{return request("/api/state").value("task_state","")=="STANDBY";},18));
  EXPECT_EQ(sha256(root/saved/"graph.posegraph"),original);
  ASSERT_TRUE(wait([&]{return node->get_publishers_info_by_topic("/mapping/cmd_vel").empty();},5));
  const auto list=request("/api/bundles").at("bundles"); size_t valid=0; for(const auto& b:list)if(b.value("valid",false))++valid;
  EXPECT_EQ(valid,2u); EXPECT_TRUE(request("/api/diagnostics").contains("task_log"));
}
TEST_F(Integration, EmptyRestoreResponseWithoutNewPoseCannotUnlock){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);
  ASSERT_EQ(request("/api/save",json::object())["http_status"],202);ASSERT_TRUE(wait([&]{auto s=request("/api/state");return !s.value("busy",true)&&s.value("result","").find("保存完整")!=std::string::npos;},12));
  std::string bundle;for(auto&item:std::filesystem::directory_iterator(root))bundle=item.path().filename();ASSERT_FALSE(bundle.empty());
  ws.reset();kill_child(gateway);emit_pose=false;relay_count=0;start_gateway("resume",bundle);connect_control();usleep(500000);
  EXPECT_EQ(request("/api/resume",{{"pose",{2000,0,0}}})["http_status"],400);
  ASSERT_EQ(request("/api/resume",{{"pose",{0,0,0}}})["http_status"],202);
  ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="FAILED";},10));
  EXPECT_EQ(relay_count.load(),1);EXPECT_EQ(request("/api/confirm",json::object())["http_status"],409);EXPECT_EQ(nonzero.load(),0);
}
TEST_F(Integration, PausedGatewayCannotKeepOldMotionAlive){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);
  send({{"action","enable"},{"frame",get_video()}});ASSERT_TRUE(wait([&]{return request("/api/state")["control"].value("armed",false);}));
  bool moved=false;uint64_t sequence=0;
  for(int i=0;i<60;++i){auto s=read_ws();if(!s.value("armed",false))continue;send({{"action","intent"},{"sequence",++sequence},{"challenge",s["challenge"]},{"direction","forward"},{"frame",get_video()}});usleep(20000);if(nonzero>0){moved=true;break;}}
  ASSERT_TRUE(moved);kill(gateway,SIGSTOP);
  const bool stopped=wait([&]{return output.load()==0;},1);
  kill(gateway,SIGCONT);EXPECT_TRUE(stopped);
  EXPECT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);},1));
}
TEST_F(Integration, InvalidSlamPoseDuringSaveLocksSessionUntilRestart){
  start_gateway();ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="CURRENT";}));connect_control();usleep(500000);
  send({{"action","enable"},{"frame",get_video()}});ASSERT_TRUE(wait([&]{return request("/api/state")["control"].value("armed",false);}));
  ASSERT_EQ(request("/api/save",json::object())["http_status"],202);
  ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="SAVING";}));
  emit_pose=false;
  geometry_msgs::msg::PoseWithCovarianceStamped invalid;invalid.header.stamp=node->now();invalid.header.frame_id="map";invalid.pose.pose.orientation.w=1;invalid.pose.pose.position.x=std::numeric_limits<double>::quiet_NaN();pose->publish(invalid);
  ASSERT_TRUE(wait([&]{return request("/api/state").value("phase","")=="FAILED";}));
  ASSERT_TRUE(wait([&]{return !request("/api/state")["control"].value("armed",true);}));
  ASSERT_TRUE(wait([&]{return !request("/api/state").value("busy",true);}));
  EXPECT_EQ(request("/api/state").value("phase",""),"FAILED");
  EXPECT_EQ(request("/api/confirm",json::object())["http_status"],409);EXPECT_EQ(request("/api/save",json::object())["http_status"],409);EXPECT_EQ(nonzero.load(),0);
}
