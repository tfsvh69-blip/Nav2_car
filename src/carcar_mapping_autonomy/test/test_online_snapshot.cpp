#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <spawn.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <slam_toolbox/srv/deserialize_pose_graph.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "carcar_mapping_autonomy/recovery_artifact.hpp"
extern char**environ;
namespace {
using Json=nlohmann::json;
struct SlamProcess {
  pid_t pid=-1;
  ~SlamProcess(){if(pid<=0)return;kill(-pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid)return;std::this_thread::sleep_for(std::chrono::milliseconds(20));}kill(-pid,SIGKILL);waitpid(pid,nullptr,0);}
  bool start(const std::filesystem::path&root,const std::filesystem::path&log){
    std::vector<std::string>args{SLAM_EXE,"--ros-args","-p","online_snapshots:=true","-p",std::string("snapshot_worker:=")+SNAPSHOT_WORKER,
      "-p","snapshot_root:="+root.string(),"-p","use_map_saver:=false","-p","minimum_travel_distance:=0.05","-p","minimum_time_interval:=0.10",
      "-p","map_update_interval:=0.20","-p","base_frame:=base_footprint","-p","max_laser_range:=3.0","-p","do_loop_closing:=false"};
    std::vector<char*>argv;for(auto&arg:args)argv.push_back(arg.data());argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    const int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC,0644);if(fd<0)return false;
    posix_spawn_file_actions_adddup2(&actions,fd,STDOUT_FILENO);posix_spawn_file_actions_adddup2(&actions,fd,STDERR_FILENO);
    posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
    const int result=posix_spawn(&pid,argv[0],&actions,&attr,argv.data(),environ);
    posix_spawnattr_destroy(&attr);posix_spawn_file_actions_destroy(&actions);close(fd);return result==0;
  }
};
}
// 真实SLAM和独立工作进程，连续移动的矩形房间软件真值；不启动任何执行器。
TEST(OnlineSnapshot, MovingSlamCapturesOneHundredTimesWithoutStaleScansAndRestoresRealArchive){
  ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):""),"228");
  rclcpp::init(0,nullptr);struct Shutdown{~Shutdown(){rclcpp::shutdown();}} shutdown;
  char temporary[]="/tmp/carcar-snapshot-live-XXXXXX";ASSERT_NE(mkdtemp(temporary),nullptr);const std::filesystem::path root=temporary;
  struct Cleanup{std::filesystem::path root;~Cleanup(){std::filesystem::remove_all(root);}}cleanup{root};
  const std::filesystem::path evidence="log/nav018_r26_snapshot/live";std::filesystem::create_directories(evidence);
  auto node=std::make_shared<rclcpp::Node>("snapshot_live_truth");SlamProcess child;
  const std::string prior=std::getenv("LD_LIBRARY_PATH")?std::getenv("LD_LIBRARY_PATH"):"";
  const auto compat=ament_index_cpp::get_package_prefix("carcar_tf2_compat")+"/lib/carcar_tf2_compat";
  setenv("LD_LIBRARY_PATH",(compat+":"+prior).c_str(),1);ASSERT_TRUE(child.start(root,evidence/"slam.log"));setenv("LD_LIBRARY_PATH",prior.c_str(),1);
  auto scans=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
  auto autonomy=node->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
  auto requests=node->create_publisher<std_msgs::msg::String>("/mapping/slam/snapshot/request",10);
  Json state=Json::object(),slam=Json::object();double heartbeat_at=0,maximum_gap=0;bool moving=false;double theta=0;uint64_t generation=1;
  auto state_sub=node->create_subscription<std_msgs::msg::String>("/mapping/slam/snapshot/state",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){state=Json::parse(m.data);});
  auto slam_sub=node->create_subscription<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){
    const auto t=node->now().seconds();if(moving&&heartbeat_at>0)maximum_gap=std::max(maximum_gap,t-heartbeat_at);heartbeat_at=t;slam=Json::parse(m.data);});
  tf2_ros::TransformBroadcaster tf(*node);tf2_ros::StaticTransformBroadcaster fixed(*node);
  geometry_msgs::msg::TransformStamped laser;laser.header.frame_id="base_footprint";laser.child_frame_id="laser_frame";laser.transform.rotation.w=1;fixed.sendTransform(laser);
  auto feed=[&]{const auto now=node->now();if(moving)theta+=.01;const double x=.6*std::sin(theta),y=.6*(1-std::cos(theta));
    geometry_msgs::msg::TransformStamped tr;tr.header.stamp=now;tr.header.frame_id="odom";tr.child_frame_id="base_footprint";tr.transform.translation.x=x;tr.transform.translation.y=y;tr.transform.rotation.w=1;tf.sendTransform(tr);
    std_msgs::msg::String status;status.data=Json{{"task_id","live-snapshot"},{"generation",generation},{"navigation_generation",generation},{"healthy",true},
      {"phase","EXPLORING"},{"origin",{0,0,0}},{"still",!moving}}.dump();autonomy->publish(status);
    sensor_msgs::msg::LaserScan scan;scan.header.stamp=now;scan.header.frame_id="laser_frame";scan.angle_min=-M_PI;scan.angle_increment=M_PI/180;scan.angle_max=M_PI;scan.range_min=.05;scan.range_max=12;
    for(int k=0;k<=360;++k){const double angle=-M_PI+k*M_PI/180,dx=std::cos(angle),dy=std::sin(angle);double r=12;
      if(dx>1e-8)r=std::min(r,(4-x)/dx);else if(dx< -1e-8)r=std::min(r,(-2-x)/dx);
      if(dy>1e-8)r=std::min(r,(2-y)/dy);else if(dy< -1e-8)r=std::min(r,(-2-y)/dy);scan.ranges.push_back(r);}
    scans->publish(scan);
  };
  auto timer=node->create_wall_timer(std::chrono::milliseconds(50),feed);
  auto spin_until=[&](auto done,double timeout){const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    while(rclcpp::ok()&&std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);if(done())return true;std::this_thread::sleep_for(std::chrono::milliseconds(2));}return done();};
  ASSERT_TRUE(spin_until([&]{return slam.value("sequence",uint64_t{0})>=3;},10))<<slam;
  moving=true;std::vector<double>times;std::string latest;std::ofstream transactions(evidence/"transactions.jsonl");
  for(int i=0;i<100;++i){const std::string id="capture-"+std::to_string(i);std_msgs::msg::String request;
    request.data=Json{{"task_id","live-snapshot"},{"request_id",id},{"generation",generation},{"navigation_generation",generation},{"origin",{0,0,0}}}.dump();requests->publish(request);
    ASSERT_TRUE(spin_until([&]{const auto p=state.value("phase","");return state.value("request_id","")==id&&!state.value("in_flight",true)&&(p=="READY"||p=="FAILED");},5))<<state;
    transactions<<state.dump()<<'\n';transactions.flush();ASSERT_EQ(state.value("phase",""),"READY")<<state;
    times.push_back(state.at("capture_ms"));latest=state.at("path");
  }
  std::sort(times.begin(),times.end());EXPECT_LE(times[98],100);EXPECT_LE(times.back(),200);EXPECT_LT(maximum_gap,.5);
  const double completed_duration=state.at("elapsed_seconds");spin_until([]{return false;},.2);
  EXPECT_DOUBLE_EQ(state.at("elapsed_seconds").get<double>(),completed_duration);
  EXPECT_EQ(slam.value("scan_stale_rejections",uint64_t{1}),0U);EXPECT_LT(slam.value("scan_callback_max_age_ms",1000.),500.);
  const auto manifest=carcar_mapping_autonomy::recovery_validate_artifact(latest);
  std::ofstream(evidence/"result.json")<<Json{{"hardware",false},{"samples",times.size()},{"p99_ms",times[98]},{"max_ms",times.back()},
    {"heartbeat_max_gap_seconds",maximum_gap},{"scan_callback_max_age_ms",slam.value("scan_callback_max_age_ms",-1.)},{"slam_sequence",slam.value("sequence",uint64_t{0})}}.dump(2);
  // 在只读副本移交期间改变坐标代次，迟到的工作结果不能晋升；新代次还能继续维护。
  std_msgs::msg::String cancelled_request;
  cancelled_request.data=Json{{"task_id","live-snapshot"},{"request_id","cancel-during-copy"},{"generation",generation},{"navigation_generation",generation},{"origin",{0,0,0}}}.dump();
  requests->publish(cancelled_request);
  ASSERT_TRUE(spin_until([&]{return state.value("request_id","")=="cancel-during-copy"&&state.value("in_flight",false);},3));
  ++generation;feed();
  ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED"&&!state.value("in_flight",true);},3))<<state;
  transactions<<state.dump()<<'\n';
  cancelled_request.data=Json{{"task_id","live-snapshot"},{"request_id","after-generation-change"},{"generation",generation},{"navigation_generation",generation},{"origin",{0,0,0}}}.dump();
  requests->publish(cancelled_request);
  ASSERT_TRUE(spin_until([&]{return state.value("request_id","")=="after-generation-change"&&!state.value("in_flight",true);},5))<<state;
  ASSERT_EQ(state.value("phase",""),"READY")<<state;transactions<<state.dump()<<'\n';
  // 对实际生产适配器调用原反序列化接口，验证原格式恢复入口保持兼容。
  moving=false;const auto before=slam.value("restore_generation",uint64_t{0});
  using Load=slam_toolbox::srv::DeserializePoseGraph;auto load=node->create_client<Load>("/slam_toolbox/deserialize_map");
  ASSERT_TRUE(load->service_is_ready());auto restore=std::make_shared<Load::Request>();restore->filename=latest+"/graph";
  restore->match_type=Load::Request::START_AT_GIVEN_POSE;restore->initial_pose.x=manifest.at("pose").at(0);restore->initial_pose.y=manifest.at("pose").at(1);restore->initial_pose.theta=manifest.at("pose").at(2);
  auto response=load->async_send_request(restore);ASSERT_TRUE(spin_until([&]{return response.wait_for(std::chrono::seconds(0))==std::future_status::ready;},5));
  ASSERT_TRUE(spin_until([&]{return slam.value("restore_generation",uint64_t{0})>before;},3));
}
