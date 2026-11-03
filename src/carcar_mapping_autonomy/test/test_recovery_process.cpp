#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nlohmann/json.hpp>
extern char**environ;
namespace {
using Json=nlohmann::json;
#ifdef CARCAR_ONLINE_RECOVERY
constexpr bool online=true;
constexpr const char*domain="230";
constexpr const char*evidence_path="log/nav018_r26_snapshot/recovery";
#else
constexpr bool online=false;
constexpr const char*domain="221";
constexpr const char*evidence_path="log/nav018_recovery_process";
#define SNAPSHOT_WORKER "unused-legacy-worker"
#endif
class Children {
public:
  std::vector<pid_t> pids;
  ~Children(){for(auto pid:pids)kill(pid,SIGINT);for(auto pid:pids){bool ended=false;for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){ended=true;break;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}if(!ended){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}}
  bool start(std::vector<std::string>args,const std::filesystem::path&log){
    const int fd=open(log.c_str(),O_CREAT|O_TRUNC|O_WRONLY,0644);if(fd<0)return false;
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);posix_spawn_file_actions_adddup2(&actions,fd,1);posix_spawn_file_actions_adddup2(&actions,fd,2);
    std::vector<char*>argv;for(auto&s:args)argv.push_back(s.data());argv.push_back(nullptr);pid_t pid=-1;
    const int result=posix_spawn(&pid,argv[0],&actions,nullptr,argv.data(),environ);posix_spawn_file_actions_destroy(&actions);close(fd);
    if(result)return false;
    pids.push_back(pid);return true;
  }
};
}
// 真正的SLAM Toolbox + AMCL进程，不用假成功服务替代恢复行为；只在隔离域运行。
TEST(RecoveryProcess, CheckpointTiltRelocalizeRestoreAndCommitWithoutMotion){
  ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):""),domain);
  rclcpp::init(0,nullptr);auto n=std::make_shared<rclcpp::Node>("recovery_process_fixture");
  const std::filesystem::path evidence=evidence_path;std::filesystem::create_directories(evidence);
  char temporary[]="/tmp/carcar-recovery-process-XXXXXX";ASSERT_NE(mkdtemp(temporary),nullptr);
  Children children;
  ASSERT_TRUE(children.start({SLAM_EXE,"--ros-args","-p","scan_topic:=/mapping/slam_scan","-p","use_map_saver:=false","-p","minimum_travel_distance:=0.10","-p","minimum_time_interval:=0.20","-p","map_update_interval:=0.20","-p","base_frame:=base_footprint","-p","max_laser_range:=8.0","-p","startup_stationary_scans:=3","-p",std::string("online_snapshots:=")+(online?"true":"false"),"-p",std::string("snapshot_root:=")+temporary,"-p",std::string("snapshot_worker:=")+SNAPSHOT_WORKER},evidence/"slam.log"));
  ASSERT_TRUE(children.start({"/opt/ros/humble/lib/nav2_amcl/amcl","--ros-args","-r","__ns:=/mapping/relocalizer","--params-file",RELOCALIZER_CONFIG,"-r","amcl_pose:=/mapping/recovery/amcl_pose","-r","initialpose:=/mapping/recovery/initialpose"},evidence/"amcl.log"));
  ASSERT_TRUE(children.start({RECOVERY_EXE,"--ros-args","-p","task_id:=fixture","-p",std::string("checkpoint_root:=")+temporary,"-p",std::string("online_snapshots:=")+(online?"true":"false")},evidence/"recovery.log"));
  auto state_pub=n->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
  auto request_pub=n->create_publisher<std_msgs::msg::String>("/mapping/recovery/request",10);
  auto scans=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS());
  auto relay=n->create_publisher<sensor_msgs::msg::LaserScan>("/mapping/slam_scan",rclcpp::SensorDataQoS());
  auto imu=n->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",rclcpp::SensorDataQoS());
  Json state=Json::object(),slam=Json::object();
  auto sub=n->create_subscription<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){state=Json::parse(m.data);std::ofstream(evidence/"last-state.json")<<state.dump(2);});
  auto slam_sub=n->create_subscription<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){slam=Json::parse(m.data);});
  std::ofstream candidates(evidence/"amcl-candidates.jsonl");
  auto amcl_sub=n->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/mapping/recovery/amcl_pose",rclcpp::QoS(10).reliable(),
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped&m){const auto&p=m.pose.pose;const auto&q=p.orientation;
      candidates<<Json{{"stamp_ns",rclcpp::Time(m.header.stamp).nanoseconds()},{"pose",{p.position.x,p.position.y,std::atan2(2*q.w*q.z,1-2*q.z*q.z)}},
        {"covariance",{m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]}}}.dump()<<'\n';candidates.flush();});
  auto lifecycle=n->create_client<lifecycle_msgs::srv::ChangeState>("/mapping/relocalizer/amcl/change_state");
  tf2_ros::TransformBroadcaster tf(*n);tf2_ros::StaticTransformBroadcaster fixed(*n);
  geometry_msgs::msg::TransformStamped laser;laser.header.frame_id="base_footprint";laser.child_frame_id="laser_frame";laser.transform.translation.z=.217;laser.transform.rotation.w=1;
  double odom_x=0,tilt=0;uint64_t gen=1;int tick=0;bool tf_enabled=false,static_sent=false;
  auto spin_until=[&](auto done,double timeout){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    while(rclcpp::ok()&&std::chrono::steady_clock::now()<end){
      const auto now=n->now();geometry_msgs::msg::TransformStamped t;t.header.stamp=now;t.header.frame_id="odom";t.child_frame_id="base_footprint";t.transform.translation.x=odom_x;t.transform.rotation.w=1;
      if(tf_enabled){if(!static_sent){fixed.sendTransform(laser);static_sent=true;}tf.sendTransform(t);}
      std_msgs::msg::String a;a.data=Json{{"task_id","fixture"},{"generation",gen},{"navigation_generation",gen},{"healthy",true},{"phase","MANUAL"},{"control_mode","automatic"},{"still",true},{"navigation_terminal",true},{"current_pose",{0,0,0}},{"origin",{0,0,0}}}.dump();state_pub->publish(a);
      sensor_msgs::msg::Imu im;im.header.stamp=now;im.header.frame_id="imu_link";im.orientation_covariance[0]=-1;im.linear_acceleration.x=9.81*std::sin(tilt);im.linear_acceleration.z=9.81*std::cos(tilt);imu->publish(im);
      if(++tick%2==0){
        sensor_msgs::msg::LaserScan s;s.header.stamp=now;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_increment=2*M_PI/360;s.angle_max=M_PI;s.range_min=.05;s.range_max=12;s.scan_time=.1;s.ranges.resize(361);
        for(size_t i=0;i<s.ranges.size();++i){const double angle=s.angle_min+i*s.angle_increment,dx=std::cos(angle),dy=std::sin(angle);double r=8;
          if(dx>1e-6)r=std::min(r,3.2/dx);else if(dx< -1e-6)r=std::min(r,-2.5/dx);
          if(dy>1e-6)r=std::min(r,1.8/dy);else if(dy< -1e-6)r=std::min(r,-1.8/dy);
          // 不对称特征，避免用圆形/重复走廊伪造唯一定位结论。
          for(double d=.15;d<r;d+=.015){const double x=d*dx,y=d*dy;if((x>.8&&x<1.3&&y>.3&&y<.8)||(x>-.9&&x<-.5&&y>1.0&&y<1.4)){r=d;break;}}
          s.ranges[i]=r;
        }
        scans->publish(s);if(!state.value("gate_write",false))relay->publish(s);
      }
      rclcpp::spin_some(n);if(done())return true;std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }return done();
  };
  // 复现实车驱动晚于SLAM服务就绪：先连续发扫描但不提供静态／轮式TF，参数应尚未声明。
  spin_until([]{return false;},3.1);
  ASSERT_FALSE(state.empty());EXPECT_NE(state.value("phase",""),"FAILED")<<state.dump();
  EXPECT_FALSE(state.value("gate_write",true));EXPECT_FALSE(state.value("mapping_range_ready",true));
  EXPECT_GT(state.value("mapping_range_waits",0),0)<<"必须实际覆盖PARAMETER_NOT_SET读回，而非仅等待无服务";
  const Json waiting_startup=state;tf_enabled=true;
  ASSERT_TRUE(spin_until([&]{return lifecycle->service_is_ready()&&state.value("imu_ready",false)&&state.value("mapping_range_ready",false)&&slam.value("sequence",0)>=4;},12))<<state.dump();
  for(unsigned transition:{1u,3u}){
    auto req=std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();req->transition.id=transition;auto future=lifecycle->async_send_request(req).future.share();
    ASSERT_TRUE(spin_until([&]{return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;},5));ASSERT_TRUE(future.get()->success);
  }
  auto request=[&](const std::string&action,const std::string&id){std_msgs::msg::String m;m.data=Json{{"task_id","fixture"},{"generation",gen},{"navigation_generation",gen},{"origin",{0,0,0}},{"request_id",id},{"action",action}}.dump();request_pub->publish(m);};
  ASSERT_TRUE(spin_until([&]{return state.value("imu_stable",false);},5));
  request(online?"snapshot":"checkpoint","checkpoint-1");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="CHECKPOINT_READY"||state.value("phase","")=="FAILED"||state.value("snapshot",Json::object()).value("phase","")=="FAILED";},15));
  ASSERT_EQ(state.value("phase",""),"CHECKPOINT_READY")<<state.dump();
  const auto checkpoint=state.value("checkpoint_path","");ASSERT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"graph.posegraph"));
  tilt=.25;ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="DISTURBED";},3));
  EXPECT_TRUE(state.value("gate_write",false));odom_x=.35;
  // 倾斜持续时不能继续，恢复平放后才允许旁路定位。
  ++gen;request("recover","recover-1");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="WAITING_STABLE";},3));
  spin_until([]{return false;},1);EXPECT_EQ(state.value("phase",""),"WAITING_STABLE");tilt=0;
  const bool verified=spin_until([&]{return state.value("phase","")=="VERIFY_READY"||state.value("phase","")=="FAILED";},45);
  EXPECT_TRUE(verified);ASSERT_EQ(state.value("phase",""),"VERIFY_READY")<<state.dump();
  EXPECT_TRUE(state.value("gate_write",false));EXPECT_GE(slam.value("restore_generation",0),1);
  request("commit_verified","commit-1");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="SUCCEEDED";},5))<<state.dump();
  EXPECT_FALSE(state.value("gate_write",true));EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"graph.posegraph"));
  const Json success=state;
  --gen;request("recover","stale-old-generation");spin_until([]{return false;},.5);
  EXPECT_EQ(state.value("phase",""),"SUCCEEDED");EXPECT_FALSE(state.value("gate_write",true));++gen;
  bool suspect=false;for(const auto&entry:std::filesystem::directory_iterator(std::filesystem::path(checkpoint).parent_path()))
    if(entry.path().filename().string().find("_suspect")!=std::string::npos&&std::filesystem::exists(entry.path()/"evidence.json"))suspect=true;
  EXPECT_TRUE(suspect);
  tilt=.25;ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="DISTURBED";},3));
  ++gen;request("recover","persistent-tilt");
  ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},25));
  EXPECT_TRUE(state.value("gate_write",false));EXPECT_TRUE(state.value("requires_stop",false));
  EXPECT_NE(state.value("reason","").find("平稳"),std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"graph.posegraph"));
  std::ofstream(evidence/"result.json")<<Json{{"waiting_before_first_tf",waiting_startup},{"successful_transaction",success},{"persistent_tilt_failure",state},{"slam",slam},{"hardware",false},{"odom_disturbance_m",.35},{"suspect_evidence_preserved",suspect}}.dump(2);
  rclcpp::shutdown();
}
