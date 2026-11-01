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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    if(args.front()==SLAM_EXE&&std::getenv("CARCAR_TEST_LOOP_ROLLBACK"))
      args.insert(args.end(),{"-r","/mapping/slam/status:=/fixture/slam/status"});
    if(args.front()==SLAM_EXE&&std::getenv("CARCAR_TEST_LONG_STARTUP")){
      for(auto &arg:args)if(arg=="startup_stationary_scans:=3")arg="startup_stationary_scans:=12";
      args.insert(args.end(),{"-p","startup_stationary_window:=5.0"});
    }
    // 固定旧二进制用于保存反例，默认仍运行本次构建产物。
    if(args.front()==SLAM_EXE&&std::getenv("CARCAR_SLAM_BASELINE_EXE"))args.front()=std::getenv("CARCAR_SLAM_BASELINE_EXE");
    if(args.front()==RECOVERY_EXE&&std::getenv("CARCAR_RECOVERY_BASELINE_EXE"))args.front()=std::getenv("CARCAR_RECOVERY_BASELINE_EXE");
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
  const std::filesystem::path evidence=std::getenv("CARCAR_TEST_EVIDENCE_DIR")?std::getenv("CARCAR_TEST_EVIDENCE_DIR"):evidence_path;std::filesystem::create_directories(evidence);
  char temporary[]="/tmp/carcar-recovery-process-XXXXXX";ASSERT_NE(mkdtemp(temporary),nullptr);
  Children children;
  ASSERT_TRUE(children.start({SLAM_EXE,"--ros-args","-p","scan_topic:=/mapping/slam_scan","-p","use_map_saver:=false","-p","minimum_travel_distance:=0.10","-p","minimum_time_interval:=0.50","-p","map_update_interval:=0.20","-p","base_frame:=base_footprint","-p","max_laser_range:=8.0","-p","startup_stationary_scans:=3","-p",std::string("online_snapshots:=")+(online?"true":"false"),"-p",std::string("snapshot_root:=")+temporary,"-p",std::string("snapshot_worker:=")+SNAPSHOT_WORKER},evidence/"slam.log"));
  ASSERT_TRUE(children.start({"/opt/ros/humble/lib/nav2_amcl/amcl","--ros-args","-r","__ns:=/mapping/relocalizer","--params-file",RELOCALIZER_CONFIG,"-r","amcl_pose:=/mapping/recovery/amcl_pose","-r","initialpose:=/mapping/recovery/initialpose"},evidence/"amcl.log"));
  ASSERT_TRUE(children.start({RECOVERY_EXE,"--ros-args","-p","task_id:=fixture","-p",std::string("checkpoint_root:=")+temporary,"-p",std::string("online_snapshots:=")+(online?"true":"false")},evidence/"recovery.log"));
  auto state_pub=n->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
  auto request_pub=n->create_publisher<std_msgs::msg::String>("/mapping/recovery/request",10);
  auto scans=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS());
  auto relay=n->create_publisher<sensor_msgs::msg::LaserScan>("/mapping/slam_scan",rclcpp::SensorDataQoS());
  auto imu=n->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",rclcpp::SensorDataQoS());
  Json state=Json::object(),slam=Json::object(),first_snapshot_ready=Json::object(),snapshot_during_startup=Json::object();
  const std::string rollback_case=std::getenv("CARCAR_TEST_LOOP_ROLLBACK")?std::getenv("CARCAR_TEST_LOOP_ROLLBACK"):"";
  bool inject_rollback=false;
  auto slam_relay=n->create_publisher<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local());
  auto sub=n->create_subscription<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){
    state=Json::parse(m.data);std::ofstream(evidence/"last-state.json")<<state.dump(2);
    // 留住首次提升的原始状态，不能用后续心跳覆盖缺少新帧验证的首次READY。
    if(online&&first_snapshot_ready.empty()&&state.value("snapshot",Json::object()).value("phase","")=="READY"){
      first_snapshot_ready=state;std::ofstream(evidence/"first-snapshot-ready.json")<<state.dump(2);
    }
  });
  auto slam_sub=n->create_subscription<std_msgs::msg::String>(rollback_case.empty()?"/mapping/slam/status":"/fixture/slam/status",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){
    slam=Json::parse(m.data);
    if(!rollback_case.empty()){
      // 这里只模拟已撤销通知；地图、扫描、保存、六帧验证及入图门均由真实进程执行。
      if(inject_rollback)slam["loop_guard"]={{"rolled_back",true},{"rejections",1}};
      std_msgs::msg::String forwarded;forwarded.data=slam.dump();slam_relay->publish(forwarded);
    }
  });
  std::ofstream candidates(evidence/"amcl-candidates.jsonl");
  auto amcl_sub=n->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/mapping/recovery/amcl_pose",rclcpp::QoS(10).reliable(),
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped&m){const auto&p=m.pose.pose;const auto&q=p.orientation;
      candidates<<Json{{"stamp_ns",rclcpp::Time(m.header.stamp).nanoseconds()},{"pose",{p.position.x,p.position.y,std::atan2(2*q.w*q.z,1-2*q.z*q.z)}},
        {"covariance",{m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]}}}.dump()<<'\n';candidates.flush();});
  auto lifecycle=n->create_client<lifecycle_msgs::srv::ChangeState>("/mapping/relocalizer/amcl/change_state");
  tf2_ros::TransformBroadcaster tf(*n);tf2_ros::StaticTransformBroadcaster fixed(*n);
  tf2_ros::Buffer observed_tf(n->get_clock());tf2_ros::TransformListener tf_listener(observed_tf);
  geometry_msgs::msg::TransformStamped laser;laser.header.frame_id="base_footprint";laser.child_frame_id="laser_frame";laser.transform.translation.z=.217;laser.transform.rotation.w=1;
  double odom_x=0,truth_x=0,tilt=0,turn_yaw=0;uint64_t gen=1;int tick=0;bool tf_enabled=false,static_sent=false,emit_scans=true;
  const std::string checkpoint_case=std::getenv("CARCAR_TEST_CHECKPOINT_STABILITY")?std::getenv("CARCAR_TEST_CHECKPOINT_STABILITY"):"";
  const std::string snapshot_post_capture_case=std::getenv("CARCAR_TEST_SNAPSHOT_POST_CAPTURE")?std::getenv("CARCAR_TEST_SNAPSHOT_POST_CAPTURE"):"";
  const std::string post_capture_target=snapshot_post_capture_case=="bad_scan"?"checkpoint-1":"snapshot-post-capture-negative";
  ASSERT_TRUE(snapshot_post_capture_case.empty()||(online&&(snapshot_post_capture_case=="bad_scan"||snapshot_post_capture_case=="bad_scan_existing"||snapshot_post_capture_case=="cancel"||snapshot_post_capture_case=="generation")));
  const bool small_translation=std::getenv("CARCAR_TEST_SMALL_TRANSLATION")!=nullptr;
  const double translation_m=small_translation?std::stod(std::getenv("CARCAR_TEST_SMALL_TRANSLATION")):0;
  ASSERT_TRUE(!small_translation||(std::isfinite(translation_m)&&translation_m>0&&translation_m<=.20));
  bool post_capture_injected=false;double post_capture_injection_time=0;Json post_capture_before=Json::object();
  const std::string commit_case=std::getenv("CARCAR_TEST_COMMIT_GUARD")?std::getenv("CARCAR_TEST_COMMIT_GUARD"):"";
  bool commit_bad_scan=false;
  bool injected=false,cancel_sent=false;int injection_tick=0;double injection_time=0;
  auto spin_until=[&](auto done,double timeout){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    while(rclcpp::ok()&&std::chrono::steady_clock::now()<end){
      const auto now=n->now();geometry_msgs::msg::TransformStamped t;t.header.stamp=now;t.header.frame_id="odom";t.child_frame_id="base_footprint";t.transform.translation.x=odom_x;t.transform.rotation.w=1;
      t.transform.rotation.z=std::sin(turn_yaw/2);t.transform.rotation.w=std::cos(turn_yaw/2);
      if(tf_enabled){if(!static_sent){fixed.sendTransform(laser);static_sent=true;}tf.sendTransform(t);}
      std_msgs::msg::String a;a.data=Json{{"task_id","fixture"},{"generation",gen},{"navigation_generation",gen},{"healthy",true},{"phase","MANUAL"},{"control_mode","automatic"},{"still",!(injected&&checkpoint_case=="movement")},{"navigation_terminal",true},{"current_pose",{0,0,0}},{"origin",{0,0,0}}}.dump();state_pub->publish(a);
      // 现场40毫秒单帧比力方向越界；这里只重现相对夹角，不声称车身发生真实倾斜。
      if(!checkpoint_case.empty()&&!injected&&state.value("phase","")=="CHECKPOINTING"){
        injected=true;injection_tick=tick;injection_time=now.seconds();tilt=checkpoint_case=="hard"?.25:4.236456*M_PI/180;
      }else if(injected&&checkpoint_case=="transient"&&tick>injection_tick)tilt=0;
      if(injected&&checkpoint_case=="burst")tilt=tick-injection_tick<100&&(tick-injection_tick)%16==0?4.236456*M_PI/180:0;
      if(injected&&checkpoint_case=="cancel"&&!cancel_sent&&tick-injection_tick>=3){
        std_msgs::msg::String cancel;cancel.data=Json{{"task_id","fixture"},{"generation",gen+1},{"request_id","cancel-checkpoint"},{"action","cancel"}}.dump();request_pub->publish(cancel);cancel_sent=true;tilt=0;
      }
      const auto snapshot=state.value("snapshot",Json::object());
      if(online&&!snapshot_post_capture_case.empty()&&!post_capture_injected&&snapshot.value("request_id","")==post_capture_target&&snapshot.value("phase","")=="VALIDATING"){
        // 只在候选文件完成后施加反例，保持原地图、TF及全部验收阈值不变。
        post_capture_injected=true;post_capture_injection_time=now.seconds();post_capture_before=state;
        std::ofstream(evidence/"post-capture-before.json")<<state.dump(2);
        if(snapshot_post_capture_case=="generation")++gen;
        if(snapshot_post_capture_case=="cancel"){
          ++gen;std_msgs::msg::String cancel;cancel.data=Json{{"task_id","fixture"},{"generation",gen},{"request_id","cancel-snapshot-candidate"},{"action","cancel"}}.dump();request_pub->publish(cancel);
        }
      }
      sensor_msgs::msg::Imu im;im.header.stamp=now;im.header.frame_id="imu_link";im.orientation_covariance[0]=-1;im.linear_acceleration.x=9.81*std::sin(tilt);im.linear_acceleration.z=9.81*std::cos(tilt);
      // 先用一帧波动让保存确实进入等待，再断流；不能把尚未超过0.5秒的正常完成误判为陈旧。
      if(!(injected&&checkpoint_case=="stale"&&tick>injection_tick))imu->publish(im);
      if(++tick%2==0&&emit_scans){
        sensor_msgs::msg::LaserScan s;s.header.stamp=now;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_increment=2*M_PI/360;s.angle_max=M_PI;s.range_min=.05;s.range_max=12;s.scan_time=.1;s.ranges.resize(361);
        for(size_t i=0;i<s.ranges.size();++i){const double angle=turn_yaw+s.angle_min+i*s.angle_increment,dx=std::cos(angle),dy=std::sin(angle);double r=8;
          if(dx>1e-6)r=std::min(r,(3.2-truth_x)/dx);else if(dx< -1e-6)r=std::min(r,(-2.5-truth_x)/dx);
          if(dy>1e-6)r=std::min(r,1.8/dy);else if(dy< -1e-6)r=std::min(r,-1.8/dy);
          // 不对称特征，避免用圆形/重复走廊伪造唯一定位结论。
          for(double d=.15;d<r;d+=.015){const double x=truth_x+d*dx,y=d*dy;if((x>.8&&x<1.3&&y>.3&&y<.8)||(x>-.9&&x<-.5&&y>1.0&&y<1.4)){r=d;break;}}
          s.ranges[i]=r;
        }
        auto validation_scan=s;
        if(post_capture_injected&&snapshot_post_capture_case.rfind("bad_scan",0)==0)std::fill(validation_scan.ranges.begin(),validation_scan.ranges.end(),.25f);
        if(commit_bad_scan)std::fill(validation_scan.ranges.begin(),validation_scan.ranges.end(),.25f);
        // 坏扫描只进入独立验证支路，生产SLAM继续接收原始几何，避免把改图当成拒绝证据。
        scans->publish(validation_scan);if(!state.value("gate_write",false))relay->publish(s);
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
  auto request=[&](const std::string&action,const std::string&id,bool operator_revalidation=false){std_msgs::msg::String m;m.data=Json{{"task_id","fixture"},{"generation",gen},{"navigation_generation",gen},{"origin",{0,0,0}},{"request_id",id},{"action",action},{"operator_revalidation",operator_revalidation}}.dump();request_pub->publish(m);};
  ASSERT_TRUE(spin_until([&]{return state.value("imu_stable",false);},5));
  if(std::getenv("CARCAR_TEST_LONG_STARTUP")){
    ASSERT_FALSE(slam.value("startup_closed",true));
    if(online){
      // 在线入口必须实际发snapshot，不能用旧checkpoint请求掩盖补观测前取快照的缺口。
      request("snapshot","snapshot-during-observation");
      ASSERT_TRUE(spin_until([&]{return state.value("snapshot",Json::object()).value("request_id","")=="snapshot-during-observation";},2));
      snapshot_during_startup=state;std::ofstream(evidence/"snapshot-during-startup.json")<<state.dump(2);
      EXPECT_EQ(state.value("snapshot",Json::object()).value("phase",""),"WAITING_STARTUP")<<state.dump();
      EXPECT_NE(state.value("phase",""),"CHECKPOINT_READY")<<state.dump();
      EXPECT_TRUE(state.value("checkpoint_path",std::string()).empty())<<"补观测尚未结束时不能提升首个可信参考";
    }else{
      request("checkpoint","checkpoint-during-observation");
      ASSERT_TRUE(spin_until([&]{return state.value("request_id","")=="checkpoint-during-observation";},2));
      EXPECT_EQ(state.value("phase",""),"WARMING")<<state.dump();
    }
    EXPECT_FALSE(state.value("gate_write",true));
    EXPECT_TRUE(slam.value("admission_enabled",false));
    ASSERT_TRUE(spin_until([&]{return slam.value("startup_closed",false);},6));
  }
  // 补采封窗状态经独立话题到达；至少再送一轮，避免把尚在WARMING的正常拒绝当成检查点故障。
  ASSERT_TRUE(spin_until([&]{return slam.value("startup_closed",true);},6));
  spin_until([]{return false;},.3);
  request(online?"snapshot":"checkpoint","checkpoint-1");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="CHECKPOINT_READY"||state.value("phase","")=="FAILED"||state.value("snapshot",Json::object()).value("phase","")=="FAILED";},20));
  if(online&&snapshot_post_capture_case=="bad_scan"){
    ASSERT_TRUE(post_capture_injected)<<"必须真实进入捕获后验证阶段，不能用启动失败代替几何反例";
    EXPECT_EQ(state.value("snapshot",Json::object()).value("phase",""),"FAILED")<<state.dump();
    EXPECT_TRUE(state.value("checkpoint_id",std::string()).empty());EXPECT_TRUE(state.value("checkpoint_path",std::string()).empty());
    EXPECT_TRUE(first_snapshot_ready.empty())<<"首个坏候选不能出现过READY";
    EXPECT_FALSE(state.value("gate_write",true))<<"拒绝例行候选不能伪装成停止生产入图";
    EXPECT_LE(n->now().seconds()-post_capture_injection_time,5.6)<<"原5秒验证预算之外只允许消息运输和调度余量";
    std::ofstream(evidence/"post-capture-result.json")<<Json{{"case",snapshot_post_capture_case},{"before",post_capture_before},{"after",state},{"hardware",false}}.dump(2);
    rclcpp::shutdown();return;
  }
  if(!checkpoint_case.empty()){
    ASSERT_TRUE(injected);
    if(checkpoint_case=="transient"||checkpoint_case=="burst"){
      EXPECT_EQ(state.value("phase",""),"CHECKPOINT_READY")<<state.dump();
      EXPECT_FALSE(state.value("gate_write",true));
      EXPECT_GE(state.value("checkpoint_revalidation",Json::object()).value("attempts",0),1);
      EXPECT_GE(n->now().seconds()-injection_time,2.0)<<"不能跳过原连续稳定2秒门槛";
    }else{
      EXPECT_EQ(state.value("phase",""),"FAILED")<<state.dump();
      EXPECT_TRUE(state.value("gate_write",false));
      EXPECT_TRUE(state.value("checkpoint_id","").empty());
      if(checkpoint_case=="persistent"){EXPECT_NE(state.value("reason","").find("12秒"),std::string::npos);}
      if(checkpoint_case=="cancel"){
        EXPECT_TRUE(cancel_sent);tilt=0;spin_until([]{return false;},2.5);
        EXPECT_EQ(state.value("phase",""),"FAILED");EXPECT_TRUE(state.value("gate_write",false));
        EXPECT_EQ(state.value("request_id",""),"cancel-checkpoint");
      }
    }
    std::ofstream(evidence/("checkpoint-"+checkpoint_case+".json"))<<state.dump(2);
    rclcpp::shutdown();return;
  }
  ASSERT_EQ(state.value("phase",""),"CHECKPOINT_READY")<<state.dump();
  if(online){
    ASSERT_FALSE(first_snapshot_ready.empty())<<"必须观察到首次真实READY，而不是只检查文件已写出";
    const auto snapshot=first_snapshot_ready.value("snapshot",Json::object());
    const auto capture_stamp=snapshot.value("capture_stamp_ns",int64_t{0});
    const auto validation_stamp=snapshot.value("validation_scan_stamp_ns",int64_t{0});
    EXPECT_GE(snapshot.value("validation_frames",size_t{0}),size_t{3})<<snapshot.dump();
    EXPECT_GT(capture_stamp,int64_t{0})<<snapshot.dump();
    EXPECT_GT(validation_stamp,capture_stamp)<<"捕获帧本身不能充当捕获之后的新扫描验证";
    EXPECT_FALSE(first_snapshot_ready.value("gate_write",true))<<"例行快照不能为了验证关闭生产入图";
    if(!snapshot_post_capture_case.empty()){
      const auto old_id=state.value("checkpoint_id",std::string()),old_path=state.value("checkpoint_path",std::string());
      ASSERT_FALSE(old_id.empty());ASSERT_FALSE(old_path.empty());
      request("snapshot",post_capture_target);
      ASSERT_TRUE(spin_until([&]{const auto s=state.value("snapshot",Json::object());return s.value("request_id","")==post_capture_target&&(s.value("phase","")=="FAILED"||s.value("phase","")=="READY");},8))<<state.dump();
      ASSERT_TRUE(post_capture_injected)<<"负例必须发生在第二个候选的捕获后验证阶段";
      EXPECT_EQ(state.value("snapshot",Json::object()).value("phase",""),"FAILED")<<state.dump();
      EXPECT_EQ(state.value("checkpoint_id",std::string()),old_id);EXPECT_EQ(state.value("checkpoint_path",std::string()),old_path);
      EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(old_path)/"manifest.json"));
      EXPECT_LE(n->now().seconds()-post_capture_injection_time,5.6);
      std::ofstream(evidence/"post-capture-result.json")<<Json{{"case",snapshot_post_capture_case},{"before",post_capture_before},{"after",state},{"old_checkpoint_id",old_id},{"old_checkpoint_path",old_path},{"hardware",false}}.dump(2);
      rclcpp::shutdown();return;
    }
    if(std::getenv("CARCAR_TEST_SNAPSHOT_PREFLIGHT_ONLY")){
      std::ofstream(evidence/"snapshot-preflight-result.json")<<Json{{"startup_request",snapshot_during_startup},{"first_ready",first_snapshot_ready},{"slam",slam},{"hardware",false}}.dump(2);
      rclcpp::shutdown();return;
    }
  }
  const auto checkpoint=state.value("checkpoint_path","");ASSERT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"graph.posegraph"));
  if(!rollback_case.empty()){
    inject_rollback=true;spin_until([]{return false;},.5);
    request("recover_rollback","rollback-check");
    if(rollback_case=="cancel"){
      ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="VERIFYING_ROLLBACK";},15))<<state;
      request("cancel","cancel-rollback");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},3));
      spin_until([]{return false;},2);EXPECT_TRUE(state.value("gate_write",false));EXPECT_EQ(state.value("phase",""),"FAILED");
      EXPECT_EQ(state.value("checkpoint_path",""),checkpoint);
    }else{
      ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="VERIFY_READY"||state.value("phase","")=="FAILED";},35))<<state;
      ASSERT_EQ(state.value("phase",""),"VERIFY_READY")<<state;
      EXPECT_TRUE(state.value("gate_write",false));EXPECT_GE(state.value("localization_verified_frames",0),6);
      EXPECT_GE(state.value("verified_frames",0),3);EXPECT_GE(slam.value("restore_generation",0),1);
      EXPECT_EQ(state.value("checkpoint_path",""),checkpoint);
      const auto sequence=slam.value("sequence",0);spin_until([]{return false;},1.2);EXPECT_EQ(slam.value("sequence",0),sequence);
      request("commit_verified","rollback-commit");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="SUCCEEDED";},3));
      EXPECT_FALSE(state.value("gate_write",true));EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"manifest.json"));
    }
    rclcpp::shutdown();return;
  }
  tilt=.25;ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="DISTURBED";},3));
  EXPECT_TRUE(state.value("gate_write",false));
  // 抬起／侧滑模式只改变激光的真实原点，轮式TF仍为零；默认保留原0.35米轮式反例。
  if(small_translation)truth_x=translation_m;else odom_x=.35;
  // 复现实车冻结后的正常转向：轮式与激光同转150度，旧SLAM姿态保持不变。
  if(std::getenv("CARCAR_TEST_RECORDED_TURN"))turn_yaw=-2.6184;
  // 冻结超过Nav2默认TF缓存长度，真实扫描仍更新运输时间，但禁止写图。
  const auto frozen_sequence=slam.value("sequence",0);
  const bool long_pause=std::getenv("CARCAR_TEST_LONG_PAUSE")!=nullptr;
  spin_until([]{return false;},long_pause?121.2:11.2);
  ASSERT_NO_THROW(observed_tf.lookupTransform("map","odom",tf2::TimePointZero));
  auto frozen_tf=observed_tf.lookupTransform("map","odom",tf2::TimePointZero);
  EXPECT_LT((n->now()-rclcpp::Time(frozen_tf.header.stamp)).seconds(),.5);
  EXPECT_EQ(slam.value("sequence",0),frozen_sequence);
  EXPECT_FALSE(slam.value("admission_enabled",true));
  // 没有新扫描不能靠定时器伪造新时间或定位健康。
  emit_scans=false;spin_until([]{return false;},1.2);
  frozen_tf=observed_tf.lookupTransform("map","odom",tf2::TimePointZero);
  EXPECT_GT((n->now()-rclcpp::Time(frozen_tf.header.stamp)).seconds(),.5);
  emit_scans=true;
  // 倾斜持续时不能继续，恢复平放后才允许旁路定位。
  ++gen;
  if(long_pause){
    request("recover","expired-automatic");
    ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},3));
    EXPECT_NE(state.value("reason","").find("120秒"),std::string::npos);
    EXPECT_TRUE(state.value("gate_write",false));
  }
  request("recover","recover-1",long_pause);ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="WAITING_STABLE";},3));
  spin_until([]{return false;},1);EXPECT_EQ(state.value("phase",""),"WAITING_STABLE");tilt=0;
  const bool verified=spin_until([&]{return state.value("phase","")=="VERIFY_READY"||state.value("phase","")=="FAILED";},45);
  EXPECT_TRUE(verified);ASSERT_EQ(state.value("phase",""),"VERIFY_READY")<<state.dump();
  EXPECT_TRUE(state.value("gate_write",false));EXPECT_GE(slam.value("restore_generation",0),1);
  if(small_translation){
    EXPECT_GE(state.value("localization_verified_frames",0),6);EXPECT_GE(state.value("verified_frames",0),3);
    ASSERT_TRUE(state.contains("verified_pose"));const auto pose=state.at("verified_pose");
    EXPECT_NEAR(pose.at(0).get<double>(),truth_x,.08);EXPECT_NEAR(pose.at(1).get<double>(),0,.08);EXPECT_NEAR(pose.at(2).get<double>(),0,.08);
    const auto wheel=observed_tf.lookupTransform("odom","base_footprint",tf2::TimePointZero);
    EXPECT_DOUBLE_EQ(wheel.transform.translation.x,0);EXPECT_DOUBLE_EQ(wheel.transform.translation.y,0);
    EXPECT_DOUBLE_EQ(odom_x,0);
  }
  // 等导航确认期间只读新扫描，不得以心跳名义继续改图并推进接纳时间。
  const auto verified_sequence=slam.value("sequence",uint64_t{0});
  spin_until([]{return false;},1.3);
  EXPECT_EQ(slam.value("sequence",uint64_t{0}),verified_sequence);
  EXPECT_EQ(state.value("phase",""),"VERIFY_READY");
  if(!commit_case.empty()){
    // 资格来自新的独立扫描，不能用相同时间戳或先前AMCL六帧充当提交确认。
    // 旧正式二进制不提供该字段，仍继续施加坏帧以记录其立即FAILED的真实反例。
    if(state.contains("commit_ready")){ASSERT_TRUE(state.at("commit_ready").get<bool>())<<state.dump();}
    const auto before=state;
    if(commit_case=="stale"){
      emit_scans=false;
      ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},2))<<state.dump();
      EXPECT_FALSE(state.value("commit_ready",true));EXPECT_TRUE(state.value("gate_write",false));
      EXPECT_NE(state.value("reason","").find("扫描"),std::string::npos);
    }else{
      commit_bad_scan=true;
      ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED"||!state.value("commit_ready",true);},1))<<state.dump();
      ASSERT_EQ(state.value("phase",""),"VERIFY_READY")<<"单帧坏几何应冻结重观察，不立即永久失败"<<state.dump();
      EXPECT_TRUE(state.value("gate_write",false));EXPECT_FALSE(slam.value("admission_enabled",true));
      request("commit_verified","delayed-commit");
      ASSERT_TRUE(spin_until([&]{return state.value("commit_guard",Json::object()).value("pending_request_id","")=="delayed-commit";},1))<<state.dump();
      EXPECT_NE(state.value("phase",""),"SUCCEEDED");EXPECT_TRUE(state.value("gate_write",false));
      if(commit_case=="persistent"){
        ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},6))<<state.dump();
        EXPECT_NE(state.value("reason","").find("5秒"),std::string::npos);
      }else if(commit_case=="cancel"){
        ++gen;request("cancel","cancel-pending-commit");commit_bad_scan=false;
        ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},2));
        spin_until([]{return false;},1.0);
        EXPECT_EQ(state.value("phase",""),"FAILED");EXPECT_EQ(state.value("request_id",""),"cancel-pending-commit");
        EXPECT_FALSE(state.value("commit_ready",true));EXPECT_FALSE(slam.value("admission_enabled",true));
      }else{
        ASSERT_EQ(commit_case,"transient");
        // 延迟到达的提交在坏帧期间不能开门，也不能被静默丢弃。
        spin_until([]{return false;},.35);
        EXPECT_EQ(state.value("phase",""),"VERIFY_READY");EXPECT_FALSE(slam.value("admission_enabled",true));
        commit_bad_scan=false;
        ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="SUCCEEDED";},4))<<state.dump();
        EXPECT_EQ(state.value("request_id",""),"delayed-commit");EXPECT_FALSE(state.value("gate_write",true));
      }
    }
    std::ofstream(evidence/"commit-guard-result.json")<<Json{{"case",commit_case},{"before",before},{"after",state},{"slam",slam},{"hardware",false}}.dump(2);
    rclcpp::shutdown();return;
  }
  request("commit_verified","commit-1");ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="SUCCEEDED";},5))<<state.dump();
  EXPECT_FALSE(state.value("gate_write",true));EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"graph.posegraph"));
  const Json success=state;
  --gen;request("recover","stale-old-generation");spin_until([]{return false;},.5);
  EXPECT_EQ(state.value("phase",""),"SUCCEEDED");EXPECT_FALSE(state.value("gate_write",true));++gen;
  bool suspect=false;for(const auto&entry:std::filesystem::directory_iterator(std::filesystem::path(checkpoint).parent_path()))
    if(entry.path().filename().string().find("_suspect")!=std::string::npos&&std::filesystem::exists(entry.path()/"evidence.json"))suspect=true;
  EXPECT_TRUE(suspect);
  tilt=.25;ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="DISTURBED";},3));
  ++gen;request("recover","persistent-tilt",long_pause);
  ASSERT_TRUE(spin_until([&]{return state.value("phase","")=="FAILED";},25));
  EXPECT_TRUE(state.value("gate_write",false));EXPECT_TRUE(state.value("requires_stop",false));
  EXPECT_NE(state.value("reason","").find("平稳"),std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(checkpoint)/"graph.posegraph"));
  std::ofstream(evidence/"result.json")<<Json{{"waiting_before_first_tf",waiting_startup},{"successful_transaction",success},{"persistent_tilt_failure",state},{"slam",slam},{"hardware",false},{"odom_disturbance_m",odom_x},{"truth_translation_m",truth_x},{"small_translation_without_wheel_motion",small_translation},{"suspect_evidence_preserved",suspect}}.dump(2);
  rclcpp::shutdown();
}
