#include <chrono>
#include <deque>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <slam_toolbox/srv/deserialize_pose_graph.hpp>
#include <tf2_ros/buffer.h>
#include "carcar_mapping_autonomy/scan_pose.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <nav2_map_server/map_io.hpp>
#include "carcar_mapping_autonomy/recovery_core.hpp"
#include "carcar_mapping_autonomy/recovery_commit_guard.hpp"
#include "carcar_mapping_autonomy/recovery_artifact.hpp"
#include "carcar_mapping_autonomy/recovery_initializer.hpp"
#include "carcar_mapping_autonomy/snapshot_protocol.hpp"
namespace ca=carcar_mapping_autonomy;
using Json=nlohmann::json;
using Gate=std_srvs::srv::SetBool;
using Once=std_srvs::srv::Trigger;
using Empty=std_srvs::srv::Empty;
using Save=slam_toolbox::srv::SerializePoseGraph;
using Load=slam_toolbox::srv::DeserializePoseGraph;
static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static ca::Pose planar(const geometry_msgs::msg::Pose&p){return {p.position.x,p.position.y,std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z))};}
template<class T>static bool ready(const std::shared_future<T>&f){return f.valid()&&f.wait_for(std::chrono::seconds(0))==std::future_status::ready;}

// 恢复编排不发布运动命令，也不发布TF。检查点和候选未验证时保持入图硬门关闭。
class Recovery:public rclcpp::Node {
  struct Artifact{std::filesystem::path path;std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map;std::shared_ptr<ca::ReferenceMatcher> matcher;Json manifest;};
  struct Checked{ca::Pose pose;int64_t stamp=0;ca::ReferenceMatcher::Verification result;};
public:
  Recovery():Node("mapping_recovery"),tf_(get_clock()),listener_(tf_){
    root_=declare_parameter("checkpoint_root",std::string("/home/jetson/luhao/my_nav_carcar/log/mapping_checkpoints"));
    task_id_=declare_parameter("task_id",std::string(""));
    timeout_=std::clamp(declare_parameter("recovery_timeout_seconds",60.0),10.0,120.0);
    imu_required_=declare_parameter("require_imu",true);
    scan_topic_=declare_parameter("scan_topic",std::string("/scan/filtered"));
    mapping_range_=declare_parameter("mapping_range_m",3.0);
    online_snapshots_=declare_parameter("online_snapshots",false);
    slam_parameters_=create_client<rcl_interfaces::srv::GetParameters>("/slam_toolbox/get_parameters");
    gate_=create_client<Gate>("/mapping/slam/set_admission");once_=create_client<Once>("/mapping/slam/admit_once");
    save_=create_client<Save>("/slam_toolbox/serialize_map");load_=create_client<Load>("/slam_toolbox/deserialize_map");
    nomotion_=create_client<Empty>("/mapping/relocalizer/request_nomotion_update");
    state_pub_=create_publisher<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local());
    if(online_snapshots_){
      snapshot_request_pub_=create_publisher<std_msgs::msg::String>("/mapping/slam/snapshot/request",10);
      snapshot_sub_=create_subscription<std_msgs::msg::String>("/mapping/slam/snapshot/state",rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String&m){snapshot_result(m.data);});
      snapshot_state_={{"phase","IDLE"}};
    }
    map_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/mapping/recovery/reference_map",rclcpp::QoS(1).reliable().transient_local());
    initial_pub_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mapping/recovery/initialpose",rclcpp::QoS(1).reliable());
    scan_pub_=create_publisher<sensor_msgs::msg::LaserScan>("/mapping/recovery/scan",rclcpp::SensorDataQoS());
    trial_pub_=create_publisher<sensor_msgs::msg::LaserScan>("/mapping/slam_scan",rclcpp::SensorDataQoS());
    requests_=create_subscription<std_msgs::msg::String>("/mapping/recovery/request",10,[this](std_msgs::msg::String::ConstSharedPtr m){request(m->data);});
    autonomy_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){
      try{auto j=Json::parse(m->data);if(!task_id_.empty()&&j.value("task_id","")!=task_id_)return;
        if(task_id_.empty())task_id_=j.value("task_id",std::string(""));
        autonomy_state_=j;autonomy_at_=steady();still_=j.value("still",false);terminal_=j.value("navigation_terminal",false);
      }catch(const std::exception&){}
    });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
      if(!fresh(m->header.stamp))return;
      imu_.update({m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z},
        {m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z},still_&&steady()-autonomy_at_<.5,steady());
    });
    scans_=create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_,rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
      if(!fresh(m->header.stamp)||m->ranges.empty()||m->ranges.size()>100000||!std::isfinite(m->angle_min)||
        !std::isfinite(m->angle_increment)||m->angle_increment<=0||m->header.frame_id.empty())return;
      scan_=m;scan_at_=steady();scan_history_.push_back(m);while(scan_history_.size()>30)scan_history_.pop_front();
      if(phase_=="LOCALIZING"&&steady()-last_scan_sent_>.18){scan_pub_->publish(reference_scan(*m));last_scan_sent_=steady();}
      // 受控验证帧间隔留出原SLAM的0.20秒门槛，不用未接纳帧消耗一次性许可。
      if(trial_permit_&&steady()-trial_sent_>=.25){trial_permit_=false;trial_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();trial_pub_->publish(*m);trial_sent_=steady();}
    });
    maps_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){if(m->header.frame_id=="map"){map_=m;map_at_=steady();++map_count_;}});
    poses_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",rclcpp::QoS(1).reliable().transient_local(),[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
      if(m->header.frame_id!="map")return;
      pose_=m;pose_at_=steady();pose_odometry_.reset();cache_pose_odometry();
      if((phase_=="VERIFYING"||phase_=="VERIFY_READY")&&trial_stamp_>0&&rclcpp::Time(m->header.stamp).nanoseconds()==trial_stamp_)trial_pose_=m;
    });
    slam_=create_subscription<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){try{slam_state_=Json::parse(m->data);slam_at_=steady();}catch(const std::exception&){};});
    amcl_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/mapping/recovery/amcl_pose",rclcpp::QoS(10).reliable(),[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){amcl(m);});
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{try{tick();}catch(const std::exception&e){fail(e.what());}publish();});
  }
private:
  bool snapshot_current()const{
    return idle()&&!freeze_&&!imu_.disturbed&&(!imu_required_||(imu_.ready&&steady()-imu_.last_at<.5))&&
      steady()-autonomy_at_<1&&autonomy_state_.value("healthy",false)&&
      ca::snapshot_same_generation(snapshot_request_,autonomy_state_);
  }
  void snapshot_failed(const std::string&reason){
    snapshot_pending_=false;snapshot_candidate_.reset();snapshot_observations_.reset();
    snapshot_state_["phase"]="FAILED";snapshot_state_["reason"]=reason;snapshot_state_["in_flight"]=false;
    RCLCPP_WARN(get_logger(),"快照维护未提交 request_id=%s reason=%s；旧可信快照保留，定位健康时继续任务",snapshot_request_.value("request_id",std::string()).c_str(),reason.c_str());
  }
  void start_snapshot(const Json&request){
    if(!online_snapshots_||snapshot_pending_||snapshot_future_.valid()||request.value("request_id","")==snapshot_request_.value("request_id",""))return;
    snapshot_request_=request;snapshot_state_=request;snapshot_state_["phase"]="REQUESTED";snapshot_state_["in_flight"]=true;
    try{
      ca::validate_snapshot_request(request);
      // 在线首点也必须等待原有静止补观测结束，不能凭较早的三帧提前放行自动起步。
      if(!checkpoint_&&!slam_state_.value("startup_closed",true)){
        phase_="WARMING";snapshot_state_["phase"]="WAITING_STARTUP";snapshot_state_["in_flight"]=false;
        snapshot_state_["reason"]="首个可信快照等待有界静止补观测结束，车辆保持停车，入图继续开放";
        return;
      }
      if(!snapshot_current()||!mapping_range_ready_||(!checkpoint_&&!settled()))throw std::runtime_error("首个可信快照要求停稳；维护快照要求生产定位健康且任务代次一致");
      snapshot_candidate_.reset();snapshot_observations_.reset();
      snapshot_pending_=true;snapshot_begin_=steady();std_msgs::msg::String m;m.data=request.dump();snapshot_request_pub_->publish(m);
    }catch(const std::exception&e){snapshot_failed(e.what());}
  }
  void snapshot_result(const std::string&text){
    try{
      const auto result=Json::parse(text);
      if(!snapshot_pending_||result.value("request_id","")!=snapshot_request_.value("request_id","")||!ca::snapshot_same_generation(result,snapshot_request_))return;
      // 生产端READY只表示候选已发布；接收方未校验完时保持VALIDATING，不能提前宣告提升成功。
      // 同一READY会随心跳重复发布；进入新扫描验证后不能重启文件读取或清空已累计帧。
      if(snapshot_future_.valid()||snapshot_candidate_)return;
      snapshot_state_=result;
      if(result.value("phase","")=="FAILED"){snapshot_failed(result.value("reason",std::string("快照工作进程失败")));return;}
      if(result.value("phase","")!="READY"||result.value("in_flight",true)||snapshot_future_.valid())return;
      if(!snapshot_current())throw std::runtime_error("快照返回时任务或健康状态已改变");
      const auto path=std::filesystem::path(result.at("path").get<std::string>());
      const auto expected=root_/task_id_/snapshot_request_.at("request_id").get<std::string>();
      if(path.lexically_normal()!=expected.lexically_normal())throw std::runtime_error("快照返回目录与本次请求不一致");
      snapshot_state_["phase"]="VALIDATING";snapshot_state_["in_flight"]=true;const auto request=snapshot_request_;
      snapshot_future_=std::async(std::launch::async,[path,request]{
        const auto manifest=ca::recovery_validate_artifact(path);
        if(!ca::snapshot_same_generation(manifest,request)||manifest.at("request_id")!=request.at("request_id"))throw std::runtime_error("快照文件关联标识不一致");
        auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
        if(nav2_map_server::loadMapFromYaml((path/"map.yaml").string(),*map)!=nav2_map_server::LOAD_MAP_SUCCESS)throw std::runtime_error("快照栅格读取失败");
        map->header.frame_id="map";map->header.stamp=rclcpp::Time(manifest.at("capture_stamp_ns").get<int64_t>());
        auto matcher=std::make_shared<ca::ReferenceMatcher>(*map);return std::make_shared<Artifact>(Artifact{path,map,matcher,manifest});
      });
    }catch(const std::exception&e){snapshot_failed(e.what());}
  }
  void snapshot_tick(){
    try{
      if(snapshot_pending_&&(!snapshot_current()||steady()-snapshot_begin_>35))snapshot_failed("快照维护超时或任务／定位状态已改变，未提升候选");
      if(snapshot_future_.valid()&&snapshot_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        const auto candidate=snapshot_future_.get();if(!snapshot_pending_||!snapshot_current())return;
        snapshot_candidate_=candidate;snapshot_validation_begin_=steady();snapshot_observations_.reset();
        snapshot_state_["phase"]="VALIDATING";snapshot_state_["in_flight"]=true;snapshot_state_["validation_frames"]=0;
        snapshot_state_["capture_stamp_ns"]=candidate->manifest.at("capture_stamp_ns");
        snapshot_state_["scan_sequence"]=candidate->manifest.at("sequence");
        snapshot_state_["new_graph_observations"]=!checkpoint_||candidate->manifest.at("sequence").get<uint64_t>()>checkpoint_->manifest.value("sequence",uint64_t{0});
        snapshot_state_["reason"]="成对图档已校验，等待捕获之后三个新扫描核对当前位置，尚未提升可信参考";
      }
      if(!snapshot_pending_||!snapshot_candidate_||!snapshot_current())return;
      const auto candidate=snapshot_candidate_;
      const auto&meta=candidate->manifest;const double captured=meta.at("capture_steady_seconds");
      if(steady()-captured<0||steady()-captured>120||meta.at("loop_generation")!=slam_state_.value("loop_generation",uint64_t{0})||
        meta.at("restore_generation")!=slam_state_.value("restore_generation",uint64_t{0})||slam_state_.value("loop_active",false))
        throw std::runtime_error("快照已过期或闭环／恢复代次变化，拒绝提升");
      if(steady()-snapshot_validation_begin_>5)throw std::runtime_error("快照未通过捕获后连续新扫描的原几何检查，保留旧可信参考");
      if(!scan_||!pose_||steady()-scan_at_>.5)return;
      const auto stamp=rclcpp::Time(scan_->header.stamp).nanoseconds();
      if(stamp<=meta.at("capture_stamp_ns").get<int64_t>()||stamp<=snapshot_observations_.last_stamp)return;
      // 独立话题可能先到新位姿、后到对应扫描；等待下一帧，不把正常到达先后当作快照失败。
      if(stamp<rclcpp::Time(pose_->header.stamp).nanoseconds()){
        snapshot_state_["reason"]="等待与最新位姿时间顺序一致的新扫描，旧可信参考保留";return;
      }
      ca::Pose current_pose;std::vector<ca::ReferenceMatcher::Ray> current_rays;
      try{
        current_pose=ca::pose_at_scan(*pose_,scan_->header.stamp,tf_,pose_odometry_?&*pose_odometry_:nullptr);
        current_rays=rays(*scan_);
      }catch(const tf2::TransformException&){
        snapshot_state_["reason"]="等待捕获后新扫描的精确时刻TF，旧可信参考保留";return;
      }
      // 工作进程的构图扫描只能证明内部一致；用捕获之后的独立新观测审查原几何门槛。
      const auto geometry=candidate->matcher->score(current_pose,current_rays);
      const bool validated=snapshot_observations_.observe(current_pose,stamp,steady(),geometry.acceptable(),3,.2);
      snapshot_state_["validation_frames"]=snapshot_observations_.count;snapshot_state_["validation_scan_stamp_ns"]=stamp;
      snapshot_state_["validation_geometry"]={{"points",geometry.points},{"coverage",geometry.coverage},{"hit_ratio",geometry.hit_ratio},
        {"mean_residual_m",geometry.mean},{"free_conflict",geometry.free_conflict}};
      if(!validated)return;
      checkpoint_=candidate;checkpoint_at_=captured;map_pub_->publish(*checkpoint_->map);
      snapshot_candidate_.reset();
      snapshot_pending_=false;snapshot_state_["phase"]="READY";snapshot_state_["in_flight"]=false;snapshot_state_["reason"]="后台快照已成为可信恢复点";
      phase_="CHECKPOINT_READY";reason_="可信恢复点已更新，导航运行状态不受例行快照影响";
    }catch(const std::exception&e){snapshot_failed(e.what());}
  }
  void cache_pose_odometry(){
    if(!pose_||pose_odometry_)return;
    try{pose_odometry_=tf_.lookupTransform("odom","base_footprint",rclcpp::Time(pose_->header.stamp),rclcpp::Duration::from_seconds(0));}
    catch(const tf2::TransformException&){} // 话题可能先于对应TF到达，下一次定时回调再取精确时刻。
  }
  bool fresh(const builtin_interfaces::msg::Time&s)const{const double age=(now()-rclcpp::Time(s)).seconds();return age>=-.1&&age<.6;}
  bool settled()const{return still_&&terminal_&&steady()-autonomy_at_<.5&&(!imu_required_||imu_.settled(steady()));}
  bool idle()const{return phase_=="IDLE"||phase_=="WARMING"||phase_=="CHECKPOINT_READY"||phase_=="SUCCEEDED";}
  std::shared_ptr<Artifact> active_reference()const{return rollback_candidate_?rollback_candidate_:checkpoint_;}
  void request(const std::string&text){
    try{
      const auto j=Json::parse(text);const auto id=j.value("request_id",std::string(""));
      if(id.empty()||id==request_id_||id==pending_commit_request_||(!task_id_.empty()&&j.value("task_id","")!=task_id_))return;
      const auto generation=j.at("generation").get<uint64_t>();if(generation<generation_)return;
      const auto action=j.at("action").get<std::string>();
      if(action=="snapshot"){start_snapshot(j);return;}
      if(action=="freeze"){
        if(!idle()&&phase_!="DISTURBED")return;
        generation_=generation;request_id_=id;++token_;begin_=steady();reason_="监督器检测定位异常，冻结入图并等待导航终态";
        close_then("AWAIT_START");return;
      }
      if(action=="cancel"||action=="abort"){generation_=generation;request_id_=id;++token_;fail("恢复已取消；入图保持冻结，工程和原始数据保留");return;}
      if(action=="commit_verified"){
        if(generation!=generation_||phase_!="VERIFY_READY")return;
        // 监督器可能刚看到旧READY；保留请求，必须再核查本节点最新扫描，不能丢请求或提前开门。
        pending_commit_request_=id;reason_="已收到恢复提交，保持冻结，等待本轮新扫描资格核验";
        RCLCPP_INFO(get_logger(),"恢复提交请求已保留 request_id=%s generation=%lu；先核验最新扫描资格，尚未开门",id.c_str(),static_cast<unsigned long>(generation));
        return;
      }
      if(action!="checkpoint"&&action!="recover"&&action!="recover_rollback")return;
      if(!idle()&&phase_!="DISTURBED"&&phase_!="FAILED")return;
      if(initializer_future_.valid()){
        if(initializer_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready){
          reason_="前次已取消的只读匹配尚未结束，保持冻结；未启动重叠事务";return;
        }
        try{initializer_future_.get();}catch(const std::exception&){}
      }
      generation_=generation;request_id_=id;++token_;begin_=steady();reason_.clear();verification_.reset();
      pending_commit_request_.clear();commit_anchor_.reset();commit_odometry_.reset();commit_guard_.reset(steady());
      rollback_recovery_=action=="recover_rollback";rollback_candidate_.reset();trial_pose_.reset();localization_verified_frames_=0;
      if(rollback_recovery_&&!slam_state_.value("loop_guard",Json::object()).value("rolled_back",false))
        throw std::runtime_error("没有SLAM事务撤销确认，不能把当前地图用于原位复验");
      if(action=="checkpoint"||rollback_recovery_){
        checkpoint_retry_pending_=false;checkpoint_retry_count_=0;checkpoint_revalidation_deadline_=-1;
        if(imu_.disturbed||(!rollback_recovery_&&phase_=="DISTURBED"))throw std::runtime_error("受扰状态尚未恢复，不能把可疑生产图提升为可信检查点");
        if(rollback_recovery_){close_then("WAITING_ROLLBACK_STABLE");return;}
        // 首个检查点不能提前闭门打断有限补观测；普通旧入口未提供该字段时保持兼容。
        if(!checkpoint_&&!slam_state_.value("startup_closed",true)){
          phase_="WARMING";reason_="检查点尚未生成：等待有界静止补观测结束，入图保持开放，车辆保持停车";return;
        }
        if(!mapping_range_ready_||!settled()||!map_||!pose_||steady()-map_at_>10||steady()-slam_at_>1||slam_state_.value("sequence",uint64_t(0))<3||slam_state_.value("loop_active",false)){
          phase_="WARMING";reason_="检查点尚未生成：等待停稳、IMU稳定、三次SLAM接纳和有效地图";return;
        }
        close_then("SAVE_GRAPH");
      }else{
        if(!checkpoint_){fail("没有可信检查点，无法自动回退；保留工程并等待重新建图");return;}
        // 自动续跑只使用近期检查点；人工静止复核允许旧参考，但不跳过完整性、范围和多帧几何验证。
        if(steady()-checkpoint_at_>120&&!j.value("operator_revalidation",false)){fail("可信检查点距今超过120秒，超出自动回退范围；可在停车后请求人工重新检查");return;}
        close_then("VALIDATE_CHECKPOINT");
      }
    }catch(const std::exception&e){fail(e.what());}
  }
  void close_then(const std::string&next){
    freeze_=true;requires_stop_=true;
    if(!gate_->service_is_ready())throw std::runtime_error("SLAM入图硬门服务未就绪，保持停车");
    auto req=std::make_shared<Gate::Request>();req->data=false;
    gate_future_=gate_->async_send_request(req).future.share();gate_next_=next;
    phase_=next=="SAVE_GRAPH"?"CHECKPOINTING":(next=="AWAIT_START"?"DISTURBED":"WAITING_STABLE");gate_deadline_=steady()+2;
  }
  void open_then(const std::string&next){
    if(!gate_->service_is_ready())throw std::runtime_error("SLAM入图门未能恢复，保持停车");
    auto req=std::make_shared<Gate::Request>();req->data=true;
    gate_future_=gate_->async_send_request(req).future.share();gate_next_=next;gate_deadline_=steady()+2;opening_=true;
  }
  void save_graph(){
    if(!settled()||!save_->service_is_ready())throw std::runtime_error("稳定检查点保存条件已失效");
    checkpoint_map_=std::make_shared<nav_msgs::msg::OccupancyGrid>(*map_);checkpoint_sequence_=slam_state_.value("sequence",uint64_t(0));
    const auto timestamp=std::to_string(now().nanoseconds());
    const auto task=task_id_.empty()?std::string("local"):task_id_;
    if(task.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)throw std::runtime_error("检查点任务ID无效");
    pending_path_=root_/task/(timestamp+"_g"+std::to_string(generation_));std::filesystem::create_directories(pending_path_);
    auto req=std::make_shared<Save::Request>();req->filename=(pending_path_/"graph").string();
    save_future_=save_->async_send_request(req).future.share();deadline_=steady()+25;
    phase_="CHECKPOINTING";reason_="停车生成可信检查点：成对保存位姿图与栅格并校验完整性";
    checkpoint_wait_map_=false;
  }
  bool checkpoint_stability(double t){
    // 小幅比力方向波动只使检查点等待；轮式运动、数据陈旧和明显受扰仍立即拒绝。
    if(!still_||!terminal_||t-autonomy_at_>=.5)
      throw std::runtime_error("检查点未提交：轮式未停稳、导航未结束或监督状态陈旧；保持冻结，请检查停车反馈");
    if(imu_required_&&(!imu_.ready||t-imu_.last_at>=.5||!std::isfinite(imu_.norm)||!std::isfinite(imu_.gyro)||
        !std::isfinite(imu_.tilt)||imu_.disturbed||imu_.tilt>=.14||imu_.norm<=8||imu_.norm>=11.6||imu_.gyro>=.12))
      throw std::runtime_error("检查点未提交：IMU数据失效或超出轻微波动范围；保持冻结，请检查姿态和传感数据");
    if(checkpoint_retry_pending_&&t>checkpoint_revalidation_deadline_)
      throw std::runtime_error("检查点重验等待超过12秒：仍未连续稳定2秒或旧保存尚未结束；保持冻结，请检查振动和数据状态");
    if(!settled()){
      if(!checkpoint_retry_pending_){
        if(checkpoint_retry_count_>=2)throw std::runtime_error("检查点重验已达2次：反复波动，未提升候选；保持冻结等待检查");
        checkpoint_retry_pending_=true;++checkpoint_retry_count_;++token_;
        if(checkpoint_revalidation_deadline_<0)checkpoint_revalidation_deadline_=t+12;
        RCLCPP_WARN(get_logger(),"检查点暂缓 request_id=%s retry=%u reason=轻微IMU波动；停车冻结，连续稳定2秒后重新保存校验",request_id_.c_str(),checkpoint_retry_count_);
      }
      reason_="检查点尚未提交：轻微IMU波动，保持停车和冻结；等待连续稳定2秒后重新保存校验（最多等待12秒）";
      return false;
    }
    if(!checkpoint_retry_pending_)return true;
    // 不抢占仍在写文件的操作，也不提升波动前启动的旧异步结果。各次文件独立保留。
    if(save_future_.valid()){
      if(!ready(save_future_))return false;
      const auto result=save_future_.get();save_future_={};
      if(result->result!=0)throw std::runtime_error("检查点重验前旧位姿图保存失败，保持冻结");
    }
    if(artifact_future_.valid()){
      if(artifact_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return false;
      try{artifact_future_.get();}catch(const std::exception&e){
        RCLCPP_WARN(get_logger(),"检查点旧候选未提交 request_id=%s reason=%s；保留证据并重新验证",request_id_.c_str(),e.what());
      }
    }
    checkpoint_retry_pending_=false;checkpoint_wait_map_=true;checkpoint_gate_at_=t;checkpoint_map_before_=map_count_;
    checkpoint_min_stamp_=slam_state_.value("accepted_stamp_ns",int64_t(0));deadline_=t+12;
    reason_="已重新连续稳定2秒，入图仍冻结；重新获取栅格、保存位姿图并执行原几何校验";
    return false;
  }
  void localize(){
    const auto reference=active_reference();
    if(!reference||!pose_||!nomotion_->service_is_ready())throw std::runtime_error("AMCL旁路或当前位置尚未就绪");
    if(!settled()||steady()-scan_at_>.5)throw std::runtime_error("重定位前姿态／扫描不稳定");
    // 冻结期间车体可能正常转动；搜索先验必须与本次扫描同一时刻，不能沿用旧SLAM朝向。
    // 精确轮式历史缺失时继续保持冻结，不回退到最新TF或扩大搜索范围。
    localization_prior_=ca::pose_at_scan(*pose_,scan_->header.stamp,tf_,pose_odometry_?&*pose_odometry_:nullptr);
    initialization_json_={{"source","karto_2.6.10_correlate_scan"},{"prior_source","slam_pose_with_scan_time_odometry"},
      {"slam_stamp_ns",rclcpp::Time(pose_->header.stamp).nanoseconds()},
      {"scan_stamp_ns",rclcpp::Time(scan_->header.stamp).nanoseconds()},
      {"prior",{localization_prior_.x,localization_prior_.y,localization_prior_.yaw}}};
    const auto transform=tf_.lookupTransform("base_footprint",scan_->header.frame_id,rclcpp::Time(scan_->header.stamp),rclcpp::Duration::from_seconds(0));
    const auto&q=transform.transform.rotation;
    const ca::Pose offset{transform.transform.translation.x,transform.transform.translation.y,
      std::atan2(2*q.w*q.z,1-2*q.z*q.z)};
    const auto map=reference->map;const auto scan=std::make_shared<sensor_msgs::msg::LaserScan>(reference_scan(*scan_));const auto prior=localization_prior_;
    // 时间未过期不代表旧图覆盖当前观测；这里只记录先验处诊断，不能把先验误差直接当成拒绝依据。
    const auto support=reference->matcher->score(prior,rays(*scan));
    initialization_json_["reference_at_prior"]={{"checkpoint_id",reference->path.filename().string()},
      {"source",rollback_candidate_?"rolled_back_candidate":"trusted_checkpoint"},
      {"sequence",reference->manifest.value("sequence",uint64_t(0))},{"age_seconds",steady()-(rollback_candidate_?rollback_reference_at_:checkpoint_at_)},
      {"coverage",support.coverage},{"hit_ratio",support.hit_ratio},{"mean_residual_m",support.mean},{"points",support.points}};
    const auto support_detail=initialization_json_["reference_at_prior"].dump();
    initializer_token_=token_;initializer_future_=std::async(std::launch::async,[map,scan,prior,offset,support_detail]{
      try{return ca::recovery_initial_pose(*map,*scan,prior,offset);}
      catch(const std::exception&e){throw std::runtime_error(std::string(e.what())+
        "；先验处参考支持="+support_detail+"；未发布AMCL初值，请检查参考覆盖与当前位置，不能按低分强行续跑");}
    });
    phase_="INITIALIZING";reason_="用只读参考图的Karto相关匹配生成AMCL初值，尚未允许恢复";
    localization_begin_=steady();last_nomotion_=0;verification_.reset();
  }
  std::vector<ca::ReferenceMatcher::Ray> rays(const sensor_msgs::msg::LaserScan&m){
    const auto transform=tf_.lookupTransform("base_footprint",m.header.frame_id,rclcpp::Time(m.header.stamp),rclcpp::Duration::from_seconds(0));
    const auto&q=transform.transform.rotation;const auto&t=transform.transform.translation;
    tf2::Transform tf(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(t.x,t.y,t.z));
    const auto origin=tf*tf2::Vector3(0,0,0);std::vector<ca::ReferenceMatcher::Ray> out;
    const size_t stride=std::max<size_t>(1,m.ranges.size()/180);
    for(size_t i=0;i<m.ranges.size();i+=stride){const double r=m.ranges[i];if(!std::isfinite(r)||r<std::max(.15,double(m.range_min))||r>std::min(mapping_range_,double(m.range_max))-.05)continue;
      const double angle=m.angle_min+i*m.angle_increment;const auto p=tf*tf2::Vector3(r*std::cos(angle),r*std::sin(angle),0);
      out.push_back({p.x(),p.y(),origin.x(),origin.y()});
    }
    return out;
  }
  sensor_msgs::msg::LaserScan reference_scan(const sensor_msgs::msg::LaserScan&input)const{
    auto output=input;output.range_max=std::min(double(input.range_max),mapping_range_);
    for(auto&value:output.ranges)if(!std::isfinite(value)||value>output.range_max-.05)
      value=std::numeric_limits<float>::quiet_NaN();
    return output;
  }
  void amcl(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
    if(phase_!="LOCALIZING"||m->header.frame_id!="map"||!fresh(m->header.stamp)||!settled()||check_future_.valid())return;
    for(size_t i:{0u,7u,35u})if(!std::isfinite(m->pose.covariance[i])||m->pose.covariance[i]<0||m->pose.covariance[i]>.04)return;
    const auto candidate=planar(m->pose.pose);
    if(!std::isfinite(candidate.x)||!std::isfinite(candidate.y)||!std::isfinite(candidate.yaw))return;
    if(std::hypot(candidate.x-localization_prior_.x,candidate.y-localization_prior_.y)>.75||
      std::abs(std::remainder(candidate.yaw-localization_prior_.yaw,2*M_PI))>.60){
      reason_="AMCL候选超出本轮0.75米／0.60弧度有限恢复先验，拒绝自动提交";return;
    }
    const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();sensor_msgs::msg::LaserScan::ConstSharedPtr matched;
    for(const auto&s:scan_history_)if(rclcpp::Time(s->header.stamp).nanoseconds()==stamp){matched=s;break;}
    if(!matched)return;
    try{
      auto points=rays(*matched);auto matcher=active_reference()->matcher;const auto pose=planar(m->pose.pose);
      check_token_=token_;check_future_=std::async(std::launch::async,[points=std::move(points),matcher,pose,stamp]{return Checked{pose,stamp,matcher->verify(pose,points)};});
    }catch(const tf2::TransformException&){reason_="等待激光至底盘的新鲜TF，尚未提交重定位候选";}
  }
  void restore(const ca::Pose&p){
    // 旧生产图可能已经含有可疑扫描；覆盖前保存原样证据，绝不覆盖可信检查点。
    if(!save_->service_is_ready())throw std::runtime_error("受扰原图证据保存服务未就绪");
    verified_=p;suspect_path_=root_/(task_id_.empty()?"local":task_id_)/(std::to_string(now().nanoseconds())+"_suspect");
    std::filesystem::create_directories(suspect_path_);
    auto preserve=std::make_shared<Save::Request>();preserve->filename=(suspect_path_/"graph").string();
    save_future_=save_->async_send_request(preserve).future.share();phase_="PRESERVING_SUSPECT";deadline_=steady()+25;
    reason_="保留受扰后的可疑位姿图与栅格，随后才回退可信检查点";
  }
  void deserialize(){
    if(!load_->service_is_ready())throw std::runtime_error("SLAM恢复服务未就绪");
    auto req=std::make_shared<Load::Request>();req->filename=(active_reference()->path/"graph").string();
    req->match_type=Load::Request::START_AT_GIVEN_POSE;req->initial_pose.x=verified_.x;req->initial_pose.y=verified_.y;req->initial_pose.theta=verified_.yaw;
    before_restore_=slam_state_.value("restore_generation",uint64_t(0));load_future_=load_->async_send_request(req).future.share();
    phase_="RESTORING";deadline_=steady()+20;reason_="旁路候选已通过多帧几何检查，恢复可信位姿图";
  }
  void begin_trial(){
    if(!once_->service_is_ready())throw std::runtime_error("SLAM单帧验证服务未就绪");
    auto req=std::make_shared<Once::Request>();once_future_=once_->async_send_request(req).future.share();trial_stamp_=0;trial_pose_.reset();deadline_=steady()+3;
  }
  bool commit_observation(double t){
    if(!settled())throw std::runtime_error("监督器提交前姿态再次异常，保持冻结");
    if(!scan_||t-scan_at_>.5||!fresh(scan_->header.stamp))
      throw std::runtime_error("监督器提交前过滤扫描超过0.5秒未更新或时间戳失效，保持冻结");
    if(!commit_anchor_)throw std::runtime_error("恢复提交缺少已通过三帧SLAM的位姿锚点，保持冻结");
    const auto stamp=rclcpp::Time(scan_->header.stamp).nanoseconds();
    if(stamp<=rclcpp::Time(commit_anchor_->header.stamp).nanoseconds()){
      commit_guard_.invalidate(t);reason_="等待晚于已验证SLAM位姿的新扫描，入图保持冻结";
    }else if(stamp>commit_guard_.scan_stamp_ns){
      try{
        if(!commit_odometry_)commit_odometry_=tf_.lookupTransform("odom","base_footprint",rclcpp::Time(commit_anchor_->header.stamp),rclcpp::Duration::from_seconds(0));
        const auto aligned=ca::pose_at_scan(*commit_anchor_,scan_->header.stamp,tf_,&*commit_odometry_);
        if(!ca::RecoveryCommitGuard::agrees(aligned,verified_))
          throw std::runtime_error("监督器提交前扫描时刻位置超出原AMCL候选12厘米／0.10弧度界限，保持冻结");
        const auto geometry=active_reference()->matcher->score(aligned,rays(*scan_));
        commit_scan_pose_=aligned;commit_guard_.observe(aligned,stamp,t,geometry);
        if(!geometry.acceptable())RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
          "恢复提交资格暂撤 request_id=%s scan_stamp_ns=%ld scan_age_s=%.3f points=%zu coverage=%.6f hit_ratio=%.6f mean_residual_m=%.6f free_conflict=%.6f；保持冻结等待新扫描",
          request_id_.c_str(),static_cast<long>(stamp),t-scan_at_,geometry.points,geometry.coverage,geometry.hit_ratio,geometry.mean,geometry.free_conflict);
        reason_=commit_guard_.ready?"提交前连续新扫描已通过原几何门槛，等待监督器提交":
          "提交前观测暂未形成连续资格，保持冻结，最多5秒等待后续新扫描";
      }catch(const tf2::TransformException&){
        commit_guard_.invalidate(t);reason_="提交前等待扫描精确时刻TF，入图保持冻结";
      }
    }
    if(commit_guard_.expired(t))throw std::runtime_error("监督器提交前5秒内未重新取得连续新扫描的原几何资格，保持冻结");
    return commit_guard_.ready&&stamp==commit_guard_.scan_stamp_ns;
  }
  void try_commit(double t){
    if(!commit_observation(t)||pending_commit_request_.empty()||gate_future_.valid())return;
    if(rollback_candidate_){checkpoint_=rollback_candidate_;rollback_candidate_.reset();checkpoint_at_=t;}
    // 只有最新已接收扫描仍具本次资格时，才发出开门请求；待应答期间保留待提交标识。
    open_then("SUCCEEDED");
  }
  void fail(const std::string&why){
    phase_="FAILED";reason_=why;freeze_=true;requires_stop_=true;trial_permit_=false;++token_;
    if(!pending_commit_request_.empty())RCLCPP_WARN(get_logger(),"待处理恢复提交已终止 request_id=%s reason=%s；未确认恢复开门",pending_commit_request_.c_str(),why.c_str());
    pending_commit_request_.clear();commit_guard_.invalidate(steady());
    if(rollback_candidate_){rollback_candidate_.reset();if(checkpoint_)map_pub_->publish(*checkpoint_->map);}
    gate_future_={};save_future_={};load_future_={};once_future_={};opening_=false;gate_next_.clear();
    RCLCPP_ERROR(get_logger(),"恢复未提交 request_id=%s generation=%lu reason=%s；保持停车保留原始数据",request_id_.c_str(),static_cast<unsigned long>(generation_),why.c_str());
    if(gate_->service_is_ready()){auto req=std::make_shared<Gate::Request>();req->data=false;gate_->async_send_request(req);}
  }
  void tick(){
    cache_pose_odometry();
    if(online_snapshots_)snapshot_tick();
    const double t=steady();
    if(!mapping_range_ready_){
      if(range_future_.valid()&&ready(range_future_)){
        const auto result=range_future_.get();range_future_={};
        // Humble在第一帧激光通过TF过滤后才声明该参数；未声明是启动等待，不能闭门阻断首帧。
        if(result->values.empty()||(result->values.size()==1&&result->values[0].type==rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET)){
          ++mapping_range_waits_;
          if(phase_=="WARMING")reason_="等待首帧激光与TF就绪后读回SLAM实际入图距离；保持停车，尚未建立检查点";
        }else{
        if(result->values.size()!=1||result->values[0].type!=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE||
          !std::isfinite(result->values[0].double_value)||result->values[0].double_value<.5||result->values[0].double_value>30)
          throw std::runtime_error("SLAM实际最大入图距离读回无效，未允许创建可信检查点");
        mapping_range_=result->values[0].double_value;mapping_range_ready_=true;
        }
      }else if(range_future_.valid()&&t-range_request_at_>2){slam_parameters_->remove_pending_request(range_request_id_);range_future_={};}
      if(!range_future_.valid()&&!mapping_range_ready_&&t-range_request_at_>2&&slam_parameters_->service_is_ready()){
        auto request=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();request->names={"max_laser_range"};
        auto pending=slam_parameters_->async_send_request(request);range_request_id_=pending.request_id;range_future_=pending.future.share();range_request_at_=t;
      }
    }
    if(idle()&&imu_required_&&imu_.ready&&autonomy_state_.value("control_mode","")=="automatic"&&
      (t-imu_.last_at>.5||!std::isfinite(imu_.norm)||!std::isfinite(imu_.gyro))){
      imu_.disturbed=true;imu_.stable=false;imu_.stable_since=0;imu_.reason="IMU受扰观察陈旧或非有限，暂停入图等待恢复";
    }
    if(idle()&&autonomy_state_.value("control_mode","")=="automatic"&&imu_.disturbed){phase_="DISTURBED";freeze_=true;requires_stop_=true;reason_=imu_.reason;close_then("AWAIT_START");}
    if(opening_&&gate_next_=="SUCCEEDED"&&!commit_observation(t)){
      // 开门应答尚未到达时出现新反证，立即按同一服务的先后顺序重新闭门，再等待新资格。
      auto req=std::make_shared<Gate::Request>();req->data=false;
      gate_future_=gate_->async_send_request(req).future.share();gate_next_="VERIFY_READY";opening_=false;gate_deadline_=t+2;
    }
    if(gate_future_.valid()){
      if(ready(gate_future_)){
        if(!gate_future_.get()->success){gate_future_={};throw std::runtime_error("SLAM拒绝入图门变更");}gate_future_={};
        const auto next=gate_next_;gate_next_.clear();
        if(opening_){opening_=false;freeze_=false;requires_stop_=false;phase_=next;imu_.acknowledge();
          if(next=="SUCCEEDED"){request_id_=pending_commit_request_;pending_commit_request_.clear();}
          reason_=next=="SUCCEEDED"?"恢复提交完成，等待监督器按原任务预算继续":"可信检查点已完成，原始和历史图均保留";}
        else if(next=="SAVE_GRAPH"){
          checkpoint_wait_map_=true;checkpoint_gate_at_=t;checkpoint_map_before_=map_count_;checkpoint_min_stamp_=slam_state_.value("accepted_stamp_ns",int64_t(0));deadline_=t+12;
          reason_="入图已冻结，等待最后图内扫描生成新的栅格，再保存一致检查点";
        }
        else if(next=="WAITING_STABLE"){phase_="WAITING_STABLE";reason_="入图已冻结，等待车身平放、导航终态和轮式停稳";}
        else if(next=="WAITING_ROLLBACK_STABLE"){phase_=next;reason_="错误回环已撤销，等待连续停稳后保存候选并多帧复验";}
        else if(next=="VALIDATE_CHECKPOINT"){
          const auto path=checkpoint_->path;validation_token_=token_;
          validation_future_=std::async(std::launch::async,[path]{return ca::recovery_validate_artifact(path);});
          phase_="VALIDATING_CHECKPOINT";reason_="入图已冻结，校验可信检查点成对文件和SHA256";
        }
        else if(next=="VERIFY_READY")phase_=next;
        else phase_="DISTURBED";
      }else if(t>gate_deadline_)throw std::runtime_error("SLAM入图门等待超时");
      return;
    }
    if(phase_=="FAILED"||phase_=="DISTURBED"||idle())return;
    if(t-begin_>timeout_)throw std::runtime_error("自动恢复总时限已耗尽，未恢复运动");
    if(phase_=="WAITING_ROLLBACK_STABLE"){
      if(mapping_range_ready_&&settled()&&map_&&pose_&&t-map_at_<1&&t-slam_at_<1&&
         slam_state_.value("sequence",uint64_t(0))>=3&&!slam_state_.value("loop_active",false))close_then("SAVE_GRAPH");
      else if(t-begin_>20)throw std::runtime_error("回环撤销后20秒内未取得连续停稳与新鲜地图，保持锁停");
    }else if(phase_=="CHECKPOINTING"){
      if(!checkpoint_stability(t))return;
      if(checkpoint_wait_map_){
        if(map_&&map_count_>checkpoint_map_before_&&t-map_at_<1&&rclcpp::Time(map_->header.stamp).nanoseconds()>=checkpoint_min_stamp_&&t-checkpoint_gate_at_>=.3)save_graph();
        else if(t>deadline_)throw std::runtime_error("入图冻结后未收到新栅格，检查点未提升");
        return;
      }
      if(ready(save_future_)){
        const auto result=save_future_.get();save_future_={};if(result->result!=0)throw std::runtime_error("位姿图检查点保存失败");
        if(!scan_||!pose_)throw std::runtime_error("检查点缺少对应扫描与位姿");
        const auto current_rays=rays(*scan_);const auto current_pose=ca::pose_at_scan(*pose_,scan_->header.stamp,tf_,pose_odometry_?&*pose_odometry_:nullptr);
        const auto directory=pending_path_;const auto map=checkpoint_map_;const auto sequence=checkpoint_sequence_;const auto gen=generation_;
        Json origin=autonomy_state_.value("origin",Json::array());const auto task=task_id_;artifact_token_=token_;
        artifact_future_=std::async(std::launch::async,[directory,map,sequence,gen,origin,task,current_rays,current_pose]{
          auto matcher=std::make_shared<ca::ReferenceMatcher>(*map);const auto geometry=matcher->score(current_pose,current_rays);
          if(!geometry.acceptable())throw std::runtime_error("检查点当前激光与栅格残差／重叠不通过，拒绝提升可信检查点："+
            Json{{"points",geometry.points},{"mean_residual_m",geometry.mean},{"coverage",geometry.coverage},
              {"hit_ratio",geometry.hit_ratio},{"free_conflict",geometry.free_conflict}}.dump());
          ca::recovery_write_map(directory,*map);Json manifest{{"complete",true},{"slam_version","2.6.10"},{"base_frame","base_footprint"},{"generation",gen},{"sequence",sequence},{"origin",origin},{"task_id",task}};
          manifest["pose"]={current_pose.x,current_pose.y,current_pose.yaw};manifest["map_stamp_ns"]=rclcpp::Time(map->header.stamp).nanoseconds();
          manifest["geometry"]={{"mean_residual_m",geometry.mean},{"coverage",geometry.coverage},{"hit_ratio",geometry.hit_ratio},{"free_conflict",geometry.free_conflict}};
          for(const auto*name:{"graph.posegraph","graph.data","map.pgm","map.yaml"})manifest["sha256"][name]=ca::recovery_sha256(directory/name);
          {std::ofstream output(directory/"manifest.json.tmp");output<<manifest.dump(2)<<'\n';output.close();if(!output)throw std::runtime_error("检查点manifest写入失败");}
          std::filesystem::rename(directory/"manifest.json.tmp",directory/"manifest.json");ca::recovery_validate_artifact(directory);
          return std::make_shared<Artifact>(Artifact{directory,map,matcher,manifest});
        });
      }
      if(artifact_future_.valid()&&artifact_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        const auto artifact=artifact_future_.get();if(artifact_token_!=token_)return;
        if(rollback_recovery_){
          rollback_candidate_=artifact;rollback_reference_at_=t;map_pub_->publish(*artifact->map);
          phase_="VERIFYING_ROLLBACK";verification_.reset();localization_begin_=t;
          reason_="错误回环已撤销，当前图仍冻结；准备原Karto／AMCL多帧定位链路，候选尚未提升为可信检查点";
        }else{checkpoint_=artifact;checkpoint_at_=t;map_pub_->publish(*checkpoint_->map);open_then("CHECKPOINT_READY");}
      }
      if(t>deadline_)throw std::runtime_error("检查点文件或校验超时");
    }else if(phase_=="VERIFYING_ROLLBACK"){
      if(!settled()||!scan_||t-scan_at_>.5)throw std::runtime_error("回环撤销后观测失效或车辆未停稳，拒绝继续");
      // 先发布候选参考图，再启动原匹配链，避免地图与initialpose跨话题紧邻到达。
      if(t-rollback_reference_at_>=.5)localize();
    }else if(phase_=="VALIDATING_CHECKPOINT"){
      if(validation_future_.valid()&&validation_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        validation_future_.get();if(validation_token_!=token_)return;phase_="WAITING_STABLE";reason_="可信检查点完整，等待车身平放后重定位";
      }
    }else if(phase_=="WAITING_STABLE"){
      if(settled()&&t-scan_at_<.5)localize();else if(t-begin_>20)throw std::runtime_error("受扰后20秒内未恢复平稳，可能仍有车轮骑在障碍上");
    }else if(phase_=="INITIALIZING"){
      if(!settled())throw std::runtime_error("旁路初值生成期间姿态变化，拒绝重定位");
      if(initializer_future_.valid()&&initializer_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        const auto candidate=initializer_future_.get();if(initializer_token_!=token_)return;
        // 检查点提升时已经发布参考图；不要紧贴initialpose重发地图，避免跨话题乱序重置粒子。
        geometry_msgs::msg::PoseWithCovarianceStamped initial;initial.header.stamp=now();initial.header.frame_id="map";
        initial.pose.pose.position.x=candidate.x;initial.pose.pose.position.y=candidate.y;
        initial.pose.pose.orientation.z=std::sin(candidate.yaw/2);initial.pose.pose.orientation.w=std::cos(candidate.yaw/2);
        // 这里的协方差是已匹配初值的搜索分布，不是恢复成功或真实精度证明。
        initial.pose.covariance[0]=.01;initial.pose.covariance[7]=.01;initial.pose.covariance[35]=.0025;
        initial_pub_->publish(initial);phase_="LOCALIZING";reason_="Karto初值已生成，等待AMCL六帧与独立几何／歧义验证";
        initialization_json_["pose"]={candidate.x,candidate.y,candidate.yaw};
      }
      if(t-localization_begin_>10)throw std::runtime_error("只读参考图相关匹配超过10秒，未恢复运动");
    }else if(phase_=="LOCALIZING"){
      if(!settled())throw std::runtime_error("重定位期间再次倾斜或移动，拒绝候选");
      if(t-last_nomotion_>.2&&nomotion_->service_is_ready()){auto req=std::make_shared<Empty::Request>();nomotion_->async_send_request(req);last_nomotion_=t;}
      if(check_future_.valid()&&check_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        const auto checked=check_future_.get();if(check_token_!=token_)return;
        const auto&v=checked.result;verification_json_={{"mean_residual_m",v.score.mean},{"coverage",v.score.coverage},{"hit_ratio",v.score.hit_ratio},{"free_conflict",v.score.free_conflict},{"ambiguity_margin",v.ambiguity_margin},{"global_sample_margin",v.global_margin},{"global_samples",v.global_evaluations},{"ambiguity_scope","local_refinement_and_whole_map_sampled_competitors"},{"unique",v.unique},{"frames",verification_.count}};
        if(verification_.observe(checked.pose,checked.stamp,t,v.unique&&v.score.acceptable())){
          localization_verified_frames_=verification_.count;restore(checked.pose);
        }
      }
      if(t-localization_begin_>25)throw std::runtime_error("AMCL旁路未通过重叠、残差、多解和多帧检查；保持停车");
    }else if(phase_=="PRESERVING_SUSPECT"){
      if(ready(save_future_)){
        const auto response=save_future_.get();save_future_={};
        if(response->result!=0)throw std::runtime_error("受扰原图证据保存失败，拒绝覆盖生产图");
        const auto path=suspect_path_;const auto map=map_;suspect_token_=token_;
        Json evidence{{"kind","suspect"},{"generation",generation_},{"request_id",request_id_},{"checkpoint",active_reference()->path.string()},{"verified_pose",{verified_.x,verified_.y,verified_.yaw}}};
        suspect_future_=std::async(std::launch::async,[path,map,evidence=std::move(evidence)]()mutable{
          if(!map)throw std::runtime_error("受扰证据缺少栅格");
          ca::recovery_write_map(path,*map);
          for(const auto*name:{"graph.posegraph","graph.data","map.pgm","map.yaml"})evidence["sha256"][name]=ca::recovery_sha256(path/name);
          std::ofstream out(path/"evidence.json");out<<evidence.dump(2)<<'\n';out.close();if(!out)throw std::runtime_error("受扰证据元数据写入失败");
        });
      }
      if(suspect_future_.valid()&&suspect_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){suspect_future_.get();if(suspect_token_!=token_)return;deserialize();}
      if(t>deadline_)throw std::runtime_error("受扰图证据保存超时");
    }else if(phase_=="RESTORING"){
      if(ready(load_future_)){
        load_future_.get();load_future_={};
        if(!slam_state_.value("restore_error",std::string()).empty())throw std::runtime_error("SLAM拒绝恢复："+slam_state_.value("restore_error",std::string()));
        // 服务应答没有成功字段，必须等适配器确认真正替换了图。
        phase_="WAIT_RESTORE";
      }
      if(t>deadline_)throw std::runtime_error("SLAM图恢复服务超时");
    }else if(phase_=="WAIT_RESTORE"){
      if(slam_state_.value("restore_generation",uint64_t(0))>before_restore_){phase_="VERIFYING";verification_.reset();begin_trial();}
      if(t>deadline_)throw std::runtime_error("未收到SLAM图恢复代次确认");
    }else if(phase_=="VERIFY_READY"){
      try_commit(t);
      // 三帧验证后保持图不变，等待导航重建并提交。独立观测订阅维持TF新鲜度，
      // 不能再用反复入图作心跳，否则接纳时间持续领先地图，导航验证永远追不上。
    }else if(phase_=="VERIFYING"){
      if(!settled())throw std::runtime_error("恢复验证期间车身未保持平稳");
      if(ready(once_future_)){
        const auto response=once_future_.get();once_future_={};if(!response->success)throw std::runtime_error("单帧SLAM验证被拒绝："+response->message);trial_permit_=true;
      }
      if(trial_pose_&&slam_state_.value("accepted_stamp_ns",int64_t(0))==trial_stamp_){
        const auto p=planar(trial_pose_->pose.pose);const bool agrees=ca::RecoveryCommitGuard::agrees(p,verified_);
        if(!agrees)throw std::runtime_error("恢复后SLAM位姿与已验证旁路候选不一致");
        if(verification_.observe(p,trial_stamp_,t,true,3,.4)){
          // 后续扫描从最后已通过验证的SLAM位置传播，避免继续在旧AMCL位置上评分。
          commit_anchor_=trial_pose_;commit_odometry_.reset();commit_guard_.reset(t);pending_commit_request_.clear();
          phase_="VERIFY_READY";reason_="SLAM恢复后的多帧位姿已验证，等待提交前新扫描及地图TF／掩膜／导航核验";trial_stamp_=0;
        }else begin_trial();
      }
      if(t>deadline_)throw std::runtime_error("SLAM单帧接纳或对应位姿确认超时");
    }
  }
  void publish(){
    Json j{{"phase",phase_},{"gate_write",freeze_},{"requires_stop",requires_stop_},{"generation",generation_},{"request_id",request_id_},{"task_id",task_id_},
      {"reason",reason_},{"checkpoint_id",checkpoint_?checkpoint_->path.filename().string():""},{"checkpoint_path",checkpoint_?checkpoint_->path.string():""},
      {"checkpoint_age_seconds",checkpoint_?steady()-checkpoint_at_:-1},{"checkpoint_sequence",checkpoint_?checkpoint_->manifest.value("sequence",uint64_t(0)):0},
      {"imu_required",imu_required_},{"imu_ready",imu_.ready},{"imu_stable",!imu_required_||imu_.settled(steady())},{"imu_age_seconds",steady()-imu_.last_at},
      {"imu_valid",imu_.ready&&steady()-imu_.last_at<.5&&std::isfinite(imu_.norm)&&std::isfinite(imu_.gyro)},
      {"imu_acceleration_norm_mps2",imu_.norm},{"imu_gyro_norm_radps",imu_.gyro},{"imu_reason",imu_.reason},
      {"imu_relative_tilt_deg",imu_.tilt*180/M_PI},{"verified_pose",{verified_.x,verified_.y,verified_.yaw}},{"verification",verification_json_},
      {"verified_frames",verification_.count},{"localization_verified_frames",localization_verified_frames_},
      {"initialization",initialization_json_},{"note","IMU仅受扰观察；旁路不广播TF，失败不强行返程"}};
    j["mapping_range_m"]=mapping_range_;j["mapping_range_ready"]=mapping_range_ready_;j["mapping_range_waits"]=mapping_range_waits_;
    j["snapshot"]=snapshot_state_;j["online_snapshots"]=online_snapshots_;
    const bool commit_ready=phase_=="VERIFY_READY"&&!opening_&&commit_guard_.ready&&scan_&&
      steady()-scan_at_<=.5&&fresh(scan_->header.stamp)&&
      rclcpp::Time(scan_->header.stamp).nanoseconds()==commit_guard_.scan_stamp_ns&&settled();
    j["commit_ready"]=commit_ready;
    const auto&score=commit_guard_.score;
    j["commit_guard"]={{"phase",phase_=="FAILED"?"FAILED":(phase_!="VERIFY_READY"?"IDLE":(commit_ready?"READY":"WAITING"))},
      {"scan_stamp_ns",commit_guard_.scan_stamp_ns},{"frames",commit_guard_.observations.count},
      {"pending_request_id",pending_commit_request_},{"wait_seconds",commit_guard_.waiting_seconds(steady())},{"limit_seconds",ca::RecoveryCommitGuard::wait_limit_seconds},
      {"scan_pose",{commit_scan_pose_.x,commit_scan_pose_.y,commit_scan_pose_.yaw}},
      {"anchor_pose",commit_anchor_?Json::array({commit_anchor_->pose.pose.position.x,commit_anchor_->pose.pose.position.y,planar(commit_anchor_->pose.pose).yaw}):Json()},
      {"score",{{"points",score.points},{"coverage",score.coverage},{"hit_ratio",score.hit_ratio},{"mean_residual_m",score.mean},{"free_conflict",score.free_conflict},{"acceptable",score.acceptable()}}}};
    j["checkpoint_revalidation"]={{"waiting",checkpoint_retry_pending_},{"attempts",checkpoint_retry_count_},{"limit",2},
      {"wait_limit_seconds",12},{"reason",phase_=="CHECKPOINTING"?reason_:""}};
    std_msgs::msg::String m;m.data=j.dump();state_pub_->publish(m);
  }
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  std::filesystem::path root_,pending_path_,suspect_path_;std::string task_id_,scan_topic_,phase_="WARMING",request_id_,reason_,gate_next_;
  uint64_t generation_=0,token_=0,artifact_token_=0,check_token_=0,before_restore_=0,checkpoint_sequence_=0,map_count_=0,checkpoint_map_before_=0,validation_token_=0,suspect_token_=0,initializer_token_=0;
  bool imu_required_=true,still_=false,terminal_=false,freeze_=false,requires_stop_=false,opening_=false,trial_permit_=false,checkpoint_wait_map_=false;
  bool checkpoint_retry_pending_=false;unsigned checkpoint_retry_count_=0;double checkpoint_revalidation_deadline_=-1;
  double timeout_=60,begin_=0,deadline_=0,gate_deadline_=0,autonomy_at_=-1e9,scan_at_=-1e9,map_at_=-1e9,pose_at_=-1e9,slam_at_=-1e9,
    checkpoint_at_=0,localization_begin_=0,last_scan_sent_=0,last_nomotion_=0,trial_sent_=0,checkpoint_gate_at_=0;
  int64_t trial_stamp_=0,checkpoint_min_stamp_=0;ca::Pose verified_,localization_prior_;ca::DisturbanceMonitor imu_;ca::MultiFrameVerification verification_;
  ca::RecoveryCommitGuard commit_guard_;ca::Pose commit_scan_pose_;std::string pending_commit_request_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr commit_anchor_;
  std::optional<geometry_msgs::msg::TransformStamped>commit_odometry_;
  Json autonomy_state_=Json::object(),slam_state_=Json::object(),verification_json_=Json::object(),initialization_json_=Json::object();
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;std::shared_ptr<nav_msgs::msg::OccupancyGrid> checkpoint_map_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_,trial_pose_;
  std::optional<geometry_msgs::msg::TransformStamped>pose_odometry_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr>scan_history_;
  std::shared_ptr<Artifact>checkpoint_;std::future<std::shared_ptr<Artifact>>artifact_future_;std::future<Checked>check_future_;
  bool rollback_recovery_=false;double rollback_reference_at_=0;size_t localization_verified_frames_=0;std::shared_ptr<Artifact>rollback_candidate_;
  std::future<Json>validation_future_;std::future<void>suspect_future_;
  std::future<ca::Pose>initializer_future_;
  bool online_snapshots_=false,snapshot_pending_=false;double snapshot_begin_=0;
  Json snapshot_request_=Json::object(),snapshot_state_={{"phase","DISABLED"}};
  std::future<std::shared_ptr<Artifact>>snapshot_future_;
  std::shared_ptr<Artifact>snapshot_candidate_;ca::MultiFrameVerification snapshot_observations_;double snapshot_validation_begin_=0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr snapshot_request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr snapshot_sub_;
  double mapping_range_=3,range_request_at_=-1e9;bool mapping_range_ready_=false;int64_t range_request_id_=0;uint64_t mapping_range_waits_=0;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr slam_parameters_;
  std::shared_future<rcl_interfaces::srv::GetParameters::Response::SharedPtr>range_future_;
  std::shared_future<Gate::Response::SharedPtr>gate_future_;std::shared_future<Once::Response::SharedPtr>once_future_;
  std::shared_future<Save::Response::SharedPtr>save_future_;std::shared_future<Load::Response::SharedPtr>load_future_;
  rclcpp::Client<Gate>::SharedPtr gate_;rclcpp::Client<Once>::SharedPtr once_;rclcpp::Client<Empty>::SharedPtr nomotion_;
  rclcpp::Client<Save>::SharedPtr save_;rclcpp::Client<Load>::SharedPtr load_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_,trial_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requests_,autonomy_,slam_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scans_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poses_,amcl_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Recovery>());rclcpp::shutdown();return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
