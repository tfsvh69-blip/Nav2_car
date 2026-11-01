#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <random>
#include <sstream>
#include <Eigen/Eigenvalues>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include "carcar_mapping_autonomy/near_field.hpp"
#include "carcar_mapping_autonomy/task.hpp"
#include "carcar_mapping_autonomy/route_validation.hpp"
#include "carcar_mapping_autonomy/observation_selector.hpp"
#include "carcar_mapping_autonomy/return_selector.hpp"
#include "carcar_mapping_autonomy/topology_monitor.hpp"
#include "carcar_mapping_autonomy/navigation_speed.hpp"
#include "carcar_mapping_autonomy/costmap_rebuild.hpp"
#include "carcar_mapping_autonomy/map_fingerprint.hpp"
#include "carcar_mapping_autonomy/continuous_exploration.hpp"
#include "carcar_mapping_autonomy/supervisor_output.hpp"
namespace ca=carcar_mapping_autonomy;
using Nav=nav2_msgs::action::NavigateToPose;
using Plan=nav2_msgs::action::ComputePathToPose;
using NavHandle=rclcpp_action::ClientGoalHandle<Nav>;
using PlanHandle=rclcpp_action::ClientGoalHandle<Plan>;
using Proxy=rclcpp_action::ServerGoalHandle<Nav>;
using Clock=std::chrono::steady_clock;
static double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static double yaw(const geometry_msgs::msg::Quaternion&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
static bool finite_twist(const geometry_msgs::msg::Twist&v){return std::isfinite(v.linear.x)&&std::isfinite(v.angular.z)&&std::abs(v.linear.y)<1e-9;}
class Supervisor: public rclcpp::Node {
  struct Operation {
    uint64_t generation=0,id=0,attempt=0,path_revision=0,validated_revision=0;
    bool validation_pending=false,selecting=false,selection_pending=false;
    bool arrival_pending=false,arrival_verified=false;double arrival_started=0;
    std::shared_ptr<const nav_msgs::msg::Path> path;
    geometry_msgs::msg::Point frontier;
    std::string cancel_reason,request_id;
    std::string planner_reason,planner_code;double plan_failure_at=0;
    std::shared_ptr<Proxy> proxy;
    NavHandle::SharedPtr nav;
    PlanHandle::SharedPtr plan;
    geometry_msgs::msg::PoseStamped target;
    bool returning=false,revalidate=false,cancel=false,nav_done=true,plan_done=false,nav_pending=false,plan_pending=true;
    double created=seconds(),began=created,cancel_at=0,remaining=-1,initial_remaining=-1,best_remaining=-1;
  };
public:
  Supervisor():Node("mapping_autonomy"),tf_(get_clock()),listener_(tf_) {
    task_.limits.radius=declare_parameter("radius_m",20.0);
    task_.limits.explore_seconds=declare_parameter("explore_seconds",2400.0);
    task_.limits.return_seconds=declare_parameter("return_seconds",600.0);
    selection_stall_seconds_=declare_parameter("selection_stall_seconds",15.0);
    if(!std::isfinite(selection_stall_seconds_)||selection_stall_seconds_<1||selection_stall_seconds_>60)
      throw std::runtime_error("选点等待上限须为1～60秒");
    if(!task_.limits.valid())throw std::runtime_error("自动建图预算无效");
    requested_auto_=declare_parameter("control_mode",std::string("manual"))=="automatic";
    if(requested_auto_){task_.mode="automatic";task_.reason="等待地图和Nav2全部就绪，尚未自动行驶";}
    task_id_=declare_parameter("task_id",std::string(""));
    if(task_id_.empty()){std::ostringstream s;s<<std::hex<<std::random_device{}()<<std::random_device{}();task_id_=s.str();}
    recovery_enabled_=declare_parameter("recovery_enabled",false);
    online_snapshots_=declare_parameter("online_snapshots",false);
    snapshot_interval_seconds_=declare_parameter("snapshot_interval_seconds",30.0);
    if(!std::isfinite(snapshot_interval_seconds_)||snapshot_interval_seconds_<5||snapshot_interval_seconds_>30)
      throw std::runtime_error("可信快照维护间隔必须在5～30秒内");
    if(online_snapshots_&&!recovery_enabled_)throw std::runtime_error("在线快照要求启用恢复监督");
    if(recovery_enabled_)costmap_rebuild_=std::make_unique<ca::CostmapRebuild>(*this);
    exploration_tree_=declare_parameter("exploration_behavior_tree",
      ament_index_cpp::get_package_share_directory("carcar_mapping_autonomy")+"/behavior_trees/navigate_to_pose_exploration.xml");
    if(declare_parameter("continuous_exploration",false))continuous_=std::make_unique<ca::ContinuousExploration>(*this,
      declare_parameter("continuous_behavior_tree",ament_index_cpp::get_package_share_directory("carcar_mapping_autonomy")+"/behavior_trees/navigate_through_poses_exploration.xml"));
    navigation_speed_=std::make_unique<ca::MappingNavigationSpeed>(*this);
    direction_min_=declare_parameter("direction_ratio_min",.01);
    direction_hold_=declare_parameter("direction_risk_seconds",3.0);
    filtered_scan_=declare_parameter("filtered_scan_topic",std::string("/scan/filtered"));
    nav_=rclcpp_action::create_client<Nav>(this,"/navigate_to_pose");
    planner_=rclcpp_action::create_client<Plan>(this,"/compute_path_to_pose");
    nav2_ready_=create_client<std_srvs::srv::Trigger>("/lifecycle_manager_navigation/is_active");
    state_pub_=create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
    // 输出线程只持有发布者和日志器副本，不读取控制线程的可变状态。
    telemetry_=std::make_unique<ca::SupervisorOutput>(
      [publisher=state_pub_](const std::string&body){std_msgs::msg::String message;message.data=body;publisher->publish(message);},
      [logger=get_logger()](int severity,const std::string&body){
        if(severity>=30)RCLCPP_WARN(logger,"%s",body.c_str());else RCLCPP_INFO(logger,"%s",body.c_str());});
    input_pub_=create_publisher<geometry_msgs::msg::Twist>("/mapping/collision_input",rclcpp::QoS(1));
    output_pub_=create_publisher<geometry_msgs::msg::Twist>("/mapping/cmd_vel",rclcpp::QoS(1));
    explore_pub_=create_publisher<std_msgs::msg::Bool>("/explore/resume",rclcpp::QoS(1).reliable().transient_local());
    candidate_rejections_pub_=create_publisher<std_msgs::msg::String>("/mapping/explore_candidate_rejection",rclcpp::QoS(10));
    navigation_generation_pub_=create_publisher<std_msgs::msg::UInt64>("/mapping/navigation_generation",rclcpp::QoS(1).reliable().transient_local());
    planner_diagnostics_=create_subscription<std_msgs::msg::String>("/mapping/planner_diagnostic",rclcpp::QoS(10),
      [this](const std_msgs::msg::String&m){try{
        auto op=operation_;if(!op||op->cancel||op->nav||op->nav_pending)return;
        const auto j=ca::Json::parse(m.data);
        if(j.value("planner_id",std::string())!=(op->returning?"GridBased":"ExplorationGrid")||
           j.value("goal_stamp_ns",int64_t(0))!=rclcpp::Time(op->target.header.stamp).nanoseconds())return;
        if(!j.contains("target")||j["target"].size()!=2||
           std::abs(j["target"][0].get<double>()-op->target.pose.position.x)>1e-6||
           std::abs(j["target"][1].get<double>()-op->target.pose.position.y)>1e-6)return;
        op->planner_reason=j.value("reason",std::string());op->planner_code=j.value("code",std::string());
      }catch(...){}});
    invalidate_navigation();
    recovery_requests_=create_publisher<std_msgs::msg::String>("/mapping/recovery/request",rclcpp::QoS(10));
    recovery_sub_=create_subscription<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{auto j=ca::Json::parse(m.data);
        if(j.value("task_id",std::string())!=task_id_)return;
        const auto checkpoint=j.value("checkpoint_id",std::string());
        if(!checkpoint.empty()&&checkpoint!=last_checkpoint_id_){last_checkpoint_id_=checkpoint;checkpoint_at_=seconds();}
        if(!checkpoint.empty()&&j.contains("checkpoint_age_seconds")){
          const double age=j.value("checkpoint_age_seconds",-1.0);
          checkpoint_at_=std::isfinite(age)&&age>=0?seconds()-age:-1e9;
        }
        recovery_state_=std::move(j);recovery_at_=seconds();
      }catch(...){}});
    mask_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/mapping/known_space_mask",rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){if(ca::grid_valid(*m)){mask_=m;mask_at_=seconds();}});
    mask_status_sub_=create_subscription<std_msgs::msg::String>("/mapping/known_space/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{mask_status_=ca::Json::parse(m.data);}catch(...){}});
    scans_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
      if(!fresh(m->header.stamp)||m->ranges.empty()||m->ranges.size()>100000||m->header.frame_id.empty()||
        !std::isfinite(m->angle_min)||!std::isfinite(m->angle_increment)||m->angle_increment<=0)return;
      scan_at_=seconds();scan_frame_=m->header.frame_id;scan_stamp_=m->header.stamp;scan_=m;
      Eigen::Matrix2d information=Eigen::Matrix2d::Zero();size_t n=0,valid=0;
      for(size_t i=0;i<m->ranges.size();++i){const auto r=m->ranges[i];if(std::isfinite(r)&&r>=.1&&r<=3)++valid;
        if(!i)continue;
        const auto r0=m->ranges[i-1];
        if(!std::isfinite(r)||!std::isfinite(r0)||r<.1||r>3||r0<.1||r0>3||std::abs(r-r0)>.15)continue;
        const auto a=m->angle_min+i*m->angle_increment,a0=a-m->angle_increment;
        Eigen::Vector2d tangent(r*std::cos(a)-r0*std::cos(a0),r*std::sin(a)-r0*std::sin(a0));
        if(tangent.norm()<.003||tangent.norm()>.15)continue;
        Eigen::Vector2d normal(-tangent.y(),tangent.x());normal.normalize();information+=normal*normal.transpose();++n;
      }
      valid_points_=valid;
      if(n>=20){Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> e(information);direction_ratio_=e.eigenvalues()[0]/std::max(1e-9,e.eigenvalues()[1]);}
      else direction_ratio_=0;
    });
    filtered_=create_subscription<sensor_msgs::msg::LaserScan>(filtered_scan_,rclcpp::SensorDataQoS().keep_last(1),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){if(fresh(m->header.stamp)&&!m->ranges.empty())filtered_at_=seconds();});
    odom_=create_subscription<nav_msgs::msg::Odometry>("/wheel/odometry",rclcpp::QoS(1),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if(!fresh(m->header.stamp)||!std::isfinite(m->twist.twist.linear.x)||!std::isfinite(m->twist.twist.linear.y)||!std::isfinite(m->twist.twist.angular.z)||
        !std::isfinite(m->pose.pose.position.x)||!std::isfinite(m->pose.pose.position.y))return;
      // 全向底盘横向仍在移动时，不能把纵向速度为零当作停稳。
      odom_at_=seconds();speed_=std::hypot(m->twist.twist.linear.x,m->twist.twist.linear.y);rotation_speed_=std::abs(m->twist.twist.angular.z);
      wheel_={m->pose.pose.position.x,m->pose.pose.position.y,yaw(m->pose.pose.orientation)};
      // SLAM保留消息可能先于本进程首个编码器样本到达，不能把默认(0,0)当成已测基准。
      if(!accepted_wheel_valid_&&accepted_sequence_!=0){accepted_wheel_=wheel_;accepted_at_=seconds();accepted_wheel_valid_=true;acceptance_motion_at_=-1;}
      if(speed_<.015&&rotation_speed_<.03){if(still_since_<0)still_since_=seconds();}else still_since_=-1;
    });
    grid_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      if(!ca::grid_valid(*m))return;
      if(!map_||!ca::same_grid(*map_,*m)){++map_revision_;map_changed_=true;map_fingerprint_=ca::map_content_fingerprint(*m);}
      map_=m;map_at_=seconds();++map_sequence_;
      if(empty_wait_&&map_sequence_>empty_map_sequence_&&task_.phase=="EXPLORING"){
        empty_wait_=false;empty_map_sequence_=map_sequence_;enable_explore(true);
      }
    });
    slam_=create_subscription<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local(),[this](const std_msgs::msg::String&m){
      try{const auto j=ca::Json::parse(m.data);slam_at_=seconds();
        if(j.value("loop_generation",uint64_t{0})!=loop_generation_){
          loop_generation_=j.value("loop_generation",uint64_t{0});loop_at_=seconds();
          invalidate_navigation();
          accepted_at_=loop_at_;accepted_wheel_=wheel_;accepted_wheel_valid_=seconds()-odom_at_<.5;acceptance_motion_at_=-1;loop_revalidation_=true;loop_waiting_map_=true;loop_map_after_=map_sequence_;pending_pose_jump_at_=-1;
        }
        if(j.value("accepted",false)&&j.value("sequence",uint64_t{0})!=accepted_sequence_){
          accepted_sequence_=j.value("sequence",uint64_t{0});accepted_at_=seconds();accepted_wheel_=wheel_;accepted_wheel_valid_=seconds()-odom_at_<.5;acceptance_motion_at_=-1;
        }
        slam_status_=j;
      }catch(...){}
    });
    pose_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",rclcpp::QoS(1).reliable().transient_local(),[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
      if(m->header.frame_id!="map")return;
      const ca::Pose p{m->pose.pose.position.x,m->pose.pose.position.y,yaw(m->pose.pose.orientation)};
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)){quality_fault_="SLAM位姿无效";return;}
      const auto q=std::max(m->pose.covariance[0],m->pose.covariance[7]);
      if(!std::isfinite(q)||q<0||!std::isfinite(m->pose.covariance[35])){quality_fault_="SLAM协方差无效";return;}
      if(have_pose_&&seconds()-loop_at_>2){
        const double wheel_delta=std::hypot(wheel_.x-last_pose_wheel_.x,wheel_.y-last_pose_wheel_.y);
        if(std::hypot(p.x-last_pose_.x,p.y-last_pose_.y)>wheel_delta+.5||std::abs(std::remainder(p.yaw-last_pose_.yaw,2*M_PI))>std::abs(std::remainder(wheel_.yaw-last_pose_wheel_.yaw,2*M_PI))+.7)
          pending_pose_jump_at_=seconds();
      }
      covariance_=q;last_pose_=p;last_pose_wheel_=wheel_;have_pose_=true;
    });
    ready_=create_subscription<std_msgs::msg::String>("/mapping/session/state",rclcpp::QoS(1),[this](const std_msgs::msg::String&m){try{auto j=ca::Json::parse(m.data);session_ready_=j.value("ready",false);const auto storage=j.value("storage",ca::Json::object());storage_reason_=storage.value("start_allowed",true)?"":storage.value("reason",std::string("磁盘空间检查未通过"));session_at_=seconds();}catch(...){session_ready_=false;}});
    manual_=create_subscription<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist&m){if(finite_twist(m)){manual_velocity_=m;manual_at_=seconds();}});
    automatic_=create_subscription<geometry_msgs::msg::Twist>("/mapping/auto_cmd_vel",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist&m){if(finite_twist(m)){auto_velocity_=m;auto_at_=seconds();}});
    collision_=create_subscription<geometry_msgs::msg::Twist>("/mapping/collision_output",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist&m){if(finite_twist(m)){protected_velocity_=m;collision_at_=seconds();}});
    local_pose_=create_subscription<geometry_msgs::msg::PolygonStamped>("/local_costmap/published_footprint",rclcpp::QoS(1),[this](const geometry_msgs::msg::PolygonStamped&m){if(m.header.frame_id=="odom"&&fresh(m.header.stamp)&&m.polygon.points.size()>=3)local_pose_at_=seconds();});
    global_pose_=create_subscription<geometry_msgs::msg::PolygonStamped>("/global_costmap/published_footprint",rclcpp::QoS(1),[this](const geometry_msgs::msg::PolygonStamped&m){if(m.header.frame_id=="map"&&fresh(m.header.stamp)&&m.polygon.points.size()>=3)global_pose_at_=seconds();});
    requests_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/request",rclcpp::QoS(10),[this](const std_msgs::msg::String&m){command(m);});
    explore_status_=create_subscription<explore_lite_msgs::msg::ExploreStatus>("/explore/status",rclcpp::QoS(10).reliable().transient_local(),[this](const explore_lite_msgs::msg::ExploreStatus&m){
      if(continuous_||task_.phase!="EXPLORING")return;
      if(m.status=="exploration_waiting_candidates"){
        exploration_note_="暂时没有适合继续前进的目标，保持停车等待新地图或候选冷却；探索尚未完成";return;
      }
      if(m.status=="exploration_incomplete_unreachable")begin_stop("FINISH","部分区域不可达，保留缺口；尝试返回",seconds(),"partial_unreachable");
      else if(m.status=="exploration_complete"){
        if(!map_||!ca::free_footprint(*map_,current_)){begin_stop("FAULT","前沿搜索为空但车位不在可信自由区",seconds());return;}
        if(++empty_count_>=3)begin_stop("FINISH","连续地图更新无可达前沿，当前范围探索结束",seconds(),"complete");
        else{empty_wait_=true;empty_map_sequence_=map_sequence_;}
      }
    });
    proxy_server_=rclcpp_action::create_server<Nav>(this,"/mapping/explore_navigate_to_pose",
      [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Nav::Goal> g){
        if(continuous_||task_.phase!="EXPLORING"||operation_||loop_waiting_map_||checkpoint_requested_||recovery_pending_||
           (recovery_enabled_&&recovery_state_.value("gate_write",false))||!healthy()||g->pose.header.frame_id!="map")return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](std::shared_ptr<Proxy> h){if(operation_&&operation_->proxy==h){
        if(operation_->cancel_reason.empty())operation_->cancel_reason="探索Action客户端请求取消；进展超时或暂停原因见同goal_id探索日志";
        cancel(operation_);
      }return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Proxy> h){
        exploration_note_.clear();
        auto op=std::make_shared<Operation>();op->proxy=h;op->generation=task_.generation;op->id=++operation_sequence_;op->request_id=active_request_id_;
        auto & p=h->get_goal()->pose.pose.position;
        op->frontier=p;
        op->selecting=true;op->plan_done=true;op->plan_pending=false;
        operation_=op;select_observation(op);
      });
    topology_=std::make_unique<ca::TopologyMonitor>([this]{
      const auto pubs=get_publishers_info_by_topic("/mapping/cmd_vel");
      return pubs.size()==1&&pubs.front().node_name()==get_name()&&get_publishers_info_by_topic("/scan").size()==1&&
        get_publishers_info_by_topic("/wheel/odometry").size()==1&&get_publishers_info_by_topic("/mapping/collision_output").size()==1;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    enable_explore(false);publish_state();
  }
  ~Supervisor()override{geometry_msgs::msg::Twist z;for(int i=0;i<3;++i){try{input_pub_->publish(z);output_pub_->publish(z);}catch(...){}}telemetry_.reset();topology_.reset();}
private:
  template<class...Args>void queue_log(int severity,const char*format,Args...args){
    const int length=std::snprintf(nullptr,0,format,args...);
    if(length<0){telemetry_->event(30,"监督器事件格式化失败；原任务与停车状态保留");return;}
    std::string message(static_cast<size_t>(length)+1,'\0');std::snprintf(message.data(),message.size(),format,args...);
    message.resize(length);telemetry_->event(severity,std::move(message));
  }
  bool fresh(const builtin_interfaces::msg::Time&s)const{const double a=(now()-rclcpp::Time(s)).seconds();return std::isfinite(a)&&a>=-.1&&a<=.5;}
  bool still()const{return seconds()-odom_at_<.5&&still_since_>=0&&seconds()-still_since_>=.3;}
  bool observation_healthy(bool checking_session=false){
    const double t=seconds();
    auto reject=[this](const std::string&why){health_reason_=why;return false;};
    if(recovery_enabled_&&(t-recovery_at_>1||!recovery_state_.value("imu_ready",false)||
       !recovery_state_.value("imu_valid",true)||recovery_state_.value("imu_age_seconds",0.0)>.5))
      return reject("受扰观察尚未就绪或心跳过期；检查原始IMU与恢复节点");
    if(t-scan_at_>.5)return reject("原始 /scan 超过0.5秒未更新");
    if(t-filtered_at_>.5)return reject("过滤扫描超过0.5秒未更新");
    if(t-odom_at_>.5)return reject("轮式里程计超过0.5秒未更新");
    if(t-slam_at_>1)return reject("SLAM诊断心跳超过1秒未更新");
    if(!storage_reason_.empty())return reject(storage_reason_);
    if(t-session_at_>.5||(!session_ready_&&!checking_session))return reject("建图会话未就绪；检查首帧确认、保存或故障状态");
    if(valid_points_<12)return reject("3米内有效激光测量不足12点");
    try{
      const auto odom_base=tf_.lookupTransform("odom","base_footprint",tf2::TimePointZero);
      if(!fresh(odom_base.header.stamp))return reject("轮式坐标变换陈旧，等待底盘观测恢复");
      tf_.lookupTransform("base_footprint",scan_frame_,rclcpp::Time(scan_stamp_),rclcpp::Duration::from_seconds(0));
    }catch(const tf2::TransformException&e){return reject(std::string("恢复观测坐标变换不可用：")+e.what());}
    health_reason_.clear();return true;
  }
  bool healthy(bool ignore_quality=false,bool checking_session=false){
    const double t=seconds();
    auto reject=[this](const std::string&why){health_reason_=why;return false;};
    if(!observation_healthy(checking_session))return false;
    if(!ignore_quality&&!quality_fault_.empty())return reject(quality_fault_);
    if(!ignore_quality&&recovery_enabled_&&(recovery_state_.value("gate_write",false)||recovery_state_.value("requires_stop",false)))
      return reject("地图写入已冻结，位置尚待验证；保持停车，可在暂停后重新检查");
    if(t-map_at_>10||!map_)return reject("地图缺失或超过10秒未更新");
    if(!have_pose_)return reject("尚无有效SLAM位姿");
    if(!nav2_active_||t-nav2_ready_at_>2)return reject("Nav2生命周期组合尚未全部激活或健康应答过期");
    if(t-local_pose_at_>1||t-global_pose_at_>1)return reject("Nav2局部／全局代价图使用的车位陈旧；检查内部TF与代价图线程");
    try{
      const auto map_odom=tf_.lookupTransform("map","odom",tf2::TimePointZero);
      const auto odom_base=tf_.lookupTransform("odom","base_footprint",tf2::TimePointZero);
      map_odom_age_=(now()-rclcpp::Time(map_odom.header.stamp)).seconds();
      odom_base_age_=(now()-rclcpp::Time(odom_base.header.stamp)).seconds();
      // SLAM Toolbox按transform_timeout=0.2s前推map→odom时间戳，这是TF有效期，
      // 不是来自未来的传感器测量；原始scan/odom仍使用各自严格的新鲜度门槛。
      if(!std::isfinite(map_odom_age_)||map_odom_age_<-.25||map_odom_age_>.5||!fresh(odom_base.header.stamp))
        return reject("map→odom 或 odom→base_footprint 的独立变换时间无效／陈旧");
      const auto tr=tf_.lookupTransform("map","base_footprint",tf2::TimePointZero);
      if(!fresh(tr.header.stamp))return reject("map 到 base_footprint 变换时间无效或陈旧");
      current_={tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)};
      tf_.lookupTransform("base_footprint",scan_frame_,rclcpp::Time(scan_stamp_),rclcpp::Duration::from_seconds(0));
    }catch(const tf2::TransformException&e){return reject(std::string("扫描坐标变换不可用：")+e.what());}
    if(!topology_ok_)return reject(topology_result_.ready&&t-topology_result_.completed>1?
      "发布者图查询超过1秒未完成，停止运动；检查DDS发现与调度延迟":
      "速度出口、原始扫描、里程计或近场保护发布者不唯一／缺失");
    health_reason_.clear();return true;
  }
  void enable_explore(bool v){std_msgs::msg::Bool m;m.data=v&&!continuous_;explore_pub_->publish(m);}
  void invalidate_navigation(){if(continuous_)continuous_->stop();std_msgs::msg::UInt64 m;m.data=++navigation_generation_;navigation_generation_pub_->publish(m);}
  std::string recovery_request(const std::string&action){
    const auto id=task_id_+"-recovery-"+std::to_string(++recovery_request_sequence_);
    std_msgs::msg::String m;m.data=ca::Json{{"action",action},{"task_id",task_id_},
      {"generation",task_.generation},{"request_id",id},{"navigation_generation",navigation_generation_},
      {"operator_revalidation",operator_recovery_},{"origin",{task_.origin.x,task_.origin.y,task_.origin.yaw}}}.dump();
    recovery_requests_->publish(m);return id;
  }
  void cancel(const std::shared_ptr<Operation>&op,bool recheck=false){
    if(!op)return;
    if(!recheck)op->revalidate=false;
    if(op->cancel)return;
    op->cancel=true;op->cancel_at=seconds();
    trace(op,recheck?"revalidate_cancel":"cancel",op->cancel_reason.empty()?task_.reason:op->cancel_reason);
    if(op->nav&&!op->nav_done)nav_->async_cancel_goal(op->nav);
    if(op->plan&&!op->plan_done)planner_->async_cancel_goal(op->plan);
  }
  bool terminal()const{return (!continuous_||continuous_->terminal())&&(!operation_||(operation_->nav_done&&operation_->plan_done&&!operation_->nav_pending&&!operation_->plan_pending));}
  void begin_stop(const std::string& target,const std::string&reason,double t,const std::string&outcome=""){
    if(task_.phase=="FAULT")return; // 超时锁停保持锁存，迟到的路径超时不能重新进入STOPPING。
    if(target=="FAULT"){
      remember_fault(reason);
      if(operator_check_.value("phase",std::string())=="CHECKING")operator_check_.update({{"phase","FAILED"},{"reason",reason}});
      operator_recovery_=false;
    }
    if(continuous_)continuous_->stop();
    enable_explore(false);exploration_note_.clear();task_.stop(target,reason,t,outcome);
    if(operation_)operation_->cancel_reason=reason;
    cancel(operation_);requested_auto_=false;last_closed_=t;
  }
  void remember_fault(const std::string&reason){
    if(fault_.value("active",false))return;
    fault_pose_=current_;fault_wheel_=wheel_;
    fault_={{"active",true},{"reason",reason},{"stamp_ns",now().nanoseconds()},
      {"request_id",active_request_id_},{"generation",task_.generation},
      {"transient_scan",reason=="原始 /scan 超过0.5秒未更新"||reason=="过滤扫描超过0.5秒未更新"}};
  }
  bool unchanged_since_fault()const{
    const auto close=[](const ca::Pose&a,const ca::Pose&b){return std::hypot(a.x-b.x,a.y-b.y)<.03&&
      std::abs(std::remainder(a.yaw-b.yaw,2*M_PI))<.05;};
    return close(current_,fault_pose_)&&close(wheel_,fault_wheel_)&&quality_fault_.empty()&&
      (!recovery_enabled_||(recovery_state_.value("imu_stable",false)&&!recovery_state_.value("requires_stop",false)&&!recovery_state_.value("gate_write",false)));
  }
  std::string recheck_block(){
    if(task_.phase!="FAULT"&&task_.phase!="PAUSED")return "当前阶段不需要重新检查";
    if(!terminal()||!still())return "等待导航取消确认与轮式停稳";
    if(!observation_healthy(true))return health_reason_;
    const auto field=current_near_field();
    // 一米内没有回波不等于扫描失效；扫描新鲜度和有效点数由观测健康检查。
    if(field.inside)return "近场仍有障碍，请先移开障碍";
    if(fault_.value("transient_scan",false)&&unchanged_since_fault()&&healthy(true,true))return {};
    if(!recovery_enabled_)return "本入口未启用可信定位恢复，请结束任务后重新定位";
    if(recovery_state_.value("checkpoint_id",std::string()).empty())return "没有可信检查点，请结束任务后重新建图";
    // 人工重新检查可使用旧参考图，但仍须经过文件校验、有限搜索和全部新扫描验证。
    // 自动恢复保留120秒限制；墙钟年龄本身不能代替参考区域是否仍适用的几何验证。
    return {};
  }
  void start_recheck(){
    const auto blocked=recheck_block();if(!blocked.empty())throw std::runtime_error("重新检查未执行："+blocked+"；没有下发目标");
    operator_hold_=true;
    operator_check_={{"phase","CHECKING"},{"request_id",task_.request_id},{"reason","保持停车，检查连续新扫描、TF、净空与定位"}};
    operator_check_started_=seconds();operator_valid_since_=-1;operator_scan_count_=0;operator_scan_stamp_=0;
    requested_auto_=false;task_.save_required=false;task_.resume_phase=task_.return_result=="running"?"RETURNING":"EXPLORING";
    task_.phase="CHECKING";task_.reason="正在重新检查，车辆保持停车";++task_.generation;invalidate_navigation();enable_explore(false);
    if(!(fault_.value("transient_scan",false)&&unchanged_since_fault()&&healthy(true,true))){
      task_.phase="RECOVERING";operator_recovery_=true;recovery_pending_=true;recovery_rollback_=false;
      recovery_sent_=recovery_commit_sent_=false;recovery_started_=seconds();recovery_valid_since_=-1;
      costmap_rebuild_started_=false;recovery_map_sequence_=map_sequence_;
      recovery_request("freeze");
    }
  }
  void checked_pause(){
    task_.phase="PAUSED";task_.reason="检查通过，保持停车；请选择继续建图、返回起点或手动接管";
    task_.save_required=false;task_.stop_target.clear();++task_.generation;fault_["active"]=false;
    operator_check_["phase"]="READY";operator_check_["reason"]=task_.reason;
    operator_check_["generation"]=task_.generation;operator_recovery_=false;last_closed_=seconds();
    error_.clear();motion_note_.clear();enable_explore(false);
  }
  ca::Json available_actions(){
    ca::Json actions=ca::Json::object();const bool ok=healthy();const auto health=health_reason_;
    auto set=[&](const std::string&name,const std::string&reason){actions[name]={{"enabled",reason.empty()},{"reason",reason}};};
    const auto idle_reason=(!terminal()||!still())?"等待导航取消与轮式停稳":!ok?health:automatic_start_block();
    set("resume",task_.phase=="MANUAL"||task_.phase=="PAUSED"?idle_reason:"请先暂停或重新检查");
    set("pause",task_.automatic()?"":"当前没有自动行驶任务");
    const bool handoff=task_.automatic()||task_.phase=="PAUSED";
    set("manual",!handoff?"请先完成重新检查":!ok?health:"");
    const bool returnable=handoff||task_.phase=="MANUAL";
    set("return",task_.phase=="RETURNING"?"返程已在执行，沿用原起点与预算":!returnable?"请先完成重新检查":!task_.anchored?"尚未记录本轮起点":!ok?health:"");
    set("recheck",recheck_block());set("halt","");
    return actions;
  }
  bool checkpoint_usable()const{
    if(recovery_state_.value("checkpoint_id",std::string()).empty())return false;
    return !online_snapshots_||(seconds()-checkpoint_at_>=0&&seconds()-checkpoint_at_<=120);
  }
  std::string automatic_start_block()const{
    if(!checkpoint_usable()&&!slam_status_.value("startup_closed",true))
      return "自动尚未启动：正在原地累计激光观测，已接纳"+std::to_string(accepted_sequence_)+"帧；补观测结束后检查净空与可信地图，没有下发目标";
    if(recovery_enabled_&&(!checkpoint_usable()||
       checkpoint_requested_||recovery_state_.value("gate_write",false)||recovery_state_.value("requires_stop",false)))
      return "自动尚未启动：正在建立首个稳定检查点或验证受扰恢复，保持停车等待车端完成";
    if(!map_||!have_pose_)return "自动尚未启动：等待地图与有效车位；没有下发目标";
    ca::FootprintEvidence evidence;
    if(!ca::free_footprint(*map_,current_,.22,&evidence)){
      if(evidence.invalid)return "自动尚未启动：车端地图或车位数据无效；请检查地图与定位反馈，没有下发目标";
      if(evidence.outside)return "自动尚未启动：整车检查范围超出地图边界；请检查地图覆盖，没有下发目标";
      if(evidence.occupied)return "自动尚未启动：地图净空检查发现"+std::to_string(evidence.occupied)+"个占用格；请对照实物核对地图障碍，没有下发目标";
      return "自动尚未启动：当前车位整车范围内还有"+std::to_string(evidence.unknown)+"个未知格（未确认空闲），并非检测到障碍；请检查静止扫描覆盖，没有下发目标";
    }
    return {};
  }
  void start_automatic(double t){
    if(recovery_enabled_&&(!checkpoint_usable()||
       recovery_state_.value("gate_write",false)||recovery_state_.value("requires_stop",false)))
      throw std::runtime_error("自动尚未启动：正在建立首个稳定检查点或处理受扰；保持停车等待车端验证，没有下发目标");
    if(!healthy())throw std::runtime_error("自动尚未启动：车端健康检查拒绝（"+health_reason_+"）；没有下发目标");
    if(!still())throw std::runtime_error("自动尚未启动：尚未确认轮式停稳；请先核对实物停车；没有下发目标");
    const auto blocked=automatic_start_block();
    if(!blocked.empty())throw std::runtime_error(blocked);
    task_.start(t,current_,true,true);operator_hold_=false;enable_explore(task_.phase=="EXPLORING");empty_count_=0;empty_wait_=false;motion_note_.clear();navigation_detail_.clear();exploration_note_.clear();
    selection_wait_at_=0;selection_wait_paused_=false;
  }
  void command(const std_msgs::msg::String&m){
    std::string id;
    try{
      const auto j=ca::Json::parse(m.data);id=j.at("request_id").get<std::string>();
      if(j.value("task_id","")!=task_id_||id.empty()||id.size()>128)throw std::runtime_error("任务编号不匹配，没有下发目标");
      if(std::find(seen_.begin(),seen_.end(),id)!=seen_.end())return;
      const auto action=j.at("action").get<std::string>();
      if(action=="set_speed"){
        if(!nav2_active_)navigation_speed_->reject(id,"导航控制器尚未激活，未提交调速；等待Nav2就绪");
        else if(task_.phase!="EXPLORING"&&task_.phase!="RETURNING"&&task_.phase!="MANUAL"&&task_.phase!="PAUSED")
          navigation_speed_->reject(id,"当前正在停车、恢复或保存，未提交调速；等待阶段完成");
        else if(j.contains("speed_mps")&&j["speed_mps"].is_number())navigation_speed_->select(j["speed_mps"].get<double>(),seconds(),id);
        else navigation_speed_->reject(id,"speed_mps必须为0.05～0.18米/秒的数值");
        seen_.push_back(id);while(seen_.size()>128)seen_.pop_front();publish_state();return;
      }
      task_.request_id=id;error_.clear();
      if(task_.phase=="FAULT"&&action!="save_done"&&action!="recheck"&&action!="halt")throw std::runtime_error("故障锁停：请先重新检查，验证通过后选择后续操作；没有下发目标");
      if(action=="recheck")start_recheck();
      else if(action=="start"||action=="resume"){
        start_automatic(seconds());
      }else if(action=="pause"){
        if(!task_.automatic())throw std::runtime_error("当前阶段不在自动运动，没有重新启动任务");
        operator_hold_=true;
        begin_stop("PAUSED","操作者暂停自动建图",seconds());
      }else if(action=="manual"||action=="finish"||action=="return"){
        if(!task_.automatic()&&task_.phase!="PAUSED"&&!(action!="manual"&&task_.phase=="MANUAL"))throw std::runtime_error("当前阶段不允许接管或返程，没有下发新目标");
        if(!healthy())throw std::runtime_error("操作未执行："+health_reason_+"；请先重新检查");
        if(action!="manual"&&!task_.anchored)throw std::runtime_error("尚未记录本轮起点，未下发返程目标");
        if(action=="manual"){operator_hold_=true;begin_stop("MANUAL","请求手动接管，等待取消与停稳",seconds());}
        else {
          if(task_.phase=="RETURNING"||(task_.phase=="STOPPING"&&task_.stop_target=="FINISH"))
            throw std::runtime_error("返程已在执行，未重置起点、候选次数或返程预算");
          operator_hold_=false;begin_stop("FINISH","操作者结束探索，尝试返回起点",seconds());
        }
      }
      else if(action=="save_done")task_.save_done();
      else if(action=="halt"){
        operator_hold_=true;
        if(operator_check_.value("phase",std::string())=="CHECKING")operator_check_["phase"]="CANCELLED";
        operator_recovery_=false;
        if(recovery_pending_){recovery_request("cancel");recovery_pending_=false;begin_stop("FAULT","操作者中止恢复，保持冻结并保存证据",seconds());}
        else if(task_.phase=="CHECKING")begin_stop("FAULT","操作者中止重新检查，保持停车",seconds());
        else if(task_.phase!="REVIEW_REQUIRED"&&task_.phase!="SAVE_PENDING")begin_stop("PAUSED","停车请求，取消当前动作",seconds());
      }
      else throw std::runtime_error("不支持的建图操作，没有下发目标");
      seen_.push_back(id);while(seen_.size()>128)seen_.pop_front();
      active_request_id_=id;request_result_="accepted";
      queue_log(20,"建图请求 request_id=%s action=%s result=accepted phase=%s；以终态与停稳为准",id.c_str(),action.c_str(),task_.phase.c_str());
    }catch(const std::exception&e){error_=e.what();request_result_="rejected";task_.request_id=id;queue_log(30,"建图请求 request_id=%s result=rejected reason=%s；没有因该请求下发新目标",id.c_str(),error_.c_str());}
    remember_request_result(id);
    publish_state();
  }
  void remember_request_result(const std::string&id){
    if(id.empty())return;
    // 连续请求不能覆盖另一端的确认，同一请求后续拒绝也不能被初次接收结果覆盖。
    request_results_.erase(std::remove_if(request_results_.begin(),request_results_.end(),[&](const auto&r){return r.value("request_id",std::string())==id;}),request_results_.end());
    request_results_.push_back({{"request_id",id},{"request_result",request_result_},{"reason",error_.empty()?task_.reason:error_}});
    while(request_results_.size()>128)request_results_.pop_front();
  }
  void trace(const std::shared_ptr<Operation>&op,const std::string&event,const std::string&reason){
    last_goal_reason_=event+"："+reason;
    const auto target=op->selecting?std::string("null"):ca::Json{op->target.pose.position.x,op->target.pose.position.y}.dump();
    queue_log(20,"探索目标 task_id=%s request_id=%s control_request_id=%s operation=%lu goal_id=%s attempt=%lu map_revision=%lu event=%s elapsed=%.3f goal_seconds=%.3f remaining_m=%.3f progress_m=%.3f target=%s frontier=[%.3f,%.3f] reason=%s",
      task_id_.c_str(),op->request_id.c_str(),active_request_id_.c_str(),op->id,op->proxy?rclcpp_action::to_string(op->proxy->get_goal_id()).c_str():"return",
      op->attempt,map_revision_,event.c_str(),seconds()-op->began,seconds()-op->created,op->remaining,
      op->initial_remaining<0?0:std::max(0.,op->initial_remaining-op->best_remaining),
      target.c_str(),op->frontier.x,op->frontier.y,reason.c_str());
  }
  void select_observation(const std::shared_ptr<Operation>&op){
    ca::ObservationSelector::Job job;job.operation=op->id;job.revision=map_revision_;job.map=map_;
    job.current=current_;job.frontier={op->frontier.x,op->frontier.y,0};job.task=task_;
    op->selection_pending=true;selector_.submit(std::move(job));
  }
  void collect_observation(){
    auto result=selector_.take();if(!result||!operation_)return;
    auto op=operation_;const auto&j=result->job;
    if(j.operation!=op->id||!op->selecting)return;
    op->selection_pending=false;
    if(op->cancel||task_.phase!="EXPLORING"||op->generation!=task_.generation)return;
    if(loop_waiting_map_)return;
    if(j.revision!=map_revision_||!map_||
       ca::ObservationSelector::Search::cell(*map_,current_)!=ca::ObservationSelector::Search::cell(*map_,j.current)){
      select_observation(op);return;
    }
    const auto&target=result->target;
    if(!target||std::hypot(target->x-current_.x,target->y-current_.y)<=.25){
      const std::string reason=target?"观测点距车不超过0.25米，未下发重复近点":
        result->reason+"；请核对地图通道、未知区及规划禁行掩膜";
      complete(op,false,reason);return;
    }
    op->selecting=false;op->target.header.frame_id="map";op->target.header.stamp=now();
    op->target.pose.position.x=target->x;op->target.pose.position.y=target->y;
    const double approach=std::atan2(target->y-current_.y,target->x-current_.x);
    op->target.pose.orientation.z=std::sin(approach/2);op->target.pose.orientation.w=std::cos(approach/2);
    empty_count_=0;plan(op);
  }
  void validate(const std::shared_ptr<Operation>&op){
    if(op->cancel||op->arrival_pending||!op->path||!map_)return;
    ca::RouteValidator::Job job;job.operation=op->id;job.attempt=op->attempt;
    job.revision=map_revision_;job.path_revision=op->path_revision;job.map=map_;job.path=op->path;
    job.task=task_;job.current=current_;job.target={op->target.pose.position.x,op->target.pose.position.y,0};
    job.remaining=op->nav&&!op->nav_done;op->validation_pending=true;validator_.submit(std::move(job));
  }
  void revalidate(const std::shared_ptr<Operation>&op,const std::string&reason){
    if(op->cancel)return;
    op->revalidate=true;op->cancel_reason=reason;last_closed_=seconds();cancel(op,true);
  }
  void collect_validation(){
    auto result=validator_.take();if(!result||!operation_)return;
    auto op=operation_;const auto&j=result->job;
    if(j.operation!=op->id||j.attempt!=op->attempt||j.path_revision!=op->path_revision||op->arrival_pending)return;
    op->validation_pending=false;
    if(loop_waiting_map_){revalidate(op,"SLAM闭环修正，等待闭环结束后的新地图");return;}
    if(op->cancel||!task_.automatic()||op->generation!=task_.generation)return;
    if(j.revision!=map_revision_){validate(op);return;}
    if(!result->safe){
      if(op->nav&&!op->nav_done)revalidate(op,"最新地图中的目标或剩余路径净空失效");
      else complete(op,false,"目标或路径不可达／净空不足，未下发运动目标："+result->reason);
      return;
    }
    op->validated_revision=map_revision_;
    if(op->nav&&!op->nav_done)trace(op,"keep_goal","地图更新后固定目标及剩余路径净空有效");
    else dispatch(op);
  }
  void plan(const std::shared_ptr<Operation>&op){
    ++op->attempt;op->began=seconds();op->plan_done=false;op->plan_pending=true;
    op->remaining=op->initial_remaining=op->best_remaining=-1;
    op->path.reset();op->validation_pending=false;
    // 每次预检使用独立时间戳，诊断只能匹配本次目标，不能套用上一轮或BT重规划的失败。
    op->target.header.stamp=now();op->planner_reason.clear();op->planner_code.clear();op->plan_failure_at=0;
    const auto attempt=op->attempt;
    trace(op,"plan",op->returning?"预检返程目标":"预检固定观测目标");
    if(op->returning)return_destination_["phase"]="PLANNING";
    if(!planner_->action_server_is_ready()||!nav_->action_server_is_ready()){op->plan_done=true;op->plan_pending=false;complete(op,false,"导航规划接口未就绪");return;}
    Plan::Goal goal;goal.goal=op->target;goal.use_start=false;goal.planner_id=op->returning?"GridBased":"ExplorationGrid";
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback=[this,op,attempt](PlanHandle::SharedPtr h){
      if(attempt!=op->attempt||operation_!=op){if(h)planner_->async_cancel_goal(h);return;}
      op->plan_pending=false;op->plan=h;
      if(!h){op->plan_done=true;complete(op,false,"路径请求被拒绝");}else if(op->cancel)planner_->async_cancel_goal(h);};
    options.result_callback=[this,op,attempt](const PlanHandle::WrappedResult&r){
      if(attempt!=op->attempt||operation_!=op)return;
      op->plan_done=true;
      if(op->cancel){if(!op->revalidate)complete(op,false,"路径检查已取消；"+op->cancel_reason);return;}
      if(r.code!=rclcpp_action::ResultCode::SUCCEEDED||!r.result||r.result->path.poses.empty()){
        // Humble动作结果没有错误原因；给关联诊断最多150毫秒跨话题到达时间。
        op->plan_failure_at=seconds()+.15;return;}
      op->path=std::make_shared<nav_msgs::msg::Path>(r.result->path);++op->path_revision;validate(op);
    };
    planner_->async_send_goal(goal,options);
  }
  void dispatch(const std::shared_ptr<Operation>&op){
    if(loop_waiting_map_){revalidate(op,"SLAM闭环修正，等待闭环结束后的新地图");return;}
    if(operation_!=op||op->cancel||op->generation!=task_.generation||!task_.automatic()||!healthy()){complete(op,false,"目标已过期或健康失效，没有下发");return;}
    if(!op->returning&&op->path&&op->path->poses.size()>1){
      const auto&end=op->path->poses.back().pose.position;
      for(auto it=op->path->poses.rbegin()+1;it!=op->path->poses.rend();++it){
        const auto&p=it->pose.position;
        if(std::hypot(end.x-p.x,end.y-p.y)<.10)continue;
        const double approach=std::atan2(end.y-p.y,end.x-p.x);
        op->target.pose.orientation.z=std::sin(approach/2);op->target.pose.orientation.w=std::cos(approach/2);break;
      }
    }
    Nav::Goal goal;goal.pose=op->target;if(!op->returning)goal.behavior_tree=exploration_tree_;op->nav_pending=true;op->nav_done=false;
    const auto attempt=op->attempt;
    rclcpp_action::Client<Nav>::SendGoalOptions options;
    options.goal_response_callback=[this,op,attempt](NavHandle::SharedPtr h){
      if(attempt!=op->attempt||operation_!=op){if(h)nav_->async_cancel_goal(h);return;}
      op->nav_pending=false;op->nav=h;
      if(!h){op->nav_done=true;complete(op,false,"导航器拒绝目标");}
      else{navigation_detail_.clear();selection_wait_at_=0;selection_wait_paused_=false;if(op->returning)return_destination_["phase"]="NAVIGATING";trace(op,"dispatch",op->returning?"导航器已接受返程目标":"导航器已接受固定观测目标");if(op->cancel||op->generation!=task_.generation)nav_->async_cancel_goal(h);}
    };
    options.feedback_callback=[this,op,attempt](NavHandle::SharedPtr,const std::shared_ptr<const Nav::Feedback>f){
      if(operation_!=op||attempt!=op->attempt||op->cancel||!std::isfinite(f->distance_remaining)||f->distance_remaining<0)return;
      // Humble切换目标时黑板可能暂留旧路径，反馈会是旧的正距离或零值。
      // 相对固定观测点的直线下界留0.25m容差，容纳终点容差、路径裁剪和TF时差。
      const double straight=std::hypot(current_.x-op->target.pose.position.x,current_.y-op->target.pose.position.y);
      if(f->distance_remaining+.25<straight||(f->distance_remaining<=.15&&straight>.25))return;
      op->remaining=f->distance_remaining;
      if(op->initial_remaining<0)op->initial_remaining=op->best_remaining=op->remaining;
      else op->best_remaining=std::min(op->best_remaining,op->remaining);
      if(op->proxy&&op->proxy->is_active()&&!op->proxy->is_canceling())
        op->proxy->publish_feedback(std::make_shared<Nav::Feedback>(*f));};
    options.result_callback=[this,op,attempt](const NavHandle::WrappedResult&r){
      if(operation_!=op||attempt!=op->attempt)return;
      op->nav_done=true;
      if(op->cancel){if(!op->revalidate)complete(op,false,"导航已取消；"+op->cancel_reason);return;}
      const bool ok=r.code==rclcpp_action::ResultCode::SUCCEEDED;
      complete(op,ok,ok?"已到达导航目标":"导航规划或控制执行中止；检查该目标编号对应的 Nav2 日志");
    };
    nav_->async_send_goal(goal,options);
  }
  void complete(std::shared_ptr<Operation> op,bool success,const std::string&reason){
    if(operation_!=op||!op->nav_done||!op->plan_done||op->nav_pending||op->plan_pending)return;
    // 导航成功只进入核对阶段；不能仅凭动作服务器返回值宣称实际返程完成。
    if(success&&op->returning&&task_.phase=="RETURNING"&&!op->arrival_verified){
      op->arrival_pending=true;op->arrival_started=seconds();last_closed_=seconds();
      return_destination_["phase"]="VERIFYING";
      return_destination_["reason"]="导航器报告到达，正在核对实际地图位置、整车净空与轮式停稳";
      task_.reason=return_destination_["reason"].get<std::string>();trace(op,"verify_arrival",task_.reason);return;
    }
    trace(op,success?"succeeded":(op->cancel?"canceled":"failed"),reason);
    if(op->cancel)navigation_detail_.clear();else navigation_detail_=success?"":reason;
    if(success&&op->proxy&&task_.phase=="EXPLORING")exploration_note_="已到达当前观测点，等待更新地图后选择下一个目标";
    if(op->proxy&&op->proxy->is_active()){
      auto result=std::make_shared<Nav::Result>();
      if(op->proxy->is_canceling())op->proxy->canceled(result);
      else if(success)op->proxy->succeed(result);else{
        if(op->selecting&&!op->cancel){std_msgs::msg::String rejected;rejected.data=rclcpp_action::to_string(op->proxy->get_goal_id());candidate_rejections_pub_->publish(rejected);}
        op->proxy->abort(result);
      }
    }
    operation_.reset();
    if(!success&&!op->cancel&&op->planner_code=="START_BLOCKED"&&task_.automatic()){
      if(op->returning){return_destination_["phase"]="BLOCKED";return_destination_["reason"]=reason;}
      begin_stop("PAUSED",reason+"；当前车位受阻，继续更换目的地不能解除阻塞；请检查车旁障碍与地图，重新检查通过后再继续或返程",seconds());
      return;
    }
    if(!op->returning&&!op->cancel&&task_.phase=="EXPLORING") {
      if(success){selection_wait_at_=0;selection_wait_paused_=false;}
      else if(selection_wait_at_==0)selection_wait_at_=seconds();
    }
    if(success&&!op->returning&&recovery_enabled_&&!online_snapshots_&&!recovery_pending_&&task_.phase=="EXPLORING"&&seconds()-checkpoint_at_>=30){
      checkpoint_hold_started_=seconds();begin_stop("CHECKPOINT","到达观测点，停稳保存近期可信检查点",seconds());
    }
    if(op->returning&&task_.phase=="RETURNING"){
      if(success){
        const bool nearby=return_destination_.value("kind",std::string())=="nearby";
        return_destination_["phase"]="COMPLETED";
        return_destination_["reason"]=nearby?"已在起点附近停稳，位置为地图估计；未到达原起点":"已在原起点容差内停稳，位置为地图估计";
        task_.returned(true,reason,nearby);
      }else if(!op->cancel){
        failed_return_targets_.push_back({op->target.pose.position.x,op->target.pose.position.y,0});
        ++return_retries_;return_retry_at_=seconds()+.35;return_retry_scan_=scan_stamp_;last_closed_=seconds();
        navigation_detail_="返程目标未通过第"+std::to_string(return_retries_)+"次检查："+reason+"；停稳接收新扫描后检查其他附近目标";
        task_.reason=navigation_detail_;return_destination_["reason"]=navigation_detail_;
        return_destination_["phase"]="SELECTING";
        if(return_attempts_>=return_max_candidates_)return_failed("返程目标已达到有界尝试上限；"+reason);
      }
    }
  }
  void return_failed(const std::string&reason){
    return_destination_["phase"]="FAILED";return_destination_["reason"]=reason;
    const auto detail="返程未完成："+reason+"；原起点和附近没有通过全部检查的目标，保持停车保存候选；检查地图与Nav2日志";
    // 候选耗尽不等于底盘已经停稳；失败保存也必须经过原有取消和停车屏障。
    if(!terminal()||!still()){
      task_.return_result="failed";begin_stop("SAVE",detail,seconds());
    }else task_.returned(false,detail);
    queue_log(30,"返程检查 request_id=%s attempts=%zu reason=%s；未因本次检查下发新运动目标",active_request_id_.c_str(),return_attempts_,reason.c_str());
  }
  void select_return(const std::shared_ptr<Operation>&op){
    ca::ReturnSelector::Job job;job.operation=op->id;job.generation=op->generation;
    job.revision=map_revision_;job.request_id=op->request_id;job.map=map_;job.current=current_;job.task=task_;
    job.radius=return_radius_;job.arrival_tolerance=return_arrival_tolerance_;job.max_candidates=return_max_candidates_;
    op->selection_pending=true;return_selector_.submit(std::move(job));
  }
  void collect_return(){
    auto result=return_selector_.take();if(!result||!operation_)return;
    auto op=operation_;const auto&j=result->job;
    if(j.operation!=op->id||j.generation!=op->generation||j.request_id!=op->request_id||!op->returning||!op->selecting)return;
    op->selection_pending=false;
    if(op->cancel||task_.phase!="RETURNING"||op->generation!=task_.generation||loop_waiting_map_)return;
    if(j.revision!=map_revision_||!map_||
       ca::ObservationSelector::Search::cell(*map_,current_)!=ca::ObservationSelector::Search::cell(*map_,j.current)){
      select_return(op);return;
    }
    auto target=std::find_if(result->candidates.begin(),result->candidates.end(),[this](const ca::Pose&p){
      return std::none_of(failed_return_targets_.begin(),failed_return_targets_.end(),[&](const ca::Pose&f){
        return std::hypot(p.x-f.x,p.y-f.y)<.1;
      });
    });
    if(target==result->candidates.end()){
      operation_.reset();return_failed(result->reason+"；没有剩余可尝试的附近目标");return;
    }
    ++return_attempts_;op->selecting=false;op->target.header.frame_id="map";op->target.header.stamp=now();
    op->target.pose.position.x=target->x;op->target.pose.position.y=target->y;
    op->target.pose.orientation.z=std::sin(target->yaw/2);op->target.pose.orientation.w=std::cos(target->yaw/2);
    const double offset=std::hypot(target->x-task_.origin.x,target->y-task_.origin.y);
    return_destination_.update({{"kind",offset<1e-6?"origin":"nearby"},{"phase","PLANNING"},
      {"target",{target->x,target->y,target->yaw}},{"target_offset_m",offset},{"reason",result->reason}});
    trace(op,"return_selected",result->reason);plan(op);
  }
  void verify_return(double t){
    if(!operation_||!operation_->arrival_pending||operation_->cancel||task_.phase!="RETURNING"||operation_->generation!=task_.generation)return;
    auto op=operation_;
    const double target_distance=std::hypot(current_.x-op->target.pose.position.x,current_.y-op->target.pose.position.y);
    const double origin_distance=std::hypot(current_.x-task_.origin.x,current_.y-task_.origin.y);
    const bool at_origin=return_destination_.value("kind",std::string())=="origin";
    const bool ready=healthy()&&!loop_waiting_map_&&pending_pose_jump_at_<0&&terminal()&&still()&&
      t-std::max(still_since_,op->arrival_started)>=.3&&odom_at_>op->arrival_started&&
      local_pose_at_>op->arrival_started&&global_pose_at_>op->arrival_started&&map_&&ca::free_footprint(*map_,current_)&&
      task_.inside(current_)&&target_distance<=return_arrival_tolerance_&&
      origin_distance<=(at_origin?return_arrival_tolerance_:return_radius_)&&
      std::abs(std::remainder(current_.yaw-task_.origin.yaw,2*M_PI))<=.175&&
      std::abs(last_output_.linear.x)+std::abs(last_output_.angular.z)<1e-9;
    if(ready){op->arrival_verified=true;op->arrival_pending=false;complete(op,true,"地图位置、朝向、整车净空和轮式停稳均通过核对");}
    else if(t-op->arrival_started>3){
      op->arrival_pending=false;
      if(!still())begin_stop("FAULT","导航报告成功后未确认停稳，保持锁停并保存证据",t);
      else complete(op,false,"导航报告成功但实际地图位置、朝向、净空或停车核对未通过；没有确认返程成功");
    }
  }
  void tick(){
    if(!rclcpp::ok())return;
    const double t=seconds();
    // 参数服务先于控制器configure出现；生命周期激活后才发首次默认速度事务。
    if(nav2_active_||navigation_speed_->state().value("phase",std::string())!="WAITING")navigation_speed_->tick(t);
    if(costmap_rebuild_)costmap_rebuild_->tick(t);
    if(last_tick_at_>0){tick_gap_ms_=(t-last_tick_at_)*1000;max_tick_gap_ms_=std::max(max_tick_gap_ms_,tick_gap_ms_);}last_tick_at_=t;
    if(nav2_ready_request_>=0&&t-nav2_probe_at_>1){nav2_ready_->remove_pending_request(nav2_ready_request_);nav2_ready_request_=-1;nav2_active_=false;++nav2_probe_generation_;}
    if(nav2_ready_request_<0&&t-nav2_probe_at_>.5&&nav2_ready_->service_is_ready()){
      nav2_probe_at_=t;const auto generation=++nav2_probe_generation_;
      auto future=nav2_ready_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this,generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
        if(generation!=nav2_probe_generation_)return;
        nav2_ready_request_=-1;nav2_active_=f.get()->success;nav2_ready_at_=seconds();
      });nav2_ready_request_=future.request_id;
    }
    // pose与诊断为不同DDS话题，允许闭环事件先后到达；等待期间输出零速度。
    if(pending_pose_jump_at_>=0&&t-pending_pose_jump_at_>.35&&t-loop_at_>2)
      quality_fault_="SLAM出现非闭环大修正，需要核对工程与真实位置";
    topology_result_=topology_->result();
    topology_ok_=topology_result_.ready&&topology_result_.healthy&&t-topology_result_.completed<=1;
    const double accepted_travel=std::hypot(wheel_.x-accepted_wheel_.x,wheel_.y-accepted_wheel_.y);
    const bool admission_expected=accepted_sequence_!=0&&accepted_wheel_valid_&&!slam_status_.value("loop_active",false)&&t-loop_at_>2&&accepted_travel>.15;
    // 等待窗口从真正跨过平移接纳门槛开始，静止和原地调头不预先消耗它。
    if(!admission_expected)acceptance_motion_at_=-1;
    else if(acceptance_motion_at_<0)acceptance_motion_at_=t;
    if(ca::acceptance_stalled(admission_expected,slam_status_.value("loop_active",false),t-loop_at_,
      acceptance_motion_at_<0?0:t-acceptance_motion_at_,accepted_travel))
      quality_fault_="平移超过接纳门槛后持续无有效扫描接纳";
    if(speed_>.02&&direction_ratio_<direction_min_){if(direction_since_<0)direction_since_=t;if(t-direction_since_>direction_hold_)quality_fault_="激光轮廓方向约束不足，可能弱特征；请核对并选择可信工程";}else direction_since_=-1;
    const auto recovery_phase=recovery_state_.value("phase",std::string("WARMING"));
    const auto rejected_loops=slam_status_.value("loop_guard",ca::Json::object()).value("rejections",uint64_t{0});
    if(rejected_loops>seen_loop_rejections_){
      seen_loop_rejections_=rejected_loops;
      if(recovery_enabled_&&task_.phase=="EXPLORING"&&!operator_hold_&&!recovery_pending_&&automatic_recoveries_<2){
        ++automatic_recoveries_;recovery_rollback_=true;recovery_pending_=true;recovery_sent_=recovery_commit_sent_=false;
        recovery_started_=t;recovery_valid_since_=-1;costmap_rebuild_started_=false;recovery_map_sequence_=map_sequence_;
        invalidate_navigation();begin_stop("RECOVER","错误回环已撤销，停车并冻结；多帧地图、定位及导航检查通过后自动继续建图",t);
        recovery_request("freeze");
      }else{
        // 人工保持、返程、次数用尽以及其他事务期间均不自动启动本分支。
        operator_hold_=true;
        if(recovery_pending_){
          recovery_request("cancel");recovery_pending_=false;
          begin_stop("FAULT","恢复期间再次出现被拒绝的回环，原恢复结果作废；保持锁停并保留证据",t);
        }else{
          if(task_.automatic()||task_.phase=="CHECKPOINTING"||task_.phase=="STOPPING")
            begin_stop("PAUSED","回环校正被拒绝且当前不满足自动续建条件；保持停车，可重新检查或保存结束",t);
          if(recovery_enabled_)recovery_request("freeze");
        }
      }
    }
    const bool recoverable_pause=task_.phase=="PAUSED"&&!operator_hold_&&task_.mode=="automatic";
    if(recovery_enabled_&&(task_.automatic()||recoverable_pause)&&!recovery_pending_&&
       (!quality_fault_.empty()||(recovery_state_.value("requires_stop",false)&&recovery_phase!="CHECKPOINTING"))){
      if(automatic_recoveries_>=2||recovery_state_.value("checkpoint_id",std::string()).empty()||t-checkpoint_at_>120){
        operator_hold_=true;
        begin_stop("PAUSED","自动恢复次数已用尽或缺少近期可信检查点；保持停车，可重新检查位置或保存结束",t);
      }else{
        ++automatic_recoveries_;recovery_rollback_=false;
        recovery_pending_=true;recovery_sent_=recovery_commit_sent_=false;recovery_started_=t;recovery_valid_since_=-1;
        costmap_rebuild_started_=false;
        recovery_map_sequence_=map_sequence_;invalidate_navigation();
        begin_stop("RECOVER",quality_fault_.empty()?recovery_state_.value("reason",std::string("检测到车身受扰，暂停入图并验证定位")):quality_fault_,t);
        recovery_request("freeze");
      }
    }
    if(online_snapshots_&&!recovery_pending_){
      const auto maintenance=recovery_state_.value("snapshot",ca::Json::object());
      if(snapshot_pending_&&maintenance.value("request_id","")==snapshot_request_id_&&!maintenance.value("in_flight",true)&&
        (maintenance.value("phase","")=="READY"||maintenance.value("phase","")=="FAILED"||maintenance.value("phase","")=="WAITING_STARTUP")){
        snapshot_pending_=false;
        // 重试间隔从失败反馈开始；慢盘耗时不能吞掉失败后的10秒退避。
        if(maintenance.value("phase","")=="FAILED")snapshot_attempt_at_=t;
        // 两端接收补观测终态有先后；等待首帧保护不是保存失败，2秒后用新请求重新核验。
        if(maintenance.value("phase","")=="WAITING_STARTUP")snapshot_attempt_at_=t-8;
      }
      if(snapshot_pending_&&t-snapshot_attempt_at_>40){snapshot_pending_=false;snapshot_attempt_at_=t;}
      if(task_.automatic()&&!checkpoint_usable())begin_stop("PAUSED","可信快照超过120秒或不可用，进入保护暂停；检查快照维护原因，更新成功后再恢复",t);
      const bool initial_conditions=checkpoint_usable()||(still()&&terminal()&&recovery_state_.value("imu_stable",false));
      const bool startup_ready=checkpoint_usable()||slam_status_.value("startup_closed",true);
      if(!snapshot_pending_&&initial_conditions&&startup_ready&&healthy()&&quality_fault_.empty()&&have_pose_&&accepted_sequence_>=3&&!slam_status_.value("loop_active",false)&&
        (task_.automatic()||task_.phase=="MANUAL"||task_.phase=="PAUSED")&&t-snapshot_attempt_at_>=10&&(!checkpoint_usable()||t-checkpoint_at_>=snapshot_interval_seconds_)){
        snapshot_attempt_at_=t;snapshot_pending_=true;snapshot_request_id_=recovery_request("snapshot");
      }
    }
    if(recovery_enabled_&&!online_snapshots_&&task_.automatic()&&!recovery_pending_&&t-checkpoint_at_>=90){
      checkpoint_hold_started_=t;begin_stop("CHECKPOINT","连续运行达到90秒，停稳更新可信检查点",t);
    }
    if(recovery_enabled_&&!online_snapshots_&&!recovery_pending_&&quality_fault_.empty()&&healthy()&&still()&&terminal()&&have_pose_&&accepted_sequence_>=3&&
       recovery_state_.value("imu_stable",false)&&!slam_status_.value("loop_active",false)&&
       (task_.phase=="MANUAL"||task_.phase=="CHECKPOINTING"||task_.phase=="PAUSED")&&
       !checkpoint_requested_&&!checkpoint_verified_&&t-checkpoint_attempt_at_>=2&&(recovery_state_.value("checkpoint_id",std::string()).empty()||t-checkpoint_at_>=30)){
      checkpoint_requested_=true;checkpoint_attempt_at_=t;checkpoint_request_id_=recovery_request("checkpoint");
    }
    if(checkpoint_requested_&&recovery_state_.value("request_id",std::string())==checkpoint_request_id_&&
       (recovery_phase=="CHECKPOINT_READY"||recovery_phase=="FAILED"||recovery_phase=="WARMING")){
      checkpoint_requested_=false;
      if(task_.phase=="CHECKPOINTING"){
        if(recovery_phase=="CHECKPOINT_READY")checkpoint_verified_=true;
        else if(recovery_phase=="FAILED"){
          task_.phase="PAUSED";
          task_.reason="检查点生成失败，保持停车，准备验证最近可信位置："+recovery_state_.value("reason",std::string());
        }
      }else if(recovery_phase=="FAILED"&&(task_.phase=="PAUSED"||task_.phase=="MANUAL")){
        task_.phase="PAUSED";task_.reason="检查点生成失败，地图写入保持冻结，可重新检查位置："+recovery_state_.value("reason",std::string());
      }
    }
    if(task_.phase=="CHECKPOINTING"&&(t-checkpoint_hold_started_>65||t-recovery_at_>1))
      begin_stop("FAULT","检查点等待超时或恢复节点断联，保持停车保存证据",t);
    if(task_.phase=="CHECKPOINTING"&&(!quality_fault_.empty()||recovery_phase=="DISTURBED")){
      recovery_request("cancel");begin_stop("FAULT","检查点期间检测到受扰或定位异常，未允许续跑；保留旧可信图和当前证据",t);
    }
    if(recovery_pending_&&task_.phase=="RECOVERING"){
      if(!recovery_sent_){recovery_sent_=true;recovery_request(recovery_rollback_?"recover_rollback":"recover");}
      const bool matching=recovery_state_.value("generation",uint64_t{0})==task_.generation;
      if(t-recovery_started_>65||t-recovery_at_>1||(matching&&recovery_phase=="FAILED")){
        recovery_request("cancel");recovery_pending_=false;
        begin_stop("FAULT","受扰恢复未通过："+recovery_state_.value("reason",std::string("恢复超时或心跳陈旧"))+"；保持冻结，保存可信图和故障证据",t);
      }else if(matching&&recovery_phase=="VERIFY_READY"){
        // 清质量字符串不是恢复依据：仍在RECOVERING零输出，独立检查新图、TF及代价图车位。
        if(!costmap_rebuild_started_){costmap_rebuild_started_=true;costmap_rebuild_->begin(t);}
        if(costmap_rebuild_->failed()){
          recovery_request("cancel");recovery_pending_=false;
          begin_stop("FAULT","定位已验证但导航代价图重建失败；保持停车保存证据："+costmap_rebuild_->state().value("error",std::string()),t);
        }
        const auto global=costmap_rebuild_->global_costmap();
        const bool aligned=global&&map_&&global->header.frame_id==map_->header.frame_id&&
          global->metadata.size_x==map_->info.width&&global->metadata.size_y==map_->info.height&&
          std::abs(global->metadata.resolution-map_->info.resolution)<1e-6&&global->metadata.origin==map_->info.origin;
        const bool mask_ready=mask_&&map_&&mask_at_>=recovery_started_&&ca::same_map_geometry(*mask_,*map_)&&
          mask_status_.value("source_fingerprint",std::string())==std::to_string(map_fingerprint_)&&
          mask_status_.value("navigation_generation",uint64_t{0})==navigation_generation_;
        const auto accepted_stamp=slam_status_.value("accepted_stamp_ns",int64_t{0});
        const bool ready=recovery_state_.value("commit_ready",false)&&costmap_rebuild_->ready()&&aligned&&mask_ready&&map_sequence_>recovery_map_sequence_&&healthy(true,operator_recovery_)&&still()&&terminal()&&
          map_&&accepted_stamp>0&&rclcpp::Time(map_->header.stamp).nanoseconds()>=accepted_stamp&&
          ca::free_footprint(*map_,current_)&&t-local_pose_at_<.5&&t-global_pose_at_<.5;
        if(!ready)recovery_valid_since_=-1;
        else if(recovery_valid_since_<0)recovery_valid_since_=t;
        else if(t-recovery_valid_since_>=1&&!recovery_commit_sent_){recovery_commit_sent_=true;recovery_request("commit_verified");}
      }else if(matching&&recovery_phase=="SUCCEEDED"&&recovery_commit_sent_){
        quality_fault_.clear();pending_pose_jump_at_=-1;direction_since_=-1;
        acceptance_motion_at_=-1;accepted_wheel_=wheel_;accepted_at_=t;accepted_wheel_valid_=true;
        recovery_pending_=false;recovery_sent_=false;loop_waiting_map_=false;loop_revalidation_=false;
        if(operator_recovery_)checked_pause();else task_.recovered();
        selection_wait_at_=0;selection_wait_paused_=false;empty_wait_=false;empty_count_=0;
        invalidate_navigation();last_closed_=t;return_retry_at_=0;
        enable_explore(task_.phase=="EXPLORING");
      }
    }
    if(task_.phase=="CHECKING"){
      const auto field=current_near_field();
      const bool ready=healthy(false,true)&&still()&&terminal()&&unchanged_since_fault()&&
        map_&&ca::free_footprint(*map_,current_)&&!field.inside;
      if(!ready){operator_valid_since_=-1;operator_scan_count_=0;
        operator_check_["reason"]=health_reason_.empty()?"等待稳定车位、连续停稳及整车净空；未恢复运动":health_reason_;}
      else{
        if(operator_valid_since_<0)operator_valid_since_=t;
        const auto stamp=rclcpp::Time(scan_stamp_).nanoseconds();
        if(stamp!=operator_scan_stamp_){operator_scan_stamp_=stamp;++operator_scan_count_;}
        if(t-operator_valid_since_>=1&&operator_scan_count_>=5)checked_pause();
      }
      if(task_.phase=="CHECKING"&&t-operator_check_started_>10){
        operator_check_["phase"]="FAILED";begin_stop("FAULT","重新检查未通过："+operator_check_.value("reason",std::string("连续健康条件不足")),t);
      }
    }
    const bool ok=healthy();
    if(task_.phase=="CHECKPOINTING"&&checkpoint_verified_&&ok&&still()&&terminal()&&!recovery_state_.value("gate_write",false)){
      checkpoint_verified_=false;task_.checkpointed();enable_explore(task_.phase=="EXPLORING");last_closed_=t;
    }
    if(task_.phase!="CHECKPOINTING")checkpoint_verified_=false;
    if(!ok&&task_.phase=="MANUAL"&&!quality_fault_.empty())begin_stop("FAULT",quality_fault_,t);
    // 人工保持或退出自动恢复时也须关闭入图门，让独立观测继续；不启动恢复或增加重试次数。
    const bool recovery_idle=recovery_phase=="IDLE"||recovery_phase=="WARMING"||
      recovery_phase=="CHECKPOINT_READY"||recovery_phase=="SUCCEEDED";
    const bool gate_closed=recovery_state_.value("gate_write",false);
    // 等待闭门反馈期间只发一次；已闭门或故障消失后复位，避免后续重新开门遗漏冻结。
    if(quality_fault_.empty()||gate_closed)quality_freeze_requested_=false;
    if(recovery_enabled_&&!recovery_pending_&&!quality_fault_.empty()&&recovery_idle&&!gate_closed&&
       !quality_freeze_requested_&&(task_.phase=="PAUSED"||task_.phase=="STOPPING"||task_.phase=="FAULT")){
      recovery_request("freeze");quality_freeze_requested_=true;
    }
    const auto old_phase=task_.phase;
    if(task_.automatic()&&!ok)remember_fault(health_reason_);
    task_.tick(t,current_,ok,health_reason_);
    if(task_.phase=="STOPPING"&&old_phase!="STOPPING"){if(continuous_)continuous_->stop();enable_explore(false);cancel(operation_);last_closed_=t;}
    if(operation_&&operation_->cancel&&terminal()&&!operation_->revalidate)complete(operation_,false,"导航已取消；"+task_.reason);
    if(operation_&&!operation_->cancel&&operation_->plan_failure_at>0&&t>=operation_->plan_failure_at){
      auto op=operation_;op->plan_failure_at=0;
      complete(op,false,op->planner_reason.empty()?"路径规划失败，未下发运动目标；检查 /plan 与规划器日志":
        op->planner_reason+"；未下发运动目标");
    }
    if(operation_&&t-operation_->began>5&&operation_->plan_pending)begin_stop("FAULT","路径请求接受超时，保持锁停",t);
    if(operation_&&t-operation_->began>8&&!operation_->plan_done)begin_stop("FAULT","路径检查超时，保持锁停",t);
    if(operation_&&operation_->selecting&&!operation_->cancel&&t-operation_->created>8)
      begin_stop("FAULT",operation_->returning?"返程候选筛选超过8秒，未下发运动目标，保持锁停；检查地图规模与计算负载":"观测点可达性筛选超过8秒，未下发运动目标，保持锁停；检查地图规模与计算负载",t);
    if(operation_&&operation_->cancel&&t-operation_->cancel_at>5&&task_.phase!="FAULT")
      begin_stop("FAULT","目标重新验证期间取消确认超时，保持锁停",t);
    if(loop_waiting_map_&&!slam_status_.value("loop_active",false)&&map_sequence_>loop_map_after_)loop_waiting_map_=false;
    collect_observation();
    collect_return();
    // 连续候选拒绝不能无限保持“运行中”；暂停沿用取消和轮式停稳屏障，不误报完成。
    if(!continuous_&&task_.phase=="EXPLORING"&&selection_wait_at_>0&&
       t-selection_wait_at_>=selection_stall_seconds_) {
      selection_wait_paused_=true;
      begin_stop("PAUSED","车端连续"+std::to_string(int(selection_stall_seconds_))+
        "秒没有可执行目标，取消当前动作并保持停车；最近失败："+navigation_detail_+
        "；未下发新的行驶目标，请检查地图未知区、车位净空和通道后选择继续或接管",t);
    }
    if(operation_&&operation_->selecting&&!operation_->selection_pending&&!operation_->cancel&&!loop_waiting_map_&&task_.phase=="EXPLORING")select_observation(operation_);
    if(operation_&&operation_->returning&&operation_->selecting&&!operation_->selection_pending&&!operation_->cancel&&!loop_waiting_map_&&task_.phase=="RETURNING")select_return(operation_);
    collect_validation();
    if(loop_revalidation_){
      if(operation_&&!operation_->selecting&&!operation_->arrival_pending&&task_.automatic())revalidate(operation_,"SLAM闭环修正，取消确认后重新预检同一观测目标");
      loop_revalidation_=false;
    }
    if(map_changed_&&task_.automatic()){
      if(operation_&&!operation_->cancel&&!operation_->validation_pending)validate(operation_);
      map_changed_=false;
    }
    if(operation_&&operation_->revalidate&&terminal()&&task_.automatic()&&ok&&still()&&!loop_waiting_map_&&t-last_closed_>.3){
      auto op=operation_;op->cancel=false;op->revalidate=false;op->nav.reset();op->plan.reset();
      op->cancel_reason.clear();trace(op,"revalidate_resume","取消与停稳已确认，保持原探索目标");plan(op);
    }
    if(continuous_){
      ca::ContinuousExploration::Snapshot snapshot;
      snapshot.route.map=map_;snapshot.route.current=current_;snapshot.route.task=task_;snapshot.route.now=t;
      snapshot.route.revision=map_revision_;snapshot.route.generation=task_.generation;
      snapshot.route.navigation_generation=navigation_generation_;snapshot.route.speed=navigation_speed_->limit();
      snapshot.map_sequence=map_sequence_;snapshot.task_id=task_id_;snapshot.request_id=active_request_id_;
      snapshot.enabled=task_.phase=="EXPLORING"&&!checkpoint_requested_&&!recovery_pending_&&!loop_waiting_map_&&
        !(recovery_enabled_&&recovery_state_.value("gate_write",false));
      snapshot.healthy=ok&&pending_pose_jump_at_<0;snapshot.still=still();continuous_->tick(std::move(snapshot));
      if(task_.phase=="EXPLORING"&&!continuous_->fault().empty())begin_stop("FAULT",continuous_->fault(),t);
      else if(task_.phase=="EXPLORING"&&continuous_->complete())begin_stop("FINISH","连续地图更新没有剩余前沿，返回起点",t,"complete");
    }
    if(task_.phase=="STOPPING"){
      task_.stopped(t,terminal(),still(),ok);
      if(terminal())operation_.reset();
    }
    const bool checkpoint_ready=!recovery_enabled_||(checkpoint_usable()&&
      !recovery_state_.value("gate_write",false)&&!recovery_state_.value("requires_stop",false));
    if(requested_auto_&&ok&&still()&&task_.phase=="MANUAL"&&checkpoint_ready){
      try{start_automatic(t);requested_auto_=false;}
      catch(const std::exception&e){error_=e.what();requested_auto_=false;task_.mode="manual";task_.reason=error_;request_result_="rejected";
        remember_request_result(task_.request_id);
        queue_log(30,"自动启动预检拒绝：%s；保持手动待命，没有下发目标",e.what());}
    }
    if(task_.phase=="RETURNING"&&!operation_&&terminal()&&t>=return_retry_at_&&
       (return_retries_==0||rclcpp::Time(scan_stamp_)>rclcpp::Time(return_retry_scan_))){
      if(ok&&still()&&!loop_waiting_map_){
        if(return_attempts_>=return_max_candidates_)return_failed("返程目标已达到有界尝试上限");
        else{
          auto op=std::make_shared<Operation>();op->generation=task_.generation;op->id=++operation_sequence_;
          op->request_id=active_request_id_;op->returning=true;op->selecting=true;op->plan_done=true;op->plan_pending=false;
          return_destination_["phase"]="SELECTING";return_destination_["reason"]="优先检查原起点，再筛选限定范围内的安全可达位置";
          operation_=op;select_return(op);
        }
      }
    }
    geometry_msgs::msg::Twist input;
    const bool write_hold=recovery_enabled_&&(recovery_pending_||checkpoint_requested_||recovery_state_.value("gate_write",false));
    const bool auto_motion=task_.automatic()&&!write_hold&&!loop_waiting_map_&&
      ((operation_&&operation_->nav&&!operation_->nav_done&&!operation_->cancel)||(continuous_&&task_.phase=="EXPLORING"&&continuous_->motion()));
    if(ok&&!write_hold&&pending_pose_jump_at_<0&&t-last_closed_>.3){
      if(task_.phase=="MANUAL"&&!requested_auto_&&t-manual_at_<.3)input=manual_velocity_;
      else if(auto_motion&&t-auto_at_<.3)input=auto_velocity_;
    }
    if(task_.phase!="MANUAL"){input.linear.x=std::clamp(input.linear.x,-.05,navigation_speed_->limit());input.angular.z=std::clamp(input.angular.z,-.36,.36);}
    input_pub_->publish(input);
    geometry_msgs::msg::Twist output;
    if(ok&&!write_hold&&pending_pose_jump_at_<0&&t-last_closed_>.3&&t-collision_at_<.2&&(task_.phase=="MANUAL"||auto_motion)){
      output=protected_velocity_;
      // 近场模块不得放大当前选择的速度；旧输入不会通过零输出屏障。
      // 同向反馈可能晚一拍；减速时裁剪幅值，避免把正常延迟变成非零／零交替。
      if(input.linear.x*output.linear.x<0)output.linear.x=0;
      else output.linear.x=std::copysign(std::min(std::abs(output.linear.x),std::abs(input.linear.x)),input.linear.x);
      if(input.angular.z*output.angular.z<0)output.angular.z=0;
      else output.angular.z=std::copysign(std::min(std::abs(output.angular.z),std::abs(input.angular.z)),input.angular.z);
    }
    output_pub_->publish(output);last_output_=output;
    verify_return(t);
    if(t-last_state_>=.1){publish_state();last_state_=t;}
    max_tick_work_ms_=std::max(max_tick_work_ms_,(seconds()-t)*1000);
  }
  ca::NearField current_near_field()const{
    if(!scan_||seconds()-scan_at_>.5)return {};
    try{const auto transform=tf_.lookupTransform("base_footprint",scan_->header.frame_id,tf2::TimePointZero);
      std::vector<std::pair<double,double>> ranges;ranges.reserve(scan_->ranges.size());
      for(size_t i=0;i<scan_->ranges.size();++i)ranges.emplace_back(scan_->ranges[i],scan_->angle_min+i*scan_->angle_increment);
      return ca::assess_near_field(ranges,transform.transform.translation.x,transform.transform.translation.y,yaw(transform.transform.rotation));
    }catch(const tf2::TransformException&){return {};}
  }
  void publish_state(){auto j=task_.status();j["task_id"]=task_id_;j["healthy"]=healthy();j["quality_fault"]=quality_fault_;
    j["health_reason"]=health_reason_;
    const auto motion_health=health_reason_;
    j["observation_ready"]=observation_healthy(true);
    j["observation_reason"]=health_reason_;
    j["localization_requires_verification"]=!quality_fault_.empty()||
      (recovery_enabled_&&(recovery_state_.value("gate_write",false)||recovery_state_.value("requires_stop",false)));
    health_reason_=motion_health;
    j["fault"]=fault_;j["operator_check"]=operator_check_;j["actions"]=available_actions();
    j["current_pose"]={current_.x,current_.y,current_.yaw};j["speed"]=navigation_speed_->state();
    j["continuous_exploration"]=continuous_?continuous_->status():ca::Json::object();
    j["navigation_generation"]=navigation_generation_;
    j["online_snapshots"]=online_snapshots_;j["snapshot_maintenance"]=recovery_state_.value("snapshot",ca::Json::object());
    j["checkpoint_usable"]=!recovery_enabled_||checkpoint_usable();
    if(continuous_&&online_snapshots_)j["continuous_exploration"]["checkpoint_policy"]="后台维护一致快照；120秒内无有效快照时保护暂停";
    j["recovery"]=recovery_state_;j["recovery_pending"]=recovery_pending_;
    j["recovery_navigation"]=costmap_rebuild_?costmap_rebuild_->state():ca::Json::object();
    j["return_retries"]=return_retries_;
    auto destination=return_destination_;
    destination["distance_to_origin_m"]=task_.anchored&&have_pose_?ca::Json(std::hypot(current_.x-task_.origin.x,current_.y-task_.origin.y)):ca::Json(nullptr);
    destination["attempts"]=return_attempts_;destination["max_candidates"]=return_max_candidates_;
    j["return_destination"]=std::move(destination);
    j["auto_start_pending"]=requested_auto_;
    const auto start_block=automatic_start_block();
    j["automatic_start_ready"]=start_block.empty();j["automatic_start_reason"]=start_block;
    j["acceptance_wheel_ready"]=accepted_wheel_valid_;
    j["wheel_position"]={wheel_.x,wheel_.y};j["accepted_wheel_position"]={accepted_wheel_.x,accepted_wheel_.y};
    j["travel_since_acceptance"]=std::hypot(wheel_.x-accepted_wheel_.x,wheel_.y-accepted_wheel_.y);
    j["seconds_since_acceptance"]=seconds()-accepted_at_;
    j["seconds_waiting_expected_acceptance"]=acceptance_motion_at_<0?0:seconds()-acceptance_motion_at_;
    j["pose_correction_pending"]=pending_pose_jump_at_>=0;
    j["tf_map_odom_age"]=map_odom_age_;j["tf_odom_base_age"]=odom_base_age_;
    j["request_result"]=request_result_;j["request_results"]=request_results_;j["error"]=error_;j["still"]=still();j["navigation_terminal"]=terminal();
    j["control_tick_gap_ms"]=tick_gap_ms_;j["control_max_tick_gap_ms"]=max_tick_gap_ms_;j["control_max_tick_work_ms"]=max_tick_work_ms_;
    j["topology_query_ms"]=topology_result_.duration_ms;j["topology_age_s"]=topology_result_.ready?seconds()-topology_result_.completed:-1;
    j["active_request_id"]=active_request_id_;j["goal_request_id"]=operation_?operation_->request_id:"";
    j["goal_seconds"]=operation_?seconds()-operation_->created:0;
    j["navigation_attempt_seconds"]=operation_?seconds()-operation_->began:0;
    j["navigation_remaining_m"]=operation_?operation_->remaining:-1;
    j["navigation_progress_m"]=operation_&&operation_->initial_remaining>=0?std::max(0.,operation_->initial_remaining-operation_->best_remaining):0;
    j["goal"]=operation_&&!operation_->selecting?ca::Json{operation_->target.pose.position.x,operation_->target.pose.position.y}:ca::Json(nullptr);
    j["goal_selecting"]=operation_&&operation_->selecting;
    j["frontier_goal"]=operation_?ca::Json{operation_->frontier.x,operation_->frontier.y}:ca::Json(nullptr);
    j["observation_goal"]=j["goal"];j["map_revision"]=map_revision_;j["map_sequence"]=map_sequence_;
    j["goal_id"]=operation_&&operation_->proxy?rclcpp_action::to_string(operation_->proxy->get_goal_id()):"";
    j["nav_goal_id"]=operation_&&operation_->nav?rclcpp_action::to_string(operation_->nav->get_goal_id()):"";
    j["navigation_attempt"]=operation_?operation_->attempt:0;j["goal_change_reason"]=last_goal_reason_;
    j["loop_waiting_map"]=loop_waiting_map_;j["goal_revalidating"]=operation_&&operation_->revalidate;j["exploration_note"]=exploration_note_;
    if(continuous_&&task_.phase=="EXPLORING"){
      const auto route=continuous_->status();j["exploration_note"]=route.value("reason","");
      j["goal_selecting"]=route.value("prefetch_pending",false)&&route.at("route").empty();
      if(!route.at("route").empty()){
        j["goal"]=route.at("route").at(0).at("target");j["observation_goal"]=j["goal"];
        j["frontier_goal"]=route.at("route").at(0).at("frontier");
        j["goal_id"]="route-"+std::to_string(route.value("route_id",uint64_t(0)));
      }
    }
    if(task_.return_started==0)return_remaining_=task_.limits.return_seconds;
    else if(task_.return_result=="running")return_remaining_=std::max(0.0,task_.limits.return_seconds-(seconds()-task_.return_started));
    j["return_remaining_seconds"]=return_remaining_;
    j["direction_ratio"]=direction_ratio_;j["covariance_xy"]=covariance_;j["accepted_sequence"]=accepted_sequence_;j["loop_generation"]=loop_generation_;
    j["output"]={last_output_.linear.x,last_output_.angular.z};j["slam"]=slam_status_;j["experimental"]=true;
    const auto field=current_near_field();
    const bool commanded=task_.automatic()&&operation_&&operation_->nav&&!operation_->nav_done&&!operation_->cancel&&
      seconds()-auto_at_<.4&&std::abs(auto_velocity_.linear.x)+std::abs(auto_velocity_.angular.z)>0.01;
    const bool cleared=std::abs(last_output_.linear.x)+std::abs(last_output_.angular.z)<0.005;
    if(const auto note=ca::motion_block_note(commanded,cleared,field);!note.empty()&&motion_note_.empty())motion_note_=note;
    else if(speed_>0.02||rotation_speed_>0.05)motion_note_.clear();
    j["motion_note"]=motion_note_;j["navigation_detail"]=navigation_detail_;
    j["automatic_recovery"]={{"attempts",automatic_recoveries_},{"limit",2},{"operator_hold",operator_hold_}};
    j["exploration_wait"]={{"phase",selection_wait_paused_?"PAUSED":
      selection_wait_at_>0&&task_.phase=="EXPLORING"?"WAITING":"IDLE"},
      {"seconds",selection_wait_at_>0?std::min(selection_stall_seconds_,std::max(0.,seconds()-selection_wait_at_)):0.},
      {"limit_seconds",selection_stall_seconds_}};
    j["near_field"]={{"valid",field.valid},{"inside",field.inside},{"count",field.inside_count},
      {"range",field.range},{"x",field.x},{"y",field.y},{"side",field.side}};
    const auto key=task_.phase+"|"+task_.reason+"|"+quality_fault_+"|"+motion_note_+"|"+navigation_detail_+"|"+exploration_note_;
    const bool changed=key!=last_event_;telemetry_->submit(std::move(j),changed);if(changed)last_event_=key;}
  ca::Json fault_=ca::Json::object(),operator_check_=ca::Json::object();
  ca::Pose fault_pose_,fault_wheel_;
  bool operator_recovery_=false;
  bool operator_hold_=false,quality_freeze_requested_=false;unsigned automatic_recoveries_=0;
  double operator_check_started_=0,operator_valid_since_=-1;
  int operator_scan_count_=0;int64_t operator_scan_stamp_=0;
  ca::Task task_;tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  std::unique_ptr<ca::MappingNavigationSpeed>navigation_speed_;
  std::unique_ptr<ca::CostmapRebuild>costmap_rebuild_;bool costmap_rebuild_started_=false;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask_;ca::Json mask_status_=ca::Json::object();
  uint64_t map_fingerprint_=0;double mask_at_=-1e9;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mask_status_sub_;
  ca::Json recovery_state_=ca::Json::object();bool recovery_enabled_=false,recovery_pending_=false;
  double recovery_at_=-1e9,recovery_started_=0,checkpoint_at_=-1e9,checkpoint_attempt_at_=-1e9,recovery_valid_since_=-1,checkpoint_hold_started_=0;
  bool recovery_sent_=false,recovery_commit_sent_=false,checkpoint_requested_=false,checkpoint_verified_=false;
  bool online_snapshots_=false,snapshot_pending_=false;double snapshot_attempt_at_=-1e9,snapshot_interval_seconds_=30.;std::string snapshot_request_id_;
  std::string exploration_tree_,checkpoint_request_id_,last_checkpoint_id_;uint64_t navigation_generation_=0,recovery_request_sequence_=0,recovery_map_sequence_=0;
  static constexpr double return_radius_=.8,return_arrival_tolerance_=.15;
  static constexpr size_t return_max_candidates_=12;
  ca::ReturnSelector return_selector_;std::vector<ca::Pose>failed_return_targets_;size_t return_attempts_=0;
  ca::Json return_destination_={{"kind","none"},{"phase","IDLE"},{"target",nullptr},
    {"search_radius_m",return_radius_},{"arrival_tolerance_m",return_arrival_tolerance_},
    {"target_offset_m",nullptr},{"distance_to_origin_m",nullptr},{"reason","尚未开始返程"}};
  size_t return_retries_=0;double return_retry_at_=0;builtin_interfaces::msg::Time return_retry_scan_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr navigation_generation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_requests_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_sub_;
  ca::Pose wheel_,accepted_wheel_,last_pose_,last_pose_wheel_,current_;
  ca::Json slam_status_=ca::Json::object();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr nav2_ready_;
  int64_t nav2_ready_request_=-1;uint64_t nav2_probe_generation_=0;
  bool nav2_active_=false;double nav2_ready_at_=-1e9,nav2_probe_at_=0;
  std::string storage_reason_;
  bool requested_auto_=false,session_ready_=false,have_pose_=false,accepted_wheel_valid_=false,topology_ok_=false,map_changed_=false,empty_wait_=false,loop_revalidation_=false,loop_waiting_map_=false;
  bool recovery_rollback_=false;uint64_t seen_loop_rejections_=0;
  std::string task_id_,active_request_id_,error_,request_result_,quality_fault_,health_reason_,scan_frame_,filtered_scan_,last_event_,motion_note_,navigation_detail_,last_goal_reason_,exploration_note_;
  double selection_stall_seconds_=15,selection_wait_at_=0;
  bool selection_wait_paused_=false;
  std::unique_ptr<ca::ContinuousExploration> continuous_;
  ca::RouteValidator validator_;ca::ObservationSelector selector_;uint64_t map_revision_=0,operation_sequence_=0,loop_map_after_=0;
  std::unique_ptr<ca::TopologyMonitor>topology_;ca::TopologyMonitor::Result topology_result_;
  double last_tick_at_=0,tick_gap_ms_=0,max_tick_gap_ms_=0,max_tick_work_ms_=0;
  double scan_at_=-1e9,filtered_at_=-1e9,odom_at_=-1e9,map_at_=-1e9,slam_at_=-1e9,session_at_=-1e9,local_pose_at_=-1e9,global_pose_at_=-1e9;
  double manual_at_=-1e9,auto_at_=-1e9,collision_at_=-1e9,still_since_=-1,accepted_at_=0,acceptance_motion_at_=-1,loop_at_=-1e9,map_odom_age_=0,odom_base_age_=0;
  double last_state_=0,last_closed_=seconds(),direction_ratio_=0,direction_since_=-1,direction_min_=0,direction_hold_=3,covariance_=0,speed_=0,rotation_speed_=0,pending_pose_jump_at_=-1,return_remaining_=600;
  size_t valid_points_=0,empty_count_=0;uint64_t accepted_sequence_=0,loop_generation_=0,map_sequence_=0,empty_map_sequence_=0;
  builtin_interfaces::msg::Time scan_stamp_;
  std::deque<std::string>seen_;
  std::deque<ca::Json>request_results_;
  geometry_msgs::msg::Twist manual_velocity_,auto_velocity_,protected_velocity_,last_output_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;std::shared_ptr<Operation> operation_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;rclcpp_action::Client<Plan>::SharedPtr planner_;rclcpp_action::Server<Nav>::SharedPtr proxy_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
  std::unique_ptr<ca::SupervisorOutput>telemetry_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_rejections_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input_pub_,output_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ready_,requests_,slam_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_diagnostics_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scans_,filtered_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_,automatic_,collision_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr local_pose_,global_pose_;
  rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr explore_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{auto n=std::make_shared<Supervisor>();rclcpp::spin(n);n.reset();rclcpp::shutdown();return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
