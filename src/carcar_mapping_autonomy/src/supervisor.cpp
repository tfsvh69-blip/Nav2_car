#include <chrono>
#include <deque>
#include <memory>
#include <random>
#include <sstream>
#include <Eigen/Eigenvalues>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include "carcar_mapping_autonomy/topology_monitor.hpp"
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
    std::shared_ptr<const nav_msgs::msg::Path> path;
    geometry_msgs::msg::Point frontier;
    std::string cancel_reason,request_id;
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
    if(!task_.limits.valid())throw std::runtime_error("自动建图预算无效");
    requested_auto_=declare_parameter("control_mode",std::string("manual"))=="automatic";
    if(requested_auto_){task_.mode="automatic";task_.reason="等待地图和Nav2全部就绪，尚未自动行驶";}
    task_id_=declare_parameter("task_id",std::string(""));
    if(task_id_.empty()){std::ostringstream s;s<<std::hex<<std::random_device{}()<<std::random_device{}();task_id_=s.str();}
    direction_min_=declare_parameter("direction_ratio_min",.01);
    direction_hold_=declare_parameter("direction_risk_seconds",3.0);
    filtered_scan_=declare_parameter("filtered_scan_topic",std::string("/scan/filtered"));
    nav_=rclcpp_action::create_client<Nav>(this,"/navigate_to_pose");
    planner_=rclcpp_action::create_client<Plan>(this,"/compute_path_to_pose");
    nav2_ready_=create_client<std_srvs::srv::Trigger>("/lifecycle_manager_navigation/is_active");
    state_pub_=create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
    input_pub_=create_publisher<geometry_msgs::msg::Twist>("/mapping/collision_input",rclcpp::QoS(1));
    output_pub_=create_publisher<geometry_msgs::msg::Twist>("/mapping/cmd_vel",rclcpp::QoS(1));
    explore_pub_=create_publisher<std_msgs::msg::Bool>("/explore/resume",rclcpp::QoS(1).reliable().transient_local());
    candidate_rejections_pub_=create_publisher<std_msgs::msg::String>("/mapping/explore_candidate_rejection",rclcpp::QoS(10));
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
      if(!fresh(m->header.stamp)||!std::isfinite(m->twist.twist.linear.x)||!std::isfinite(m->twist.twist.angular.z)||
        !std::isfinite(m->pose.pose.position.x)||!std::isfinite(m->pose.pose.position.y))return;
      odom_at_=seconds();speed_=std::abs(m->twist.twist.linear.x);rotation_speed_=std::abs(m->twist.twist.angular.z);
      wheel_={m->pose.pose.position.x,m->pose.pose.position.y,yaw(m->pose.pose.orientation)};
      // SLAM保留消息可能先于本进程首个编码器样本到达，不能把默认(0,0)当成已测基准。
      if(!accepted_wheel_valid_&&accepted_sequence_!=0){accepted_wheel_=wheel_;accepted_at_=seconds();accepted_wheel_valid_=true;acceptance_motion_at_=-1;}
      if(speed_<.015&&rotation_speed_<.03){if(still_since_<0)still_since_=seconds();}else still_since_=-1;
    });
    grid_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      if(!ca::grid_valid(*m))return;
      if(!map_||!ca::same_grid(*map_,*m)){++map_revision_;map_changed_=true;}
      map_=m;map_at_=seconds();++map_sequence_;
      if(empty_wait_&&map_sequence_>empty_map_sequence_&&task_.phase=="EXPLORING"){
        empty_wait_=false;empty_map_sequence_=map_sequence_;enable_explore(true);
      }
    });
    slam_=create_subscription<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local(),[this](const std_msgs::msg::String&m){
      try{const auto j=ca::Json::parse(m.data);slam_at_=seconds();
        if(j.value("loop_generation",uint64_t{0})!=loop_generation_){
          loop_generation_=j.value("loop_generation",uint64_t{0});loop_at_=seconds();
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
    ready_=create_subscription<std_msgs::msg::String>("/mapping/session/state",rclcpp::QoS(1),[this](const std_msgs::msg::String&m){try{auto j=ca::Json::parse(m.data);session_ready_=j.value("ready",false);session_at_=seconds();}catch(...){session_ready_=false;}});
    manual_=create_subscription<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist&m){if(finite_twist(m)){manual_velocity_=m;manual_at_=seconds();}});
    automatic_=create_subscription<geometry_msgs::msg::Twist>("/mapping/auto_cmd_vel",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist&m){if(finite_twist(m)){auto_velocity_=m;auto_at_=seconds();}});
    collision_=create_subscription<geometry_msgs::msg::Twist>("/mapping/collision_output",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist&m){if(finite_twist(m)){protected_velocity_=m;collision_at_=seconds();}});
    local_pose_=create_subscription<geometry_msgs::msg::PolygonStamped>("/local_costmap/published_footprint",rclcpp::QoS(1),[this](const geometry_msgs::msg::PolygonStamped&m){if(m.header.frame_id=="odom"&&fresh(m.header.stamp)&&m.polygon.points.size()>=3)local_pose_at_=seconds();});
    global_pose_=create_subscription<geometry_msgs::msg::PolygonStamped>("/global_costmap/published_footprint",rclcpp::QoS(1),[this](const geometry_msgs::msg::PolygonStamped&m){if(m.header.frame_id=="map"&&fresh(m.header.stamp)&&m.polygon.points.size()>=3)global_pose_at_=seconds();});
    requests_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/request",rclcpp::QoS(10),[this](const std_msgs::msg::String&m){command(m);});
    explore_status_=create_subscription<explore_lite_msgs::msg::ExploreStatus>("/explore/status",rclcpp::QoS(10).reliable().transient_local(),[this](const explore_lite_msgs::msg::ExploreStatus&m){
      if(task_.phase!="EXPLORING")return;
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
        if(task_.phase!="EXPLORING"||operation_||loop_waiting_map_||!healthy()||g->pose.header.frame_id!="map")return rclcpp_action::GoalResponse::REJECT;
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
  ~Supervisor()override{geometry_msgs::msg::Twist z;for(int i=0;i<3;++i){try{input_pub_->publish(z);output_pub_->publish(z);}catch(...){}}topology_.reset();}
private:
  bool fresh(const builtin_interfaces::msg::Time&s)const{const double a=(now()-rclcpp::Time(s)).seconds();return std::isfinite(a)&&a>=-.1&&a<=.5;}
  bool still()const{return seconds()-odom_at_<.5&&still_since_>=0&&seconds()-still_since_>=.3;}
  bool healthy(){
    const double t=seconds();
    auto reject=[this](const std::string&why){health_reason_=why;return false;};
    if(!quality_fault_.empty())return reject(quality_fault_);
    if(t-scan_at_>.5)return reject("原始 /scan 超过0.5秒未更新");
    if(t-filtered_at_>.5)return reject("过滤扫描超过0.5秒未更新");
    if(t-odom_at_>.5)return reject("轮式里程计超过0.5秒未更新");
    if(t-slam_at_>1)return reject("SLAM诊断心跳超过1秒未更新");
    if(t-map_at_>10||!map_)return reject("地图缺失或超过10秒未更新");
    if(t-session_at_>.5||!session_ready_)return reject("建图会话未就绪；检查首帧确认、保存或故障状态");
    if(!have_pose_)return reject("尚无有效SLAM位姿");
    if(valid_points_<12)return reject("3米内有效激光测量不足12点");
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
  void enable_explore(bool v){std_msgs::msg::Bool m;m.data=v;explore_pub_->publish(m);}
  void cancel(const std::shared_ptr<Operation>&op,bool recheck=false){
    if(!op)return;
    if(!recheck)op->revalidate=false;
    if(op->cancel)return;
    op->cancel=true;op->cancel_at=seconds();
    trace(op,recheck?"revalidate_cancel":"cancel",op->cancel_reason.empty()?task_.reason:op->cancel_reason);
    if(op->nav&&!op->nav_done)nav_->async_cancel_goal(op->nav);
    if(op->plan&&!op->plan_done)planner_->async_cancel_goal(op->plan);
  }
  bool terminal()const{return !operation_||(operation_->nav_done&&operation_->plan_done&&!operation_->nav_pending&&!operation_->plan_pending);}
  void begin_stop(const std::string& target,const std::string&reason,double t,const std::string&outcome=""){
    if(task_.phase=="FAULT")return; // 超时锁停保持锁存，迟到的路径超时不能重新进入STOPPING。
    enable_explore(false);exploration_note_.clear();task_.stop(target,reason,t,outcome);
    if(operation_)operation_->cancel_reason=reason;
    cancel(operation_);requested_auto_=false;last_closed_=t;
  }
  std::string automatic_start_block()const{
    if(!map_||!have_pose_)return "自动尚未启动：等待地图与有效车位；没有下发目标";
    if(!ca::free_footprint(*map_,current_))
      return "自动尚未启动：当前车位周围存在未知、占用或图外栅格，尚不能确认整车净空；请保持停车，核对雷达遮挡、周边障碍与地图；没有下发目标";
    return {};
  }
  void start_automatic(double t){
    if(!healthy())throw std::runtime_error("自动尚未启动：车端健康检查拒绝（"+health_reason_+"）；没有下发目标");
    if(!still())throw std::runtime_error("自动尚未启动：尚未确认轮式停稳；请先核对实物停车；没有下发目标");
    const auto blocked=automatic_start_block();
    if(!blocked.empty())throw std::runtime_error(blocked);
    task_.start(t,current_,true,true);enable_explore(task_.phase=="EXPLORING");empty_count_=0;empty_wait_=false;motion_note_.clear();navigation_detail_.clear();exploration_note_.clear();
  }
  void command(const std_msgs::msg::String&m){
    std::string id;
    try{
      const auto j=ca::Json::parse(m.data);id=j.at("request_id").get<std::string>();
      if(j.value("task_id","")!=task_id_||id.empty()||id.size()>128)throw std::runtime_error("任务编号不匹配，没有下发目标");
      if(std::find(seen_.begin(),seen_.end(),id)!=seen_.end())return;
      const auto action=j.at("action").get<std::string>();task_.request_id=id;error_.clear();
      if(task_.phase=="FAULT"&&action!="save_done")throw std::runtime_error("故障锁停不能原地解锁；请选择可信已保存工程重新定位后恢复");
      if(action=="start"||action=="resume"){
        start_automatic(seconds());
      }else if(action=="pause"){
        if(!task_.automatic())throw std::runtime_error("当前阶段不在自动运动，没有重新启动任务");
        begin_stop("PAUSED","操作者暂停自动建图",seconds());
      }else if(action=="manual"||action=="finish"){
        if(task_.phase!="EXPLORING"&&task_.phase!="PAUSED")throw std::runtime_error("当前阶段不允许接管或结束探索，没有下发新目标");
        if(action=="manual")begin_stop("MANUAL","请求手动接管，等待取消与停稳",seconds());
        else begin_stop("FINISH","操作者结束探索，尝试返回起点",seconds());
      }
      else if(action=="save_done")task_.save_done();
      else if(action=="halt"){
        if(task_.phase!="REVIEW_REQUIRED"&&task_.phase!="SAVE_PENDING")begin_stop("PAUSED","停车请求，取消当前动作",seconds());
      }
      else throw std::runtime_error("不支持的建图操作，没有下发目标");
      seen_.push_back(id);while(seen_.size()>128)seen_.pop_front();
      active_request_id_=id;request_result_="accepted";
      RCLCPP_INFO(get_logger(),"建图请求 request_id=%s action=%s result=accepted phase=%s；以终态与停稳为准",id.c_str(),action.c_str(),task_.phase.c_str());
    }catch(const std::exception&e){error_=e.what();request_result_="rejected";task_.request_id=id;RCLCPP_WARN(get_logger(),"建图请求 request_id=%s result=rejected reason=%s；没有因该请求下发新目标",id.c_str(),error_.c_str());}
    publish_state();
  }
  void trace(const std::shared_ptr<Operation>&op,const std::string&event,const std::string&reason){
    last_goal_reason_=event+"："+reason;
    const auto target=op->selecting?std::string("null"):ca::Json{op->target.pose.position.x,op->target.pose.position.y}.dump();
    RCLCPP_INFO(get_logger(),"探索目标 task_id=%s request_id=%s control_request_id=%s operation=%lu goal_id=%s attempt=%lu map_revision=%lu event=%s elapsed=%.3f goal_seconds=%.3f remaining_m=%.3f progress_m=%.3f target=%s frontier=[%.3f,%.3f] reason=%s",
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
        "前沿附近没有同时满足净空、视线和车位连通性的观测点；未提交路径预检，未下发运动目标；请核对地图通道、未知区及规划禁行掩膜";
      complete(op,false,reason);return;
    }
    op->selecting=false;op->target.header.frame_id="map";op->target.header.stamp=now();
    op->target.pose.position.x=target->x;op->target.pose.position.y=target->y;op->target.pose.orientation.w=1;
    empty_count_=0;plan(op);
  }
  void validate(const std::shared_ptr<Operation>&op){
    if(op->cancel||!op->path||!map_)return;
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
    if(j.operation!=op->id||j.attempt!=op->attempt||j.path_revision!=op->path_revision)return;
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
    const auto attempt=op->attempt;
    trace(op,"plan","预检固定观测目标");
    if(!planner_->action_server_is_ready()||!nav_->action_server_is_ready()){op->plan_done=true;op->plan_pending=false;complete(op,false,"导航规划接口未就绪");return;}
    Plan::Goal goal;goal.goal=op->target;goal.use_start=false;goal.planner_id="GridBased";
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
        complete(op,false,"路径规划失败，未下发运动目标；检查 /plan 与规划器日志");return;}
      op->path=std::make_shared<nav_msgs::msg::Path>(r.result->path);++op->path_revision;validate(op);
    };
    planner_->async_send_goal(goal,options);
  }
  void dispatch(const std::shared_ptr<Operation>&op){
    if(loop_waiting_map_){revalidate(op,"SLAM闭环修正，等待闭环结束后的新地图");return;}
    if(operation_!=op||op->cancel||op->generation!=task_.generation||!task_.automatic()||!healthy()){complete(op,false,"目标已过期或健康失效，没有下发");return;}
    Nav::Goal goal;goal.pose=op->target;op->nav_pending=true;op->nav_done=false;
    const auto attempt=op->attempt;
    rclcpp_action::Client<Nav>::SendGoalOptions options;
    options.goal_response_callback=[this,op,attempt](NavHandle::SharedPtr h){
      if(attempt!=op->attempt||operation_!=op){if(h)nav_->async_cancel_goal(h);return;}
      op->nav_pending=false;op->nav=h;
      if(!h){op->nav_done=true;complete(op,false,"导航器拒绝目标");}
      else{navigation_detail_.clear();trace(op,"dispatch","导航器已接受固定观测目标");if(op->cancel||op->generation!=task_.generation)nav_->async_cancel_goal(h);}
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
    if(op->returning&&task_.phase=="RETURNING")task_.returned(success,success?"已返回":("返程未完成："+reason+"；停车保存候选"));
  }
  void tick(){
    if(!rclcpp::ok())return;
    const double t=seconds();
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
    const bool ok=healthy();
    if(!ok&&task_.phase=="MANUAL"&&!quality_fault_.empty())begin_stop("FAULT",quality_fault_,t);
    const auto old_phase=task_.phase;task_.tick(t,current_,ok);
    if(task_.phase=="STOPPING"&&old_phase!="STOPPING"){enable_explore(false);cancel(operation_);last_closed_=t;}
    if(operation_&&operation_->cancel&&terminal()&&!operation_->revalidate)complete(operation_,false,"导航已取消；"+task_.reason);
    if(operation_&&t-operation_->began>5&&operation_->plan_pending)begin_stop("FAULT","路径请求接受超时，保持锁停",t);
    if(operation_&&t-operation_->began>8&&!operation_->plan_done)begin_stop("FAULT","路径检查超时，保持锁停",t);
    if(operation_&&operation_->selecting&&!operation_->cancel&&t-operation_->created>8)
      begin_stop("FAULT","观测点可达性筛选超过8秒，未下发运动目标，保持锁停；检查地图规模与计算负载",t);
    if(operation_&&operation_->cancel&&t-operation_->cancel_at>5&&task_.phase!="FAULT")
      begin_stop("FAULT","目标重新验证期间取消确认超时，保持锁停",t);
    if(loop_waiting_map_&&!slam_status_.value("loop_active",false)&&map_sequence_>loop_map_after_)loop_waiting_map_=false;
    collect_observation();
    if(operation_&&operation_->selecting&&!operation_->selection_pending&&!operation_->cancel&&!loop_waiting_map_&&task_.phase=="EXPLORING")select_observation(operation_);
    collect_validation();
    if(loop_revalidation_){
      if(operation_&&!operation_->selecting&&task_.automatic())revalidate(operation_,"SLAM闭环修正，取消确认后重新预检同一观测目标");
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
    if(task_.phase=="STOPPING"){
      task_.stopped(t,terminal(),still(),ok);
      if(terminal())operation_.reset();
    }
    if(requested_auto_&&ok&&still()&&task_.phase=="MANUAL"){
      try{start_automatic(t);requested_auto_=false;}
      catch(const std::exception&e){error_=e.what();requested_auto_=false;task_.mode="manual";task_.reason=error_;request_result_="rejected";
        RCLCPP_WARN(get_logger(),"自动启动预检拒绝：%s；保持手动待命，没有下发目标",e.what());}
    }
    if(task_.phase=="RETURNING"&&!operation_){
      if(!map_||!ca::free_footprint(*map_,task_.origin))task_.returned(false,"起点不可达，原地停稳保存候选");
      else{auto op=std::make_shared<Operation>();op->generation=task_.generation;op->id=++operation_sequence_;op->request_id=active_request_id_;op->returning=true;op->target.header.frame_id="map";op->target.header.stamp=now();
        op->target.pose.position.x=task_.origin.x;op->target.pose.position.y=task_.origin.y;op->target.pose.orientation.z=std::sin(task_.origin.yaw/2);op->target.pose.orientation.w=std::cos(task_.origin.yaw/2);operation_=op;plan(op);}
    }
    geometry_msgs::msg::Twist input;
    const bool auto_motion=task_.automatic()&&!loop_waiting_map_&&operation_&&operation_->nav&&!operation_->nav_done&&!operation_->cancel;
    if(ok&&pending_pose_jump_at_<0&&t-last_closed_>.3){
      if(task_.phase=="MANUAL"&&!requested_auto_&&t-manual_at_<.3)input=manual_velocity_;
      else if(auto_motion&&t-auto_at_<.3)input=auto_velocity_;
    }
    if(task_.phase!="MANUAL"){input.linear.x=std::clamp(input.linear.x,-.05,.10);input.angular.z=std::clamp(input.angular.z,-.36,.36);}
    input_pub_->publish(input);
    geometry_msgs::msg::Twist output;
    if(ok&&pending_pose_jump_at_<0&&t-last_closed_>.3&&t-collision_at_<.2&&(task_.phase=="MANUAL"||auto_motion)){
      output=protected_velocity_;
      // 近场模块不得放大当前选择的速度；旧输入不会通过零输出屏障。
      if(input.linear.x*output.linear.x<0||std::abs(output.linear.x)>std::abs(input.linear.x)+1e-6)output.linear.x=0;
      if(input.angular.z*output.angular.z<0||std::abs(output.angular.z)>std::abs(input.angular.z)+1e-6)output.angular.z=0;
    }
    output_pub_->publish(output);last_output_=output;
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
    j["request_result"]=request_result_;j["error"]=error_;j["still"]=still();j["navigation_terminal"]=terminal();
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
    j["near_field"]={{"valid",field.valid},{"inside",field.inside},{"count",field.inside_count},
      {"range",field.range},{"x",field.x},{"y",field.y},{"side",field.side}};
    std_msgs::msg::String m;m.data=j.dump();state_pub_->publish(m);
    const auto key=task_.phase+"|"+task_.reason+"|"+quality_fault_+"|"+motion_note_+"|"+navigation_detail_+"|"+exploration_note_;if(key!=last_event_){RCLCPP_INFO(get_logger(),"自动建图状态 %s",m.data.c_str());last_event_=key;}}
  ca::Task task_;tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  ca::Pose wheel_,accepted_wheel_,last_pose_,last_pose_wheel_,current_;
  ca::Json slam_status_=ca::Json::object();
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr nav2_ready_;
  int64_t nav2_ready_request_=-1;uint64_t nav2_probe_generation_=0;
  bool nav2_active_=false;double nav2_ready_at_=-1e9,nav2_probe_at_=0;
  bool requested_auto_=false,session_ready_=false,have_pose_=false,accepted_wheel_valid_=false,topology_ok_=false,map_changed_=false,empty_wait_=false,loop_revalidation_=false,loop_waiting_map_=false;
  std::string task_id_,active_request_id_,error_,request_result_,quality_fault_,health_reason_,scan_frame_,filtered_scan_,last_event_,motion_note_,navigation_detail_,last_goal_reason_,exploration_note_;
  ca::RouteValidator validator_;ca::ObservationSelector selector_;uint64_t map_revision_=0,operation_sequence_=0,loop_map_after_=0;
  std::unique_ptr<ca::TopologyMonitor>topology_;ca::TopologyMonitor::Result topology_result_;
  double last_tick_at_=0,tick_gap_ms_=0,max_tick_gap_ms_=0,max_tick_work_ms_=0;
  double scan_at_=-1e9,filtered_at_=-1e9,odom_at_=-1e9,map_at_=-1e9,slam_at_=-1e9,session_at_=-1e9,local_pose_at_=-1e9,global_pose_at_=-1e9;
  double manual_at_=-1e9,auto_at_=-1e9,collision_at_=-1e9,still_since_=-1,accepted_at_=0,acceptance_motion_at_=-1,loop_at_=-1e9,map_odom_age_=0,odom_base_age_=0;
  double last_state_=0,last_closed_=seconds(),direction_ratio_=0,direction_since_=-1,direction_min_=0,direction_hold_=3,covariance_=0,speed_=0,rotation_speed_=0,pending_pose_jump_at_=-1,return_remaining_=600;
  size_t valid_points_=0,empty_count_=0;uint64_t accepted_sequence_=0,loop_generation_=0,map_sequence_=0,empty_map_sequence_=0;
  builtin_interfaces::msg::Time scan_stamp_;
  std::deque<std::string>seen_;
  geometry_msgs::msg::Twist manual_velocity_,auto_velocity_,protected_velocity_,last_output_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;std::shared_ptr<Operation> operation_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;rclcpp_action::Client<Plan>::SharedPtr planner_;rclcpp_action::Server<Nav>::SharedPtr proxy_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_rejections_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input_pub_,output_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ready_,requests_,slam_;
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
