// NAV-012 系统性修复：监督状态机、有界 Action、无副作用净空条件。
#include "carcar_navigation/recovery_runtime.hpp"
#include "carcar_navigation/bounded_action.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace carcar_navigation {
namespace {
std::string input_string(BT::TreeNode & node,const std::string & key,const std::string & fallback) {
  std::string result=fallback; node.getInput(key,result); return result;
}
geometry_msgs::msg::PoseStamped final_goal(BT::TreeNode & node) {
  geometry_msgs::msg::PoseStamped goal;
  if (node.getInput("goal",goal)) {return goal;}
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (node.getInput("goals",goals) && !goals.empty()) {return goals.back();}
  return goal;
}
void ensure_blackboard(const BT::NodeConfiguration & config) {
  for (auto pair : {std::pair<std::string,int>{"bt_loop_duration",10},
      {"server_timeout",1000},{"wait_for_service_timeout",1000}}) {
    std::chrono::milliseconds value;
    if (!config.blackboard->get(pair.first,value)) {
      config.blackboard->set(pair.first,std::chrono::milliseconds(pair.second));
    }
  }
}
bool same_path_poses(const nav_msgs::msg::Path & a,const nav_msgs::msg::Path & b) {
  if (a.header.frame_id!=b.header.frame_id || a.poses.size()!=b.poses.size()) {return false;}
  for (size_t i=0;i<a.poses.size();++i) {
    if (a.poses[i].pose!=b.poses[i].pose) {return false;}
  }
  return true;
}

class RecoveryInputsReady : public BT::ConditionNode {
public:
  RecoveryInputsReady(const std::string & n,const BT::NodeConfiguration & c)
  :BT::ConditionNode(n,c),runtime_(RecoveryRuntime::get(c)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override {return runtime_->inputs_ready().ok?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;}
private:
  std::shared_ptr<RecoveryRuntime> runtime_;
};
class ReplanNotRequired : public BT::ConditionNode {
public:
  ReplanNotRequired(const std::string & n,const BT::NodeConfiguration & c)
  :BT::ConditionNode(n,c),runtime_(RecoveryRuntime::get(c)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override {
    return runtime_->replan_required?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS;
  }
private:
  std::shared_ptr<RecoveryRuntime> runtime_;
};
class RearClear : public BT::ConditionNode {
public:
  RearClear(const std::string & name,const BT::NodeConfiguration & config)
  : BT::ConditionNode(name,config),runtime_(RecoveryRuntime::get(config)) {
    cache_=runtime_->sensors(input_string(*this,"scan_topic","/scan"),
      input_string(*this,"costmap_topic","/local_costmap/costmap_raw"),
      input_string(*this,"footprint_topic","/local_costmap/published_footprint"));
    publisher_=runtime_->node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/rear_clear/status",10);
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("scan_topic","/scan",""),
      BT::InputPort<std::string>("costmap_topic","/local_costmap/costmap_raw",""),
      BT::InputPort<std::string>("footprint_topic","/local_costmap/published_footprint",""),
      BT::InputPort<double>("backup_distance",kMaxSingleBackupDistance,""),
      BT::OutputPort<double>("selected_distance"),
      BT::InputPort<double>("max_data_age",0.5,"")};
  }
  BT::NodeStatus tick() override {
    double distance=kMaxSingleBackupDistance,age=0.5; getInput("backup_distance",distance); getInput("max_data_age",age);
    double chosen=0;
    SafetyResult result;
    if (runtime_->backup_cooling()) {
      result={false,"BACKUP_COOLDOWN","倒车冷却中，先跟随已重规划路径或受控转向"};
    } else {
      const auto data=cache_->snapshot();
      const auto selected=select_backup_distance(
        distance,kMaxBackupBudget-runtime_->backup_used,
        [&](double sweep) {return swept_clear(
          data,runtime_->tf,runtime_->node->now(),runtime_->base_frame,-sweep,0,age);});
      result=selected.result;
      if (selected.ok) {
        chosen=selected.selected;
        setOutput("selected_distance",selected.selected);
        result.detail="选定倒车 "+std::to_string(selected.selected)+
          "m，含停车余量扫掠 "+std::to_string(selected.swept_distance)+"m";
      }
    }
    diagnostic_msgs::msg::DiagnosticStatus msg;
    msg.name="RearClear"; msg.hardware_id="carcar_nav_bt_nodes"; msg.level=result.ok?0:1;
    msg.message=result.ok?"通过":"拒绝倒车";
    diagnostic_msgs::msg::KeyValue kv; kv.key="reason_code";kv.value=result.code;msg.values.push_back(kv);
    kv.key="detail";kv.value=result.detail;msg.values.push_back(kv);
    kv.key="requested_distance";kv.value=std::to_string(distance);msg.values.push_back(kv);
    kv.key="selected_distance";kv.value=std::to_string(chosen);msg.values.push_back(kv);
    kv.key="remaining_budget";kv.value=std::to_string(
      std::max(0.0,kMaxBackupBudget-runtime_->backup_used));msg.values.push_back(kv);
    publisher_->publish(msg);
    // 条件检查无额度副作用，反复 tick 不会消耗后退预算。
    return result.ok?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<RecoveryRuntime> runtime_;
  std::shared_ptr<SensorCache> cache_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr publisher_;
};

template<class ActionT>
class SafeActionLeaf : public BT::StatefulActionNode {
public:
  SafeActionLeaf(const std::string & name,const BT::NodeConfiguration & config,
    std::string action,bool motion)
  : BT::StatefulActionNode(name,config),runtime_(RecoveryRuntime::get(config)),
    action_(std::move(action)),motion_(motion) {
    ensure_blackboard(config);
    probe_=rclcpp_action::create_client<ActionT>(runtime_->node,action_,runtime_->group);
  }
  ~SafeActionLeaf() override {
    if (session_ && !session_->state().terminal) {
      session_->cancel(); runtime_->retiring.push_back(session_);
    }
  }
protected:
  virtual bool prepare(typename ActionT::Goal &) = 0;
  virtual bool reserve() {return true;}
  virtual void rejected() {}
  virtual void failed() {}
  virtual void completed(typename ActionT::Result::SharedPtr) {}
  virtual void update() {}
  BT::NodeStatus onStart() override {
    if (runtime_->fault) {return BT::NodeStatus::FAILURE;}
    session_.reset();settling_=TimePoint{}; released_=false; no_motion_=false;
    goal_=typename ActionT::Goal{};
    if (!prepare(goal_)) {return BT::NodeStatus::FAILURE;}
    if (no_motion_) {return BT::NodeStatus::SUCCESS;}
    discovery_deadline_=after(runtime_->response_timeout);
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    if (!session_) {
      if (!probe_->action_server_is_ready()) {
        if (Steady::now()>=discovery_deadline_) {
          runtime_->diagnostic(action_,"SERVER_UNAVAILABLE");return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
      if (motion_ && runtime_->motion && !runtime_->motion->state().terminal) {
        runtime_->fail("OVERLAPPING_MOTION_REJECTED");return BT::NodeStatus::FAILURE;
      }
      try {
        session_=std::make_shared<BoundedAction<ActionT>>(name()+"_client",action_,config(),
          runtime_->response_timeout,runtime_->cancel_timeout);
      } catch (const std::exception & e) {
        runtime_->diagnostic(action_,std::string("SERVER_UNAVAILABLE: ")+e.what());
        return BT::NodeStatus::FAILURE;
      }
      if (!reserve()) {session_.reset();return BT::NodeStatus::FAILURE;}
      if (motion_) {runtime_->motion=session_;++runtime_->action_sequence;}
      session_->send(goal_,result_timeout_);
    }
    session_->poll();
    auto state=session_->state();
    if (state.reason=="CANCEL_UNCONFIRMED") {
      if (motion_) {runtime_->fail(state.reason);}
      runtime_->retiring.push_back(session_);
      return BT::NodeStatus::FAILURE;
    }
    if (!state.terminal) {
      if (!state.cancel_requested && state.accepted) {update();}
      return BT::NodeStatus::RUNNING;
    }
    if (state.rejected && !released_) {rejected();released_=true;}
    if (motion_ && runtime_->supervised) {
      runtime_->permit(false);
      if (settling_==TimePoint{}) {settling_=Steady::now();runtime_->restart_stillness();}
      if (!runtime_->still(runtime_->still_duration)) {
        if (seconds(settling_)>runtime_->settle_timeout) {
          runtime_->fail("STOP_NOT_CONFIRMED");return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    if (motion_ && runtime_->motion==session_) {runtime_->motion.reset();}
    if (state.success && !state.cancel_requested && !state.timed_out) {
      completed(session_->result());return BT::NodeStatus::SUCCESS;
    }
    if (!released_) {failed();released_=true;}
    return BT::NodeStatus::FAILURE;
  }
  void onHalted() override {
    if (motion_) {runtime_->permit(false);}
    if (session_ && !session_->state().terminal) {
      session_->cancel();
      if (!motion_) {runtime_->retiring.push_back(session_);}
    }
  }
  std::shared_ptr<RecoveryRuntime> runtime_;
  std::shared_ptr<BoundedAction<ActionT>> session_;
  typename ActionT::Goal goal_;
  double result_timeout_{0};
  bool no_motion_{false};
private:
  std::string action_;
  bool motion_,released_{false};
  TimePoint discovery_deadline_{},settling_{};
  typename rclcpp_action::Client<ActionT>::SharedPtr probe_;
};

class ControlledSpin : public SafeActionLeaf<nav2_msgs::action::Spin> {
public:
  ControlledSpin(const std::string & n,const BT::NodeConfiguration & c)
  : SafeActionLeaf(n,c,"/spin",true) {
    diagnostic_pub_=runtime_->node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/controlled_spin/status",10);
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::InputPort<double>("max_spin_angle",1.570796,""),BT::InputPort<double>("time_allowance",10.0,"")};
  }
private:
  bool prepare(nav2_msgs::action::Spin::Goal & goal) override {
    if (runtime_->spin_used) {return false;}
    nav_msgs::msg::Path path; geometry_msgs::msg::PoseStamped pose;
    if (!getInput("path",path) || !runtime_->pose(pose,path.header.frame_id)) {return false;}
    auto ref=heading_reference(pose,path,final_goal(*this)); if (!ref.valid) {return false;}
    double max_angle=1.570796,time=10; getInput("max_spin_angle",max_angle);getInput("time_allowance",time);
    if (!std::isfinite(max_angle) || max_angle<=0 || max_angle>1.570797 ||
      !std::isfinite(time) || time<=0 || time>10.0) {return false;}
    std::vector<double> candidates;
    for (double degrees : {15.0,30.0,45.0,60.0,75.0,90.0}) {
      const double angle=degrees*M_PI/180.0;
      if (angle<=max_angle+1e-6) {candidates.push_back(angle);candidates.push_back(-angle);}
    }
    const double path_angle=std::clamp(ref.error,-max_angle,max_angle);
    if (std::abs(path_angle)>=0.10) {candidates.push_back(path_angle);}
    std::sort(candidates.begin(),candidates.end());
    candidates.erase(std::unique(candidates.begin(),candidates.end(),
      [](double a,double b) {return std::abs(a-b)<1e-6;}),candidates.end());
    const auto data=runtime_->sensors()->snapshot();
    SpinCandidateEvaluation best;
    double left_clear=0,right_clear=0;unsigned rejected=0;
    for (double angle : candidates) {
      auto rotation=swept_pose_clear(data,runtime_->tf,runtime_->node->now(),runtime_->base_frame,
        0,0,0,0,0,angle,runtime_->data_age);
      if (!rotation.ok) {++rejected;continue;}
      double forward=0;
      for (double distance=.05;distance<=.350001;distance+=.05) {
        auto travel=swept_pose_clear(data,runtime_->tf,runtime_->node->now(),runtime_->base_frame,
          0,0,angle,distance*std::cos(angle),distance*std::sin(angle),angle,runtime_->data_age);
        if (!travel.ok) {break;}
        forward=distance;
      }
      if (angle>0) {left_clear=std::max(left_clear,forward);}
      else {right_clear=std::max(right_clear,forward);}
      SpinCandidateEvaluation candidate{forward>=.05,angle,forward,
        std::abs(normalize(ref.error-angle))};
      if (better_spin_candidate(candidate,best,runtime_->preferred_spin_direction)) {best=candidate;}
    }
    publish_selection(best,left_clear,right_clear,rejected);
    if (!best.valid) {return false;}
    goal.target_yaw=best.angle;
    runtime_->preferred_spin_direction=best.angle>=0?1:-1;
    goal.time_allowance=rclcpp::Duration::from_seconds(time);result_timeout_=time+1;
    if (std::abs(goal.target_yaw)<0.10) {no_motion_=true;runtime_->spin_used=true;runtime_->escape_stage=2;}
    return true;
  }
  bool reserve() override {runtime_->spin_used=true;runtime_->escape_stage=2;return true;}
  void rejected() override {runtime_->spin_used=false;}
  void publish_selection(const SpinCandidateEvaluation & best,double left,double right,unsigned rejected) {
    diagnostic_msgs::msg::DiagnosticStatus msg;msg.name="ControlledSpin";
    msg.hardware_id="carcar_nav_bt_nodes";msg.level=best.valid?0:1;
    msg.message=best.valid?"已选择双向评估后的恢复转向":"左右均无安全转向后前进候选";
    auto add=[&](const std::string & key,double value) {
      diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=std::to_string(value);msg.values.push_back(kv);
    };
    add("selected_angle",best.angle);add("selected_forward_clearance",best.forward_clearance);
    add("left_forward_clearance",left);add("right_forward_clearance",right);
    add("path_error_after_turn",best.path_error);add("rejected_candidates",rejected);
    diagnostic_pub_->publish(msg);
  }
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_pub_;
};
class SafeBackUp : public SafeActionLeaf<nav2_msgs::action::BackUp> {
public:
  SafeBackUp(const std::string & n,const BT::NodeConfiguration & c):SafeActionLeaf(n,c,"/backup",true) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("backup_dist",kMaxSingleBackupDistance,""),
      BT::InputPort<double>("backup_speed",kMaxBackupSpeed,""),
      BT::InputPort<double>("time_allowance",kMaxBackupTimeAllowance,"")};
  }
private:
  bool prepare(nav2_msgs::action::BackUp::Goal & goal) override {
    double distance=kMaxSingleBackupDistance,speed=kMaxBackupSpeed,time=kMaxBackupTimeAllowance;
    getInput("backup_dist",distance);getInput("backup_speed",speed);getInput("time_allowance",time);
    if (!std::isfinite(distance)||distance<=0||distance>kMaxSingleBackupDistance+1e-6||
      !std::isfinite(speed)||speed<=0||speed>kMaxBackupSpeed+1e-6||
      !std::isfinite(time)||time<=0||time>kMaxBackupTimeAllowance+1e-6) {return false;}
    if (runtime_->backup_cooling()) {runtime_->diagnostic("BACKUP","BACKUP_COOLDOWN");return false;}
    const auto data=runtime_->sensors()->snapshot();
    const auto selected=select_backup_distance(
      distance,kMaxBackupBudget-runtime_->backup_used,
      [&](double sweep) {return swept_clear(data,runtime_->tf,runtime_->node->now(),
        runtime_->base_frame,-sweep,0,runtime_->data_age);});
    if (!selected.ok) {runtime_->diagnostic("BACKUP",selected.result.code);return false;}
    requested_distance_=distance;
    distance_=selected.selected; goal.target.x=-distance_; goal.speed=speed;
    goal.time_allowance=rclcpp::Duration::from_seconds(time);result_timeout_=time+1;return true;
  }
  bool reserve() override {
    // Action server 可在 prepare 后才变为就绪；在真正提交前用最新数据再检查一次。
    // 该次只允许保持或缩短 RearClear 已选距离，绝不因环境变化扩大。
    const auto data=runtime_->sensors()->snapshot();
    const auto selected=select_backup_distance(
      distance_,kMaxBackupBudget-runtime_->backup_used,
      [&](double sweep) {return swept_clear(data,runtime_->tf,runtime_->node->now(),
        runtime_->base_frame,-sweep,0,runtime_->data_age);});
    if (!selected.ok) {
      runtime_->diagnostic("BACKUP_RECHECK_REJECTED",selected.result.code);
      return false;
    }
    distance_=selected.selected;goal_.target.x=-distance_;
    if (!runtime_->reserve_backup(distance_)) {return false;}
    initial_odom_=runtime_->odom();
    runtime_->diagnostic("BACKUP_SELECTED","requested="+std::to_string(requested_distance_)+
      ",selected="+std::to_string(distance_)+",remaining_budget="+
      std::to_string(std::max(0.0,kMaxBackupBudget-runtime_->backup_used)));
    runtime_->escape_stage=1;return true;
  }
  void rejected() override {runtime_->release_backup(distance_);runtime_->note_backup(false);}
  void failed() override {
    const auto progress=actual_progress();
    runtime_->reconcile_backup(distance_,progress.second,progress.first);
    runtime_->diagnostic("BACKUP_FAILED","requested="+std::to_string(distance_)+
      ",actual="+std::to_string(progress.second)+",remaining_budget="+
      std::to_string(std::max(0.0,kMaxBackupBudget-runtime_->backup_used))+
      ",odom_reliable="+(progress.first?"true":"false"));
    runtime_->note_backup(false);
  }
  void completed(nav2_msgs::action::BackUp::Result::SharedPtr) override {
    const auto progress=actual_progress();
    // Action 报成功但里程计仍近似零位移属于相互矛盾证据，不能把预留额度退回。
    const bool displacement_confirmed=progress.first && progress.second>0.01;
    runtime_->reconcile_backup(distance_,progress.second,displacement_confirmed);
    runtime_->diagnostic("BACKUP_COMPLETED","requested="+std::to_string(distance_)+
      ",actual="+std::to_string(progress.second)+",remaining_budget="+
      std::to_string(std::max(0.0,kMaxBackupBudget-runtime_->backup_used))+
      ",odom_reliable="+(displacement_confirmed?"true":"false"));
    runtime_->note_backup(true);
  }
  std::pair<bool,double> actual_progress() const {
    auto current=runtime_->odom();
    if (!initial_odom_ || !current || !runtime_->odom_fresh()) {return {false,0};}
    const double yaw=tf2::getYaw(initial_odom_->pose.pose.orientation);
    const double dx=current->pose.pose.position.x-initial_odom_->pose.pose.position.x;
    const double dy=current->pose.pose.position.y-initial_odom_->pose.pose.position.y;
    const double value=-(std::cos(yaw)*dx+std::sin(yaw)*dy);
    if (!std::isfinite(value) || value<-.02) {return {false,0};}
    return {true,std::clamp(value,0.0,distance_)};
  }
  double requested_distance_{kMaxSingleBackupDistance};
  double distance_{kMaxSingleBackupDistance};
  nav_msgs::msg::Odometry::ConstSharedPtr initial_odom_;
};
class SafeFollowPath : public SafeActionLeaf<nav2_msgs::action::FollowPath> {
public:
  SafeFollowPath(const std::string & n,const BT::NodeConfiguration & c):SafeActionLeaf(n,c,"/follow_path",true) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("controller_id","FollowPath",""),
      BT::InputPort<std::string>("goal_checker_id","general_goal_checker","")};
  }
private:
  bool prepare(nav2_msgs::action::FollowPath::Goal & goal) override {
    getInput("controller_id",goal.controller_id);getInput("goal_checker_id",goal.goal_checker_id);
    return getInput("path",goal.path) && !goal.path.poses.empty();
  }
  void update() override {
    nav_msgs::msg::Path path;
    if (!getInput("path",path) || path.poses.empty() || same_path_poses(path,goal_.path)) {return;}
    goal_.path=path;
    session_->send(goal_,0);
  }
};
template<bool Through>
class SafePlanner : public SafeActionLeaf<std::conditional_t<Through,
    nav2_msgs::action::ComputePathThroughPoses,nav2_msgs::action::ComputePathToPose>> {
  using Action=std::conditional_t<Through,nav2_msgs::action::ComputePathThroughPoses,nav2_msgs::action::ComputePathToPose>;
  using Base=SafeActionLeaf<Action>;
public:
  SafePlanner(const std::string & n,const BT::NodeConfiguration & c)
  :Base(n,c,Through?"/compute_path_through_poses":"/compute_path_to_pose",false) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("planner_id","GridBased",""),
      BT::InputPort<double>("timeout",5.0,"")};
  }
private:
  bool prepare(typename Action::Goal & goal) override {
    this->getInput("planner_id",goal.planner_id);
    this->getInput("timeout",this->result_timeout_);
    if (!std::isfinite(this->result_timeout_) || this->result_timeout_<=0 || this->result_timeout_>5) {return false;}
    if constexpr (Through) {return this->getInput("goals",goal.goals) && !goal.goals.empty();}
    else {return bool(this->getInput("goal",goal.goal));}
  }
  void completed(typename Action::Result::SharedPtr result) override {
    if (result && !result->path.poses.empty()) {
      this->setOutput("path",result->path);this->runtime_->replan_required=false;
    }
  }
};

class ProgressGuard : public BT::DecoratorNode {
public:
  ProgressGuard(const std::string & n,const BT::NodeConfiguration & c)
  : BT::DecoratorNode(n,c),runtime_(RecoveryRuntime::get(c)) {
    pub_=runtime_->node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/progress_guard/status",10);
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::InputPort<double>("linear_stagnation_timeout",3.0,""),
      BT::InputPort<double>("linear_displacement_threshold",0.03,""),
      BT::InputPort<double>("angular_stagnation_timeout",3.0,""),
      BT::InputPort<double>("angular_convergence_threshold",0.10,""),
      BT::InputPort<double>("max_rotation_budget",8.0,""),
      BT::InputPort<double>("heading_flip_threshold",0.35,""),
      BT::InputPort<int>("max_heading_flips",3,"")};
  }
  void halt() override {initialized_=false;in_turn_=false;heading_flips_=0;BT::DecoratorNode::halt();}
  BT::NodeStatus tick() override {
    if (status()==BT::NodeStatus::IDLE) {initialized_=false;in_turn_=false;heading_flips_=0;}
    if (runtime_->motion && runtime_->motion->state().terminal) {
      initialized_=false;in_turn_=false;heading_flips_=0;
      return child_node_->executeTick(); // 动作结束后的停车确认不计入行驶进展窗口。
    }
    if (!runtime_->healthy() || !runtime_->odom_fresh()) {return failure("DATA_EXPIRED","定位或轮式里程计无效");}
    nav_msgs::msg::Path path; geometry_msgs::msg::PoseStamped pose;
    if (!getInput("path",path) || !runtime_->pose(pose,path.header.frame_id)) {return failure("TF_UNAVAILABLE","路径或 TF 不可用");}
    auto ref=heading_reference(pose,path,final_goal(*this));
    if (!ref.valid) {return failure("INVALID_PATH","无法计算路径前向参考");}
    auto odom=runtime_->odom();
    reference_yaw_=ref.yaw;robot_yaw_=tf2::getYaw(pose.pose.orientation);measured_wz_=odom->twist.twist.angular.z;
    bool path_changed=path.poses.size()!=previous_path_.poses.size();
    if (!path_changed) {
      for (size_t i=0;i<path.poses.size();++i) {
        if (path.poses[i].pose!=previous_path_.poses[i].pose) {path_changed=true;break;}
      }
    }
    if (path_changed) {++path_revision_;previous_path_=path;}
    const auto now=Steady::now();
    if (!initialized_ || epoch_!=runtime_->epoch) {
      initialized_=true;epoch_=runtime_->epoch;baseline_=odom->pose.pose;cruise_=now;
      in_turn_=false;heading_flips_=0;
    }
    double linear_timeout=3,linear_threshold=.03,angular_timeout=3,angular_threshold=.1,total=8;
    double flip_threshold=.35; int max_flips=3;
    getInput("linear_stagnation_timeout",linear_timeout);getInput("linear_displacement_threshold",linear_threshold);
    getInput("angular_stagnation_timeout",angular_timeout);getInput("angular_convergence_threshold",angular_threshold);
    getInput("max_rotation_budget",total);
    getInput("heading_flip_threshold",flip_threshold);getInput("max_heading_flips",max_flips);
    for (double value : {linear_timeout,linear_threshold,angular_timeout,angular_threshold,total,flip_threshold}) {
      if (!std::isfinite(value)||value<=0) {return failure("INVALID_PARAMETER","进展守卫阈值无效");}
    }
    if (max_flips<1) {return failure("INVALID_PARAMETER","进展守卫阈值无效");}
    const double dx=odom->pose.pose.position.x-baseline_.position.x;
    const double dy=odom->pose.pose.position.y-baseline_.position.y;
    const double baseline_yaw=tf2::getYaw(baseline_.orientation);
    const auto progress=classify_forward_progress(dx,dy,baseline_yaw,linear_threshold);
    last_forward_step_=0.0;
    if (progress.displaced) {
      baseline_=odom->pose.pose;
      if (progress.plausible) {
        runtime_->forward_progress(progress.forward);
        last_forward_step_=progress.forward;
        if (progress.forward>=linear_threshold) {
          // 边转边前进属于有效连续绕行：刷新停滞窗口，但不靠路径版本变化续期。
          cruise_=now;heading_flips_=0;
          if (in_turn_) {
            turn_=window_=now;target_yaw_=ref.yaw;error_=std::abs(ref.error);
          }
        }
      }
    }
    if (std::abs(ref.error)>=.35) {
      auto event=classify_heading_align(in_turn_,target_yaw_,ref.yaw,heading_flips_,
        flip_threshold,static_cast<unsigned>(max_flips));
      if (event==HeadingAlignEvent::Oscillating) {
        return failure("PATH_HEADING_OSCILLATION","路径参考航向反复翻转，停止原地对准");
      }
      if (!in_turn_) {
        in_turn_=true;turn_=window_=now;error_=std::abs(ref.error);target_yaw_=ref.yaw;
      } else if (event==HeadingAlignEvent::NewTarget) {
        // 明显换向只建立新的改善参考；同一受阻回合的总转向预算不得刷新。
        ++heading_flips_;window_=now;error_=std::abs(ref.error);target_yaw_=ref.yaw;
      }
      if (seconds(turn_)>=total) {return failure("ROTATION_BUDGET_EXCEEDED","单次转向总预算耗尽");}
      if (seconds(window_)>=angular_timeout) {
        const double tracked_error=std::abs(normalize(target_yaw_-robot_yaw_));
        double improvement=error_-tracked_error;
        if (improvement<angular_threshold) {
          return failure("ANGULAR_CONVERGENCE_FAIL",std::to_string(angular_timeout)+
            "s 内朝向改善 "+std::to_string(improvement)+" rad");
        }
        window_=now;error_=tracked_error;
      }
    } else {
      if (in_turn_) {in_turn_=false;heading_flips_=0;baseline_=odom->pose.pose;cruise_=now;}
      if (seconds(cruise_)>=linear_timeout) {return failure("LINEAR_STAGNATION","轮式里程计有效前向位移不足");}
    }
    if (seconds(last_diag_)>=1) {
      last_diag_=now;
      publish("NONE",in_turn_?"转向对准":"正常跟随",ref.yaw,
        tf2::getYaw(pose.pose.orientation),odom->twist.twist.angular.z);
    }
    return child_node_->executeTick();
  }
private:
  BT::NodeStatus failure(const std::string & code,const std::string & detail) {
    runtime_->permit(false);runtime_->begin_recovery();
    publish(code,detail,reference_yaw_,robot_yaw_,measured_wz_);
    if (child_node_->status()==BT::NodeStatus::RUNNING) {child_node_->halt();}
    return BT::NodeStatus::FAILURE;
  }
  void publish(const std::string & code,const std::string & detail,double target,double yaw,double wz) {
    diagnostic_msgs::msg::DiagnosticStatus msg;msg.name="ProgressGuard";
    msg.level=code=="NONE"?0:1;msg.message=detail;msg.hardware_id="carcar_nav_bt_nodes";
    for (auto pair : {std::pair<std::string,std::string>{"reason_code",code},{"detail",detail},
        {"reference_yaw",std::to_string(target)},{"robot_yaw",std::to_string(yaw)},
        {"tracked_reference_yaw",std::to_string(target_yaw_)},
        {"measured_wz",std::to_string(wz)},{"path_revision",std::to_string(path_revision_)},
        {"heading_flips",std::to_string(heading_flips_)},
        {"forward_step",std::to_string(last_forward_step_)},
        {"turn_elapsed_s",in_turn_?std::to_string(seconds(turn_)):"0"},
        {"window_elapsed_s",in_turn_?std::to_string(seconds(window_)):"0"}}) {
      diagnostic_msgs::msg::KeyValue kv;kv.key=pair.first;kv.value=pair.second;msg.values.push_back(kv);
    }
    pub_->publish(msg);
  }
  std::shared_ptr<RecoveryRuntime> runtime_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_;
  bool initialized_{false},in_turn_{false};unsigned heading_flips_{0};uint64_t epoch_{0};
  geometry_msgs::msg::Pose baseline_;
  TimePoint cruise_{},turn_{},window_{},last_diag_{};
  double error_{0},target_yaw_{0};
  double reference_yaw_{0},robot_yaw_{0},measured_wz_{0},last_forward_step_{0};
  uint64_t path_revision_{0};nav_msgs::msg::Path previous_path_;
};

class ParkAndObserve : public BT::StatefulActionNode {
public:
  ParkAndObserve(const std::string & n,const BT::NodeConfiguration & c)
  :BT::StatefulActionNode(n,c),runtime_(RecoveryRuntime::get(c)) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("observe_duration",8.0,""),BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals")};
  }
private:
  BT::NodeStatus onStart() override {
    double duration=8;getInput("observe_duration",duration);
    if (!std::isfinite(duration)||duration<=0||duration>30) {return BT::NodeStatus::FAILURE;}
    if (runtime_->observe_started==TimePoint{}) {runtime_->observe_started=Steady::now();}
    deadline_=runtime_->observe_started+
      std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(duration));
    last_check_=TimePoint{};runtime_->permit(false);return onRunning();
  }
  BT::NodeStatus onRunning() override {
    runtime_->permit(false);
    if (Steady::now()>=deadline_) {
      if (runtime_->request_observe_replan()) {
        runtime_->diagnostic("PARK_AND_OBSERVE","REPLAN_AFTER_OBSERVE");
        return BT::NodeStatus::SUCCESS;
      }
      runtime_->diagnostic("PARK_AND_OBSERVE","OBSERVE_TIMEOUT");
      return BT::NodeStatus::FAILURE;
    }
    if (last_check_!=TimePoint{} && seconds(last_check_)<0.2) {return BT::NodeStatus::RUNNING;}
    last_check_=Steady::now();
    auto ready=runtime_->inputs_ready();
    nav_msgs::msg::Path path;geometry_msgs::msg::PoseStamped pose;
    if (ready.ok && getInput("path",path) && runtime_->pose(pose,path.header.frame_id)) {
      auto ref=heading_reference(pose,path,final_goal(*this));
      if (ref.valid) {
        auto check=swept_clear(runtime_->sensors()->snapshot(),runtime_->tf,runtime_->node->now(),
          runtime_->base_frame,.35*std::cos(ref.error),.35*std::sin(ref.error),runtime_->data_age);
        if (check.ok) {runtime_->diagnostic("PARK_AND_OBSERVE","PATH_CLEAR");return BT::NodeStatus::SUCCESS;}
        ready=check;
      }
    }
    runtime_->diagnostic("PARK_AND_OBSERVE",ready.ok?"WAITING_PATH":ready.code);
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {runtime_->permit(false);}
  std::shared_ptr<RecoveryRuntime> runtime_;
  TimePoint deadline_{},last_check_{};
};

// 两个子节点：正常规划跟随、有序恢复。切换前必须跨越取消和停稳屏障。
class RecoverySupervisor : public BT::ControlNode {
public:
  RecoverySupervisor(const std::string & n,const BT::NodeConfiguration & c)
  :BT::ControlNode(n,c),runtime_(RecoveryRuntime::get(c)) {runtime_->supervised=true;}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
      BT::InputPort<bool>("through_poses",false,"")};
  }
  BT::NodeStatus tick() override {
    if (childrenCount()!=2) {throw BT::RuntimeError("RecoverySupervisor 必须有两个子节点");}
    const bool entered_from_idle=status()==BT::NodeStatus::IDLE;
    runtime_->note_tick();
    setStatus(BT::NodeStatus::RUNNING);
    for (auto & s : runtime_->retiring) {s->poll();}
    runtime_->retiring.erase(std::remove_if(runtime_->retiring.begin(),runtime_->retiring.end(),
      [](auto s) {return s->state().terminal;}),runtime_->retiring.end());
    if (runtime_->motion) {runtime_->motion->poll();}
    bool through=false;getInput("through_poses",through);
    auto uuid=runtime_->goal_uuid(through);
    if (entered_from_idle) {
      starting_task_=true;starting_uuid_.clear();uuid_wait_=Steady::now();
    }
    if (starting_task_) {
      return start_task(uuid);
    }
    if (runtime_->fault) {return fail(runtime_->fault_reason);}
    if (runtime_->recovery_active && seconds(runtime_->recovery_started)>=runtime_->recovery_timeout) {
      return fail("RECOVERY_TOTAL_TIMEOUT");
    }
    if (!uuid.empty() && runtime_->navigation_uuid!=uuid) {
      runtime_->previous_navigation_uuid=runtime_->navigation_uuid;runtime_->navigation_uuid=uuid;
      if (!runtime_->previous_navigation_uuid.empty()) {runtime_->diagnostic("STOPPING","GOAL_REPLACED");}
    }
    geometry_msgs::msg::PoseStamped goal;std::vector<geometry_msgs::msg::PoseStamped> goals;
    getInput("goal",goal);getInput("goals",goals);
    bool changed=initialized_ && ((!uuid.empty() && !uuid_.empty() && uuid!=uuid_) || goal!=goal_ || goals!=goals_);
    if (!initialized_ || changed) {
      initialized_=true;goal_=goal;goals_=goals;
      if (!uuid.empty()) {uuid_=uuid;}
      ++runtime_->epoch;replan_tried_=false;
      stop_then(0); // 仅重置任务基准；受阻预算与总时间继续保留。
    } else if (uuid_.empty() && !uuid.empty()) {uuid_=uuid;}
    if (switching_) {
      auto status=finish_stop();if (status!=BT::NodeStatus::SUCCESS) {return status;}
    }
    auto ready=runtime_->inputs_ready();
    if (!ready.ok && active_==0) {
      stop_then(1);runtime_->begin_recovery();
      runtime_->diagnostic("INPUT_WAIT",ready.code);
      return BT::NodeStatus::RUNNING;
    }
    // 无 fresh UUID 时只允许等待和规划入口准备，禁止真实运动提交。
    if (uuid.empty() && active_==0) {
      runtime_->permit(false);
      if (uuid_wait_==TimePoint{}) {uuid_wait_=Steady::now();}
      if (seconds(uuid_wait_)>2) {return fail("NAVIGATION_UUID_MISSING");}
      return BT::NodeStatus::RUNNING;
    }
    auto child_status=children_nodes_[active_]->executeTick();
    if (through) {
      std::vector<geometry_msgs::msg::PoseStamped> remaining;
      if (getInput("goals",remaining) && !remaining.empty() && remaining.size()<goals_.size() &&
        std::equal(remaining.rbegin(),remaining.rend(),goals_.rbegin())) {
        goals_=remaining; // 本次 tick 内 RemovePassedGoals 删除已到达前缀，不是新的导航任务。
      }
    }
    bool allow=false;
    if (runtime_->motion) {
      auto s=runtime_->motion->state();
      allow=ready.ok && s.accepted && !s.terminal && !s.cancel_requested && !s.timed_out;
      if (!ready.ok && !s.terminal) {
        runtime_->permit(false);runtime_->motion->cancel();stop_then(1);
      }
    }
    runtime_->permit(allow && !switching_);
    runtime_->diagnostic(active_==0?"NAVIGATING":"RECOVERING",ready.code);
    if (runtime_->fault) {return fail(runtime_->fault_reason);}
    if (switching_ || child_status==BT::NodeStatus::RUNNING) {return BT::NodeStatus::RUNNING;}
    runtime_->permit(false);
    if (child_status==BT::NodeStatus::SUCCESS && active_==0) {
      haltChildren();initialized_=false;return BT::NodeStatus::SUCCESS;
    }
    if (child_status==BT::NodeStatus::FAILURE && active_==1) {return fail("RECOVERY_EXHAUSTED");}
    if (active_==0) {
      runtime_->begin_recovery();
      if (!replan_tried_) {replan_tried_=true;stop_then(0);} else {stop_then(1);}
    } else {runtime_->replan_required=true;stop_then(0);}
    return BT::NodeStatus::RUNNING;
  }
  void halt() override {
    runtime_->note_tick();
    runtime_->permit(false);haltChildren();
    if (runtime_->motion && !runtime_->motion->state().terminal) {runtime_->motion->cancel();}
    initialized_=false;switching_=false;starting_task_=false;starting_uuid_.clear();
    uuid_wait_=TimePoint{};
    BT::ControlNode::halt();
  }
private:
  BT::NodeStatus start_task(const std::string & uuid) {
    runtime_->permit(false);
    if (starting_uuid_.empty()) {
      // 回调缓存可能仍是上一任务 UUID；同一位置重发也必须等新的 Action UUID。
      if (uuid.empty() || (!uuid_.empty() && uuid==uuid_)) {
        runtime_->diagnostic("TASK_START_PREPARING","WAITING_NEW_UUID");
        if (seconds(uuid_wait_)>2) {return fail("NAVIGATION_UUID_MISSING");}
        return BT::NodeStatus::RUNNING;
      }
      starting_uuid_=uuid;
      runtime_->previous_navigation_uuid=runtime_->navigation_uuid;
      runtime_->navigation_uuid=starting_uuid_;
      runtime_->diagnostic("TASK_START_PREPARING","STOPPING_PREVIOUS_TASK");
      stop_then(0);
    }
    if (switching_) {
      auto status=finish_stop();
      if (status!=BT::NodeStatus::SUCCESS) {return status;}
    }
    auto ready=runtime_->inputs_ready();
    if (!ready.ok) {
      runtime_->diagnostic("TASK_START_BLOCKED",ready.code);
      return BT::NodeStatus::RUNNING;
    }
    if (!runtime_->still(runtime_->still_duration)) {
      runtime_->diagnostic("TASK_START_PREPARING","WAITING_STILLNESS");
      stop_then(0);
      return BT::NodeStatus::RUNNING;
    }
    geometry_msgs::msg::PoseStamped goal;std::vector<geometry_msgs::msg::PoseStamped> goals;
    getInput("goal",goal);getInput("goals",goals);
    runtime_->reset_for_new_task(starting_uuid_);
    initialized_=true;goal_=goal;goals_=goals;uuid_=starting_uuid_;
    replan_tried_=false;active_=0;starting_task_=false;starting_uuid_.clear();
    uuid_wait_=TimePoint{};
    return BT::NodeStatus::RUNNING;
  }
  void stop_then(size_t next) {
    runtime_->permit(false);haltChildren();
    if (runtime_->motion && !runtime_->motion->state().terminal) {runtime_->motion->cancel();}
    next_=next;
    if (!switching_) {switching_=true;cancel_deadline_=after(runtime_->cancel_timeout);settle_started_=TimePoint{};}
  }
  BT::NodeStatus finish_stop() {
    runtime_->permit(false);
    if (runtime_->motion && !runtime_->motion->state().terminal) {
      runtime_->motion->poll();
      if (Steady::now()>=cancel_deadline_) {return fail("CANCEL_UNCONFIRMED");}
      return BT::NodeStatus::RUNNING;
    }
    if (settle_started_==TimePoint{}) {settle_started_=Steady::now();runtime_->restart_stillness();}
    if (!runtime_->still(runtime_->still_duration)) {
      if (seconds(settle_started_)>=runtime_->settle_timeout) {return fail("STOP_NOT_CONFIRMED");}
      return BT::NodeStatus::RUNNING;
    }
    runtime_->motion.reset();switching_=false;active_=next_;
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus fail(const std::string & reason) {
    runtime_->fail(reason);haltChildren();
    if (runtime_->motion) {runtime_->motion->cancel();}
    return BT::NodeStatus::FAILURE;
  }
  std::shared_ptr<RecoveryRuntime> runtime_;
  bool initialized_{false},switching_{false},replan_tried_{false},starting_task_{false};
  size_t active_{0},next_{0};
  TimePoint cancel_deadline_{},settle_started_{},uuid_wait_{};
  std::string uuid_,starting_uuid_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};
}  // namespace
void register_nav012_nodes(BT::BehaviorTreeFactory & factory) {
  factory.registerNodeType<RecoveryInputsReady>("RecoveryInputsReady");
  factory.registerNodeType<ReplanNotRequired>("ReplanNotRequired");
  factory.registerNodeType<RearClear>("RearClear");
  factory.registerNodeType<ProgressGuard>("ProgressGuard");
  factory.registerNodeType<ControlledSpin>("ControlledSpin");
  factory.registerNodeType<ParkAndObserve>("ParkAndObserve");
  factory.registerNodeType<RecoverySupervisor>("RecoverySupervisor");
  factory.registerNodeType<SafeBackUp>("SafeBackUp");
  factory.registerNodeType<SafeFollowPath>("SafeFollowPath");
  factory.registerNodeType<SafePlanner<false>>("SafeComputePathToPose");
  factory.registerNodeType<SafePlanner<true>>("SafeComputePathThroughPoses");
}
}  // namespace carcar_navigation
