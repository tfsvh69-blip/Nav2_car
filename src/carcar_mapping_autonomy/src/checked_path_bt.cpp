#include <algorithm>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include "carcar_mapping_autonomy/handoff.hpp"
#include "carcar_mapping_autonomy/route_validation.hpp"

namespace {
using HandoffClock=std::chrono::steady_clock;
// 同一黑板会被Nav2抢占复用；过点资格必须绑定实际审查的路径、目标与坐标版本。
struct CheckedHandoffPath {
  nav_msgs::msg::Path path;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  std::string task_id;
  uint64_t navigation_generation=0,map_revision=0;
  HandoffClock::time_point checked_at;
};
constexpr const char* handoff_proof_key="mapping_checked_handoff_path";
constexpr const char* retained_path_key="mapping_retained_checked_path";
bool handoff_context(const carcar_mapping_autonomy::Json&state,std::string&task,uint64_t&generation,uint64_t&revision){
  try{
    if(!state.is_object()||!state.contains("task_id")||!state["task_id"].is_string())return false;
    task=state["task_id"].get<std::string>();if(task.empty())return false;
    for(const auto*key:{"navigation_generation","map_revision"})
      if(!state.contains(key)||!state[key].is_number_integer()||
         (!state[key].is_number_unsigned()&&state[key].get<int64_t>()<0))return false;
    generation=state["navigation_generation"].get<uint64_t>();revision=state["map_revision"].get<uint64_t>();return true;
  }catch(const std::exception&){return false;}
}
}

// 审查行为树刚生成的实际路径；只有审查成功的副本能送入FollowPath。
// 使用独立回调组主动处理地图/任务状态，不依赖Nav2 Action节点的私有executor。
class CheckedMappingPath:public BT::SyncActionNode {
public:
  CheckedMappingPath(const std::string&name,const BT::NodeConfiguration&config):SyncActionNode(name,config){
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_=config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    maps_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){raw_map_=m;raw_map_at_=std::chrono::steady_clock::now();},options);
    constrained_maps_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/mapping/navigation_map",rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){constrained_map_=m;constrained_map_at_=std::chrono::steady_clock::now();},options);
    states_=node_->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{state_=carcar_mapping_autonomy::Json::parse(m.data);state_at_=std::chrono::steady_clock::now();}catch(...){state_=nullptr;}},options);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
  }
  static BT::PortsList providedPorts(){return {BT::InputPort<nav_msgs::msg::Path>("path"),BT::OutputPort<nav_msgs::msg::Path>("checked_path"),BT::InputPort<bool>("reuse_checked_path",false,"仅复核短时保留路线")};}
  BT::NodeStatus tick()override{
    executor_.spin_some();const auto now=std::chrono::steady_clock::now();
    config().blackboard->set<std::shared_ptr<const CheckedHandoffPath>>(handoff_proof_key,nullptr);
    const bool constrained=state_.is_object()&&state_.value("live_controls_enabled",false);
    map_=constrained?constrained_map_:raw_map_;map_at_=constrained?constrained_map_at_:raw_map_at_;
    if(!map_||state_.is_null()||std::chrono::duration<double>(now-map_at_).count()>10||
      std::chrono::duration<double>(now-state_at_).count()>1||state_.value("phase","")!="EXPLORING")return BT::NodeStatus::FAILURE;
    auto path=getInput<nav_msgs::msg::Path>("path");
    const bool reuse=getInput<bool>("reuse_checked_path").value();
    if(reuse){
      // 仅延续本任务/定位代次内刚刚确认的同一剩余目标序列；不为抢占后的新目标沿用旧路。
      std::shared_ptr<const CheckedHandoffPath> retained;std::vector<geometry_msgs::msg::PoseStamped> goals;
      std::string task;uint64_t generation=0,revision=0;
      if(!config().blackboard->get(retained_path_key,retained)||!retained||
         !config().blackboard->get("goals",goals)||goals.empty()||goals.size()>retained->goals.size()||
         !handoff_context(state_,task,generation,revision)||task!=retained->task_id||generation!=retained->navigation_generation||
         std::chrono::duration<double>(now-retained->checked_at).count()>2.0||
         !std::equal(goals.rbegin(),goals.rend(),retained->goals.rbegin()))return BT::NodeStatus::FAILURE;
      path=retained->path;
    }
    if(!path||path.value().poses.empty())return BT::NodeStatus::FAILURE;
    try{
      const auto tr=tf_->lookupTransform("map","base_footprint",tf2::TimePointZero);
      const auto age=(node_->now()-rclcpp::Time(tr.header.stamp)).seconds();if(age<-.25||age>.5)return BT::NodeStatus::FAILURE;
      carcar_mapping_autonomy::RouteValidator::Job job;job.map=map_;job.path=std::make_shared<nav_msgs::msg::Path>(path.value());
      job.current={tr.transform.translation.x,tr.transform.translation.y,0};job.remaining=true;
      const auto&end=path.value().poses.back().pose.position;job.target={end.x,end.y,0};
      const auto origin=state_.at("origin");job.task.anchored=true;job.task.origin={origin.at(0),origin.at(1),origin.at(2)};
      job.task.limits.radius=state_.at("radius_m");std::string reason;
      if(!carcar_mapping_autonomy::RouteValidator::check(job,&reason)){
        RCLCPP_WARN(node_->get_logger(),"实际执行路径审查拒绝：%s",reason.c_str());return BT::NodeStatus::FAILURE;
      }
      std::vector<geometry_msgs::msg::PoseStamped>goals;std::string task;uint64_t generation=0,revision=0;
      if(config().blackboard->get("goals",goals)&&!goals.empty()&&handoff_context(state_,task,generation,revision)){
        auto proof=std::make_shared<CheckedHandoffPath>();proof->path=path.value();proof->goals=std::move(goals);
        proof->task_id=std::move(task);proof->navigation_generation=generation;proof->map_revision=revision;proof->checked_at=now;
        // 复核旧路线不能给两秒保留期限续租；只有真正的新规划成功才更新保留凭据。
        if(!reuse)config().blackboard->set<std::shared_ptr<const CheckedHandoffPath>>(retained_path_key,proof);
        config().blackboard->set<std::shared_ptr<const CheckedHandoffPath>>(handoff_proof_key,std::move(proof));
      }
      if(reuse)RCLCPP_INFO(node_->get_logger(),"重规划暂未通过；原路线已重新通过当前地图完整净空审查，短暂保留同一FollowPath，未放宽停车保护");
      setOutput("checked_path",path.value());return BT::NodeStatus::SUCCESS;
    }catch(const std::exception&e){RCLCPP_WARN(node_->get_logger(),"路径审查数据未就绪：%s",e.what());return BT::NodeStatus::FAILURE;}
  }
private:
  rclcpp::Node::SharedPtr node_;std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr group_;rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_,constrained_maps_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr states_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_,raw_map_,constrained_map_;carcar_mapping_autonomy::Json state_;
  std::chrono::steady_clock::time_point map_at_,state_at_,raw_map_at_,constrained_map_at_;
};
// Humble的RateController不会因goals抢占立即重置；仅重启规划子树，继续执行已检查的路径。
class ExplorationReplan:public BT::DecoratorNode {
public:
  ExplorationReplan(const std::string&name,const BT::NodeConfiguration&config):DecoratorNode(name,config){}
  static BT::PortsList providedPorts(){return {};}
  BT::NodeStatus tick()override{
    const auto goals=config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
    const auto now=std::chrono::steady_clock::now();
    if(goals!=goals_){if(child_node_->status()!=BT::NodeStatus::IDLE)haltChild();ready_=true;}
    setStatus(BT::NodeStatus::RUNNING);
    if(ready_||child_node_->status()==BT::NodeStatus::RUNNING||now-last_>=std::chrono::seconds(1)){
      ready_=false;const auto result=child_node_->executeTick();
      // RemovePassedGoals也会修改黑板；保存处理后的值，避免过点导致重复重启规划。
      goals_=config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
      if(result==BT::NodeStatus::SUCCESS)last_=now;
      return result;
    }
    return BT::NodeStatus::RUNNING;
  }
  void halt()override{BT::DecoratorNode::halt();ready_=true;goals_.clear();config().blackboard->set<std::shared_ptr<const CheckedHandoffPath>>(retained_path_key,nullptr);}
private:
  bool ready_=true;std::chrono::steady_clock::time_point last_;
  std::vector<geometry_msgs::msg::PoseStamped>goals_;
};

// 旧FollowPath可能在新路径准备前成功；整个多点任务只能在当前末点真正到达后成功。
class ExplorationFollow:public BT::DecoratorNode {
public:
  ExplorationFollow(const std::string&name,const BT::NodeConfiguration&config):DecoratorNode(name,config){
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_=config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }
  static BT::PortsList providedPorts(){return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick()override{
    setStatus(BT::NodeStatus::RUNNING);
    const auto goals=config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
    const auto path=getInput<nav_msgs::msg::Path>("path");
    if(goals.empty())return BT::NodeStatus::FAILURE;
    if(!path||path.value().poses.empty())return BT::NodeStatus::RUNNING;
    const auto&target=goals.back().pose.position;const auto&end=path.value().poses.back().pose.position;
    // Smac终点落在5厘米栅格中心，允许原规划离散误差；该条件不放宽路径净空。
    if(std::hypot(target.x-end.x,target.y-end.y)>.10)return BT::NodeStatus::RUNNING;
    const auto result=child_node_->executeTick();
    if(result!=BT::NodeStatus::SUCCESS)return result;
    try{
      const auto tr=tf_->lookupTransform("map","base_footprint",tf2::TimePointZero);
      const auto age=(node_->now()-rclcpp::Time(tr.header.stamp)).seconds();
      if(age<-.25||age>.5)return BT::NodeStatus::FAILURE;
      if(std::hypot(target.x-tr.transform.translation.x,target.y-tr.transform.translation.y)<=.20)return BT::NodeStatus::SUCCESS;
    }catch(const std::exception&){return BT::NodeStatus::FAILURE;}
    haltChild();return BT::NodeStatus::RUNNING;
  }
private:
  rclcpp::Node::SharedPtr node_;std::shared_ptr<tf2_ros::Buffer>tf_;
};
// 读取监督器的任务内参数，并使用当前已审查路径的剩余弧长通过中间点。
class MappingPassedGoals:public BT::SyncActionNode {
public:
 MappingPassedGoals(const std::string&name,const BT::NodeConfiguration&config):SyncActionNode(name,config){
  node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");tf_=config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  rclcpp::SubscriptionOptions options;options.callback_group=group_;
  states_=node_->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
   [this](const std_msgs::msg::String&m){try{state_=carcar_mapping_autonomy::Json::parse(m.data);at_=std::chrono::steady_clock::now();}catch(...){}},options);
  executor_.add_callback_group(group_,node_->get_node_base_interface());
 }
 static BT::PortsList providedPorts(){return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals"),BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals"),BT::InputPort<nav_msgs::msg::Path>("path")};}
 BT::NodeStatus tick()override{
  executor_.spin_some();auto goals=getInput<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals");
  if(!goals)return BT::NodeStatus::FAILURE;
  auto result=goals.value();const auto path=getInput<nav_msgs::msg::Path>("path");
  std::shared_ptr<const CheckedHandoffPath>proof;std::string task;uint64_t generation=0,revision=0;
  const auto now=HandoffClock::now();
  try{
    const bool qualified=path&&config().blackboard->get(handoff_proof_key,proof)&&proof&&
      handoff_context(state_,task,generation,revision)&&proof->task_id==task&&
      proof->navigation_generation==generation&&proof->map_revision==revision&&
      proof->goals==result&&proof->path==path.value()&&
      std::chrono::duration<double>(now-proof->checked_at).count()<=1.5&&
      std::chrono::duration<double>(now-at_).count()<1&&state_.value("phase",std::string())=="EXPLORING";
    if(qualified){
      const double radius=state_.value("handoff",carcar_mapping_autonomy::Json::object()).value("distance_m",.25);
      if(std::isfinite(radius)&&radius>=.15&&radius<=1.5){
        const auto tr=tf_->lookupTransform("map","base_footprint",tf2::TimePointZero);const auto age=(node_->now()-rclcpp::Time(tr.header.stamp)).seconds();
        if(age>=-.1&&age<=.5)while(result.size()>1){const auto&p=result.front().pose.position;
          if(carcar_mapping_autonomy::handoff_path_distance(path.value(),{tr.transform.translation.x,tr.transform.translation.y,0},{p.x,p.y,0})>radius)break;
          result.erase(result.begin());
        }
      }
    }
  }catch(const std::exception&){}
  // 资格不匹配时保留全部目标，让后续真实规划和审查先更新路径，不停树、不虚构到达。
  setOutput("output_goals",result);return BT::NodeStatus::SUCCESS;
 }
private:
 rclcpp::Node::SharedPtr node_;std::shared_ptr<tf2_ros::Buffer>tf_;rclcpp::CallbackGroup::SharedPtr group_;
 rclcpp::executors::SingleThreadedExecutor executor_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr states_;
 carcar_mapping_autonomy::Json state_=carcar_mapping_autonomy::Json::object();std::chrono::steady_clock::time_point at_;
};

BT_REGISTER_NODES(factory){
  factory.registerNodeType<MappingPassedGoals>("MappingPassedGoals");
  factory.registerNodeType<CheckedMappingPath>("CheckedMappingPath");
  factory.registerNodeType<ExplorationReplan>("ExplorationReplan");
  factory.registerNodeType<ExplorationFollow>("ExplorationFollow");
}
