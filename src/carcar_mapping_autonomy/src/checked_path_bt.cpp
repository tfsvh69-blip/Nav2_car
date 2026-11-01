#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include "carcar_mapping_autonomy/route_validation.hpp"

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
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map_=m;map_at_=std::chrono::steady_clock::now();},options);
    states_=node_->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{state_=carcar_mapping_autonomy::Json::parse(m.data);state_at_=std::chrono::steady_clock::now();}catch(...){state_=nullptr;}},options);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
  }
  static BT::PortsList providedPorts(){return {BT::InputPort<nav_msgs::msg::Path>("path"),BT::OutputPort<nav_msgs::msg::Path>("checked_path")};}
  BT::NodeStatus tick()override{
    executor_.spin_some();const auto now=std::chrono::steady_clock::now();
    if(!map_||state_.is_null()||std::chrono::duration<double>(now-map_at_).count()>10||
      std::chrono::duration<double>(now-state_at_).count()>1||state_.value("phase","")!="EXPLORING")return BT::NodeStatus::FAILURE;
    const auto path=getInput<nav_msgs::msg::Path>("path");if(!path||path.value().poses.empty())return BT::NodeStatus::FAILURE;
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
      setOutput("checked_path",path.value());return BT::NodeStatus::SUCCESS;
    }catch(const std::exception&e){RCLCPP_WARN(node_->get_logger(),"路径审查数据未就绪：%s",e.what());return BT::NodeStatus::FAILURE;}
  }
private:
  rclcpp::Node::SharedPtr node_;std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr group_;rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr states_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;carcar_mapping_autonomy::Json state_;
  std::chrono::steady_clock::time_point map_at_,state_at_;
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
  void halt()override{BT::DecoratorNode::halt();ready_=true;goals_.clear();}
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
BT_REGISTER_NODES(factory){
  factory.registerNodeType<CheckedMappingPath>("CheckedMappingPath");
  factory.registerNodeType<ExplorationReplan>("ExplorationReplan");
  factory.registerNodeType<ExplorationFollow>("ExplorationFollow");
}
