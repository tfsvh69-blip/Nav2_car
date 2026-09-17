// NAV-006/NAV-008: Groot 1 实时显示桥接。
// Nav2 Humble 已移除原生 Groot 实时监视。本节点只镜像 /behavior_tree_log 的
// 状态变化到一个不含导航逻辑的显示树；它不加载 Nav2 BT 插件、不 tick 导航树、
// 不发送 action、不发布速度，也不访问串口。

#include <cstdint>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// ros-humble-behaviortree-cpp-v3 3.8.7 的头文件声明了此方法，安装库却没有
// 导出它；TreeNode::executeTick() 本身已正确读取 pre_condition_callback_。
// 在本桥接可执行文件中补齐与该头文件一致的轻量实现，使每个显示节点都能在
// 执行前返回真实 ROS 日志记录的状态，而不会运行其 tick()。
namespace BT
{
void TreeNode::setPreTickOverrideFunction(PreTickOverrideCallback callback)
{
  pre_condition_callback_ = std::move(callback);
}
}  // namespace BT

namespace
{
std::string file_digest(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return "FILE_NOT_FOUND";
  }
  uint64_t hash = 14695981039346656037ULL;
  char byte;
  while (stream.get(byte)) {
    hash ^= static_cast<uint8_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream result;
  result << "fnv64:" << std::hex << hash;
  return result.str();
}

BT::Optional<BT::NodeStatus> parse_status(const std::string & status)
{
  if (status == "IDLE") return BT::NodeStatus::IDLE;
  if (status == "RUNNING") return BT::NodeStatus::RUNNING;
  if (status == "SUCCESS") return BT::NodeStatus::SUCCESS;
  if (status == "FAILURE") return BT::NodeStatus::FAILURE;
  return {};
}

BT::PortsList generic_ports()
{
  return {
    BT::InputPort<std::string>("goal"),
    BT::InputPort<std::string>("goals"),
    BT::InputPort<std::string>("path"),
    BT::InputPort<std::string>("planner_id"),
    BT::InputPort<std::string>("controller_id"),
    BT::InputPort<std::string>("goal_checker_id"),
    BT::InputPort<std::string>("through_poses"),
    BT::InputPort<std::string>("input_goals"),BT::InputPort<std::string>("output_goals"),
    BT::InputPort<std::string>("radius"),BT::InputPort<std::string>("robot_base_frame"),
    BT::InputPort<std::string>("costmap_topic"),
    BT::InputPort<std::string>("scan_topic"),
    BT::InputPort<std::string>("nomotion_service"),
    BT::InputPort<std::string>("global_service"),
    BT::InputPort<std::string>("hz"),
    BT::InputPort<std::string>("timeout"),
    BT::InputPort<std::string>("wait_duration"),
    BT::InputPort<std::string>("spin_dist"),
    BT::InputPort<std::string>("time_allowance"),
    BT::InputPort<std::string>("backup_dist"),
    BT::InputPort<std::string>("backup_speed"),
    BT::InputPort<std::string>("backup_distance"),
    BT::InputPort<std::string>("max_data_age"),
    BT::InputPort<std::string>("local_timeout"),
    BT::InputPort<std::string>("global_timeout"),
    BT::InputPort<std::string>("number_of_retries"),
    BT::InputPort<std::string>("linear_stagnation_timeout"),
    BT::InputPort<std::string>("linear_displacement_threshold"),
    BT::InputPort<std::string>("angular_stagnation_timeout"),
    BT::InputPort<std::string>("angular_convergence_threshold"),
    BT::InputPort<std::string>("max_rotation_budget"),
    BT::InputPort<std::string>("max_spin_angle"),
    BT::InputPort<std::string>("observe_duration")
  };
}

// XML 的真实拓扑由 BT.CPP 保存，下面的节点从不运行导航业务；每个节点在
// executeTick 前均由 pre-tick 回调直接注入真实日志中最后一次的状态。
class MirrorAction : public BT::SyncActionNode
{
public:
  MirrorAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return generic_ports(); }

private:
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

class MirrorCondition : public BT::ConditionNode
{
public:
  MirrorCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() { return generic_ports(); }

private:
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

class MirrorControl : public BT::ControlNode
{
public:
  MirrorControl(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}
  static BT::PortsList providedPorts() { return generic_ports(); }

private:
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  void halt() override { haltChildren(); }
};

class MirrorDecorator : public BT::DecoratorNode
{
public:
  MirrorDecorator(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}
  static BT::PortsList providedPorts() { return generic_ports(); }

private:
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};
}  // namespace

class BtMonitorNode : public rclcpp::Node
{
public:
  BtMonitorNode()
  : Node("bt_monitor_node")
  {
    bt_xml_path_ = this->declare_parameter<std::string>("bt_xml_path", "");
    zmq_pub_port_ = static_cast<unsigned>(this->declare_parameter<int>("zmq_publisher_port", 1666));
    zmq_server_port_ = static_cast<unsigned>(this->declare_parameter<int>("zmq_server_port", 1667));
    max_messages_per_second_ = static_cast<unsigned>(this->declare_parameter<int>("max_msg_per_second", 25));
    bt_log_topic_ = this->declare_parameter<std::string>("bt_log_topic", "/behavior_tree_log");
    action_status_topic_ = this->declare_parameter<std::string>(
      "action_status_topic", "/navigate_to_pose/_action/status");

    if (bt_xml_path_.empty()) {
      throw std::runtime_error("bt_xml_path 不能为空");
    }
    xml_digest_ = file_digest(bt_xml_path_);
    if (xml_digest_ == "FILE_NOT_FOUND") {
      throw std::runtime_error("找不到行为树 XML: " + bt_xml_path_);
    }
    build_display_tree();
    zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(
      *tree_, max_messages_per_second_, zmq_pub_port_, zmq_server_port_);

    bt_log_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      bt_log_topic_, rclcpp::QoS(100).reliable(),
      std::bind(&BtMonitorNode::on_bt_log, this, std::placeholders::_1));
    action_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_status_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&BtMonitorNode::on_action_status, this, std::placeholders::_1));
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/bt_monitor/diagnostics", rclcpp::QoS(10).reliable());
    diagnostics_timer_ = this->create_wall_timer(1s, std::bind(&BtMonitorNode::publish_diagnostics, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Groot 显示桥接已就绪：XML=%s，ZMQ=127.0.0.1:%u/%u，等待真实 BT 状态日志",
      bt_xml_path_.c_str(), zmq_pub_port_, zmq_server_port_);
  }

private:
  void build_display_tree()
  {
    for (const auto & tag : {
        "ComputePathToPose", "ComputePathThroughPoses", "FollowPath", "Wait", "Spin", "BackUp",
        "RecoverLocalization", "WaitForLocalizationStatus", "ProtectedBackUp", "ControlledSpin", "ParkAndObserve",
        "SafeFollowPath", "SafeBackUp", "SafeComputePathToPose", "SafeComputePathThroughPoses", "RemovePassedGoals"})
    {
      factory_.registerNodeType<MirrorAction>(tag);
    }
    for (const auto & tag : {"GoalUpdated", "LocalizationHealthy", "RearClear", "RecoveryInputsReady"}) {
      factory_.registerNodeType<MirrorCondition>(tag);
    }
    for (const auto & tag : {"PipelineSequence", "RecoveryNode", "RoundRobin", "RecoverySupervisor"}) {
      factory_.registerNodeType<MirrorControl>(tag);
    }
    for (const auto & tag : {"RateController", "ProgressGuard"}) {
      factory_.registerNodeType<MirrorDecorator>(tag);
    }

    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(bt_xml_path_));
    for (const auto & node : tree_->nodes) {
      node_by_name_[node->name()] = node.get();
      status_by_name_[node->name()] = BT::NodeStatus::IDLE;
      node->setPreTickOverrideFunction(
        [this](BT::TreeNode & tree_node, BT::NodeStatus) -> BT::Optional<BT::NodeStatus> {
          std::lock_guard<std::mutex> lock(status_mutex_);
          const auto it = status_by_name_.find(tree_node.name());
          return it == status_by_name_.end() ? BT::Optional<BT::NodeStatus>(BT::NodeStatus::IDLE) :
            BT::Optional<BT::NodeStatus>(it->second);
        });
    }
  }

  void on_bt_log(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr message)
  {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      for (const auto & event : message->event_log) {
        const auto parsed = parse_status(event.current_status);
        const auto node = node_by_name_.find(event.node_name);
        if (!parsed) {
          unknown_statuses_.insert(event.current_status);
          continue;
        }
        if (node == node_by_name_.end()) {
          mismatched_node_names_.insert(event.node_name);
          continue;
        }
        status_by_name_[event.node_name] = *parsed;
        changed = true;
      }
      last_log_time_ = this->now();
      have_log_ = true;
    }
    // 逐个注入而非 tick 根节点，避免显示树的控制节点自行推演子节点状态。
    if (changed) {
      for (const auto & entry : node_by_name_) {
        entry.second->executeTick();
      }
    }
  }

  void on_action_status(const action_msgs::msg::GoalStatusArray::SharedPtr message)
  {
    bool active = false;
    for (const auto & status : message->status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
      {
        active = true;
        break;
      }
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    action_active_ = active;
    action_status_received_ = true;
  }

  void publish_diagnostics()
  {
    const auto now = this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "bt_monitor";
    status.hardware_id = "carcar_bt_monitor";

    bool have_log;
    bool action_active;
    bool action_status_received;
    rclcpp::Time last_log(0, 0, RCL_ROS_TIME);
    size_t mismatches;
    size_t unknown_statuses;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      have_log = have_log_;
      action_active = action_active_;
      action_status_received = action_status_received_;
      last_log = last_log_time_;
      mismatches = mismatched_node_names_.size();
      unknown_statuses = unknown_statuses_.size();
    }

    const double log_age = have_log ? (now - last_log).seconds() : -1.0;
    if (mismatches > 0 || unknown_statuses > 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "日志中的节点名或状态与显示树不匹配";
    } else if (!have_log) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "等待第一条行为树状态变化；此时 Groot 仍可显示静态拓扑";
    } else if (action_active && log_age > 10.0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "导航任务仍活动但 10 秒无状态变化；这不是断线结论，请结合 action 与 /rosout 判断";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = action_active ? "正在镜像真实行为树状态" : "最近会话已结束或尚未开始";
    }

    add_value(status, "bt_xml_path", bt_xml_path_);
    add_value(status, "xml_digest", xml_digest_);
    add_value(status, "zmq_publisher_port", std::to_string(zmq_pub_port_));
    add_value(status, "zmq_server_port", std::to_string(zmq_server_port_));
    add_value(status, "named_nodes", std::to_string(node_by_name_.size()));
    add_value(status, "has_bt_log", have_log ? "true" : "false");
    add_value(status, "action_status_received", action_status_received ? "true" : "false");
    add_value(status, "action_active", action_active ? "true" : "false");
    add_value(status, "seconds_since_last_transition", std::to_string(log_age));
    add_value(status, "mismatched_node_names", std::to_string(mismatches));
    add_value(status, "unknown_statuses", std::to_string(unknown_statuses));
    add_value(status, "source", "/behavior_tree_log 的真实状态变化；显示树不执行业务逻辑");

    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = now;
    output.status.push_back(std::move(status));
    diagnostics_pub_->publish(output);
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(std::move(entry));
  }

  std::string bt_xml_path_;
  std::string xml_digest_;
  unsigned zmq_pub_port_{1666};
  unsigned zmq_server_port_{1667};
  unsigned max_messages_per_second_{25};
  std::string bt_log_topic_;
  std::string action_status_topic_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
  std::map<std::string, BT::TreeNode *> node_by_name_;
  std::map<std::string, BT::NodeStatus> status_by_name_;
  std::mutex status_mutex_;
  std::set<std::string> mismatched_node_names_;
  std::set<std::string> unknown_statuses_;
  bool have_log_{false};
  bool action_active_{false};
  bool action_status_received_{false};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BtMonitorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("bt_monitor_node"), "启动 Groot 桥接失败：%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
