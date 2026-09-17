// 定位等待/恢复叶节点；运动由 behavior_server 与 controller_server 执行。

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include "carcar_navigation/recovery_runtime.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

rclcpp::Node::SharedPtr nav_node(const BT::NodeConfiguration & config)
{
  if (config.blackboard) {
    try {
      auto n = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
      if (n) return n;
    } catch (...) {}
  }
  static auto fallback_node = std::make_shared<rclcpp::Node>("carcar_nav_bt_fallback");
  return fallback_node;
}

// 定位健康等待；等待时限使用单调时钟。快照由 RecoveryRuntime 的执行线程填充。
class WaitForLocalizationStatus : public BT::StatefulActionNode
{
public:
  WaitForLocalizationStatus(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
    node_(nav_node(config)),
    runtime_(carcar_navigation::RecoveryRuntime::get(config))
  {
    getInput("timeout", timeout_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout", 2.0, "等待健康信号超时时间，单位秒")
    };
  }

private:
  BT::NodeStatus onStart() override
  {
    getInput("timeout", timeout_);
    if (!std::isfinite(timeout_) || timeout_<=0) {return BT::NodeStatus::FAILURE;}
    deadline_ = carcar_navigation::after(timeout_);
    return evaluate();
  }

  BT::NodeStatus onRunning() override {return evaluate();}

  BT::NodeStatus evaluate()
  {
    const auto health = runtime_->localization_ready();
    if (health.fresh(1.0)) {
      if (health.value) {
        RCLCPP_INFO(node_->get_logger(), "定位健康就绪，允许规划跟随");
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(node_->get_logger(), "定位监控报告未就绪，进入恢复分支");
      return BT::NodeStatus::FAILURE;
    }
    if (carcar_navigation::Steady::now() >= deadline_) {
      RCLCPP_ERROR(node_->get_logger(), "等待定位健康信号超时 (%.1f s)", timeout_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<carcar_navigation::RecoveryRuntime> runtime_;
  double timeout_{2.0};
  carcar_navigation::TimePoint deadline_{};
};

class LocalizationHealthy : public BT::ConditionNode
{
public:
  LocalizationHealthy(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    runtime_(carcar_navigation::RecoveryRuntime::get(config)) {}

  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override
  {
    return runtime_->healthy() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<carcar_navigation::RecoveryRuntime> runtime_;
};

class RecoverLocalization : public BT::StatefulActionNode
{
public:
  RecoverLocalization(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
    node_(nav_node(config)),
    runtime_(carcar_navigation::RecoveryRuntime::get(config))
  {
    nomotion_client_ = runtime_->node->create_client<std_srvs::srv::Empty>(
      input_string("nomotion_service", "/request_nomotion_update"),rmw_qos_profile_services_default,runtime_->group);
    global_client_ = runtime_->node->create_client<std_srvs::srv::Empty>(
      input_string("global_service", "/reinitialize_global_localization"),rmw_qos_profile_services_default,runtime_->group);
  }

  ~RecoverLocalization() override {clear_requests();}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("nomotion_service", "/request_nomotion_update"),
      BT::InputPort<std::string>("global_service", "/reinitialize_global_localization"),
      BT::InputPort<double>("local_timeout", 10.0, "局部静止更新预算，单位秒"),
      BT::InputPort<double>("global_timeout", 20.0, "全局重定位预算，单位秒")
    };
  }

private:
  BT::NodeStatus onStart() override
  {
    if (runtime_->healthy()) {
      return BT::NodeStatus::SUCCESS;
    }
    if (!runtime_->recovery_allowed().healthy()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "定位输入链路不完整或已过期：不请求 AMCL 恢复，保持停车并结束导航任务");
      return BT::NodeStatus::FAILURE;
    }
    getInput("local_timeout", local_timeout_);
    getInput("global_timeout", global_timeout_);
    if (!std::isfinite(local_timeout_) || !std::isfinite(global_timeout_) ||
      local_timeout_<=0 || local_timeout_>10 || global_timeout_<=0 || global_timeout_>20) {
      return BT::NodeStatus::FAILURE;
    }
    global_requested_ = false;
    local_deadline_ = carcar_navigation::after(local_timeout_);
    request_nomotion_update();
    RCLCPP_WARN(node_->get_logger(), "定位门控未通过：请求 AMCL 静止更新，最多等待 %.1f 秒", local_timeout_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    reap_requests();
    if (runtime_->healthy()) {
      RCLCPP_INFO(node_->get_logger(), "定位健康恢复，继续原导航目标并重新规划");
      return BT::NodeStatus::SUCCESS;
    }
    if (!runtime_->recovery_allowed().healthy()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "定位恢复期间扫描、里程计或扫描时刻 TF 失效：停止恢复并保持停车");
      return BT::NodeStatus::FAILURE;
    }

    const auto now = carcar_navigation::Steady::now();
    if (!global_requested_ && now >= local_deadline_) {
      global_requested_ = true;
      global_deadline_ = carcar_navigation::after(global_timeout_);
      request_global_localization();
      request_nomotion_update();
      RCLCPP_WARN(node_->get_logger(), "局部定位恢复超时：请求 AMCL 全局重定位，最多等待 %.1f 秒", global_timeout_);
      return BT::NodeStatus::RUNNING;
    }

    if (global_requested_ && now >= global_deadline_) {
      RCLCPP_ERROR(node_->get_logger(), "AMCL 全局重定位超时，保持停车并结束当前导航任务");
      return BT::NodeStatus::FAILURE;
    }

    if (carcar_navigation::seconds(last_nomotion_request_) >= 0.5) {
      request_nomotion_update();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {clear_requests();}

  std::string input_string(const std::string & key, const std::string & fallback)
  {
    std::string value = fallback;
    getInput(key, value);
    return value;
  }

  void request_nomotion_update()
  {
    last_nomotion_request_ = carcar_navigation::Steady::now();
    reap_requests();
    if (!nomotion_pending_ && nomotion_client_->service_is_ready()) {
      nomotion_pending_.emplace(nomotion_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>()));
      nomotion_sent_=carcar_navigation::Steady::now();
    }
  }

  void request_global_localization()
  {
    reap_requests();
    if (!global_pending_ && global_client_->service_is_ready()) {
      global_pending_.emplace(global_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>()));
      global_sent_=carcar_navigation::Steady::now();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "AMCL 全局重定位服务不可用：/reinitialize_global_localization");
    }
  }

  using Pending = rclcpp::Client<std_srvs::srv::Empty>::FutureAndRequestId;
  void reap(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client,
    std::optional<Pending> & request, carcar_navigation::TimePoint sent) {
    if (!request) return;
    if (request->future.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      request->future.get(); request.reset();
    } else if (carcar_navigation::seconds(sent)>1.0) {
      client->remove_pending_request(request->request_id); request.reset();
    }
  }
  void reap_requests() {
    reap(nomotion_client_,nomotion_pending_,nomotion_sent_);
    reap(global_client_,global_pending_,global_sent_);
  }
  void clear_requests() {
    if (nomotion_pending_) nomotion_client_->remove_pending_request(nomotion_pending_->request_id);
    if (global_pending_) global_client_->remove_pending_request(global_pending_->request_id);
    nomotion_pending_.reset();global_pending_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<carcar_navigation::RecoveryRuntime> runtime_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_client_;
  std::optional<Pending> nomotion_pending_,global_pending_;
  carcar_navigation::TimePoint local_deadline_{},global_deadline_{},last_nomotion_request_{};
  carcar_navigation::TimePoint nomotion_sent_{},global_sent_{};
  double local_timeout_{10.0};
  double global_timeout_{20.0};
  bool global_requested_{false};
};

}  // namespace
namespace carcar_navigation {
void register_nav012_nodes(BT::BehaviorTreeFactory & factory);
}
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<WaitForLocalizationStatus>("WaitForLocalizationStatus");
  factory.registerNodeType<LocalizationHealthy>("LocalizationHealthy");
  factory.registerNodeType<RecoverLocalization>("RecoverLocalization");
  carcar_navigation::register_nav012_nodes(factory);
}
