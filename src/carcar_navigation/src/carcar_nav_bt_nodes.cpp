// NAV-007/NAV-008: 本项目 Nav2 行为树节点。
// 这些节点不发布速度。RecoverLocalization 只调用 AMCL 服务，RearClear 只读
// 代价地图、扫描和 TF；Spin/BackUp/Wait 的实际运动统一由 behavior_server 完成。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
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

class HealthSubscription
{
public:
  HealthSubscription(const BT::NodeConfiguration & config, const std::string & topic)
  : node_(nav_node(config)),
    last_stamp_(0, 0, RCL_ROS_TIME)
  {
    rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
    latching_qos.reliable();
    latching_qos.transient_local();

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic, latching_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        received_ = true;
        healthy_ = msg->data;
        last_stamp_ = node_->now();
      }, sub_option);
  }

  void update()
  {
    callback_group_executor_.spin_some();
  }

  bool has_received() const { return received_; }
  bool raw_healthy() const { return healthy_; }
  rclcpp::Time last_stamp() const { return last_stamp_; }

  bool is_fresh(double max_age = 1.0) const
  {
    if (!received_) {
      return false;
    }
    return (node_->now() - last_stamp_).seconds() <= max_age;
  }

  bool healthy() const
  {
    if (!received_ || !healthy_) {
      return false;
    }
    // 超过 1.0 秒未收到门控心跳，判定为监控节点断流或异常
    return is_fresh(1.0);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool received_{false};
  bool healthy_{false};
  rclcpp::Time last_stamp_;
};

// NAV-008/NAV-009: 等待定位健康就绪节点
// 默认在导航任务启动时等待最多 2 秒以获取健康信号。健康返回 SUCCESS，
// 不健康返回 FAILURE 进入恢复分支，超时无消息返回 FAILURE。
class WaitForLocalizationStatus : public BT::StatefulActionNode
{
public:
  WaitForLocalizationStatus(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
    node_(nav_node(config)),
    topic_(resolve_topic("health_topic", "status_topic", "/localization_monitor/ready")),
    health_(config, topic_)
  {
    getInput("timeout", timeout_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("health_topic", "/localization_monitor/ready", "健康状态话题 (std_msgs/msg/Bool)"),
      BT::InputPort<std::string>("status_topic", "", "旧版话题兼容别名"),
      BT::InputPort<double>("timeout", 2.0, "等待健康信号超时时间，单位秒")
    };
  }

private:
  std::string resolve_topic(
    const std::string & primary_key, const std::string & fallback_key,
    const std::string & default_val)
  {
    std::string val;
    if (getInput(primary_key, val) && !val.empty()) {
      return val;
    }
    if (!fallback_key.empty() && getInput(fallback_key, val) && !val.empty()) {
      return val;
    }
    return default_val;
  }

  BT::NodeStatus onStart() override
  {
    getInput("timeout", timeout_);
    deadline_ = node_->now() + rclcpp::Duration::from_seconds(timeout_);
    health_.update();

    if (health_.has_received() && health_.is_fresh(1.0)) {
      if (health_.raw_healthy()) {
        RCLCPP_INFO(node_->get_logger(), "定位健康就绪 (topic: %s)，允许规划跟随", topic_.c_str());
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(node_->get_logger(), "定位监控报告未就绪 (topic: %s)，进入恢复分支", topic_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    if (node_->now() >= deadline_) {
      RCLCPP_ERROR(node_->get_logger(), "等待定位健康信号超时 (%.1f s, topic: %s)", timeout_, topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "等待定位健康信号 (topic: %s, 预算 %.1f s)...", topic_.c_str(), timeout_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    health_.update();
    const auto now = node_->now();

    if (health_.has_received() && health_.is_fresh(1.0)) {
      if (health_.raw_healthy()) {
        RCLCPP_INFO(node_->get_logger(), "定位健康就绪 (topic: %s)，允许规划跟随", topic_.c_str());
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_WARN(node_->get_logger(), "定位监控报告未就绪 (topic: %s)，进入恢复分支", topic_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    if (now >= deadline_) {
      RCLCPP_ERROR(node_->get_logger(), "等待定位健康信号超时 (%.1f s, topic: %s)", timeout_, topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "WaitForLocalizationStatus 等待被中止 (halted)");
  }

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  HealthSubscription health_;
  double timeout_{2.0};
  rclcpp::Time deadline_{0, 0, RCL_ROS_TIME};
};

class LocalizationHealthy : public BT::ConditionNode
{
public:
  LocalizationHealthy(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    health_(config, resolve_topic("health_topic", "status_topic", "/localization_monitor/ready")) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("health_topic", "/localization_monitor/ready"),
      BT::InputPort<std::string>("status_topic", "", "旧版话题兼容别名")
    };
  }

private:
  std::string resolve_topic(
    const std::string & primary_key, const std::string & fallback_key,
    const std::string & default_val)
  {
    std::string val;
    if (getInput(primary_key, val) && !val.empty()) {
      return val;
    }
    if (!fallback_key.empty() && getInput(fallback_key, val) && !val.empty()) {
      return val;
    }
    return default_val;
  }

  BT::NodeStatus tick() override
  {
    health_.update();
    return health_.healthy() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  HealthSubscription health_;
};

class RecoverLocalization : public BT::StatefulActionNode
{
public:
  RecoverLocalization(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
    node_(nav_node(config)),
    health_(config, resolve_topic("health_topic", "status_topic", "/localization_monitor/ready")),
    recovery_allowed_(config, resolve_topic(
      "recovery_allowed_topic", "", "/localization_monitor/recovery_allowed"))
  {
    nomotion_client_ = node_->create_client<std_srvs::srv::Empty>(
      input_string("nomotion_service", "/request_nomotion_update"));
    global_client_ = node_->create_client<std_srvs::srv::Empty>(
      input_string("global_service", "/reinitialize_global_localization"));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("health_topic", "/localization_monitor/ready"),
      BT::InputPort<std::string>("status_topic", "", "旧版话题兼容别名"),
      BT::InputPort<std::string>(
        "recovery_allowed_topic", "/localization_monitor/recovery_allowed"),
      BT::InputPort<std::string>("nomotion_service", "/request_nomotion_update"),
      BT::InputPort<std::string>("global_service", "/reinitialize_global_localization"),
      BT::InputPort<double>("local_timeout", 10.0, "局部静止更新预算，单位秒"),
      BT::InputPort<double>("global_timeout", 20.0, "全局重定位预算，单位秒")
    };
  }

private:
  BT::NodeStatus onStart() override
  {
    health_.update();
    recovery_allowed_.update();
    if (health_.healthy()) {
      return BT::NodeStatus::SUCCESS;
    }
    if (!recovery_allowed_.healthy()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "定位输入链路不完整或已过期：不请求 AMCL 恢复，保持停车并结束导航任务");
      return BT::NodeStatus::FAILURE;
    }
    getInput("local_timeout", local_timeout_);
    getInput("global_timeout", global_timeout_);
    global_requested_ = false;
    local_deadline_ = node_->now() + rclcpp::Duration::from_seconds(local_timeout_);
    request_nomotion_update();
    RCLCPP_WARN(node_->get_logger(), "定位门控未通过：请求 AMCL 静止更新，最多等待 %.1f 秒", local_timeout_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    health_.update();
    recovery_allowed_.update();
    if (health_.healthy()) {
      RCLCPP_INFO(node_->get_logger(), "定位健康恢复，继续原导航目标并重新规划");
      return BT::NodeStatus::SUCCESS;
    }
    if (!recovery_allowed_.healthy()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "定位恢复期间扫描、里程计或扫描时刻 TF 失效：停止恢复并保持停车");
      return BT::NodeStatus::FAILURE;
    }

    const auto now = node_->now();
    if (!global_requested_ && now >= local_deadline_) {
      global_requested_ = true;
      global_deadline_ = now + rclcpp::Duration::from_seconds(global_timeout_);
      request_global_localization();
      request_nomotion_update();
      RCLCPP_WARN(node_->get_logger(), "局部定位恢复超时：请求 AMCL 全局重定位，最多等待 %.1f 秒", global_timeout_);
      return BT::NodeStatus::RUNNING;
    }

    if (global_requested_ && now >= global_deadline_) {
      RCLCPP_ERROR(node_->get_logger(), "AMCL 全局重定位超时，保持停车并结束当前导航任务");
      return BT::NodeStatus::FAILURE;
    }

    if ((now - last_nomotion_request_).seconds() >= 0.5) {
      request_nomotion_update();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

  std::string resolve_topic(
    const std::string & primary_key, const std::string & fallback_key,
    const std::string & default_val)
  {
    std::string val;
    if (getInput(primary_key, val) && !val.empty()) {
      return val;
    }
    if (!fallback_key.empty() && getInput(fallback_key, val) && !val.empty()) {
      return val;
    }
    return default_val;
  }

  std::string input_string(const std::string & key, const std::string & fallback)
  {
    std::string value = fallback;
    getInput(key, value);
    return value;
  }

  void request_nomotion_update()
  {
    last_nomotion_request_ = node_->now();
    if (nomotion_client_->service_is_ready()) {
      nomotion_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    }
  }

  void request_global_localization()
  {
    if (global_client_->service_is_ready()) {
      global_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "AMCL 全局重定位服务不可用：/reinitialize_global_localization");
    }
  }

  rclcpp::Node::SharedPtr node_;
  HealthSubscription health_;
  HealthSubscription recovery_allowed_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_client_;
  rclcpp::Time local_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time global_deadline_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nomotion_request_{0, 0, RCL_ROS_TIME};
  double local_timeout_{10.0};
  double global_timeout_{20.0};
  bool global_requested_{false};
};

class RearClear : public BT::ConditionNode
{
public:
  RearClear(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config),
    node_(nav_node(config)),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    costmap_topic_ = input_string("costmap_topic", "/local_costmap/costmap_raw");
    scan_topic_ = input_string("scan_topic", "/scan");
    getInput("max_data_age", max_data_age_);
    getInput("backup_distance", backup_distance_);

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav2_msgs::msg::Costmap::SharedPtr msg) { costmap_ = msg; }, sub_option);
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_stamp_ = msg->header.stamp; have_scan_ = true; }, sub_option);

    status_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/rear_clear/status", rclcpp::QoS(10).reliable());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("costmap_topic", "/local_costmap/costmap_raw"),
      BT::InputPort<std::string>("scan_topic", "/scan"),
      BT::InputPort<double>("max_data_age", 0.5, "扫描与代价地图最大允许年龄，单位秒"),
      BT::InputPort<double>("backup_distance", 0.15, "后退距离，单位米")
    };
  }

private:
  void publish_result(bool passed, const std::string & reason, const std::string & detail = "")
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "RearClear";
    status.level = passed ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = reason;
    status.hardware_id = "carcar_nav_bt_nodes";
    if (!detail.empty()) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "detail";
      kv.value = detail;
      status.values.push_back(kv);
    }
    if (status_pub_) {
      status_pub_->publish(status);
    }
    if (!passed) {
      RCLCPP_WARN(node_->get_logger(), "[RearClear] 拒绝倒车: 原因=%s (%s)", reason.c_str(), detail.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "[RearClear] 后方检查通过: 净空正常，允许受限倒车 (%.2f m)", backup_distance_);
    }
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();
    if (!costmap_ || !have_scan_ || costmap_->data.empty()) {
      publish_result(false, "缺少数据", "未收到扫描或代价地图数据");
      return BT::NodeStatus::FAILURE;
    }
    const auto now = node_->now();
    const double scan_age = (now - rclcpp::Time(last_scan_stamp_)).seconds();
    const double costmap_age = (now - rclcpp::Time(costmap_->header.stamp)).seconds();
    if (scan_age < 0.0 || scan_age > max_data_age_ || costmap_age < 0.0 || costmap_age > max_data_age_) {
      std::ostringstream ss;
      ss << "扫描年龄=" << std::fixed << std::setprecision(2) << scan_age
         << "s, 代价地图年龄=" << costmap_age << "s (门限=" << max_data_age_ << "s)";
      publish_result(false, "数据过期", ss.str());
      return BT::NodeStatus::FAILURE;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        costmap_->header.frame_id, "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      const double yaw = tf2::getYaw(transform.transform.rotation);
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      constexpr double half_length = 0.14;
      constexpr double half_width = 0.13;
      constexpr double margin = 0.05;
      const auto & metadata = costmap_->metadata;
      if (metadata.resolution <= 0.0F || metadata.size_x == 0 || metadata.size_y == 0) {
        publish_result(false, "缺少数据", "代价地图元数据无效");
        return BT::NodeStatus::FAILURE;
      }

      const double step = std::max(0.02, static_cast<double>(metadata.resolution));
      for (double local_x = -(half_length + margin);
        local_x >= -(half_length + backup_distance_ + margin); local_x -= step)
      {
        for (double local_y = -(half_width + margin); local_y <= half_width + margin; local_y += step) {
          const double world_x = transform.transform.translation.x + cos_yaw * local_x - sin_yaw * local_y;
          const double world_y = transform.transform.translation.y + sin_yaw * local_x + cos_yaw * local_y;
          const int map_x = static_cast<int>((world_x - metadata.origin.position.x) / metadata.resolution);
          const int map_y = static_cast<int>((world_y - metadata.origin.position.y) / metadata.resolution);
          if (map_x < 0 || map_y < 0 || map_x >= static_cast<int>(metadata.size_x) ||
            map_y >= static_cast<int>(metadata.size_y))
          {
            publish_result(false, "超出地图", "回退轨迹栅格坐标超出代价地图边界");
            return BT::NodeStatus::FAILURE;
          }
          const size_t index = static_cast<size_t>(map_y) * metadata.size_x + static_cast<size_t>(map_x);
          if (index >= costmap_->data.size()) {
            publish_result(false, "超出地图", "栅格数据索引越界");
            return BT::NodeStatus::FAILURE;
          }
          const uint8_t val = costmap_->data[index];
          if (val == 255U) {
            publish_result(false, "未知区域", "后方回退带包含未知区域 (NO_INFORMATION)");
            return BT::NodeStatus::FAILURE;
          }
          if (val >= 253U) {
            publish_result(false, "障碍占用", "后方回退带检测到障碍或内切代价 (cost=" + std::to_string(static_cast<int>(val)) + ")");
            return BT::NodeStatus::FAILURE;
          }
        }
      }
    } catch (const tf2::TransformException & e) {
      publish_result(false, "TF不可用", std::string("坐标变换查询失败: ") + e.what());
      return BT::NodeStatus::FAILURE;
    }
    publish_result(true, "通过", "后方检查通过");
    return BT::NodeStatus::SUCCESS;
  }

  std::string input_string(const std::string & key, const std::string & fallback)
  {
    std::string value = fallback;
    getInput(key, value);
    return value;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::string costmap_topic_;
  std::string scan_topic_;
  double max_data_age_{0.5};
  double backup_distance_{0.15};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_;
  bool have_scan_{false};
  builtin_interfaces::msg::Time last_scan_stamp_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<WaitForLocalizationStatus>("WaitForLocalizationStatus");
  factory.registerNodeType<LocalizationHealthy>("LocalizationHealthy");
  factory.registerNodeType<RecoverLocalization>("RecoverLocalization");
  factory.registerNodeType<RearClear>("RearClear");
}
