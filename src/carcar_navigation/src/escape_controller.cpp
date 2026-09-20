// 已停用：接手前草稿，仅保留审计。无恢复预算/数据年龄保护，不构建、不导出、不启动。
// 替代入口：行为树 SafeBackUp + ProtectedBackUp。
#include "carcar_navigation/escape_policy.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/utils.h"
#include <cmath>
#include <optional>

namespace carcar_navigation
{

class EscapeRotationShimController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent;
    plugin_name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    auto node = parent.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".escape_backup_speed", rclcpp::ParameterValue(0.05));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".escape_rotate_speed", rclcpp::ParameterValue(0.30));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".escape_simulate_time", rclcpp::ParameterValue(0.40));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".inner_controller",
      rclcpp::ParameterValue(std::string("nav2_rotation_shim_controller::RotationShimController")));
    node->get_parameter(name + ".escape_backup_speed", backup_speed_);
    node->get_parameter(name + ".escape_rotate_speed", rotate_speed_);
    node->get_parameter(name + ".escape_simulate_time", simulate_time_);
    std::string inner_type;
    node->get_parameter(name + ".inner_controller", inner_type);
    inner_ = loader_.createUniqueInstance(inner_type);
    inner_->configure(parent, name, tf, costmap_ros);
    checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
      costmap_ros_->getCostmap());
  }

  void cleanup() override {if (inner_) {inner_->cleanup();}}
  void activate() override {if (inner_) {inner_->activate();}}
  void deactivate() override {if (inner_) {inner_->deactivate();}}
  void setPlan(const nav_msgs::msg::Path & path) override {if (inner_) {inner_->setPlan(path);}}
  void setSpeedLimit(const double & limit, const bool & percentage) override
  {
    if (inner_) {inner_->setSpeedLimit(limit, percentage);}
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    try {
      return inner_->computeVelocityCommands(pose, velocity, goal_checker);
    } catch (const std::exception & ex) {
      auto escaped = tryEscape(pose, ex.what());
      if (escaped.has_value()) {return escaped.value();}
      throw;
    }
  }

private:
  bool inCollision(double x, double y, double yaw)
  {
    using namespace nav2_costmap_2d;
    checker_->setCostmap(costmap_ros_->getCostmap());
    const double cost = checker_->footprintCostAtPose(x, y, yaw, costmap_ros_->getRobotFootprint());
    return cost >= static_cast<double>(LETHAL_OBSTACLE) ||
           cost == static_cast<double>(NO_INFORMATION);
  }

  bool rotationClear(const geometry_msgs::msg::PoseStamped & pose, double w)
  {
    const double yaw0 = tf2::getYaw(pose.pose.orientation);
    const double dt = 0.1;
    for (double t = dt; t <= simulate_time_ + 1e-6; t += dt) {
      if (inCollision(pose.pose.position.x, pose.pose.position.y, yaw0 + w * t)) {
        return false;
      }
    }
    return true;
  }

  bool backupClear(const geometry_msgs::msg::PoseStamped & pose)
  {
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double step = 0.04;
    const double dist = std::max(0.08, backup_speed_ * simulate_time_);
    for (double d = step; d <= dist + 1e-6; d += step) {
      const double x = pose.pose.position.x - std::cos(yaw) * d;
      const double y = pose.pose.position.y - std::sin(yaw) * d;
      if (inCollision(x, y, yaw)) {return false;}
    }
    return true;
  }

  std::optional<geometry_msgs::msg::TwistStamped> tryEscape(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & reason)
  {
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const bool colliding = inCollision(pose.pose.position.x, pose.pose.position.y, yaw);
    const auto action = choose_escape(
      colliding, backupClear(pose),
      rotationClear(pose, rotate_speed_), rotationClear(pose, -rotate_speed_));
    if (action == EscapeAction::None) {return std::nullopt;}
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    if (action == EscapeAction::Backup) {
      cmd.twist.linear.x = -std::abs(backup_speed_);
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "包络已压致死格，发布倒车逃生 (%.2f m/s): %s", cmd.twist.linear.x, reason.c_str());
    } else {
      cmd.twist.angular.z = (action == EscapeAction::RotatePositive) ? rotate_speed_ : -rotate_speed_;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "包络已压致死格且后方不可退，发布短转逃生 (%.2f rad/s): %s", cmd.twist.angular.z, reason.c_str());
    }
    return cmd;
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("EscapeRotationShim")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  pluginlib::ClassLoader<nav2_core::Controller> loader_{"nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr inner_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> checker_;
  double backup_speed_{0.05};
  double rotate_speed_{0.30};
  double simulate_time_{0.40};
};

}  // namespace carcar_navigation

PLUGINLIB_EXPORT_CLASS(
  carcar_navigation::EscapeRotationShimController, nav2_core::Controller)
