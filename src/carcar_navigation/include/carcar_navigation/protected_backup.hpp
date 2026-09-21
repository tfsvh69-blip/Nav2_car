// Copyright (c) 2026
// NAV-012: 倒车周期安全防护插件 ProtectedBackUp
// 继承自 Nav2 DriveOnHeading<BackUpAction>，在每个执行周期高频校验激光新鲜度、
// 里程计新鲜度、后方扇区点云避障，并对单段后退距离与速度进行硬件保护。

#ifndef CARCAR_NAVIGATION__PROTECTED_BACKUP_HPP_
#define CARCAR_NAVIGATION__PROTECTED_BACKUP_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <chrono>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "carcar_navigation/recovery_runtime.hpp"

namespace carcar_navigation
{

using BackUpAction = nav2_msgs::action::BackUp;

class ProtectedBackUp : public nav2_behaviors::DriveOnHeading<BackUpAction>
{
public:
  ProtectedBackUp();
  ~ProtectedBackUp() override = default;

  void onConfigure() override;
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  SensorSnapshot safety_data_;

  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_recv_time_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point scan_received_{},odom_received_{},deadline_{};
  std::string base_frame_{"base_footprint"};
  std::string odom_topic_{"/wheel/odometry"};

  double max_data_age_{0.5};
  double rear_clearance_limit_{0.08};
  double half_length_{0.14};
  double requested_distance_{kMaxSingleBackupDistance};
};

}  // namespace carcar_navigation

#endif  // CARCAR_NAVIGATION__PROTECTED_BACKUP_HPP_
