// Copyright (c) 2026
// NAV-012: 倒车周期安全防护插件 ProtectedBackUp 实现

#include "carcar_navigation/protected_backup.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "carcar_navigation/recovery_runtime.hpp"

namespace carcar_navigation
{

ProtectedBackUp::ProtectedBackUp()
: nav2_behaviors::DriveOnHeading<BackUpAction>()
{
}

void ProtectedBackUp::onConfigure()
{
  nav2_behaviors::DriveOnHeading<BackUpAction>::onConfigure();

  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".max_data_age", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".rear_clearance_limit", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, this->behavior_name_ + ".half_length", rclcpp::ParameterValue(0.14));

  node->get_parameter(this->behavior_name_ + ".max_data_age", max_data_age_);
  node->get_parameter(this->behavior_name_ + ".rear_clearance_limit", rear_clearance_limit_);
  node->get_parameter(this->behavior_name_ + ".half_length", half_length_);
  node->get_parameter_or("robot_base_frame",base_frame_,base_frame_);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&ProtectedBackUp::on_scan, this, std::placeholders::_1));

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/wheel/odometry", rclcpp::SensorDataQoS(),
    std::bind(&ProtectedBackUp::on_odom, this, std::placeholders::_1));
  costmap_sub_=node->create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap_raw",rclcpp::QoS(1).transient_local(),
    [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      safety_data_.costmap=msg;safety_data_.costmap_received=Steady::now();
    });
  footprint_sub_=node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/local_costmap/published_footprint",rclcpp::QoS(1),
    [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      safety_data_.footprint=msg;safety_data_.footprint_received=Steady::now();
    });

  RCLCPP_INFO(
    this->logger_,
    "[ProtectedBackUp] 已配置防护倒车插件: 最大数据年龄=%.2fs, 后方安全间隙=%.3fm, 车体半长=%.3fm",
    max_data_age_, rear_clearance_limit_, half_length_);
}

void ProtectedBackUp::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_scan_ = msg;
  last_scan_stamp_ = msg->header.stamp;
  last_scan_recv_time_ = this->clock_->now();
  scan_received_=std::chrono::steady_clock::now();
  safety_data_.scan=msg;safety_data_.scan_received=scan_received_;
}

void ProtectedBackUp::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_odom_ = msg;
  last_odom_stamp_ = msg->header.stamp;
  odom_received_=std::chrono::steady_clock::now();
}

nav2_behaviors::Status ProtectedBackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  auto modified_cmd = std::make_shared<BackUpAction::Goal>(*command);
  if (!std::isfinite(command->target.x) || !std::isfinite(command->target.y) ||
    !std::isfinite(command->target.z) || !std::isfinite(command->speed) ||
    command->target.y!=0 || command->target.z!=0 || command->target.x==0 || command->speed<=0) {
    this->stopRobot();return nav2_behaviors::Status::FAILED;
  }
  // BackUp 的契约始终为负 X；DriveOnHeading 本身不会替我们校正方向。
  modified_cmd->target.x=-std::min(0.10,std::abs(command->target.x));
  const double budget=rclcpp::Duration(command->time_allowance).seconds();
  if (budget<=0 || budget>4.0) {this->stopRobot();return nav2_behaviors::Status::FAILED;}
  deadline_=carcar_navigation::after(budget);
  modified_cmd->speed=-std::min(0.05,std::abs(static_cast<double>(modified_cmd->speed)));

  RCLCPP_INFO(
    this->logger_,
    "[ProtectedBackUp] 开始执行受控受限倒车: 目标距离=%.2fm, 速度=%.2fm/s",
    modified_cmd->target.x, modified_cmd->speed);

  return nav2_behaviors::DriveOnHeading<BackUpAction>::onRun(modified_cmd);
}

nav2_behaviors::Status ProtectedBackUp::onCycleUpdate()
{
  const auto now = this->clock_->now();
  if (std::chrono::steady_clock::now()>=deadline_) {
    this->stopRobot();return nav2_behaviors::Status::FAILED;
  }

  // 1. 数据时效性与生命周期校验
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!last_scan_ || !last_odom_) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "[ProtectedBackUp] 尚未收到激光扫描数据，安全停止倒车");
      return nav2_behaviors::Status::FAILED;
    }

    const double scan_stamp_age = (now - last_scan_stamp_).seconds();
    const double scan_recv_age = (now - last_scan_recv_time_).seconds();
    if (carcar_navigation::seconds(scan_received_)>max_data_age_ ||
        carcar_navigation::seconds(odom_received_)>max_data_age_ ||
        scan_stamp_age < 0.0 || scan_stamp_age > max_data_age_ ||
        scan_recv_age < 0.0 || scan_recv_age > max_data_age_)
    {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "[ProtectedBackUp] 扫描数据过期 (stamp_age=%.2fs, recv_age=%.2fs > %.2fs)，安全中止倒车",
        scan_stamp_age, scan_recv_age, max_data_age_);
      return nav2_behaviors::Status::FAILED;
    }

    if (last_odom_) {
      const double odom_stamp_age = (now - last_odom_stamp_).seconds();
      if (odom_stamp_age < 0.0 || odom_stamp_age > max_data_age_) {
        this->stopRobot();
        RCLCPP_WARN(
          this->logger_,
          "[ProtectedBackUp] 里程计数据过期 (odom_age=%.2fs > %.2fs)，安全中止倒车",
          odom_stamp_age, max_data_age_);
        return nav2_behaviors::Status::FAILED;
      }
    }

    // 2. 与 BT 后方检查共用完整包络扫掠；后退期间新出现的内部障碍也必须中止。
    auto check=swept_clear(safety_data_,*this->tf_,now,base_frame_,-0.10,0,max_data_age_);
    if (!check.ok) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_,"[ProtectedBackUp] %s: %s",check.code.c_str(),check.detail.c_str());
      return nav2_behaviors::Status::FAILED;
    }
    // 按扫描时刻安装 TF 变换后的车体后方近距离守护。
    const double danger_threshold = half_length_ + rear_clearance_limit_;
    std::vector<geometry_msgs::msg::Point> points;
    if (!carcar_navigation::scan_points_in_base(*last_scan_,*this->tf_,base_frame_,points)) {
      this->stopRobot();return nav2_behaviors::Status::FAILED;
    }
    for (const auto & point : points) {
      const double r=std::hypot(point.x,point.y);
      if (point.x<0) {
        if (point.x>=-danger_threshold && std::abs(point.y)<=0.16) {
          this->stopRobot();
          RCLCPP_WARN(
            this->logger_,
            "[ProtectedBackUp] 倒车中途后方扇区出现障碍点 (r=%.3fm <= %.3fm)，立即急停",
            r, danger_threshold);
          return nav2_behaviors::Status::FAILED;
        }
      }
    }
  }

  // 3. 调用基类执行代价地图碰撞模拟与速度发布
  return nav2_behaviors::DriveOnHeading<BackUpAction>::onCycleUpdate();
}

}  // namespace carcar_navigation

PLUGINLIB_EXPORT_CLASS(carcar_navigation::ProtectedBackUp, nav2_core::Behavior)
