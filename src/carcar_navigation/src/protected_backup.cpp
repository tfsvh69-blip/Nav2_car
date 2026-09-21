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
  node->get_parameter_or("odom_topic",odom_topic_,odom_topic_);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&ProtectedBackUp::on_scan, this, std::placeholders::_1));

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
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
  this->stopRobot();
  auto modified_cmd = std::make_shared<BackUpAction::Goal>(*command);
  if (!std::isfinite(command->target.x) || !std::isfinite(command->target.y) ||
    !std::isfinite(command->target.z) || !std::isfinite(command->speed) ||
    command->target.y!=0 || command->target.z!=0 || command->target.x==0 || command->speed<=0) {
    this->stopRobot();return nav2_behaviors::Status::FAILED;
  }
  // BackUp 的契约始终为负 X；DriveOnHeading 本身不会替我们校正方向。
  requested_distance_=std::min(kMaxSingleBackupDistance,std::abs(command->target.x));
  modified_cmd->target.x=-requested_distance_;
  const double budget=rclcpp::Duration(command->time_allowance).seconds();
  if (budget<=0 || budget>kMaxBackupTimeAllowance) {this->stopRobot();return nav2_behaviors::Status::FAILED;}
  deadline_=carcar_navigation::after(budget);
  modified_cmd->speed=-std::min(0.05,std::abs(static_cast<double>(modified_cmd->speed)));

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto check=swept_clear(safety_data_,*this->tf_,this->clock_->now(),base_frame_,
      -(requested_distance_+rear_clearance_limit_),0,max_data_age_);
    if (!check.ok) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_,"[ProtectedBackUp] 启动复核拒绝: %s: %s",
        check.code.c_str(),check.detail.c_str());
      return nav2_behaviors::Status::FAILED;
    }
  }

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

  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(pose,*this->tf_,this->global_frame_,
      this->robot_base_frame_,this->transform_tolerance_)) {
    this->stopRobot();return nav2_behaviors::Status::FAILED;
  }
  const double yaw=tf2::getYaw(initial_pose_.pose.orientation);
  const double dx=pose.pose.position.x-initial_pose_.pose.position.x;
  const double dy=pose.pose.position.y-initial_pose_.pose.position.y;
  const double traveled=-(std::cos(yaw)*dx+std::sin(yaw)*dy);
  const double lateral=-std::sin(yaw)*dx+std::cos(yaw)*dy;
  if (!std::isfinite(traveled) || !std::isfinite(lateral) ||
      !std::isfinite(tf2::getYaw(pose.pose.orientation)) || traveled < -0.02 ||
      std::abs(lateral)>0.03 || std::abs(normalize(tf2::getYaw(pose.pose.orientation)-yaw))>0.10) {
    this->stopRobot();return nav2_behaviors::Status::FAILED;
  }
  feedback_->distance_traveled=std::max(0.0,traveled);
  this->action_server_->publish_feedback(feedback_);
  if (traveled>=requested_distance_) {
    this->stopRobot();return nav2_behaviors::Status::SUCCEEDED;
  }

  // 1. 数据时效性与生命周期校验
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!last_scan_ || !last_odom_ ||
        scan_received_==std::chrono::steady_clock::time_point{} ||
        odom_received_==std::chrono::steady_clock::time_point{}) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "[ProtectedBackUp] 尚未收到激光扫描数据，安全停止倒车");
      return nav2_behaviors::Status::FAILED;
    }

    // 在锁内取时，避免回调刚写入更新的 recv 时刻后，锁外旧 now 算出负年龄。
    const auto freshness_now = this->clock_->now();
    const double scan_stamp_age = (freshness_now - last_scan_stamp_).seconds();
    const double scan_recv_age = (freshness_now - last_scan_recv_time_).seconds();
    const double scan_steady_age = carcar_navigation::seconds(scan_received_);
    const double odom_steady_age = carcar_navigation::seconds(odom_received_);
    if (scan_steady_age>max_data_age_ || odom_steady_age>max_data_age_ ||
        !carcar_navigation::ros_age_fresh(scan_stamp_age,max_data_age_) ||
        !carcar_navigation::ros_age_fresh(scan_recv_age,max_data_age_))
    {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "[ProtectedBackUp] 扫描数据过期 (stamp_age=%.3fs, recv_age=%.3fs, "
        "steady_age=%.3fs, limit=%.2fs)，安全中止倒车",
        scan_stamp_age, scan_recv_age, scan_steady_age, max_data_age_);
      return nav2_behaviors::Status::FAILED;
    }

    const double odom_stamp_age = (freshness_now - last_odom_stamp_).seconds();
    if (!carcar_navigation::ros_age_fresh(odom_stamp_age,max_data_age_)) {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "[ProtectedBackUp] 里程计数据过期 (odom_age=%.3fs, limit=%.2fs)，安全中止倒车",
        odom_stamp_age, max_data_age_);
      return nav2_behaviors::Status::FAILED;
    }

    // 2. 与 BT 后方检查共用完整包络扫掠；后退期间新出现的内部障碍也必须中止。
    auto check=swept_clear(safety_data_,*this->tf_,now,base_frame_,
      -(requested_distance_-std::max(0.0,traveled)+rear_clearance_limit_),0,max_data_age_);
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

  // Humble 基类从 t=0 再次检查轮廓，会拒绝离开既有车头碰撞格。
  // 以上完整扫掠替代该检查；仍复用 TimedBehavior 的取消、退出停车和 Action 生命周期。
  // 外部速度平滑器负责加减速，最终运动联锁继续约束这一路输出。
  auto cmd=std::make_unique<geometry_msgs::msg::Twist>();
  cmd->linear.x=command_speed_;
  this->vel_pub_->publish(std::move(cmd));
  return nav2_behaviors::Status::RUNNING;
}

}  // namespace carcar_navigation

PLUGINLIB_EXPORT_CLASS(carcar_navigation::ProtectedBackUp, nav2_core::Behavior)
