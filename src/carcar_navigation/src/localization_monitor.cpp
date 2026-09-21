// NAV-007: 静态地图定位健康监控。
// 本节点只读取扫描、所选里程计、AMCL、粒子、地图和 TF，
// 发布健康门控与 diagnostics；它不发布速度，不调用电机或串口。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{
double age_seconds(const rclcpp::Time & now, const builtin_interfaces::msg::Time & stamp)
{
  return (now - rclcpp::Time(stamp)).seconds();
}

bool finite_range(const float range, const sensor_msgs::msg::LaserScan & scan)
{
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}
}  // namespace

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor()
  : Node("localization_monitor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    ready_topic_ = this->declare_parameter<std::string>("ready_topic", "/localization_monitor/ready");
    fault_topic_ = this->declare_parameter<std::string>("fault_topic", "/localization_monitor/fault");
    recovery_allowed_topic_ = this->declare_parameter<std::string>(
      "recovery_allowed_topic", "/localization_monitor/recovery_allowed");
    max_data_age_ = this->declare_parameter<double>("max_data_age", 0.5);
    min_endpoint_count_ = this->declare_parameter<int>("min_endpoint_count", 12);
    min_hit_ratio_ = this->declare_parameter<double>("min_hit_ratio", 0.30);
    endpoint_match_radius_ = this->declare_parameter<double>("endpoint_match_radius", 0.15);
    min_particles_ = this->declare_parameter<int>("min_particles", 100);
    max_particle_spread_ = this->declare_parameter<double>("max_particle_spread", 0.75);
    required_healthy_cycles_ = this->declare_parameter<int>("required_healthy_cycles", 5);
    required_fault_cycles_ = this->declare_parameter<int>("required_fault_cycles", 3);
    scan_stride_ = this->declare_parameter<int>("scan_stride", 4);

    auto scan_qos = rclcpp::SensorDataQoS();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      std::bind(&LocalizationMonitor::on_scan, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LocalizationMonitor::on_odom, this, std::placeholders::_1));
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(10).reliable(),
      std::bind(&LocalizationMonitor::on_amcl_pose, this, std::placeholders::_1));
    particle_sub_ = this->create_subscription<nav2_msgs::msg::ParticleCloud>(
      "/particle_cloud", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitor::on_particles, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalizationMonitor::on_map, this, std::placeholders::_1));

    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(ready_topic_, rclcpp::QoS(1).reliable());
    fault_pub_ = this->create_publisher<std_msgs::msg::Bool>(fault_topic_, rclcpp::QoS(1).reliable());
    rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
    latching_qos.reliable();
    latching_qos.transient_local();

    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(ready_topic_, latching_qos);
    fault_pub_ = this->create_publisher<std_msgs::msg::Bool>(fault_topic_, latching_qos);
    recovery_allowed_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      recovery_allowed_topic_, latching_qos);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/localization_monitor/diagnostics", rclcpp::QoS(10).reliable());
    timer_ = this->create_wall_timer(200ms, std::bind(&LocalizationMonitor::evaluate, this));

    RCLCPP_INFO(
      this->get_logger(),
      "NAV-007 定位健康监控已启动：扫描新鲜度 <= %.2fs，端点命中率 >= %.2f，连续 %d 次才放行",
      max_data_age_, min_hit_ratio_, required_healthy_cycles_);
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    have_odom_ = true;
    last_odom_stamp_ = msg->header.stamp;
  }

  void on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    have_amcl_pose_ = true;
    last_amcl_stamp_ = msg->header.stamp;
  }

  void on_particles(const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
  {
    particle_count_ = static_cast<int>(msg->particles.size());
    if (msg->particles.empty()) {
      particle_spread_ = std::numeric_limits<double>::infinity();
      have_particles_ = false;
      return;
    }

    double weight_sum = 0.0;
    for (const auto & particle : msg->particles) {
      weight_sum += std::max(0.0, particle.weight);
    }
    const double default_weight = weight_sum > 1e-12 ? 0.0 : 1.0;
    if (weight_sum <= 1e-12) {
      weight_sum = static_cast<double>(msg->particles.size());
    }

    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto & particle : msg->particles) {
      const double weight = default_weight > 0.0 ? default_weight : std::max(0.0, particle.weight);
      mean_x += weight * particle.pose.position.x;
      mean_y += weight * particle.pose.position.y;
    }
    mean_x /= weight_sum;
    mean_y /= weight_sum;

    double variance = 0.0;
    for (const auto & particle : msg->particles) {
      const double weight = default_weight > 0.0 ? default_weight : std::max(0.0, particle.weight);
      const double dx = particle.pose.position.x - mean_x;
      const double dy = particle.pose.position.y - mean_y;
      variance += weight * (dx * dx + dy * dy);
    }
    particle_spread_ = std::sqrt(variance / weight_sum);
    have_particles_ = true;
  }

  bool occupied_near(const double x, const double y) const
  {
    if (!map_ || map_->info.resolution <= 0.0 || map_->data.empty()) {
      return false;
    }
    const auto & info = map_->info;
    const int center_x = static_cast<int>((x - info.origin.position.x) / info.resolution);
    const int center_y = static_cast<int>((y - info.origin.position.y) / info.resolution);
    const int radius = std::max(1, static_cast<int>(std::ceil(endpoint_match_radius_ / info.resolution)));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int cell_x = center_x + dx;
        const int cell_y = center_y + dy;
        if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(info.width) || cell_y >= static_cast<int>(info.height))
        {
          continue;
        }
        const auto index = static_cast<size_t>(cell_y) * info.width + static_cast<size_t>(cell_x);
        if (index < map_->data.size() && map_->data[index] >= 65) {
          return true;
        }
      }
    }
    return false;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    have_scan_ = true;
    last_scan_stamp_ = msg->header.stamp;
    last_scan_tf_valid_ = false;
    last_endpoint_count_ = 0;
    last_hit_ratio_ = 0.0;

    if (!map_ || msg->header.frame_id.empty()) {
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.0));
      const double yaw = tf2::getYaw(transform.transform.rotation);
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      int hits = 0;
      const int stride = std::max(1, scan_stride_);
      for (size_t i = 0; i < msg->ranges.size(); i += static_cast<size_t>(stride)) {
        const float range = msg->ranges[i];
        if (!finite_range(range, *msg) || range >= msg->range_max * 0.98F) {
          continue;
        }
        const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        const double local_x = range * std::cos(angle);
        const double local_y = range * std::sin(angle);
        const double map_x = transform.transform.translation.x + cos_yaw * local_x - sin_yaw * local_y;
        const double map_y = transform.transform.translation.y + sin_yaw * local_x + cos_yaw * local_y;
        ++last_endpoint_count_;
        if (occupied_near(map_x, map_y)) {
          ++hits;
        }
      }
      if (last_endpoint_count_ > 0) {
        last_hit_ratio_ = static_cast<double>(hits) / last_endpoint_count_;
      }
      last_scan_tf_valid_ = true;
    } catch (const tf2::TransformException & ex) {
      last_tf_error_ = ex.what();
    }
  }

  void evaluate()
  {
    const auto now = this->now();
    const double scan_age = have_scan_ ? age_seconds(now, last_scan_stamp_) : std::numeric_limits<double>::infinity();
    const double odom_age = have_odom_ ? age_seconds(now, last_odom_stamp_) : std::numeric_limits<double>::infinity();
    const bool scan_fresh = scan_age >= 0.0 && scan_age <= max_data_age_;
    const bool odom_fresh = odom_age >= 0.0 && odom_age <= max_data_age_;
    const bool map_alignment = last_scan_tf_valid_ &&
      last_endpoint_count_ >= min_endpoint_count_ && last_hit_ratio_ >= min_hit_ratio_;
    const bool particle_ok = have_particles_ && particle_count_ >= min_particles_ &&
      particle_spread_ <= max_particle_spread_;
    const bool candidate = have_amcl_pose_ && scan_fresh && odom_fresh && map_alignment && particle_ok;
    const bool source_chain_valid = have_amcl_pose_ && map_ && scan_fresh && odom_fresh &&
      last_scan_tf_valid_;

    if (candidate) {
      ++healthy_cycles_;
      fault_cycles_ = 0;
    } else {
      healthy_cycles_ = 0;
      ++fault_cycles_;
    }
    if (!source_chain_valid) {
      ready_latched_ = false;
    } else if (!ready_latched_ && healthy_cycles_ >= required_healthy_cycles_) {
      ready_latched_ = true;
    } else if (ready_latched_ && fault_cycles_ >= required_fault_cycles_) {
      ready_latched_ = false;
    }
    const bool ready = ready_latched_;

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = ready;
    ready_pub_->publish(ready_msg);
    std_msgs::msg::Bool fault_msg;
    fault_msg.data = !ready;
    fault_pub_->publish(fault_msg);
    std_msgs::msg::Bool recovery_allowed_msg;
    recovery_allowed_msg.data = source_chain_valid;
    recovery_allowed_pub_->publish(recovery_allowed_msg);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "localization_monitor";
    status.hardware_id = "carcar_navigation";
    status.level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = ready ? "定位健康：允许路径跟随" : failure_reason(scan_fresh, odom_fresh, map_alignment, particle_ok);
    add_value(status, "ready", ready ? "true" : "false");
    add_value(status, "recovery_allowed", source_chain_valid ? "true" : "false");
    add_value(status, "scan_topic", scan_topic_);
    add_value(status, "odom_topic", odom_topic_);
    add_value(status, "map_topic", map_topic_);
    add_value(status, "map_frame", map_ ? map_->header.frame_id : "");
    add_value(status, "scan_age_s", as_string(scan_age));
    add_value(status, "odom_age_s", as_string(odom_age));
    add_value(status, "amcl_pose_received", have_amcl_pose_ ? "true" : "false");
    add_value(status, "scan_tf_at_stamp", last_scan_tf_valid_ ? "true" : "false");
    add_value(status, "static_endpoint_count", std::to_string(last_endpoint_count_));
    add_value(status, "static_endpoint_hit_ratio", as_string(last_hit_ratio_));
    add_value(status, "particle_count", std::to_string(particle_count_));
    add_value(status, "particle_spread_m", as_string(particle_spread_));
    add_value(status, "healthy_cycles", std::to_string(healthy_cycles_));
    add_value(status, "fault_cycles", std::to_string(fault_cycles_));
    add_value(status, "tf_error", last_tf_error_);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    array.status.push_back(std::move(status));
    diag_pub_->publish(array);
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(std::move(entry));
  }

  static std::string as_string(const double value)
  {
    std::ostringstream output;
    output.setf(std::ios::fixed);
    output.precision(3);
    output << value;
    return output.str();
  }

  std::string failure_reason(bool scan_fresh, bool odom_fresh, bool map_alignment, bool particle_ok) const
  {
    if (!have_amcl_pose_) return "等待 AMCL 初始位姿";
    if (!scan_fresh) return "扫描数据过期：不执行自动重定位";
    if (!odom_fresh) return "所选里程计数据过期：不执行自动重定位";
    if (!last_scan_tf_valid_) return "扫描时刻 TF 不可用：不执行自动重定位";
    if (!map_alignment) return "激光端点与静态地图匹配不足";
    if (!particle_ok) return "AMCL 粒子尚未收敛或分布多解";
    return "等待连续定位健康样本";
  }

  std::string global_frame_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string ready_topic_;
  std::string fault_topic_;
  std::string recovery_allowed_topic_;
  double max_data_age_{0.5};
  int min_endpoint_count_{12};
  double min_hit_ratio_{0.30};
  double endpoint_match_radius_{0.15};
  int min_particles_{100};
  double max_particle_spread_{0.75};
  int required_healthy_cycles_{5};
  int required_fault_cycles_{3};
  int scan_stride_{4};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_allowed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool have_scan_{false};
  bool have_odom_{false};
  bool have_amcl_pose_{false};
  bool have_particles_{false};
  builtin_interfaces::msg::Time last_scan_stamp_;
  builtin_interfaces::msg::Time last_odom_stamp_;
  builtin_interfaces::msg::Time last_amcl_stamp_;
  bool last_scan_tf_valid_{false};
  int last_endpoint_count_{0};
  double last_hit_ratio_{0.0};
  int particle_count_{0};
  double particle_spread_{std::numeric_limits<double>::infinity()};
  int healthy_cycles_{0};
  int fault_cycles_{0};
  bool ready_latched_{false};
  std::string last_tf_error_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
