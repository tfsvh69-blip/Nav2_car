#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "carcar_camera/depth_logic.hpp"

namespace {
using Steady = std::chrono::steady_clock;

struct Xyz {
  float x;
  float y;
  float z;
};

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = value;
  status.values.push_back(entry);
}

sensor_msgs::msg::PointCloud2 make_cloud(
  const std_msgs::msg::Header & header, const std::vector<Xyz> & points)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = header;
  output.height = 1;
  output.width = static_cast<uint32_t>(points.size());
  output.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  for (const auto & point : points) {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    ++x;
    ++y;
    ++z;
  }
  return output;
}
}  // namespace

class DepthCloudFilter : public rclcpp::Node
{
public:
  DepthCloudFilter()
  : Node("depth_cloud_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
    input_topic_ = declare_parameter("input_topic", std::string("/camera/depth/points"));
    limits_.min_range = declare_parameter("min_range", 0.30);
    limits_.max_range = declare_parameter("max_range", 2.00);
    limits_.max_clearing_range = declare_parameter("max_clearing_range", limits_.max_range);
    limits_.min_obstacle_height = declare_parameter("min_obstacle_height", 0.03);
    limits_.max_obstacle_height = declare_parameter("max_obstacle_height", 0.26);
    limits_.self_min_x = declare_parameter("self_min_x", -0.14);
    limits_.self_max_x = declare_parameter("self_max_x", 0.14);
    limits_.self_min_y = declare_parameter("self_min_y", -0.13);
    limits_.self_max_y = declare_parameter("self_max_y", 0.13);
    limits_.self_min_z = declare_parameter("self_min_z", 0.00);
    limits_.self_max_z = declare_parameter("self_max_z", 0.26);
    max_stamp_age_s_ = declare_parameter("max_stamp_age_s", 0.50);
    transform_timeout_s_ = declare_parameter("transform_timeout_s", 0.10);
    if (!carcar_camera::valid_limits(limits_) || max_stamp_age_s_ <= 0.0 ||
      transform_timeout_s_ <= 0.0) {
      throw std::invalid_argument("D435 点云筛选参数无效");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/navigation/obstacles", qos);
    clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/navigation/clearing", qos);
    healthy_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/camera/navigation/healthy", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/navigation/diagnostics", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&DepthCloudFilter::on_cloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&DepthCloudFilter::publish_health, this));
    publish_bool(false);
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const auto received = Steady::now();
    std::lock_guard<std::mutex> lock(mutex_);
    last_raw_received_ = received;
    last_frame_ = msg->header.frame_id;
    raw_points_ = static_cast<size_t>(msg->width) * msg->height;
    last_error_.clear();

    const double stamp_age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    last_stamp_age_s_ = stamp_age;
    if (!std::isfinite(stamp_age) || stamp_age < -0.10 || stamp_age > max_stamp_age_s_) {
      healthy_ = false;
      last_error_ = "POINTCLOUD_STAMP_EXPIRED";
      return;
    }
    if (msg->header.frame_id.empty()) {
      healthy_ = false;
      last_error_ = "POINTCLOUD_FRAME_EMPTY";
      return;
    }

    tf2::Transform base_from_sensor;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_s_));
      tf2::fromMsg(transform.transform, base_from_sensor);
    } catch (const tf2::TransformException & error) {
      healthy_ = false;
      last_error_ = std::string("TF_UNAVAILABLE: ") + error.what();
      return;
    }

    if (msg->point_step == 0 || msg->data.empty()) {
      obstacle_points_ = 0;
      clearing_points_ = 0;
      valid_measurements_ = 0;
      healthy_ = false;
      last_error_ = "POINTCLOUD_NO_VALID_DEPTH";
      obstacle_pub_->publish(make_cloud(msg->header, {}));
      clearing_pub_->publish(make_cloud(msg->header, {}));
      return;
    }

    std::vector<Xyz> obstacles;
    std::vector<Xyz> clearing;
    size_t valid_measurements = 0;
    obstacles.reserve(raw_points_ / 8);
    clearing.reserve(raw_points_);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (carcar_camera::valid_depth_measurement(*x, *y, *z)) {
          ++valid_measurements;
        }
        const tf2::Vector3 source(*x, *y, *z);
        const tf2::Vector3 base = base_from_sensor * source;
        const auto decision = carcar_camera::classify_point(
          *x, *y, *z, base.x(), base.y(), base.z(), limits_);
        if (decision.valid_for_clearing) {
          clearing.push_back({*x, *y, *z});
        }
        if (decision.obstacle) {
          obstacles.push_back({*x, *y, *z});
        }
      }
    } catch (const std::runtime_error & error) {
      healthy_ = false;
      last_error_ = std::string("POINTCLOUD_FORMAT_INVALID: ") + error.what();
      return;
    }

    obstacle_points_ = obstacles.size();
    clearing_points_ = clearing.size();
    valid_measurements_ = valid_measurements;
    healthy_ = valid_measurements > 0;
    if (healthy_) {
      last_valid_received_ = received;
    } else {
      last_error_ = "POINTCLOUD_NO_VALID_DEPTH";
    }
    obstacle_pub_->publish(make_cloud(msg->header, obstacles));
    clearing_pub_->publish(make_cloud(msg->header, clearing));
  }

  static double age(const Steady::time_point & point)
  {
    if (point == Steady::time_point{}) {
      return INFINITY;
    }
    return std::chrono::duration<double>(Steady::now() - point).count();
  }

  void publish_bool(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    healthy_pub_->publish(msg);
  }

  void publish_health()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double raw_age = age(last_raw_received_);
    const double valid_age = age(last_valid_received_);
    const bool effective = healthy_ && raw_age <= max_stamp_age_s_ &&
      valid_age <= max_stamp_age_s_;
    publish_bool(effective);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435DepthFilter";
    status.hardware_id = "D435";
    status.level = effective ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = effective ? "HEALTHY" : (last_error_.empty() ? "POINTCLOUD_TIMEOUT" : last_error_);
    add_value(status, "input_topic", input_topic_);
    add_value(status, "frame_id", last_frame_);
    add_value(status, "raw_age_s", std::to_string(raw_age));
    add_value(status, "valid_age_s", std::to_string(valid_age));
    add_value(status, "stamp_age_s", std::to_string(last_stamp_age_s_));
    add_value(status, "raw_points", std::to_string(raw_points_));
    add_value(status, "valid_measurements", std::to_string(valid_measurements_));
    add_value(status, "obstacle_points", std::to_string(obstacle_points_));
    add_value(status, "clearing_points", std::to_string(clearing_points_));
    add_value(status, "reason_code", effective ? "NONE" :
      (last_error_.empty() ? "POINTCLOUD_TIMEOUT" : last_error_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  carcar_camera::FilterLimits limits_;
  std::string base_frame_;
  std::string input_topic_;
  double max_stamp_age_s_{0.5};
  double transform_timeout_s_{0.1};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex mutex_;
  Steady::time_point last_raw_received_{};
  Steady::time_point last_valid_received_{};
  bool healthy_{false};
  double last_stamp_age_s_{INFINITY};
  size_t raw_points_{0};
  size_t valid_measurements_{0};
  size_t obstacle_points_{0};
  size_t clearing_points_{0};
  std::string last_frame_;
  std::string last_error_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
