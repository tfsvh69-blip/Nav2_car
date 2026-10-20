#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "carcar_camera/depth_logic.hpp"
#include "carcar_camera/depth_cloud_filter_factory.hpp"
#include "carcar_camera/obstacle_voxel_grid.hpp"
#include "carcar_camera/temporal_confirmation.hpp"

namespace {
using Steady = std::chrono::steady_clock;

using Xyz = carcar_camera::DepthPoint;

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

namespace carcar_camera {
class DepthCloudFilter : public rclcpp::Node
{
public:
  explicit DepthCloudFilter(const rclcpp::NodeOptions & options)
  : Node("depth_cloud_filter", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
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
    voxel_enabled_ = declare_parameter("voxel_grid_enable", false);
    voxel_leaf_size_ = declare_parameter("voxel_leaf_size", 0.01);
    // 时间确认默认关闭，保证加入本功能前后行为完全一致；开启后才生效。
    temporal_enabled_ = declare_parameter("temporal_confirm_enable", false);
    const auto confirm_resolution = declare_parameter("temporal_confirm_resolution", 0.02);
    const auto confirm_window = declare_parameter("temporal_confirm_window_frames", 3);
    const auto confirm_min_hits = declare_parameter("temporal_confirm_min_hits", 2);
    confirmation_ = carcar_camera::TemporalConfirmation(
      confirm_resolution, confirm_window, confirm_min_hits);
    if (!carcar_camera::valid_limits(limits_) || max_stamp_age_s_ <= 0.0 ||
      transform_timeout_s_ <= 0.0 || !valid_voxel_leaf_size(voxel_leaf_size_) ||
      (temporal_enabled_ && !confirmation_.valid())) {
      throw std::invalid_argument("D435 点云筛选参数无效");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/navigation/obstacles", qos);
    clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/navigation/clearing", qos);
    // Humble 的进程内通信不支持 Transient Local；保留停车联锁的持久健康状态。
    rclcpp::PublisherOptions health_options;
    health_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    healthy_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/camera/navigation/healthy", rclcpp::QoS(1).reliable().transient_local(), health_options);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/navigation/diagnostics", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        on_cloud(msg, info);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&DepthCloudFilter::publish_health, this));
    publish_bool(false);
    RCLCPP_INFO(get_logger(), "IPC=%s, obstacle VoxelGrid=%s, leaf=%.3fm (CPU PCL)",
      options.use_intra_process_comms() ? "true" : "false",
      voxel_enabled_ ? "true" : "false", voxel_leaf_size_);
  }

private:
  void on_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const rclcpp::MessageInfo & info)
  {
    const auto received = Steady::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (info.get_rmw_message_info().from_intra_process) {++intra_process_frames_;}
    else {++inter_process_frames_;}
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
    size_t rejected_by_confirmation = 0;
    if (temporal_enabled_) {confirmation_.begin_frame(++frame_index_);}
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
          // 清障点云保持全密度；只有标障点云需要时间确认，避免单帧跳变
          // 在物体表面之内留下射线永远清不掉的永久黄色体素。
          if (temporal_enabled_ &&
            !confirmation_.accept(base.x(), base.y(), base.z(), frame_index_))
          {
            ++rejected_by_confirmation;
          } else {
            obstacles.push_back({*x, *y, *z});
          }
        }
      }
    } catch (const std::runtime_error & error) {
      healthy_ = false;
      last_error_ = std::string("POINTCLOUD_FORMAT_INVALID: ") + error.what();
      return;
    }

    obstacle_points_before_voxel_ = obstacles.size();
    const auto voxel_started = Steady::now();
    if (voxel_enabled_) {obstacles = downsample_obstacles(obstacles, voxel_leaf_size_);}
    voxel_filter_ms_ = std::chrono::duration<double, std::milli>(
      Steady::now() - voxel_started).count();
    obstacle_points_ = obstacles.size();
    clearing_points_ = clearing.size();
    valid_measurements_ = valid_measurements;
    rejected_points_ = rejected_by_confirmation;
    tracked_voxels_ = temporal_enabled_ ? confirmation_.tracked_voxels() : 0;
    healthy_ = valid_measurements > 0;
    if (healthy_) {
      last_valid_received_ = received;
    } else {
      last_error_ = "POINTCLOUD_NO_VALID_DEPTH";
    }
    obstacle_pub_->publish(
      std::make_unique<sensor_msgs::msg::PointCloud2>(make_cloud(msg->header, obstacles)));
    // STVL 当前不订阅清障点云时，避免白白构建大缓冲；计数和筛选仍照常执行。
    if (clearing_pub_->get_subscription_count() > 0 ||
      clearing_pub_->get_intra_process_subscription_count() > 0)
    {
      clearing_pub_->publish(
        std::make_unique<sensor_msgs::msg::PointCloud2>(make_cloud(msg->header, clearing)));
    }
    processing_ms_ = std::chrono::duration<double, std::milli>(Steady::now() - received).count();
    ++processed_frames_;
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
    add_value(status, "obstacle_points_before_voxel", std::to_string(obstacle_points_before_voxel_));
    add_value(status, "voxel_grid_enable", voxel_enabled_ ? "true" : "false");
    add_value(status, "voxel_leaf_size", std::to_string(voxel_leaf_size_));
    add_value(status, "voxel_filter_ms", std::to_string(voxel_filter_ms_));
    add_value(status, "processing_ms", std::to_string(processing_ms_));
    add_value(status, "processed_frames", std::to_string(processed_frames_));
    add_value(status, "intra_process_comms",
      get_node_options().use_intra_process_comms() ? "true" : "false");
    add_value(status, "process_id", std::to_string(getpid()));
    add_value(status, "intra_process_frames", std::to_string(intra_process_frames_));
    add_value(status, "inter_process_frames", std::to_string(inter_process_frames_));
    add_value(status, "compute_backend", "CPU_PCL");
    add_value(status, "clearing_points", std::to_string(clearing_points_));
    add_value(status, "temporal_confirm_enable", temporal_enabled_ ? "true" : "false");
    add_value(status, "temporal_rejected_points", std::to_string(rejected_points_));
    add_value(status, "temporal_tracked_voxels", std::to_string(tracked_voxels_));
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
  size_t obstacle_points_before_voxel_{0};
  size_t processed_frames_{0};
  size_t intra_process_frames_{0};
  size_t inter_process_frames_{0};
  bool voxel_enabled_{false};
  double voxel_leaf_size_{0.01};
  double voxel_filter_ms_{0.0};
  double processing_ms_{0.0};
  size_t clearing_points_{0};
  size_t rejected_points_{0};
  size_t tracked_voxels_{0};
  bool temporal_enabled_{false};
  int64_t frame_index_{0};
  carcar_camera::TemporalConfirmation confirmation_{0.02, 3, 2};
  std::string last_frame_;
  std::string last_error_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

rclcpp::Node::SharedPtr make_depth_cloud_filter(const rclcpp::NodeOptions & options)
{
  return std::make_shared<DepthCloudFilter>(options);
}
}  // namespace carcar_camera

RCLCPP_COMPONENTS_REGISTER_NODE(carcar_camera::DepthCloudFilter)
