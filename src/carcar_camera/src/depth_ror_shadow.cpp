#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "carcar_camera/radius_outlier_shadow.hpp"

namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
using Point = pcl::PointXYZ;
using Points = pcl::PointCloud<Point>;

void add_value(diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

Cloud to_ros_cloud(const std_msgs::msg::Header & header, const Points & points)
{
  Cloud output;
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

class DepthRorShadow : public rclcpp::Node
{
public:
  DepthRorShadow() : Node("depth_ror_shadow")
  {
    radius_m_ = declare_parameter("radius_m", 0.05);
    min_neighbors_ = declare_parameter("min_neighbors", 2);
    if (!std::isfinite(radius_m_) || radius_m_ <= 0.0 || radius_m_ > 0.20 ||
      min_neighbors_ < 1 || min_neighbors_ > 64) {
      throw std::invalid_argument("ROR shadow 参数无效");
    }
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    output_ = create_publisher<Cloud>("/camera/navigation/obstacles_ror_shadow", qos);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/navigation/ror_shadow/diagnostics", rclcpp::QoS(10));
    input_ = create_subscription<Cloud>(
      "/camera/navigation/obstacles", qos,
      [this](Cloud::ConstSharedPtr msg) {on_cloud(*msg);});
  }

private:
  void on_cloud(const Cloud & msg)
  {
    const auto began = std::chrono::steady_clock::now();
    auto points = Points::Ptr(new Points);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
      points->reserve(static_cast<size_t>(msg.width) * msg.height);
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          points->emplace_back(*x, *y, *z);
        }
      }
      points->width = static_cast<uint32_t>(points->size());
      points->height = 1;
      points->is_dense = true;
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "障碍点云缺少 XYZ 字段：%s", error.what());
      return;
    }

    const auto kept = carcar_camera::radius_outlier_shadow(points, radius_m_, min_neighbors_);
    output_->publish(to_ros_cloud(msg.header, kept));
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - began).count();

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435ObstacleRorShadow";
    status.hardware_id = "D435";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "SHADOW_ONLY";
    add_value(status, "input_points", std::to_string(points->size()));
    add_value(status, "kept_points", std::to_string(kept.size()));
    add_value(status, "removed_points", std::to_string(points->size() - kept.size()));
    add_value(status, "radius_m", std::to_string(radius_m_));
    add_value(status, "min_neighbors", std::to_string(min_neighbors_));
    add_value(status, "processing_ms", std::to_string(elapsed_ms));
    array.status.push_back(status);
    diagnostics_->publish(array);
  }

  double radius_m_;
  int min_neighbors_;
  rclcpp::Subscription<Cloud>::SharedPtr input_;
  rclcpp::Publisher<Cloud>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthRorShadow>());
  rclcpp::shutdown();
  return 0;
}
