#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// RViz 专用限流预览。保留输入帧、时间戳、字段及字节序，不供导航使用。
class DepthCloudPreview : public rclcpp::Node
{
public:
  DepthCloudPreview() : Node("depth_cloud_preview")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/camera/depth/points");
    const auto output = declare_parameter<std::string>("output_topic", "/camera/debug/points");
    const auto max_points = declare_parameter<int>("max_points", 8000);
    const auto max_fps = declare_parameter<double>("max_fps", 1.0);
    if (max_points < 1 || !std::isfinite(max_fps) || max_fps <= 0.0 || input == output) {
      throw std::invalid_argument("Preview requires positive limits and distinct topics");
    }
    max_points_ = static_cast<size_t>(max_points);
    interval_ = std::chrono::duration<double>(1.0 / max_fps);
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(input, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {preview(*msg);});
    RCLCPP_INFO(get_logger(), "RViz ONLY: at most %zu points at %.2f Hz; navigation unchanged",
      max_points_, max_fps);
  }

private:
  void preview(const sensor_msgs::msg::PointCloud2 & input)
  {
    if (publisher_->get_subscription_count() == 0) {return;}
    const auto current = std::chrono::steady_clock::now();
    if (sent_ && current - last_sent_ < interval_) {return;}
    const size_t count = static_cast<size_t>(input.width) * input.height;
    const size_t row_bytes = static_cast<size_t>(input.width) * input.point_step;
    const size_t required = static_cast<size_t>(input.height) * input.row_step;
    if (input.point_step == 0 || input.row_step < row_bytes ||
      input.data.size() < required)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected malformed cloud");
      return;
    }
    // 空帧也是有效的场景更新；转发以让预览清除上一帧，避免零点数减一溢出。
    if (count == 0) {
      publisher_->publish(input);
      last_sent_ = current;
      sent_ = true;
      return;
    }
    const size_t stride = (count - 1) / max_points_ + 1;
    const size_t selected = (count - 1) / stride + 1;
    const size_t bytes = selected * input.point_step;
    if (bytes > UINT32_MAX) {return;}
    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.is_dense = input.is_dense;
    output.point_step = input.point_step;
    output.height = 1;
    output.width = static_cast<uint32_t>(selected);
    output.row_step = static_cast<uint32_t>(bytes);
    output.data.resize(bytes);
    size_t destination = 0;
    for (size_t index = 0; index < count; index += stride) {
      const size_t source = (index / input.width) * input.row_step +
        (index % input.width) * input.point_step;
      std::copy_n(input.data.begin() + source, input.point_step,
        output.data.begin() + destination);
      destination += input.point_step;
    }
    publisher_->publish(std::move(output));
    last_sent_ = current;
    sent_ = true;
  }

  size_t max_points_{};
  bool sent_{false};
  std::chrono::duration<double> interval_{};
  std::chrono::steady_clock::time_point last_sent_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthCloudPreview>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_cloud_preview"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
