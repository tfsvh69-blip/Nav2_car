#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "carcar_camera/image_reference.hpp"

class ImageReferencePreview : public rclcpp::Node
{
public:
  ImageReferencePreview() : Node("image_reference_preview")
  {
    depth_ = declare_parameter("depth", true);
    width_ = declare_parameter("max_width", 320);
    far_ = declare_parameter("far_m", 3.0);
    quality_ = declare_parameter("jpeg_quality", 70);
    const auto fps = declare_parameter("max_fps", 1.0);
    const auto input = declare_parameter("input_topic",
      std::string("/camera/camera/depth/image_rect_raw"));
    const auto output = declare_parameter("output_topic",
      std::string("/camera/debug/depth_reference/compressed"));
    if (!std::isfinite(fps) || fps <= 0 || fps > 6 || width_ < 1 || width_ > 4096 ||
      !std::isfinite(far_) || far_ <= 0 || quality_ < 1 || quality_ > 100 || input == output)
    {throw std::invalid_argument("参照图参数无效");}
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(input, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) {latest_ = std::move(image);});
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this]() {
      if (!latest_ || !publisher_->get_subscription_count()) {return;}
      const auto age = (now() - rclcpp::Time(latest_->header.stamp)).seconds();
      if (!std::isfinite(age) || age < -0.1 || age > 0.5) {return;}
      try {
        publisher_->publish(carcar_camera::image_reference(*latest_, depth_, width_, far_, quality_));
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      }
    });
    RCLCPP_INFO(get_logger(), "DISPLAY ONLY: %s -> %s, width <= %d, %.1f Hz",
      input.c_str(), output.c_str(), width_, fps);
  }
private:
  bool depth_;
  int width_, quality_;
  double far_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<ImageReferencePreview>());}
  catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
  rclcpp::shutdown();
  return 0;
}
