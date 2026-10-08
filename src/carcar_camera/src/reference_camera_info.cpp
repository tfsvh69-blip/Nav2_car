#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "carcar_camera/reference_camera_info.hpp"

class ReferenceCameraInfo : public rclcpp::Node
{
public:
  ReferenceCameraInfo() : Node("reference_camera_info")
  {
    const int width = declare_parameter("max_width", 320);
    if (width < 1 || width > 4096) {throw std::invalid_argument("显示宽度无效");}
    output_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/debug/rgb_reference/camera_info", rclcpp::QoS(1).reliable());
    input_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", rclcpp::SensorDataQoS().keep_last(1),
      [this, width](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        const double age = (now() - rclcpp::Time(info->header.stamp)).seconds();
        if (!std::isfinite(age) || age < -0.1 || age > 0.5) {return;}
        try {output_->publish(carcar_camera::reference_camera_info(*info, width));}
        catch (const std::exception & error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
        }
      });
  }
private:
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr output_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr input_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<ReferenceCameraInfo>());}
  catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
  rclcpp::shutdown();
  return 0;
}
