#include "carcar_camera/depth_cloud_filter_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(carcar_camera::make_depth_cloud_filter(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
