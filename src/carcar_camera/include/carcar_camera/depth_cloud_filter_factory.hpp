#pragma once

#include "rclcpp/rclcpp.hpp"

namespace carcar_camera {
rclcpp::Node::SharedPtr make_depth_cloud_filter(const rclcpp::NodeOptions & options);
}  // namespace carcar_camera
