#pragma once
#include <cmath>
#include <stdexcept>
#include <sensor_msgs/msg/camera_info.hpp>

namespace carcar_camera {
// 只用于显示目标网格；CameraInfo的binning由image_geometry统一解释。
inline sensor_msgs::msg::CameraInfo reference_camera_info(
  const sensor_msgs::msg::CameraInfo & input, int max_width)
{
  if (max_width < 1 || !input.width || !input.height ||
    !std::isfinite(input.k[0]) || !std::isfinite(input.k[4]) ||
    !std::isfinite(input.p[0]) || !std::isfinite(input.p[5]) || input.k[0] <= 0 ||
    input.k[4] <= 0 || input.p[0] <= 0 || input.p[5] <= 0 ||
    input.binning_x > 1 || input.binning_y > 1 || input.roi.width || input.roi.height ||
    input.roi.x_offset || input.roi.y_offset || input.roi.do_rectify) {
    throw std::invalid_argument("仅支持已标定、无裁剪的原生RGB内参");
  }
  const unsigned factor = input.width <= static_cast<unsigned>(max_width) ? 1 :
    input.width / static_cast<unsigned>(max_width);
  if ((input.width > static_cast<unsigned>(max_width) &&
    input.width % static_cast<unsigned>(max_width)) ||
    input.height % factor) {
    throw std::invalid_argument("显示目标必须是原生尺寸的整数缩放");
  }
  auto output = input;
  output.binning_x = factor;
  output.binning_y = factor;
  return output;
}
}  // namespace carcar_camera
