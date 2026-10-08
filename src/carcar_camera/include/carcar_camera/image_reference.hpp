#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace carcar_camera {
// 仅显示：深度毫米映射到8位灰度，不输出测距数据或修改原图。
inline sensor_msgs::msg::CompressedImage image_reference(
  const sensor_msgs::msg::Image & input, bool depth, int max_width, double far_m, int quality)
{
  if (max_width < 1 || max_width > 4096 || !std::isfinite(far_m) || far_m <= 0 ||
    quality < 1 || quality > 100 || !input.width || !input.height ||
    input.width > 8192 || input.height > 8192)
  {throw std::invalid_argument("参照图参数或尺寸无效");}
  const size_t bytes_per_pixel = depth ? 2 : 3;
  if ((depth && (input.encoding != "16UC1" || input.is_bigendian)) ||
    (!depth && input.encoding != "rgb8" && input.encoding != "bgr8") ||
    input.step < size_t(input.width) * bytes_per_pixel ||
    input.data.size() != size_t(input.step) * input.height)
  {throw std::invalid_argument("参照图编码或缓冲无效");}
  cv::Mat pixels;
  if (depth) {
    pixels.create(input.height, input.width, CV_8UC1);
    for (size_t row = 0; row < input.height; ++row) {
      for (size_t col = 0; col < input.width; ++col) {
        uint16_t mm;
        std::memcpy(&mm, input.data.data() + row * input.step + col * 2, sizeof(mm));
        pixels.at<uint8_t>(row, col) = mm == 0 ? 0 : static_cast<uint8_t>(
          std::clamp(255.0 * (1.0 - mm / (far_m * 1000.0)), 0.0, 255.0));
      }
    }
  } else {
    const cv::Mat source(input.height, input.width, CV_8UC3,
      const_cast<uint8_t *>(input.data.data()), input.step);
    if (input.encoding == "rgb8") {cv::cvtColor(source, pixels, cv::COLOR_RGB2BGR);}
    else {pixels = source.clone();}
  }
  if (input.width > static_cast<unsigned>(max_width)) {
    cv::Mat resized;
    const auto height = std::max(1, int(uint64_t(input.height) * max_width / input.width));
    cv::resize(pixels, resized, cv::Size(max_width, height), 0, 0, cv::INTER_NEAREST);
    pixels = resized;
  }
  sensor_msgs::msg::CompressedImage result;
  result.header = input.header;
  result.format = depth ? "mono8; png compressed mono8" : "bgr8; jpeg compressed bgr8";
  const bool encoded = depth ?
    cv::imencode(".png", pixels, result.data, {cv::IMWRITE_PNG_COMPRESSION, 3}) :
    cv::imencode(".jpg", pixels, result.data, {cv::IMWRITE_JPEG_QUALITY, quality});
  if (!encoded) {throw std::runtime_error("参照图压缩失败");}
  return result;
}
}  // namespace carcar_camera
