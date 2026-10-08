#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace carcar_camera {
// 固定静止地图原生PointCloud2；不外推TF，不用显示抽样计数。
template<typename Visitor>
size_t visit_stvl_roi(const sensor_msgs::msg::PointCloud2 & cloud,
  const std::array<double, 6> & bounds, const Visitor & visitor)
{
  for (size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(bounds[axis * 2]) || !std::isfinite(bounds[axis * 2 + 1]) ||
      bounds[axis * 2] >= bounds[axis * 2 + 1])
    {throw std::invalid_argument("STVL ROI边界无效");}
  }
  if (cloud.header.frame_id != "base_footprint" || cloud.is_bigendian ||
    cloud.height != 1 || cloud.point_step < 12 ||
    cloud.row_step != size_t(cloud.width) * cloud.point_step ||
    cloud.data.size() != cloud.row_step)
  {throw std::runtime_error("STVL帧或缓冲格式无效，拒绝统计");}
  std::array<size_t, 3> offsets{};
  const std::array<const char *, 3> names{"x", "y", "z"};
  for (size_t axis = 0; axis < 3; ++axis) {
    const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [&](const auto & f) {return f.name == names[axis];});
    if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      field->count != 1 || size_t(field->offset) + sizeof(float) > cloud.point_step)
    {throw std::runtime_error("STVL XYZ字段无效");}
    offsets[axis] = field->offset;
  }
  size_t count = 0;
  for (size_t point = 0; point < cloud.width; ++point) {
    bool inside = true;
    std::array<float, 3> xyz{};
    for (size_t axis = 0; axis < 3; ++axis) {
      float value;
      std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offsets[axis], sizeof(value));
      if (!std::isfinite(value)) {throw std::runtime_error("STVL含非有限体素坐标");}
      inside = inside && value >= bounds[axis * 2] && value <= bounds[axis * 2 + 1];
      xyz[axis] = value;
    }
    if (inside) {visitor(xyz); ++count;}
  }
  return count;
}

inline size_t count_stvl_roi(const sensor_msgs::msg::PointCloud2 & cloud,
  const std::array<double, 6> & bounds)
{
  return visit_stvl_roi(cloud, bounds, [](const auto &) {});
}
}  // namespace carcar_camera
