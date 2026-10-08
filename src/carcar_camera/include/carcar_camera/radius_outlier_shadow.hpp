#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace carcar_camera {

inline void validate_radius_outlier_args(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  double radius_m, int min_neighbors)
{
  if (!input || !std::isfinite(radius_m) || radius_m <= 0.0 || radius_m > 0.20 ||
    min_neighbors < 1 || min_neighbors > 64) {
    throw std::invalid_argument("ROR shadow 参数或输入无效");
  }
}

// 逐点保留掩码：与输入同序，true 表示该点被保留。
// 参数扫描需要按点判定，不能靠坐标回查（重复坐标和浮点相等都不可靠）。
inline std::vector<bool> radius_outlier_shadow_mask(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  double radius_m, int min_neighbors)
{
  validate_radius_outlier_args(input, radius_m, min_neighbors);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(radius_m);
  filter.setMinNeighborsInRadius(min_neighbors);
  std::vector<int> kept_indices;
  filter.filter(kept_indices);
  std::vector<bool> mask(input->size(), false);
  for (const int index : kept_indices) {
    if (index >= 0 && static_cast<size_t>(index) < mask.size()) {mask[index] = true;}
  }
  return mask;
}

inline pcl::PointCloud<pcl::PointXYZ> radius_outlier_shadow(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  double radius_m, int min_neighbors)
{
  validate_radius_outlier_args(input, radius_m, min_neighbors);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(radius_m);
  // PCL 的邻居数量不包括查询点本身。
  filter.setMinNeighborsInRadius(min_neighbors);
  pcl::PointCloud<pcl::PointXYZ> output;
  filter.filter(output);
  return output;
}

}  // namespace carcar_camera
