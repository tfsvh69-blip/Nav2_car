#pragma once

#include <cmath>
#include <stdexcept>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace carcar_camera {

inline pcl::PointCloud<pcl::PointXYZ> radius_outlier_shadow(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input,
  double radius_m, int min_neighbors)
{
  if (!input || !std::isfinite(radius_m) || radius_m <= 0.0 || radius_m > 0.20 ||
    min_neighbors < 1 || min_neighbors > 64) {
    throw std::invalid_argument("ROR shadow 参数或输入无效");
  }
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
