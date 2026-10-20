#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace carcar_camera {

struct DepthPoint
{
  float x;
  float y;
  float z;
};

inline bool valid_voxel_leaf_size(double leaf_size)
{
  return std::isfinite(leaf_size) && leaf_size >= 0.001 && leaf_size <= 0.05;
}

// 仅接受已完成高度/自体/时间筛选的标障点；清障射线端点不经过此函数。
// 网格在输入光学坐标系中划分，输出仍是同一坐标系里的真实点质心。
inline std::vector<DepthPoint> downsample_obstacles(
  const std::vector<DepthPoint> & points, double leaf_size)
{
  if (!valid_voxel_leaf_size(leaf_size)) {
    throw std::invalid_argument("VoxelGrid leaf size must be in [0.001, 0.05] m");
  }
  if (points.empty()) {return {};}
  auto input = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  input->reserve(points.size());
  for (const auto & point : points) {input->emplace_back(point.x, point.y, point.z);}
  input->is_dense = true;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  const float leaf = static_cast<float>(leaf_size);
  filter.setLeafSize(leaf, leaf, leaf);
  // 不设最少点数：孤立的小障碍点仍保留，不把降采样变成离群点删除。
  pcl::PointCloud<pcl::PointXYZ> output;
  filter.filter(output);
  std::vector<DepthPoint> result;
  result.reserve(output.size());
  for (const auto & point : output) {result.push_back({point.x, point.y, point.z});}
  return result;
}

}  // namespace carcar_camera
