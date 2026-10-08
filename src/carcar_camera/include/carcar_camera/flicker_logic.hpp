#pragma once

// 离线闪烁审计的纯计算部分：体素分格、出现序列的连续段统计、直方图分箱。
// 不依赖 ROS/PCL，便于单元测试。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace carcar_camera {

struct VoxelKey {
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator<(const VoxelKey & other) const
  {
    if (x != other.x) {return x < other.x;}
    if (y != other.y) {return y < other.y;}
    return z < other.z;
  }

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

inline bool valid_resolution(double resolution)
{
  return std::isfinite(resolution) && resolution > 0.0 && resolution <= 1.0;
}

// 与 Nav2 体素层一致，使用向下取整把连续坐标映射到整数格。
inline int32_t axis_index(double value, double resolution)
{
  return static_cast<int32_t>(std::floor(value / resolution));
}

inline VoxelKey voxel_key(double x, double y, double z, double resolution)
{
  return VoxelKey{
    axis_index(x, resolution), axis_index(y, resolution), axis_index(z, resolution)};
}

inline double voxel_center(int32_t index, double resolution)
{
  return (static_cast<double>(index) + 0.5) * resolution;
}

// 无效输入的哨兵值。不能用 -1：横向剖面在 y<0 一侧的合法分箱本身就是负数。
constexpr int kInvalidBin = INT32_MIN;

// 把一维值映射到直方图分箱；无效输入返回 kInvalidBin，合法分箱可为负数。
inline bool valid_bin_input(double value, double origin, double width)
{
  return std::isfinite(value) && std::isfinite(origin) && width > 0.0;
}

inline int bin_index(double value, double origin, double width)
{
  if (!valid_bin_input(value, origin, width)) {return kInvalidBin;}
  return static_cast<int>(std::floor((value - origin) / width));
}

// 给定某体素的出现样本序号（必须已升序去重），返回连续出现段数。
// 段数 1 表示整个过程连续存在；大于 1 表示中途消失过，即发生了闪烁。
inline int count_runs(const std::vector<int64_t> & ascending_samples)
{
  if (ascending_samples.empty()) {return 0;}
  int runs = 1;
  for (size_t i = 1; i < ascending_samples.size(); ++i) {
    if (ascending_samples[i] != ascending_samples[i - 1] + 1) {++runs;}
  }
  return runs;
}

// 状态翻转次数：一段连续存在有两次翻转，末段若延续到观测结束只算开始的一次。
inline int count_transitions(const std::vector<int64_t> & ascending_samples, int64_t total_samples)
{
  if (ascending_samples.empty()) {return 0;}
  const int runs = count_runs(ascending_samples);
  const bool ends_present = ascending_samples.back() == total_samples - 1;
  return ends_present ? (2 * runs - 1) : (2 * runs);
}

// 把出现/消失序列压缩成 "on3,off5,on2" 形式，便于写入 CSV 直接判读。
inline std::string run_length_encode(
  const std::vector<int64_t> & ascending_samples, int64_t total_samples)
{
  if (total_samples <= 0) {return {};}
  std::string text;
  int64_t cursor = 0;
  size_t i = 0;
  while (i < ascending_samples.size()) {
    const int64_t start = ascending_samples[i];
    if (start > cursor) {
      if (!text.empty()) {text += ',';}
      text += "off" + std::to_string(start - cursor);
    }
    int64_t end = start;
    while (i + 1 < ascending_samples.size() && ascending_samples[i + 1] == end + 1) {
      ++i;
      ++end;
    }
    if (!text.empty()) {text += ',';}
    text += "on" + std::to_string(end - start + 1);
    cursor = end + 1;
    ++i;
  }
  if (cursor < total_samples) {
    if (!text.empty()) {text += ',';}
    text += "off" + std::to_string(total_samples - cursor);
  }
  return text;
}

}  // namespace carcar_camera
