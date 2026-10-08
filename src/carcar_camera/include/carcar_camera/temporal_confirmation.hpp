#pragma once

// 标障点云的时间确认（击中计数）。
//
// 背景：Nav2 的 VoxelLayer 只靠射线清除，而射线终点落在物体表面，
// 所以“物体表面之后”的体素永远没有射线穿过，一旦被标记就不会消失。
// 单帧深度跳变（轮廓 mixed pixel、偶发飞点）恰好会把格子标在表面之内，
// 于是变成永久黄色残留。半径滤波和官方时域滤波实测都压不住（见测试记录 R14）。
//
// 做法：一个 0.02 m 体素必须在最近 window_frames 帧内被命中至少 min_hits 次，
// 才允许进入 /camera/navigation/obstacles。清障点云不做这个处理，
// 保持全密度，避免削弱射线清障能力。
//
// 不依赖 ROS/PCL，便于单元测试。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "carcar_camera/flicker_logic.hpp"

namespace carcar_camera {

class TemporalConfirmation
{
public:
  TemporalConfirmation(double resolution, int window_frames, int min_hits)
  : resolution_(resolution), window_frames_(window_frames), min_hits_(min_hits)
  {
  }

  bool valid() const
  {
    return valid_resolution(resolution_) && window_frames_ >= 1 && window_frames_ <= 60 &&
           min_hits_ >= 1 && min_hits_ <= window_frames_;
  }

  // 每帧开始时调用一次：清掉窗口外的历史体素，保证内存有界。
  void begin_frame(int64_t frame_index)
  {
    const int64_t oldest = frame_index - window_frames_ + 1;
    for (auto it = hits_.begin(); it != hits_.end(); ) {
      if (it->second.empty() || it->second.back() < oldest) {
        it = hits_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // 返回 true 表示该点所在体素已达到确认门槛，允许发布。
  // 同一帧内多个点落在同一体素只记一次。
  bool accept(double x, double y, double z, int64_t frame_index)
  {
    const VoxelKey key = voxel_key(x, y, z, resolution_);
    auto & hits = hits_[key];
    if (hits.empty() || hits.back() != frame_index) {
      hits.push_back(frame_index);
    }
    const int64_t oldest = frame_index - window_frames_ + 1;
    size_t drop = 0;
    while (drop < hits.size() && hits[drop] < oldest) {++drop;}
    if (drop > 0) {hits.erase(hits.begin(), hits.begin() + static_cast<long>(drop));}
    return static_cast<int>(hits.size()) >= min_hits_;
  }

  size_t tracked_voxels() const {return hits_.size();}

  int window_frames() const {return window_frames_;}
  int min_hits() const {return min_hits_;}

private:
  double resolution_{0.02};
  int window_frames_{3};
  int min_hits_{2};
  std::map<VoxelKey, std::vector<int64_t>> hits_;
};

}  // namespace carcar_camera
