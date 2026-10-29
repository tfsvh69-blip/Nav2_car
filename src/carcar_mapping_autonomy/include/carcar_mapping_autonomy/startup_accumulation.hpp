#pragma once
#include <algorithm>
#include <cmath>
namespace carcar_mapping_autonomy {
// 对齐 slam_toolbox 2.6.10 shouldProcessScan：首帧放行，之后平移未达到
// 0.8 * minimum_travel_distance 的扫描被拒绝。这里只在该拒绝窗口内、
// 并且尚未移动出正常门槛时，有限补收几帧，让静止起步的射线能够累积。
struct StartupAccumulation {
  bool anchored = false;
  bool closed = false;
  int extras = 0;
  double anchor_x = 0;
  double anchor_y = 0;
  double anchor_time = 0;
  double last_accept_time = 0;
};
enum class StartupDecision { ignore, accept_extra };
inline StartupDecision consider_stationary_startup(
  StartupAccumulation & state, bool parent_accepted, bool paused,
  double x, double y, double stamp, double minimum_travel, double minimum_interval,
  int extra_limit, double window_seconds)
{
  const double gate = 0.8 * minimum_travel * minimum_travel;
  const auto moved = [&] {
    const double dx = x - state.anchor_x, dy = y - state.anchor_y;
    return dx * dx + dy * dy >= gate;
  };
  if (parent_accepted) {
    if (!state.anchored) {
      state.anchored = true;
      state.anchor_x = x;
      state.anchor_y = y;
      state.anchor_time = state.last_accept_time = stamp;
    } else {
      state.last_accept_time = stamp;
      if (moved()) state.closed = true;
    }
    return StartupDecision::ignore;
  }
  if (!state.anchored || state.closed || paused) return StartupDecision::ignore;
  if (moved() || state.extras >= std::max(0, extra_limit) ||
    stamp - state.anchor_time >= window_seconds)
  {
    state.closed = true;
    return StartupDecision::ignore;
  }
  if (!std::isfinite(stamp) || stamp - state.last_accept_time + 1e-4 < minimum_interval) return StartupDecision::ignore;
  ++state.extras;
  state.last_accept_time = stamp;
  return StartupDecision::accept_extra;
}
}  // namespace carcar_mapping_autonomy
