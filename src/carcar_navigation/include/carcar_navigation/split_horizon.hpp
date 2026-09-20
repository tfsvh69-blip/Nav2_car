#pragma once
#include <cmath>

namespace carcar_navigation {

// 前进用 sim_time，纯原地转用更短的 rotate_sim_time，避免车角被过长预演扫进障碍。
inline double horizon_for_command(
  double vx, double vy, double vtheta, double sim_time, double rotate_sim_time)
{
  if (std::abs(vx) < 1e-4 && std::abs(vy) < 1e-4 && std::abs(vtheta) > 1e-4) {
    return rotate_sim_time;
  }
  return sim_time;
}

}  // namespace carcar_navigation
