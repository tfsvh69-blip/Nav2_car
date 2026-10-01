#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace carcar_camera {

struct FilterLimits {
  double min_range{0.30};
  double max_range{2.00};
  double max_clearing_range{2.00};
  double min_obstacle_height{0.03};
  double max_obstacle_height{0.26};
  double self_min_x{-0.14};
  double self_max_x{0.14};
  double self_min_y{-0.13};
  double self_max_y{0.13};
  double self_min_z{0.00};
  double self_max_z{0.26};
};

struct PointDecision {
  bool valid_for_clearing{false};
  bool obstacle{false};
};

inline bool valid_depth_measurement(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0F;
}

inline bool valid_limits(const FilterLimits & l)
{
  return std::isfinite(l.min_range) && std::isfinite(l.max_range) &&
    std::isfinite(l.max_clearing_range) && l.max_clearing_range >= l.max_range &&
    std::isfinite(l.min_obstacle_height) && std::isfinite(l.max_obstacle_height) &&
    l.min_range >= 0.0 && l.max_range > l.min_range &&
    l.max_obstacle_height > l.min_obstacle_height &&
    l.self_max_x > l.self_min_x && l.self_max_y > l.self_min_y &&
    l.self_max_z > l.self_min_z;
}

inline PointDecision classify_point(
  float sensor_x, float sensor_y, float sensor_z,
  double base_x, double base_y, double base_z, const FilterLimits & l)
{
  PointDecision result;
  if (!valid_depth_measurement(sensor_x, sensor_y, sensor_z) ||
    !std::isfinite(base_x) || !std::isfinite(base_y) || !std::isfinite(base_z)) {
    return result;
  }
  const double range = std::sqrt(
    static_cast<double>(sensor_x) * sensor_x +
    static_cast<double>(sensor_y) * sensor_y +
    static_cast<double>(sensor_z) * sensor_z);
  if (range < l.min_range || range > l.max_clearing_range) {
    return result;
  }
  result.valid_for_clearing = true;
  const bool in_self = base_x >= l.self_min_x && base_x <= l.self_max_x &&
    base_y >= l.self_min_y && base_y <= l.self_max_y &&
    base_z >= l.self_min_z && base_z <= l.self_max_z;
  result.obstacle = range <= l.max_range && !in_self && base_z >= l.min_obstacle_height &&
    base_z <= l.max_obstacle_height;
  return result;
}

enum class FusionState {LASER_ONLY, RECOVERING, FUSED};

inline const char * state_name(FusionState state)
{
  switch (state) {
    case FusionState::LASER_ONLY: return "LASER_ONLY";
    case FusionState::RECOVERING: return "RECOVERING";
    case FusionState::FUSED: return "FUSED";
  }
  return "LASER_ONLY";
}

class HealthStateMachine {
public:
  explicit HealthStateMachine(double recovery_s = 3.0) : recovery_s_(recovery_s) {}

  FusionState update(bool healthy, double now)
  {
    if (!std::isfinite(now)) {
      healthy_since_ = -1.0;
      state_ = FusionState::LASER_ONLY;
      return state_;
    }
    if (!healthy) {
      healthy_since_ = -1.0;
      state_ = FusionState::LASER_ONLY;
      return state_;
    }
    if (state_ == FusionState::FUSED) {
      return state_;
    }
    if (healthy_since_ < 0.0 || now < healthy_since_) {
      healthy_since_ = now;
    }
    state_ = now - healthy_since_ >= recovery_s_ ?
      FusionState::FUSED : FusionState::RECOVERING;
    return state_;
  }

  void force(FusionState state, double now)
  {
    state_ = state;
    healthy_since_ = state == FusionState::RECOVERING ? now : -1.0;
  }

  FusionState state() const {return state_;}

private:
  double recovery_s_{3.0};
  double healthy_since_{-1.0};
  FusionState state_{FusionState::LASER_ONLY};
};

}  // namespace carcar_camera
