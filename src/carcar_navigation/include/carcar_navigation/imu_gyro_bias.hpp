#pragma once

#include <cmath>
#include <string>

namespace carcar_navigation
{
struct GyroBiasResult
{
  bool valid{false};
  double corrected_z{0.0};
  std::string reason;
};

inline GyroBiasResult correct_gyro_z(double raw_z, double bias_z, double max_abs_z)
{
  if (!std::isfinite(raw_z) || !std::isfinite(bias_z) ||
    !std::isfinite(max_abs_z) || max_abs_z <= 0.0)
  {
    return {false, 0.0, "NONFINITE_OR_INVALID_PARAMETER"};
  }
  const double corrected = raw_z - bias_z;
  if (!std::isfinite(corrected) || std::abs(corrected) > max_abs_z) {
    return {false, 0.0, "ANGULAR_VELOCITY_OUT_OF_RANGE"};
  }
  return {true, corrected, "OK"};
}
}  // namespace carcar_navigation
