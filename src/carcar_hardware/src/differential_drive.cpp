// Copyright 2026 carcar maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carcar_hardware/differential_drive.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace carcar_hardware
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

void require_positive_finite(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive and finite");
  }
}

void require_sign(int value, const char * name)
{
  if (value != -1 && value != 1) {
    throw std::invalid_argument(std::string(name) + " must be -1 or 1");
  }
}

double clamp_symmetric(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

}  // namespace

BodyMotionCommand wheel_velocity_to_body_motion(
  double left_velocity, double right_velocity,
  double wheel_radius, double wheel_separation,
  int left_command_sign, int right_command_sign,
  double max_linear_x, double max_angular_z)
{
  if (!std::isfinite(left_velocity) || !std::isfinite(right_velocity)) {
    throw std::invalid_argument("wheel velocities must be finite");
  }
  require_positive_finite(wheel_radius, "wheel_radius");
  require_positive_finite(wheel_separation, "wheel_separation");
  require_positive_finite(max_linear_x, "max_linear_x");
  require_positive_finite(max_angular_z, "max_angular_z");
  require_sign(left_command_sign, "left_command_sign");
  require_sign(right_command_sign, "right_command_sign");

  const double corrected_left = left_velocity * left_command_sign;
  const double corrected_right = right_velocity * right_command_sign;
  BodyMotionCommand result;
  result.linear_x = clamp_symmetric(
    wheel_radius * (corrected_left + corrected_right) * 0.5,
    max_linear_x);
  result.angular_z = clamp_symmetric(
    wheel_radius * (corrected_right - corrected_left) / wheel_separation,
    max_angular_z);
  return result;
}

double encoder_count_to_position(
  std::int64_t relative_count, double counts_per_revolution,
  int encoder_sign)
{
  require_positive_finite(counts_per_revolution, "counts_per_revolution");
  require_sign(encoder_sign, "encoder_sign");
  return static_cast<double>(relative_count) * kTwoPi /
         counts_per_revolution * encoder_sign;
}

double encoder_rate_to_velocity(
  double counts_per_second, double counts_per_revolution,
  int encoder_sign)
{
  if (!std::isfinite(counts_per_second)) {
    throw std::invalid_argument("counts_per_second must be finite");
  }
  require_positive_finite(counts_per_revolution, "counts_per_revolution");
  require_sign(encoder_sign, "encoder_sign");
  return counts_per_second * kTwoPi / counts_per_revolution * encoder_sign;
}

}  // namespace carcar_hardware
