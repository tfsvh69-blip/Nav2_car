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

#ifndef CARCAR_HARDWARE__DIFFERENTIAL_DRIVE_HPP_
#define CARCAR_HARDWARE__DIFFERENTIAL_DRIVE_HPP_

#include <cstdint>

namespace carcar_hardware
{

struct BodyMotionCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
};

BodyMotionCommand wheel_velocity_to_body_motion(
  double left_velocity, double right_velocity,
  double wheel_radius, double wheel_separation,
  int left_command_sign, int right_command_sign,
  double max_linear_x, double max_angular_z);

double encoder_count_to_position(
  std::int64_t relative_count, double counts_per_revolution,
  int encoder_sign);

double encoder_rate_to_velocity(
  double counts_per_second, double counts_per_revolution,
  int encoder_sign);

}  // namespace carcar_hardware

#endif  // CARCAR_HARDWARE__DIFFERENTIAL_DRIVE_HPP_
