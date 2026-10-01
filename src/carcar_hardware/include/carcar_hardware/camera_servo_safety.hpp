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
#ifndef CARCAR_HARDWARE__CAMERA_SERVO_SAFETY_HPP_
#define CARCAR_HARDWARE__CAMERA_SERVO_SAFETY_HPP_

namespace carcar_hardware
{

// Temporary collision limits reported for the D435 mounted on S1.
constexpr int kCameraServoMinAngle = 85;
constexpr int kCameraServoMaxAngle = 160;
constexpr double kCameraServoLevelAngle = 140.5;

// Neutral is the experimentally level board command 94. Quantization is 1.5°.
constexpr double camera_tilt_from_board_command(int board_angle) noexcept
{
  return (board_angle - 94) * 1.5 * 3.14159265358979323846 / 180.0;
}

constexpr double clamp_camera_servo_angle(double angle) noexcept
{
  return angle < kCameraServoMinAngle ? kCameraServoMinAngle :
         (angle > kCameraServoMaxAngle ? kCameraServoMaxAngle : angle);
}

constexpr int clamp_camera_servo_angle(int angle) noexcept
{
  return angle < kCameraServoMinAngle ? kCameraServoMinAngle :
         (angle > kCameraServoMaxAngle ? kCameraServoMaxAngle : angle);
}

}  // namespace carcar_hardware

#endif  // CARCAR_HARDWARE__CAMERA_SERVO_SAFETY_HPP_
