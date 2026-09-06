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

#ifndef CARCAR_HARDWARE__ROSMASTER_PROTOCOL_HPP_
#define CARCAR_HARDWARE__ROSMASTER_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace carcar_hardware
{

struct RosmasterProtocolState
{
  std::array<std::int32_t, 4> encoders{{0, 0, 0, 0}};
  std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> linear_acceleration{{0.0, 0.0, 0.0}};
  std::uint64_t encoder_sequence{0};
  std::uint64_t imu_sequence{0};
  std::uint64_t valid_frame_sequence{0};
};

class RosmasterProtocolParser
{
public:
  void reset();
  void feed(const std::uint8_t * data, std::size_t size);
  const RosmasterProtocolState & state() const noexcept;

private:
  void parse_payload(
    std::uint8_t function, const std::uint8_t * payload,
    std::size_t payload_size);

  std::vector<std::uint8_t> buffer_;
  RosmasterProtocolState state_;
};

std::vector<std::uint8_t> make_auto_report_command(bool enable);
std::vector<std::uint8_t> make_zero_motor_command();
std::vector<std::uint8_t> make_motor_command(
  const std::array<std::int8_t, 4> & motor_pwm);
std::vector<std::uint8_t> make_motion_command(
  std::uint8_t car_type, double linear_x, double linear_y,
  double angular_z);
std::vector<std::uint8_t> make_zero_motion_command(std::uint8_t car_type);

}  // namespace carcar_hardware

#endif  // CARCAR_HARDWARE__ROSMASTER_PROTOCOL_HPP_
