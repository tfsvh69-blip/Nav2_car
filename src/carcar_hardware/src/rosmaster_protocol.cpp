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

#include "carcar_hardware/rosmaster_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace carcar_hardware
{
namespace
{

constexpr std::uint8_t kCommandHead = 0xFF;
constexpr std::uint8_t kCommandDevice = 0xFC;
constexpr std::uint8_t kReportDevice = 0xFB;
constexpr std::uint8_t kChecksumComplement = 5;
constexpr std::uint8_t kFunctionAutoReport = 0x01;
constexpr std::uint8_t kFunctionBeep = 0x02;
constexpr std::uint8_t kFunctionRgb = 0x05;
constexpr std::uint8_t kFunctionRgbEffect = 0x06;
constexpr std::uint8_t kFunctionReportMpuRaw = 0x0B;
constexpr std::uint8_t kFunctionReportEncoder = 0x0D;
constexpr std::uint8_t kFunctionReportIcmRaw = 0x0E;
constexpr std::uint8_t kFunctionMotor = 0x10;
constexpr std::uint8_t kFunctionMotion = 0x12;
constexpr std::size_t kMaximumReportLength = 64;

std::int16_t read_i16_le(const std::uint8_t * data)
{
  const auto value = static_cast<std::uint16_t>(data[0]) |
    (static_cast<std::uint16_t>(data[1]) << 8U);
  return static_cast<std::int16_t>(value);
}

std::int32_t read_i32_le(const std::uint8_t * data)
{
  const auto value = static_cast<std::uint32_t>(data[0]) |
    (static_cast<std::uint32_t>(data[1]) << 8U) |
    (static_cast<std::uint32_t>(data[2]) << 16U) |
    (static_cast<std::uint32_t>(data[3]) << 24U);
  return static_cast<std::int32_t>(value);
}

std::vector<std::uint8_t> finish_command(std::vector<std::uint8_t> command)
{
  command[2] = static_cast<std::uint8_t>(command.size() - 1U);
  std::uint32_t checksum = kChecksumComplement;
  for (const auto value : command) {
    checksum += value;
  }
  command.push_back(static_cast<std::uint8_t>(checksum & 0xFFU));
  return command;
}

void append_scaled_i16(std::vector<std::uint8_t> & command, double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("motion command must be finite");
  }
  const double scaled = value * 1000.0;
  if (scaled < std::numeric_limits<std::int16_t>::min() ||
    scaled > std::numeric_limits<std::int16_t>::max())
  {
    throw std::out_of_range("motion command exceeds protocol range");
  }
  const auto signed_value = static_cast<std::int16_t>(scaled);
  const auto raw = static_cast<std::uint16_t>(signed_value);
  command.push_back(static_cast<std::uint8_t>(raw & 0xFFU));
  command.push_back(static_cast<std::uint8_t>((raw >> 8U) & 0xFFU));
}

}  // namespace

void RosmasterProtocolParser::reset()
{
  buffer_.clear();
  state_ = RosmasterProtocolState{};
}

void RosmasterProtocolParser::feed(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr || size == 0U) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);

  while (true) {
    const auto head = std::find(buffer_.begin(), buffer_.end(), kCommandHead);
    if (head == buffer_.end()) {
      buffer_.clear();
      return;
    }
    buffer_.erase(buffer_.begin(), head);

    if (buffer_.size() < 2U) {
      return;
    }
    if (buffer_[1] != kReportDevice) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (buffer_.size() < 4U) {
      return;
    }

    const std::size_t extended_length = buffer_[2];
    if (extended_length < 2U || extended_length > kMaximumReportLength) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t frame_size = extended_length + 2U;
    if (buffer_.size() < frame_size) {
      return;
    }

    std::uint32_t checksum = buffer_[2] + buffer_[3];
    for (std::size_t index = 4U; index + 1U < frame_size; ++index) {
      checksum += buffer_[index];
    }
    const bool checksum_valid =
      static_cast<std::uint8_t>(checksum & 0xFFU) == buffer_[frame_size - 1U];
    if (!checksum_valid) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t payload_size = frame_size - 5U;
    parse_payload(buffer_[3], buffer_.data() + 4U, payload_size);
    ++state_.valid_frame_sequence;
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  }
}

const RosmasterProtocolState & RosmasterProtocolParser::state() const noexcept
{
  return state_;
}

void RosmasterProtocolParser::parse_payload(
  std::uint8_t function, const std::uint8_t * payload,
  std::size_t payload_size)
{
  if (function == kFunctionReportEncoder && payload_size >= 16U) {
    for (std::size_t index = 0U; index < state_.encoders.size(); ++index) {
      state_.encoders[index] = read_i32_le(payload + index * 4U);
    }
    ++state_.encoder_sequence;
    return;
  }

  if (function == kFunctionReportMpuRaw && payload_size >= 12U) {
    constexpr double kGyroRatio = 1.0 / 3754.9;
    constexpr double kAccelerationRatio = 1.0 / 1671.84;
    state_.angular_velocity[0] = read_i16_le(payload) * kGyroRatio;
    state_.angular_velocity[1] = read_i16_le(payload + 2U) * -kGyroRatio;
    state_.angular_velocity[2] = read_i16_le(payload + 4U) * -kGyroRatio;
    state_.linear_acceleration[0] = read_i16_le(payload + 6U) * kAccelerationRatio;
    state_.linear_acceleration[1] = read_i16_le(payload + 8U) * kAccelerationRatio;
    state_.linear_acceleration[2] = read_i16_le(payload + 10U) * kAccelerationRatio;
    ++state_.imu_sequence;
    return;
  }

  if (function == kFunctionReportIcmRaw && payload_size >= 12U) {
    constexpr double kIcmRatio = 1.0 / 1000.0;
    for (std::size_t index = 0U; index < 3U; ++index) {
      state_.angular_velocity[index] =
        read_i16_le(payload + index * 2U) * kIcmRatio;
      state_.linear_acceleration[index] =
        read_i16_le(payload + 6U + index * 2U) * kIcmRatio;
    }
    ++state_.imu_sequence;
  }
}

std::vector<std::uint8_t> make_auto_report_command(bool enable)
{
  return finish_command(
    {kCommandHead, kCommandDevice, 0U, kFunctionAutoReport,
      static_cast<std::uint8_t>(enable ? 1U : 0U), 0U});
}

std::vector<std::uint8_t> make_zero_motor_command()
{
  return make_motor_command({{0, 0, 0, 0}});
}

std::vector<std::uint8_t> make_motor_command(
  const std::array<std::int8_t, 4> & motor_pwm)
{
  return finish_command(
    {
      kCommandHead, kCommandDevice, 0U, kFunctionMotor,
      static_cast<std::uint8_t>(motor_pwm[0]),
      static_cast<std::uint8_t>(motor_pwm[1]),
      static_cast<std::uint8_t>(motor_pwm[2]),
      static_cast<std::uint8_t>(motor_pwm[3])});
}

std::vector<std::uint8_t> make_zero_motion_command(std::uint8_t car_type)
{
  return make_motion_command(car_type, 0.0, 0.0, 0.0);
}

std::vector<std::uint8_t> make_motion_command(
  std::uint8_t car_type, double linear_x, double linear_y,
  double angular_z)
{
  std::vector<std::uint8_t> command{
    kCommandHead, kCommandDevice, 0U, kFunctionMotion, car_type};
  append_scaled_i16(command, linear_x);
  append_scaled_i16(command, linear_y);
  append_scaled_i16(command, angular_z);
  return finish_command(std::move(command));
}

std::vector<std::uint8_t> make_beep_command(std::uint16_t on_time_ms)
{
  const auto low = static_cast<std::uint8_t>(on_time_ms & 0xFFU);
  const auto high = static_cast<std::uint8_t>((on_time_ms >> 8U) & 0xFFU);
  return finish_command(
    {kCommandHead, kCommandDevice, 0U, kFunctionBeep, low, high});
}

std::vector<std::uint8_t> make_rgb_command(
  std::uint8_t led_id, std::uint8_t red, std::uint8_t green,
  std::uint8_t blue)
{
  return finish_command(
    {kCommandHead, kCommandDevice, 0U, kFunctionRgb, led_id, red, green, blue});
}

std::vector<std::uint8_t> make_rgb_effect_command(
  std::uint8_t effect, std::uint8_t speed, std::uint8_t parm)
{
  return finish_command(
    {kCommandHead, kCommandDevice, 0U, kFunctionRgbEffect, effect, speed, parm});
}

}  // namespace carcar_hardware
