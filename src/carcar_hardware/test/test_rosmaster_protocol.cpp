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

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "carcar_hardware/rosmaster_protocol.hpp"

namespace
{

std::vector<std::uint8_t> make_report(
  std::uint8_t function, const std::vector<std::uint8_t> & payload)
{
  const auto length = static_cast<std::uint8_t>(payload.size() + 3U);
  std::vector<std::uint8_t> frame{0xFF, 0xFB, length, function};
  frame.insert(frame.end(), payload.begin(), payload.end());
  std::uint32_t checksum = length + function;
  for (const auto value : payload) {
    checksum += value;
  }
  frame.push_back(static_cast<std::uint8_t>(checksum & 0xFFU));
  return frame;
}

void append_i32(std::vector<std::uint8_t> & data, std::int32_t value)
{
  const auto raw = static_cast<std::uint32_t>(value);
  for (std::size_t index = 0U; index < 4U; ++index) {
    data.push_back(static_cast<std::uint8_t>((raw >> (index * 8U)) & 0xFFU));
  }
}

void append_i16(std::vector<std::uint8_t> & data, std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  data.push_back(static_cast<std::uint8_t>(raw & 0xFFU));
  data.push_back(static_cast<std::uint8_t>((raw >> 8U) & 0xFFU));
}

TEST(RosmasterProtocol, ParsesChunkedEncoderFrame)
{
  std::vector<std::uint8_t> payload;
  append_i32(payload, 1292);
  append_i32(payload, -32);
  append_i32(payload, 170);
  append_i32(payload, 249);
  const auto frame = make_report(0x0D, payload);

  carcar_hardware::RosmasterProtocolParser parser;
  parser.feed(frame.data(), 3U);
  EXPECT_EQ(parser.state().encoder_sequence, 0U);
  parser.feed(frame.data() + 3U, frame.size() - 3U);

  EXPECT_EQ(parser.state().encoder_sequence, 1U);
  EXPECT_EQ(parser.state().encoders[0], 1292);
  EXPECT_EQ(parser.state().encoders[1], -32);
  EXPECT_EQ(parser.state().encoders[2], 170);
  EXPECT_EQ(parser.state().encoders[3], 249);
}

TEST(RosmasterProtocol, RejectsBadChecksumAndResynchronizes)
{
  std::vector<std::uint8_t> bad_payload(16U, 0U);
  auto bad_frame = make_report(0x0D, bad_payload);
  bad_frame.back() ^= 0xFFU;

  std::vector<std::uint8_t> good_payload;
  append_i32(good_payload, 1);
  append_i32(good_payload, 2);
  append_i32(good_payload, 3);
  append_i32(good_payload, 4);
  const auto good_frame = make_report(0x0D, good_payload);

  std::vector<std::uint8_t> stream{0x11, 0x22};
  stream.insert(stream.end(), bad_frame.begin(), bad_frame.end());
  stream.insert(stream.end(), good_frame.begin(), good_frame.end());

  carcar_hardware::RosmasterProtocolParser parser;
  parser.feed(stream.data(), stream.size());
  EXPECT_EQ(parser.state().valid_frame_sequence, 1U);
  EXPECT_EQ(parser.state().encoder_sequence, 1U);
  EXPECT_EQ(parser.state().encoders[0], 1);
  EXPECT_EQ(parser.state().encoders[3], 4);
}

TEST(RosmasterProtocol, ParsesIcmImuFrame)
{
  std::vector<std::uint8_t> payload;
  append_i16(payload, 4);
  append_i16(payload, 1);
  append_i16(payload, -3);
  append_i16(payload, -597);
  append_i16(payload, 1104);
  append_i16(payload, -9066);
  append_i16(payload, 0);
  append_i16(payload, 0);
  append_i16(payload, 0);
  const auto frame = make_report(0x0E, payload);

  carcar_hardware::RosmasterProtocolParser parser;
  parser.feed(frame.data(), frame.size());

  ASSERT_EQ(parser.state().imu_sequence, 1U);
  EXPECT_DOUBLE_EQ(parser.state().angular_velocity[0], 0.004);
  EXPECT_DOUBLE_EQ(parser.state().angular_velocity[1], 0.001);
  EXPECT_DOUBLE_EQ(parser.state().angular_velocity[2], -0.003);
  EXPECT_DOUBLE_EQ(parser.state().linear_acceleration[0], -0.597);
  EXPECT_DOUBLE_EQ(parser.state().linear_acceleration[1], 1.104);
  EXPECT_DOUBLE_EQ(parser.state().linear_acceleration[2], -9.066);
}

TEST(RosmasterProtocol, BuildsOnlySafeStartupCommands)
{
  EXPECT_EQ(
    carcar_hardware::make_auto_report_command(true),
    (std::vector<std::uint8_t>{0xFF, 0xFC, 0x05, 0x01, 0x01, 0x00, 0x07}));
  EXPECT_EQ(
    carcar_hardware::make_zero_motor_command(),
    (std::vector<std::uint8_t>{0xFF, 0xFC, 0x07, 0x10, 0, 0, 0, 0, 0x17}));
  EXPECT_EQ(
    carcar_hardware::make_zero_motion_command(1U),
    (std::vector<std::uint8_t>{
      0xFF, 0xFC, 0x0A, 0x12, 0x01, 0, 0, 0, 0, 0, 0, 0x1D}));
}

TEST(RosmasterProtocol, BuildsFourIndependentMotorCommand)
{
  EXPECT_EQ(
    carcar_hardware::make_motor_command({{15, -15, 7, -7}}),
    (std::vector<std::uint8_t>{
      0xFF, 0xFC, 0x07, 0x10, 0x0F, 0xF1, 0x07, 0xF9, 0x17}));
}

TEST(RosmasterProtocol, BuildsScaledMotionCommand)
{
  const auto command = carcar_hardware::make_motion_command(
    1U, 0.10, 0.0, -0.50);
  ASSERT_EQ(command.size(), 12U);
  EXPECT_EQ(command[0], 0xFFU);
  EXPECT_EQ(command[1], 0xFCU);
  EXPECT_EQ(command[2], 0x0AU);
  EXPECT_EQ(command[3], 0x12U);
  EXPECT_EQ(command[4], 0x01U);
  EXPECT_EQ(command[5], 0x64U);
  EXPECT_EQ(command[6], 0x00U);
  EXPECT_EQ(command[7], 0x00U);
  EXPECT_EQ(command[8], 0x00U);
  EXPECT_EQ(command[9], 0x0CU);
  EXPECT_EQ(command[10], 0xFEU);
  std::uint32_t checksum = 5U;
  for (std::size_t index = 0U; index + 1U < command.size(); ++index) {
    checksum += command[index];
  }
  EXPECT_EQ(command.back(), static_cast<std::uint8_t>(checksum & 0xFFU));
}

TEST(RosmasterProtocol, RejectsInvalidMotionCommand)
{
  EXPECT_THROW(
    carcar_hardware::make_motion_command(
      1U, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    carcar_hardware::make_motion_command(1U, 40.0, 0.0, 0.0),
    std::out_of_range);
}

}  // namespace
