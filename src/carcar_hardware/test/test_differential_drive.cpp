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

#include <stdexcept>

#include "carcar_hardware/differential_drive.hpp"

TEST(DifferentialDrive, ConvertsStraightWheelVelocity)
{
  const auto command = carcar_hardware::wheel_velocity_to_body_motion(
    2.0, 2.0, 0.05, 0.30, 1, 1, 0.10, 1.0);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.10);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(DifferentialDrive, ConvertsTurnAndAppliesIndependentSigns)
{
  const auto turn = carcar_hardware::wheel_velocity_to_body_motion(
    -1.0, 1.0, 0.05, 0.20, 1, 1, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(turn.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(turn.angular_z, 0.5);

  const auto corrected = carcar_hardware::wheel_velocity_to_body_motion(
    1.0, -1.0, 0.05, 0.20, 1, -1, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(corrected.linear_x, 0.05);
  EXPECT_DOUBLE_EQ(corrected.angular_z, 0.0);
}

TEST(DifferentialDrive, ClampsBodyCommand)
{
  const auto command = carcar_hardware::wheel_velocity_to_body_motion(
    -100.0, 100.0, 0.05, 0.20, 1, 1, 0.10, 0.75);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.75);
}

TEST(DifferentialDrive, ConvertsEncoderUnits)
{
  constexpr double kPi = 3.14159265358979323846;
  EXPECT_NEAR(
    carcar_hardware::encoder_count_to_position(660, 1320.0, 1),
    kPi, 1e-12);
  EXPECT_NEAR(
    carcar_hardware::encoder_rate_to_velocity(330.0, 1320.0, -1),
    -0.5 * kPi, 1e-12);
}

TEST(DifferentialDrive, RejectsUnsafeParameters)
{
  EXPECT_THROW(
    carcar_hardware::wheel_velocity_to_body_motion(
      0.0, 0.0, 0.0, 0.20, 1, 1, 0.10, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    carcar_hardware::wheel_velocity_to_body_motion(
      0.0, 0.0, 0.05, 0.20, 0, 1, 0.10, 1.0),
    std::invalid_argument);
}
