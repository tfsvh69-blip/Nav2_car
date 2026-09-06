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

#include <memory>
#include <string>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace
{

std::string command_urdf()
{
  return
    R"(
<robot name="command_test">
  <ros2_control name="RosmasterCommandTestSystem" type="system">
    <hardware>
      <plugin>carcar_hardware/RosmasterSystem</plugin>
      <param name="serial_port">/dev/null</param>
      <param name="car_type">1</param>
      <param name="motion_enabled">false</param>
      <param name="wheel_radius">0.05</param>
      <param name="wheel_separation">0.20</param>
      <param name="encoder_counts_per_revolution">1320.0</param>
      <param name="left_wheel_command_sign">1</param>
      <param name="right_wheel_command_sign">1</param>
      <param name="left_wheel_encoder_sign">1</param>
      <param name="right_wheel_encoder_sign">1</param>
      <param name="max_linear_x">0.10</param>
      <param name="max_angular_z">0.50</param>
      <param name="encoder_timeout">0.20</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="encoder_count"/>
      <state_interface name="encoder_velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="encoder_count"/>
      <state_interface name="encoder_velocity"/>
    </joint>
    <sensor name="board_imu">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>
  </ros2_control>
</robot>)";
}

std::string four_motor_urdf()
{
  return
    R"(
<robot name="four_motor_test">
  <ros2_control name="RosmasterFourMotorTestSystem" type="system">
    <hardware>
      <plugin>carcar_hardware/RosmasterSystem</plugin>
      <param name="serial_port">/dev/null</param>
      <param name="car_type">1</param>
      <param name="motion_enabled">false</param>
      <param name="max_motor_pwm">20</param>
      <param name="encoder_timeout">0.20</param>
    </hardware>
    <joint name="motor_1_joint">
      <command_interface name="motor_pwm"/>
      <state_interface name="encoder_count"/>
      <state_interface name="encoder_velocity"/>
    </joint>
    <joint name="motor_2_joint">
      <command_interface name="motor_pwm"/>
      <state_interface name="encoder_count"/>
      <state_interface name="encoder_velocity"/>
    </joint>
    <joint name="motor_3_joint">
      <command_interface name="motor_pwm"/>
      <state_interface name="encoder_count"/>
      <state_interface name="encoder_velocity"/>
    </joint>
    <joint name="motor_4_joint">
      <command_interface name="motor_pwm"/>
      <state_interface name="encoder_count"/>
      <state_interface name="encoder_velocity"/>
    </joint>
    <sensor name="board_imu">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>
  </ros2_control>
</robot>)";
}

}  // namespace

TEST(RosmasterSystemPlugin, LoadsWithoutOpeningSerial)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto instance = loader.createSharedInstance("carcar_hardware/RosmasterSystem");
  ASSERT_NE(instance, nullptr);
}

TEST(RosmasterSystemPlugin, ExportsStandardLockedCommandDescription)
{
  auto hardware = hardware_interface::parse_control_resources_from_urdf(command_urdf());
  ASSERT_EQ(hardware.size(), 1U);
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto instance = loader.createSharedInstance("carcar_hardware/RosmasterSystem");
  ASSERT_EQ(
    instance->on_init(hardware.front()),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto state_interfaces = instance->export_state_interfaces();
  const auto command_interfaces = instance->export_command_interfaces();
  EXPECT_EQ(state_interfaces.size(), 18U);
  ASSERT_EQ(command_interfaces.size(), 2U);
  EXPECT_EQ(command_interfaces[0].get_name(), "left_wheel_joint/velocity");
  EXPECT_EQ(command_interfaces[1].get_name(), "right_wheel_joint/velocity");
}

TEST(RosmasterSystemPlugin, RejectsMissingCommandConversionParameter)
{
  auto hardware = hardware_interface::parse_control_resources_from_urdf(command_urdf());
  ASSERT_EQ(hardware.size(), 1U);
  hardware.front().hardware_parameters.erase("wheel_radius");
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto instance = loader.createSharedInstance("carcar_hardware/RosmasterSystem");
  EXPECT_EQ(
    instance->on_init(hardware.front()),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(RosmasterSystemPlugin, ExportsFourRawMotorCommandsAndEncoderStates)
{
  auto hardware = hardware_interface::parse_control_resources_from_urdf(
    four_motor_urdf());
  ASSERT_EQ(hardware.size(), 1U);
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  const auto instance = loader.createSharedInstance("carcar_hardware/RosmasterSystem");
  ASSERT_EQ(
    instance->on_init(hardware.front()),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto state_interfaces = instance->export_state_interfaces();
  const auto command_interfaces = instance->export_command_interfaces();
  EXPECT_EQ(state_interfaces.size(), 18U);
  ASSERT_EQ(command_interfaces.size(), 4U);
  for (std::size_t index = 0U; index < command_interfaces.size(); ++index) {
    EXPECT_EQ(
      command_interfaces[index].get_name(),
      "motor_" + std::to_string(index + 1U) + "_joint/motor_pwm");
  }
}
