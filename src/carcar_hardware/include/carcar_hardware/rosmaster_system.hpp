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

#ifndef CARCAR_HARDWARE__ROSMASTER_SYSTEM_HPP_
#define CARCAR_HARDWARE__ROSMASTER_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "carcar_hardware/rosmaster_protocol.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace carcar_hardware
{

class RosmasterSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RosmasterSystem)

  ~RosmasterSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool validate_description() const;
  bool open_serial();
  bool write_frame(const std::vector<std::uint8_t> & frame) const;
  bool send_zero_output() const;
  void stop_and_close() noexcept;
  void update_exported_state();
  bool encoder_feedback_is_fresh() const;

  int serial_fd_{-1};
  std::string serial_port_{"/dev/myserial"};
  std::uint8_t car_type_{1U};
  std::array<std::string, 4> wheel_joint_names_;
  std::size_t wheel_count_{0U};
  std::string imu_sensor_name_;
  bool command_mode_{false};
  bool raw_motor_command_mode_{false};
  bool standard_wheel_state_mode_{false};
  bool motion_enabled_{false};
  bool reported_motion_lock_{false};
  bool reported_waiting_for_encoder_{false};
  bool output_stopped_{true};
  double wheel_radius_{0.0};
  double wheel_separation_{0.0};
  double counts_per_revolution_{0.0};
  double max_linear_x_{0.0};
  double max_angular_z_{0.0};
  double encoder_timeout_{0.25};
  int max_motor_pwm_{20};
  int left_command_sign_{1};
  int right_command_sign_{1};
  int left_encoder_sign_{1};
  int right_encoder_sign_{1};

  RosmasterProtocolParser parser_;
  std::uint64_t observed_encoder_sequence_{0U};
  std::uint64_t observed_imu_sequence_{0U};
  bool reported_first_encoder_{false};
  bool reported_first_imu_{false};
  bool reported_first_read_call_{false};
  bool reported_first_serial_bytes_{false};
  std::array<std::int32_t, 4> previous_encoders_{{0, 0, 0, 0}};
  bool have_previous_encoder_sample_{false};
  std::chrono::steady_clock::time_point previous_encoder_time_;
  std::chrono::steady_clock::time_point configure_time_;
  std::array<std::int64_t, 4> relative_encoder_counts_{{0, 0, 0, 0}};

  std::array<double, 4> encoder_count_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> encoder_velocity_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> wheel_position_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> wheel_velocity_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> wheel_command_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> orientation_{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> angular_velocity_{{0.0, 0.0, 0.0}};
  std::array<double, 3> linear_acceleration_{{0.0, 0.0, 0.0}};
};

}  // namespace carcar_hardware

#endif  // CARCAR_HARDWARE__ROSMASTER_SYSTEM_HPP_
