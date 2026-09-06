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

#include "carcar_hardware/rosmaster_system.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "carcar_hardware/differential_drive.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace carcar_hardware
{
namespace
{

constexpr char kEncoderCountInterface[] = "encoder_count";
constexpr char kEncoderVelocityInterface[] = "encoder_velocity";
constexpr char kMotorPwmInterface[] = "motor_pwm";
constexpr std::array<const char *, 10> kImuInterfaces{{
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}};

bool has_state_interface(
  const hardware_interface::ComponentInfo & component,
  const std::string & interface_name)
{
  return std::any_of(
    component.state_interfaces.begin(), component.state_interfaces.end(),
    [&interface_name](const auto & interface) {
      return interface.name == interface_name;
    });
}

bool has_command_interface(
  const hardware_interface::ComponentInfo & component,
  const std::string & interface_name)
{
  return std::any_of(
    component.command_interfaces.begin(), component.command_interfaces.end(),
    [&interface_name](const auto & interface) {
      return interface.name == interface_name;
    });
}

double parse_positive_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name)
{
  const auto item = parameters.find(name);
  if (item == parameters.end()) {
    throw std::invalid_argument("missing parameter " + name);
  }
  std::size_t parsed_length = 0U;
  const double value = std::stod(item->second, &parsed_length);
  if (parsed_length != item->second.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be positive and finite");
  }
  return value;
}

int parse_sign_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name)
{
  const auto item = parameters.find(name);
  if (item == parameters.end()) {
    throw std::invalid_argument("missing parameter " + name);
  }
  std::size_t parsed_length = 0U;
  const int value = std::stoi(item->second, &parsed_length);
  if (parsed_length != item->second.size() || (value != -1 && value != 1)) {
    throw std::invalid_argument(name + " must be -1 or 1");
  }
  return value;
}

bool parse_motion_enabled(
  const std::unordered_map<std::string, std::string> & parameters)
{
  const auto item = parameters.find("motion_enabled");
  if (item == parameters.end() || item->second == "false" || item->second == "0") {
    return false;
  }
  if (item->second == "true" || item->second == "1") {
    return true;
  }
  throw std::invalid_argument("motion_enabled must be true/false or 1/0");
}

std::int32_t encoder_delta(std::int32_t current, std::int32_t previous)
{
  const auto current_unsigned = static_cast<std::uint32_t>(current);
  const auto previous_unsigned = static_cast<std::uint32_t>(previous);
  return static_cast<std::int32_t>(current_unsigned - previous_unsigned);
}

}  // namespace

RosmasterSystem::~RosmasterSystem()
{
  stop_and_close();
}

hardware_interface::CallbackReturn RosmasterSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto port = info_.hardware_parameters.find("serial_port");
  if (port != info_.hardware_parameters.end() && !port->second.empty()) {
    serial_port_ = port->second;
  }

  try {
    const auto car_type = info_.hardware_parameters.find("car_type");
    if (car_type != info_.hardware_parameters.end()) {
      const auto parsed = std::stoul(car_type->second);
      if (parsed > std::numeric_limits<std::uint8_t>::max()) {
        throw std::out_of_range("car_type exceeds uint8 range");
      }
      car_type_ = static_cast<std::uint8_t>(parsed);
    }

    wheel_count_ = info_.joints.size();
    if (wheel_count_ == 2U) {
      command_mode_ = has_command_interface(
        info_.joints[0], hardware_interface::HW_IF_VELOCITY);
      standard_wheel_state_mode_ =
        has_state_interface(info_.joints[0], hardware_interface::HW_IF_POSITION) &&
        has_state_interface(info_.joints[0], hardware_interface::HW_IF_VELOCITY);
    } else if (wheel_count_ == 4U) {
      raw_motor_command_mode_ = has_command_interface(
        info_.joints[0], kMotorPwmInterface);
      command_mode_ = raw_motor_command_mode_;
    }
    if (command_mode_) {
      motion_enabled_ = parse_motion_enabled(info_.hardware_parameters);
      encoder_timeout_ = parse_positive_parameter(
        info_.hardware_parameters, "encoder_timeout");
    }
    if (raw_motor_command_mode_) {
      max_motor_pwm_ = static_cast<int>(parse_positive_parameter(
          info_.hardware_parameters, "max_motor_pwm"));
      if (max_motor_pwm_ > 20) {
        throw std::out_of_range("max_motor_pwm must not exceed 20");
      }
    } else if (command_mode_) {
      wheel_radius_ = parse_positive_parameter(info_.hardware_parameters, "wheel_radius");
      wheel_separation_ = parse_positive_parameter(
        info_.hardware_parameters, "wheel_separation");
      counts_per_revolution_ = parse_positive_parameter(
        info_.hardware_parameters, "encoder_counts_per_revolution");
      max_linear_x_ = parse_positive_parameter(
        info_.hardware_parameters, "max_linear_x");
      max_angular_z_ = parse_positive_parameter(
        info_.hardware_parameters, "max_angular_z");
      left_command_sign_ = parse_sign_parameter(
        info_.hardware_parameters, "left_wheel_command_sign");
      right_command_sign_ = parse_sign_parameter(
        info_.hardware_parameters, "right_wheel_command_sign");
      left_encoder_sign_ = parse_sign_parameter(
        info_.hardware_parameters, "left_wheel_encoder_sign");
      right_encoder_sign_ = parse_sign_parameter(
        info_.hardware_parameters, "right_wheel_encoder_sign");
      if (max_linear_x_ > 32.767 || max_angular_z_ > 32.767) {
        throw std::out_of_range("motion limits exceed int16 protocol range");
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"), "Invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_description()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0U; index < wheel_count_; ++index) {
    wheel_joint_names_[index] = info_.joints[index].name;
  }
  imu_sensor_name_ = info_.sensors[0].name;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosmasterSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  parser_.reset();
  observed_encoder_sequence_ = 0U;
  observed_imu_sequence_ = 0U;
  reported_first_encoder_ = false;
  reported_first_imu_ = false;
  reported_first_read_call_ = false;
  reported_first_serial_bytes_ = false;
  reported_motion_lock_ = false;
  reported_waiting_for_encoder_ = false;
  output_stopped_ = true;
  have_previous_encoder_sample_ = false;
  relative_encoder_counts_.fill(0);
  encoder_count_.fill(0.0);
  encoder_velocity_.fill(0.0);
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  wheel_command_.fill(0.0);
  angular_velocity_.fill(0.0);
  linear_acceleration_.fill(0.0);
  configure_time_ = std::chrono::steady_clock::now();

  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!send_zero_output() || !write_frame(make_auto_report_command(true))) {
    stop_and_close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RosmasterSystem"),
    "Configured Rosmaster serial connection on %s (%s)", serial_port_.c_str(),
    command_mode_ ? "standard wheel command mode" : "read-only raw mode");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosmasterSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosmasterSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  if (serial_fd_ < 0 || !send_zero_output()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  output_stopped_ = true;
  const auto logger = rclcpp::get_logger("RosmasterSystem");
  if (!command_mode_) {
    RCLCPP_INFO(logger, "Activated read-only state interfaces; no commands exported");
  } else if (!motion_enabled_) {
    RCLCPP_WARN(logger, "Velocity interfaces active but hardware motion lock is CLOSED");
  } else {
    RCLCPP_WARN(
      logger,
      "Velocity interfaces and hardware motion are ENABLED; limits vx=%.3f m/s wz=%.3f rad/s",
      max_linear_x_, max_angular_z_);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosmasterSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  const bool stopped = send_zero_output();
  output_stopped_ = stopped;
  return stopped ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn RosmasterSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stop_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RosmasterSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  stop_and_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RosmasterSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(10U + wheel_count_ * (standard_wheel_state_mode_ ? 4U : 2U));
  for (std::size_t index = 0U; index < wheel_count_; ++index) {
    if (standard_wheel_state_mode_) {
      interfaces.emplace_back(
        wheel_joint_names_[index], hardware_interface::HW_IF_POSITION,
        &wheel_position_[index]);
      interfaces.emplace_back(
        wheel_joint_names_[index], hardware_interface::HW_IF_VELOCITY,
        &wheel_velocity_[index]);
    }
    interfaces.emplace_back(
      wheel_joint_names_[index], kEncoderCountInterface, &encoder_count_[index]);
    interfaces.emplace_back(
      wheel_joint_names_[index], kEncoderVelocityInterface, &encoder_velocity_[index]);
  }

  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[0], &orientation_[0]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[1], &orientation_[1]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[2], &orientation_[2]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[3], &orientation_[3]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[4], &angular_velocity_[0]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[5], &angular_velocity_[1]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[6], &angular_velocity_[2]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[7], &linear_acceleration_[0]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[8], &linear_acceleration_[1]);
  interfaces.emplace_back(imu_sensor_name_, kImuInterfaces[9], &linear_acceleration_[2]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RosmasterSystem::export_command_interfaces()
{
  if (!command_mode_) {
    return {};
  }
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(wheel_count_);
  const char * interface_name = raw_motor_command_mode_ ?
    kMotorPwmInterface : hardware_interface::HW_IF_VELOCITY;
  for (std::size_t index = 0U; index < wheel_count_; ++index) {
    interfaces.emplace_back(
      wheel_joint_names_[index], interface_name, &wheel_command_[index]);
  }
  return interfaces;
}

hardware_interface::return_type RosmasterSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  if (!reported_first_read_call_) {
    RCLCPP_INFO(rclcpp::get_logger("RosmasterSystem"), "First ros2_control read() call");
    reported_first_read_call_ = true;
  }

  std::array<std::uint8_t, 512> received{};
  while (true) {
    const auto size = ::read(serial_fd_, received.data(), received.size());
    if (size > 0) {
      if (!reported_first_serial_bytes_) {
        RCLCPP_INFO(
          rclcpp::get_logger("RosmasterSystem"),
          "First serial read received %zd bytes", size);
        reported_first_serial_bytes_ = true;
      }
      parser_.feed(received.data(), static_cast<std::size_t>(size));
      continue;
    }
    if (size == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"), "Serial read failed: %s", std::strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  update_exported_state();
  if (command_mode_ && motion_enabled_ && !encoder_feedback_is_fresh()) {
    wheel_velocity_.fill(0.0);
    encoder_velocity_.fill(0.0);
    send_zero_output();
    output_stopped_ = true;
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"),
      "Encoder feedback timed out after %.3f s; forced zero output",
      encoder_timeout_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RosmasterSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!command_mode_) {
    return hardware_interface::return_type::OK;
  }
  if (!motion_enabled_) {
    if (!reported_motion_lock_) {
      RCLCPP_WARN(
        rclcpp::get_logger("RosmasterSystem"),
        "Hardware motion lock is CLOSED; velocity commands are ignored");
      reported_motion_lock_ = true;
    }
    return hardware_interface::return_type::OK;
  }
  if (!have_previous_encoder_sample_) {
    if (!output_stopped_ || !reported_waiting_for_encoder_) {
      if (!send_zero_output()) {
        return hardware_interface::return_type::ERROR;
      }
      output_stopped_ = true;
    }
    if (!reported_waiting_for_encoder_) {
      RCLCPP_WARN(
        rclcpp::get_logger("RosmasterSystem"),
        "Waiting for first encoder report; forced zero output");
      reported_waiting_for_encoder_ = true;
    }
    return hardware_interface::return_type::OK;
  }
  if (!encoder_feedback_is_fresh()) {
    send_zero_output();
    output_stopped_ = true;
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"),
      "Refusing wheel command because encoder feedback is stale");
    return hardware_interface::return_type::ERROR;
  }

  try {
    if (raw_motor_command_mode_) {
      std::array<std::int8_t, 4> motor_pwm{{0, 0, 0, 0}};
      bool all_zero = true;
      for (std::size_t index = 0U; index < motor_pwm.size(); ++index) {
        const double value = wheel_command_[index];
        if (!std::isfinite(value) || std::abs(value) > max_motor_pwm_) {
          throw std::out_of_range("motor_pwm command is non-finite or exceeds safety limit");
        }
        motor_pwm[index] = static_cast<std::int8_t>(std::lround(value));
        all_zero = all_zero && motor_pwm[index] == 0;
      }
      if (!write_frame(make_motor_command(motor_pwm))) {
        send_zero_output();
        output_stopped_ = true;
        return hardware_interface::return_type::ERROR;
      }
      output_stopped_ = all_zero;
      return hardware_interface::return_type::OK;
    }
    const auto command = wheel_velocity_to_body_motion(
      wheel_command_[0], wheel_command_[1],
      wheel_radius_, wheel_separation_, left_command_sign_,
      right_command_sign_, max_linear_x_, max_angular_z_);
    if (!write_frame(make_motion_command(car_type_, command.linear_x, 0.0, command.angular_z))) {
      send_zero_output();
      output_stopped_ = true;
      return hardware_interface::return_type::ERROR;
    }
    output_stopped_ = command.linear_x == 0.0 && command.angular_z == 0.0;
  } catch (const std::exception & error) {
    send_zero_output();
    output_stopped_ = true;
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"),
      "Invalid wheel velocity command: %s; forced zero output", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool RosmasterSystem::validate_description() const
{
  const auto logger = rclcpp::get_logger("RosmasterSystem");
  if (wheel_count_ != 2U && wheel_count_ != 4U) {
    RCLCPP_ERROR(logger, "Exactly two legacy wheel joints or four motor joints are required");
    return false;
  }
  for (const auto & joint : info_.joints) {
    const bool has_expected_command = has_command_interface(
      joint, raw_motor_command_mode_ ? kMotorPwmInterface : hardware_interface::HW_IF_VELOCITY);
    if (command_mode_ && (joint.command_interfaces.size() != 1U || !has_expected_command)) {
      RCLCPP_ERROR(
        logger, "Command-capable joint %s has an invalid command interface", joint.name.c_str());
      return false;
    }
    if (!command_mode_ && !joint.command_interfaces.empty()) {
      RCLCPP_ERROR(
        logger, "Read-only wheel joint %s must not export commands", joint.name.c_str());
      return false;
    }
    if (!has_state_interface(joint, kEncoderCountInterface) ||
      !has_state_interface(joint, kEncoderVelocityInterface))
    {
      RCLCPP_ERROR(
        logger, "Wheel joint %s requires encoder_count and encoder_velocity states",
        joint.name.c_str());
      return false;
    }
    const bool has_position_state = has_state_interface(
      joint, hardware_interface::HW_IF_POSITION);
    const bool has_velocity_state = has_state_interface(
      joint, hardware_interface::HW_IF_VELOCITY);
    if (command_mode_ && !raw_motor_command_mode_ &&
      (!has_position_state || !has_velocity_state))
    {
      RCLCPP_ERROR(
        logger, "Command-capable wheel joint %s requires position and velocity states",
        joint.name.c_str());
      return false;
    }
    if ((!command_mode_ || raw_motor_command_mode_) &&
      (has_position_state || has_velocity_state))
    {
      RCLCPP_ERROR(
        logger, "Read-only raw wheel joint %s must not expose partial standard states",
        joint.name.c_str());
      return false;
    }
  }
  if (info_.sensors.size() != 1U) {
    RCLCPP_ERROR(logger, "Exactly one board IMU sensor is required");
    return false;
  }
  for (const auto * interface_name : kImuInterfaces) {
    if (!has_state_interface(info_.sensors[0], interface_name)) {
      RCLCPP_ERROR(logger, "IMU state interface %s is missing", interface_name);
      return false;
    }
  }
  return true;
}

bool RosmasterSystem::open_serial()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"), "Cannot open %s: %s",
      serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios options{};
  if (tcgetattr(serial_fd_, &options) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"), "tcgetattr failed: %s", std::strerror(errno));
    stop_and_close();
    return false;
  }
  cfmakeraw(&options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"), "tcsetattr failed: %s", std::strerror(errno));
    stop_and_close();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

bool RosmasterSystem::write_frame(const std::vector<std::uint8_t> & frame) const
{
  if (serial_fd_ < 0) {
    return false;
  }
  std::size_t offset = 0U;
  while (offset < frame.size()) {
    const auto written = ::write(serial_fd_, frame.data() + offset, frame.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd descriptor{serial_fd_, POLLOUT, 0};
      if (::poll(&descriptor, 1, 50) > 0) {
        continue;
      }
    }
    RCLCPP_ERROR(
      rclcpp::get_logger("RosmasterSystem"), "Serial write failed: %s", std::strerror(errno));
    return false;
  }
  // The vendor driver leaves 2 ms between protocol frames so the MCU can
  // finish processing one command before the next frame arrives.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return true;
}

bool RosmasterSystem::send_zero_output() const
{
  return write_frame(make_zero_motor_command()) &&
         write_frame(make_zero_motion_command(car_type_));
}

void RosmasterSystem::stop_and_close() noexcept
{
  if (serial_fd_ < 0) {
    return;
  }
  send_zero_output();
  output_stopped_ = true;
  write_frame(make_auto_report_command(false));
  tcdrain(serial_fd_);
  ::close(serial_fd_);
  serial_fd_ = -1;
}

void RosmasterSystem::update_exported_state()
{
  const auto & state = parser_.state();
  if (state.encoder_sequence != observed_encoder_sequence_) {
    const auto now = std::chrono::steady_clock::now();
    for (std::size_t index = 0U; index < wheel_count_; ++index) {
      encoder_count_[index] = static_cast<double>(state.encoders[index]);
    }
    if (have_previous_encoder_sample_) {
      const auto elapsed = std::chrono::duration<double>(now - previous_encoder_time_).count();
      for (std::size_t index = 0U; index < wheel_count_; ++index) {
        const auto delta = encoder_delta(state.encoders[index], previous_encoders_[index]);
        relative_encoder_counts_[index] += delta;
        if (elapsed > 0.0) {
          encoder_velocity_[index] = delta / elapsed;
        }
      }
    } else {
      encoder_velocity_.fill(0.0);
      have_previous_encoder_sample_ = true;
    }
    if (standard_wheel_state_mode_) {
      wheel_position_[0] = encoder_count_to_position(
        relative_encoder_counts_[0], counts_per_revolution_, left_encoder_sign_);
      wheel_position_[1] = encoder_count_to_position(
        relative_encoder_counts_[1], counts_per_revolution_, right_encoder_sign_);
      wheel_velocity_[0] = encoder_rate_to_velocity(
        encoder_velocity_[0], counts_per_revolution_, left_encoder_sign_);
      wheel_velocity_[1] = encoder_rate_to_velocity(
        encoder_velocity_[1], counts_per_revolution_, right_encoder_sign_);
    }
    previous_encoders_ = state.encoders;
    previous_encoder_time_ = now;
    observed_encoder_sequence_ = state.encoder_sequence;
    reported_waiting_for_encoder_ = false;
    if (!reported_first_encoder_) {
      RCLCPP_INFO(
        rclcpp::get_logger("RosmasterSystem"),
        "First encoder report received: M1=%d M2=%d M3=%d M4=%d",
        state.encoders[0], state.encoders[1], state.encoders[2], state.encoders[3]);
      reported_first_encoder_ = true;
    }
  }

  if (state.imu_sequence != observed_imu_sequence_) {
    angular_velocity_ = state.angular_velocity;
    linear_acceleration_ = state.linear_acceleration;
    observed_imu_sequence_ = state.imu_sequence;
    if (!reported_first_imu_) {
      RCLCPP_INFO(
        rclcpp::get_logger("RosmasterSystem"),
        "First IMU report received: gyro=[%.4f, %.4f, %.4f] accel=[%.3f, %.3f, %.3f]",
        angular_velocity_[0], angular_velocity_[1], angular_velocity_[2],
        linear_acceleration_[0], linear_acceleration_[1], linear_acceleration_[2]);
      reported_first_imu_ = true;
    }
  }
}

bool RosmasterSystem::encoder_feedback_is_fresh() const
{
  const auto now = std::chrono::steady_clock::now();
  const auto reference = have_previous_encoder_sample_ ?
    previous_encoder_time_ : configure_time_;
  return std::chrono::duration<double>(now - reference).count() <= encoder_timeout_;
}

}  // namespace carcar_hardware

PLUGINLIB_EXPORT_CLASS(carcar_hardware::RosmasterSystem, hardware_interface::SystemInterface)
