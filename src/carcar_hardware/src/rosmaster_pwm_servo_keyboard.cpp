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
// Independent keyboard test for a 270-degree PWM servo on Rosmaster S1.
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "carcar_hardware/camera_servo_safety.hpp"
#include "carcar_hardware/rosmaster_protocol.hpp"

namespace
{
std::atomic<bool> stop_requested{false};
constexpr char kSerialPort[] = "/dev/myserial";
constexpr int kAngleStep = 1;
constexpr auto kWriteTimeout = std::chrono::milliseconds(500);
constexpr auto kZeroRefresh = std::chrono::milliseconds(300);

void handle_signal(int signal_number)
{
  static_cast<void>(signal_number);
  stop_requested.store(true);
}

bool write_frame(int fd, const std::vector<std::uint8_t> & frame)
{
  const auto deadline = std::chrono::steady_clock::now() + kWriteTimeout;
  std::size_t offset = 0;
  while (offset < frame.size() && !stop_requested.load()) {
    const auto written = ::write(fd, frame.data() + offset, frame.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      return false;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) {
      return false;
    }
    pollfd descriptor{fd, POLLOUT, 0};
    if (::poll(&descriptor, 1, static_cast<int>(remaining)) <= 0) {
      return false;
    }
  }
  if (offset != frame.size()) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return true;
}

bool send_zero_motion(int fd)
{
  return write_frame(fd, carcar_hardware::make_zero_motor_command()) &&
         write_frame(fd, carcar_hardware::make_zero_motion_command(1U));
}

bool configure_serial(int fd)
{
  termios options{};
  if (::tcgetattr(fd, &options) != 0) {
    return false;
  }
  ::cfmakeraw(&options);
  if (::cfsetispeed(&options, B115200) != 0 || ::cfsetospeed(&options, B115200) != 0) {
    return false;
  }
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  return ::tcsetattr(fd, TCSANOW, &options) == 0;
}

class TerminalMode
{
public:
  bool enable()
  {
    if (::isatty(STDIN_FILENO) == 0 || ::tcgetattr(STDIN_FILENO, &original_) != 0) {
      return false;
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    enabled_ = ::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    return enabled_;
  }

  ~TerminalMode()
  {
    if (enabled_) {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

private:
  termios original_{};
  bool enabled_{false};
};

void show_angle(double physical_angle)
{
  std::cout << "S1 进行目标 " << std::fixed << std::setprecision(1)
            << physical_angle << "°\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  bool real_hardware = false;
  if (argc == 2 && std::string(argv[1]) == "--real-hardware") {
    real_hardware = true;
  } else if (argc != 1) {
    std::cerr << "用法: rosmaster_pwm_servo_keyboard [--real-hardware]\n";
    return 2;
  }
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  if (!real_hardware) {
    std::cout << "DRV-006 离线预览：S1 从默认平视角 140.5° 开始，"
              << "+ 或 = 增加 1°（低头），- 减少 1°（抬头）；"
              << "安全范围 85°～160°，q 退出；不会打开串口。\n";
    show_angle(carcar_hardware::kCameraServoLevelAngle);
    show_angle(carcar_hardware::kCameraServoLevelAngle + kAngleStep);
    show_angle(carcar_hardware::kCameraServoLevelAngle - kAngleStep);
    return 0;
  }

  TerminalMode terminal;
  if (!terminal.enable()) {
    std::cerr << "实机键盘模式必须在交互式终端运行。\n";
    return 1;
  }
  // The Python base driver does not use flock, so inspect open FDs before opening.
  const int busy = std::system("fuser -s /dev/myserial >/dev/null 2>&1");
  if (busy == -1 || !WIFEXITED(busy) || WEXITSTATUS(busy) != 1) {
    std::cerr << "串口已占用，或 fuser 检查失败；先退出其他底盘节点。\n";
    return 1;
  }
  const int fd = ::open(kSerialPort, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    std::cerr << "无法打开 " << kSerialPort << ": " << std::strerror(errno) << "\n";
    return 1;
  }
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0 || !configure_serial(fd)) {
    std::cerr << "串口独占或 115200 配置失败。\n";
    ::close(fd);
    return 1;
  }

  auto finish = [&](int result) {
      stop_requested.store(false);
      if (!send_zero_motion(fd)) {
        std::cerr << "退出零运动命令发送失败；请检查底盘并切断相关电源。\n";
        result = 1;
      }
      ::close(fd);
      std::cout << "已退出；舵机留在最后目标位置，可能继续保持扭矩。\n";
      return result;
    };
  if (!send_zero_motion(fd)) {
    std::cerr << "启动零运动命令发送失败。\n";
    return finish(1);
  }

  double angle = carcar_hardware::kCameraServoLevelAngle;
  if (!write_frame(
      fd, carcar_hardware::make_pwm_servo_command(
        1U, carcar_hardware::pwm_command_angle_from_physical(angle))))
  {
    std::cerr << "启动默认平视角命令发送失败。\n";
    return finish(1);
  }
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto state_node = std::make_shared<rclcpp::Node>("camera_servo_keyboard_state");
  auto state_pub = state_node->create_publisher<std_msgs::msg::Int32>(
    "/hardware/camera_servo/board_command_sent",
    rclcpp::QoS(1).reliable().transient_local());
  auto publish_angle = [&]() {
      std_msgs::msg::Int32 msg;
      msg.data = carcar_hardware::pwm_command_angle_from_physical(angle);
      state_pub->publish(msg);
    };
  publish_angle();
  std::cout << "按 +（或 =）低头 1°，按 - 抬头 1°；"
            << "进行目标限制在 85°～160°；按 q 或 Ctrl+C 退出。\n";
  show_angle(angle);
  auto next_zero = std::chrono::steady_clock::now() + kZeroRefresh;
  while (!stop_requested.load()) {
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int ready = ::poll(&input, 1, 50);
    if (ready < 0 && errno != EINTR) {
      std::cerr << "键盘读取失败。\n";
      return finish(1);
    }
    if (ready > 0 && (input.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      std::cerr << "终端已断开，停止控制。\n";
      return finish(1);
    }
    if (ready > 0 && (input.revents & POLLIN)) {
      char keys[32];
      const auto count = ::read(STDIN_FILENO, keys, sizeof(keys));
      if (count == 0) {
        std::cerr << "键盘输入结束，停止控制。\n";
        return finish(1);
      }
      if (count < 0 && errno != EINTR && errno != EAGAIN) {
        std::cerr << "键盘读取失败。\n";
        return finish(1);
      }
      for (ssize_t index = 0; index < count && !stop_requested.load(); ++index) {
        const char key = keys[index];
        if (key == 'q' || key == 'Q') {
          return finish(0);
        }
        const int delta = (key == '+' || key == '=') ? kAngleStep :
          (key == '-' ? -kAngleStep : 0);
        if (delta == 0) {
          continue;
        }
        const double next = carcar_hardware::clamp_camera_servo_angle(angle + delta);
        if (next == angle) {
          std::cout << "已到相机安全边界 85°～160°，不发送越界命令。\n";
          continue;
        }
        const auto board_angle = carcar_hardware::pwm_command_angle_from_physical(next);
        if (!write_frame(fd, carcar_hardware::make_pwm_servo_command(1U, board_angle))) {
          std::cerr << "舵机命令写入失败或超时，停止控制。\n";
          return finish(1);
        }
        angle = next;
        publish_angle();
        show_angle(angle);
      }
    }
    if (std::chrono::steady_clock::now() >= next_zero && !stop_requested.load()) {
      if (!send_zero_motion(fd)) {
        std::cerr << "零运动刷新失败或超时，停止控制。\n";
        return finish(1);
      }
      next_zero = std::chrono::steady_clock::now() + kZeroRefresh;
    }
  }
  return finish(0);
}
