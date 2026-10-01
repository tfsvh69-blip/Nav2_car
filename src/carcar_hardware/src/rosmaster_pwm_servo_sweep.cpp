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
// Independent S1 PWM servo test. The board receives 0..180 for 0..270 physical degrees.
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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "carcar_hardware/rosmaster_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::atomic<bool> stop_requested{false};
constexpr char kSerialPort[] = "/dev/myserial";
constexpr auto kStepInterval = std::chrono::milliseconds(100);
constexpr auto kEndpointHold = std::chrono::seconds(1);
constexpr auto kCommandTimeout = std::chrono::milliseconds(500);

void handle_signal(int signal_number)
{
  static_cast<void>(signal_number);
  stop_requested.store(true);
}

bool sleep_until_or_stop(std::chrono::steady_clock::time_point deadline)
{
  while (!stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return !stop_requested.load();
}

bool write_frame(int fd, const std::vector<std::uint8_t> & frame)
{
  const auto deadline = std::chrono::steady_clock::now() + kCommandTimeout;
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
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    pollfd descriptor{fd, POLLOUT, 0};
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0 || ::poll(&descriptor, 1, static_cast<int>(remaining)) <= 0) {
      return false;
    }
  }
  return offset == frame.size();
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
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rosmaster_pwm_servo_sweep");
  const bool real_hardware = node->declare_parameter<bool>("real_hardware", false);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  // The historical DRV-005 sweep exceeds the D435's reported 85..160-degree
  // range now that its mechanical limits are known.
  if (real_hardware) {
    std::cerr << "DRV-005 实机往返已停用：D435 已安装在舵机上，"
              << "0°～270° 会越过 85°～160° 的安全范围。"
              << "请使用 rosmaster_pwm_servo_keyboard。\n";
    rclcpp::shutdown();
    return 2;
  }

  if (!real_hardware) {
    std::cout << "DRV-005 离线预览；不会打开串口或驱动舵机。\n";
    for (const int angle : {0, 135, 270}) {
      const auto board_angle = carcar_hardware::pwm_command_angle_from_physical(angle);
      std::cout << "S1 机械角 " << angle << "° -> 控制板角 "
                << static_cast<int>(board_angle) << "°\n";
    }
    rclcpp::shutdown();
    return 0;
  }

  // The existing Python base driver does not use flock, so check its open FD too.
  const int busy = std::system("fuser -s /dev/myserial >/dev/null 2>&1");
  if (busy == -1 || !WIFEXITED(busy) || WEXITSTATUS(busy) != 1) {
    std::cerr << "串口已被占用，或 fuser 检查失败；关闭其他底盘进程后重试。\n";
    rclcpp::shutdown();
    return 1;
  }
  const int fd = ::open(kSerialPort, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    std::cerr << "无法打开 " << kSerialPort << ": " << std::strerror(errno) << "\n";
    rclcpp::shutdown();
    return 1;
  }
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0 || !configure_serial(fd)) {
    std::cerr << "串口独占或 115200 配置失败。\n";
    ::close(fd);
    rclcpp::shutdown();
    return 1;
  }

  // Cleanup must still attempt zero output after a signal; it does not change S1's position.
  auto finish = [&](int result) {
      stop_requested.store(false);
      if (!send_zero_motion(fd)) {
        std::cerr << "退出零运动命令发送失败；请检查底盘并切断相关电源。\n";
        result = 1;
      }
      ::close(fd);
      rclcpp::shutdown();
      std::cout << "舵机停在最后命令位置；PWM 可能继续保持扭矩。\n";
      return result;
    };
  if (!send_zero_motion(fd)) {
    std::cerr << "启动零运动命令发送失败。\n";
    return finish(1);
  }

  std::cout << "S1 实机连续往返：0°→270°→0°；Ctrl+C 停止。\n";
  int angle = 0;
  int direction = 1;
  while (!stop_requested.load()) {
    const auto started = std::chrono::steady_clock::now();
    const auto board_angle = carcar_hardware::pwm_command_angle_from_physical(angle);
    if (!write_frame(fd, carcar_hardware::make_pwm_servo_command(1U, board_angle))) {
      std::cerr << "舵机命令写入失败或超时，停止往返。\n";
      return finish(1);
    }
    std::cout << "S1 机械目标 " << angle << "°，控制板 "
              << static_cast<int>(board_angle) << "°，方向 "
              << (direction > 0 ? "增加" : "减小") << std::endl;
    if (std::chrono::steady_clock::now() - started > kCommandTimeout) {
      std::cerr << "命令执行超时，停止往返。\n";
      return finish(1);
    }
    const bool endpoint = (angle == 0 && direction < 0) ||
      (angle == 270 && direction > 0);
    if (!sleep_until_or_stop(started + (endpoint ? kEndpointHold : kStepInterval))) {
      break;
    }
    if (!endpoint && std::chrono::steady_clock::now() - started > kCommandTimeout) {
      std::cerr << "命令周期超时，停止往返。\n";
      return finish(1);
    }
    if (endpoint) {
      direction = -direction;
    }
    angle += direction * 5;
  }
  return finish(0);
}
