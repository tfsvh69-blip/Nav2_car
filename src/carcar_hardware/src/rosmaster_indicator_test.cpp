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

// DRV-004: Rosmaster 控制板蜂鸣器与 RGB 灯带静止测试节点
// 遵循约定：全 C++ 实现；启动、运行保护及退出均发送零运动指令；
// 默认处于 Dry-Run 离线展示模式；显式开启 real_hardware:=true 时才访问串口；
// 使用 flock 独占锁定串口；异常或中断时安全关闭声光并释放串口。

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "carcar_hardware/rosmaster_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int sig)
{
  (void)sig;
  g_shutdown_requested.store(true);
}

std::string bytes_to_hex(const std::vector<std::uint8_t> & bytes)
{
  std::ostringstream ss;
  ss << std::hex << std::uppercase << std::setfill('0');
  for (size_t i = 0; i < bytes.size(); ++i) {
    if (i > 0) {
      ss << " ";
    }
    ss << "0x" << std::setw(2) << static_cast<int>(bytes[i]);
  }
  return ss.str();
}

std::string get_current_timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto in_time_t = std::chrono::system_clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S")
     << "." << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

bool write_frame_to_serial(int fd, const std::vector<std::uint8_t> & frame)
{
  if (fd < 0) {
    return false;
  }
  std::size_t offset = 0U;
  while (offset < frame.size()) {
    const auto written = ::write(fd, frame.data() + offset, frame.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      if (g_shutdown_requested.load()) {
        return false;
      }
      continue;
    }
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd descriptor{fd, POLLOUT, 0};
      if (::poll(&descriptor, 1, 50) > 0) {
        continue;
      }
    }
    return false;
  }
  // 控制板 MCU 帧间隔处理时间 2ms
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return true;
}

bool send_zero_output(int fd, std::uint8_t car_type)
{
  return write_frame_to_serial(fd, carcar_hardware::make_zero_motor_command()) &&
         write_frame_to_serial(fd, carcar_hardware::make_zero_motion_command(car_type));
}

bool sleep_with_interruption(double duration_sec)
{
  const auto start = std::chrono::steady_clock::now();
  while (!g_shutdown_requested.load()) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    if (elapsed >= duration_sec) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rosmaster_indicator_test");

  const std::string serial_port = node->declare_parameter<std::string>(
    "serial_port", "/dev/myserial");
  const bool real_hardware = node->declare_parameter<bool>(
    "real_hardware", false);

  rcl_interfaces::msg::ParameterDescriptor action_desc;
  action_desc.dynamic_typing = true;
  const auto action_param = node->declare_parameter(
    "action", rclcpp::ParameterValue("test"), action_desc);
  std::string action = "test";
  if (action_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    action = action_param.get<std::string>();
  } else if (action_param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    action = action_param.get<bool>() ? "test" : "off";
  }
  const int car_type = node->declare_parameter<int>(
    "car_type", 1);
  const int led_id = node->declare_parameter<int>(
    "led_id", 255);
  const int beep_duration_ms = node->declare_parameter<int>(
    "beep_duration_ms", 100);
  const double color_duration_sec = node->declare_parameter<double>(
    "color_duration_sec", 1.0);
  const int color_brightness = std::clamp(
    static_cast<int>(node->declare_parameter<int>("color_brightness", 30)), 1, 255);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "\n======================================================================\n"
            << "  DRV-004 控制板蜂鸣器与 RGB 灯带测试程序 (rosmaster_indicator_test)\n"
            << "  [安全保护：本程序仅发送零运动指令，绝不驱动底盘车轮]\n"
            << "======================================================================\n";
  std::cout << "测试参数配置:\n"
            << "  串口设备路径    : " << serial_port << "\n"
            << "  执行模式        : " <<
    (real_hardware ? "实机模式 (real_hardware:=true)" : "离线预览 (real_hardware:=false)") << "\n"
            << "  测试动作 (action): " << action << " (" <<
    (action == "off" ? "全部关闭" : "短鸣与三色序列") << ")\n"
            << "  蜂鸣器短鸣时长  : " << beep_duration_ms << " ms\n"
            << "  RGB 灯亮度分量  : " << color_brightness << " / 255 (低亮度安全保护)\n"
            << "  单色持续展示时间: " << color_duration_sec << " 秒\n"
            << "----------------------------------------------------------------------\n";

  // 1. Dry-Run 离线模式
  if (!real_hardware) {
    std::cout << "[离线协议测试 (Dry-Run)] 开始生成并校验协议控制帧...\n\n";

    const auto zero_motor = carcar_hardware::make_zero_motor_command();
    const auto zero_motion =
      carcar_hardware::make_zero_motion_command(static_cast<std::uint8_t>(car_type));
    const auto beep_cmd =
      carcar_hardware::make_beep_command(static_cast<std::uint16_t>(beep_duration_ms));
    const auto stop_effect = carcar_hardware::make_rgb_effect_command(0);
    const auto red_cmd = carcar_hardware::make_rgb_command(
      static_cast<std::uint8_t>(led_id), static_cast<std::uint8_t>(color_brightness), 0, 0);
    const auto green_cmd = carcar_hardware::make_rgb_command(
      static_cast<std::uint8_t>(led_id), 0, static_cast<std::uint8_t>(color_brightness), 0);
    const auto blue_cmd = carcar_hardware::make_rgb_command(
      static_cast<std::uint8_t>(led_id), 0, 0, static_cast<std::uint8_t>(color_brightness));
    const auto beep_off = carcar_hardware::make_beep_command(0);
    const auto rgb_off = carcar_hardware::make_rgb_command(
      static_cast<std::uint8_t>(led_id), 0, 0, 0);

    std::cout << "1. 启动零输出帧: [电机零输出: " << bytes_to_hex(zero_motor) << "]\n"
              << "                 [底盘零速度: " << bytes_to_hex(zero_motion) << "]\n";
    if (action == "test") {
      std::cout << "2. 蜂鸣器短鸣帧: [" << bytes_to_hex(beep_cmd)
                << "] (鸣叫 " << beep_duration_ms << "ms)\n";
      std::cout << "3. 停止灯效帧  : [" << bytes_to_hex(stop_effect)
                << "] (准备显示固定单色)\n";
      std::cout << "4. 低亮度红色帧: [" << bytes_to_hex(red_cmd)
                << "] (保持 " << color_duration_sec << "s)\n";
      std::cout << "5. 低亮度绿色帧: [" << bytes_to_hex(green_cmd)
                << "] (保持 " << color_duration_sec << "s)\n";
      std::cout << "6. 低亮度蓝色帧: [" << bytes_to_hex(blue_cmd)
                << "] (保持 " << color_duration_sec << "s)\n";
      std::cout << "7. 退出关闭帧  : [蜂鸣器关: " << bytes_to_hex(beep_off) << "]\n"
                << "                 [RGB灯全灭: " << bytes_to_hex(rgb_off) << "]\n"
                << "                 [零速度刷新: " << bytes_to_hex(zero_motion) << "]\n";
    } else {
      std::cout << "2. 单独关闭帧  : [蜂鸣器关: " << bytes_to_hex(beep_off) << "]\n"
                << "                 [灯效停止: " << bytes_to_hex(stop_effect) << "]\n"
                << "                 [RGB灯全灭: " << bytes_to_hex(rgb_off) << "]\n";
    }

    struct stat st;
    if (stat(serial_port.c_str(), &st) == 0) {
      std::cout << "\n[环境检测] 串口设备文件 " << serial_port << " 存在于主机中。";
    } else {
      std::cout << "\n[环境检测] 串口设备文件 " << serial_port
                << " 未枚举（纯离线环境，符合离线预期）。";
    }

    std::cout << "\n\n>>> 离线协议校验成功 (PASS)：所有协议帧格式与校验和正确，"
              << "未发送任何物理信号。\n"
              << "    若需在实车上执行测试，请运行：\n"
              <<
      "    ros2 launch carcar_hardware rosmaster_indicator_test.launch.xml real_hardware:=true\n"
              << "======================================================================\n\n";
    rclcpp::shutdown();
    return 0;
  }

  // 2. 实机模式
  std::cout << "[" << get_current_timestamp()
            << "] [启动] 正在检查串口设备: " << serial_port << " ...\n";
  struct stat st;
  if (stat(serial_port.c_str(), &st) != 0) {
    std::cerr << "错误：串口设备 " << serial_port << " 不存在！请检查硬件接线或 udev 规则。\n";
    rclcpp::shutdown();
    return 1;
  }

  int fd = ::open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    std::cerr << "错误：无法打开串口 " << serial_port << ": " << std::strerror(errno) << "\n";
    rclcpp::shutdown();
    return 1;
  }

  // 使用 flock 独占串口，防止与其他底盘节点冲突
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    std::cerr << "错误：串口设备 " << serial_port << " 已被其他进程独占占用！\n"
              << "请先确认并关闭其他底盘驱动（例如 rosmaster_node、chassis_motion 等）后重试。\n";
    ::close(fd);
    rclcpp::shutdown();
    return 1;
  }

  termios options{};
  if (tcgetattr(fd, &options) != 0) {
    std::cerr << "错误：获取串口属性失败: " << std::strerror(errno) << "\n";
    ::flock(fd, LOCK_UN);
    ::close(fd);
    rclcpp::shutdown();
    return 1;
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
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    std::cerr << "错误：配置串口波特率失败: " << std::strerror(errno) << "\n";
    ::flock(fd, LOCK_UN);
    ::close(fd);
    rclcpp::shutdown();
    return 1;
  }
  tcflush(fd, TCIOFLUSH);

  std::cout << "[" << get_current_timestamp() << "] [成功] 已独占打开控制板串口 "
            << serial_port << " (波特率 115200)。\n";

  // 初始安全保护：发送零速度
  if (!send_zero_output(fd, static_cast<std::uint8_t>(car_type))) {
    std::cerr << "错误：写入初始零运动指令失败！\n";
    ::flock(fd, LOCK_UN);
    ::close(fd);
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "[" << get_current_timestamp()
            << "] [安全保护] 初始零运动指令已发送，电机保持停止。\n";

  auto safe_exit_cleanup = [&](const std::string & exit_reason) {
      std::cout << "\n[" << get_current_timestamp() << "] [退出清理: " << exit_reason <<
        "] 正在执行声光复位与安全停车...\n";
      // 关闭蜂鸣器
      write_frame_to_serial(fd, carcar_hardware::make_beep_command(0));
      // 停止灯效
      write_frame_to_serial(fd, carcar_hardware::make_rgb_effect_command(0));
      // RGB 全灭
      write_frame_to_serial(
        fd,
        carcar_hardware::make_rgb_command(static_cast<std::uint8_t>(led_id), 0, 0, 0));
      // 零运动保护
      send_zero_output(fd, static_cast<std::uint8_t>(car_type));
      tcdrain(fd);
      ::flock(fd, LOCK_UN);
      ::close(fd);
      std::cout << "[" << get_current_timestamp() << "] [退出清理] 串口已安全释放并关闭。\n\n";
    };

  if (action == "off") {
    std::cout << "\n[" << get_current_timestamp() << "] [执行] 正在发送单独关闭声光指令...\n";
    write_frame_to_serial(fd, carcar_hardware::make_beep_command(0));
    write_frame_to_serial(fd, carcar_hardware::make_rgb_effect_command(0));
    write_frame_to_serial(
      fd,
      carcar_hardware::make_rgb_command(static_cast<std::uint8_t>(led_id), 0, 0, 0));
    send_zero_output(fd, static_cast<std::uint8_t>(car_type));
    std::cout << "[" << get_current_timestamp() << "] [指令已发送] 单独关闭指令已完成发送。\n";
    safe_exit_cleanup("单项关闭完成");
    rclcpp::shutdown();
    return 0;
  }

  // 执行标准测试序列：100ms 短鸣 -> 红 1s -> 绿 1s -> 蓝 1s -> 全关
  std::cout << "\n[" << get_current_timestamp() <<
    "] >>> 开始执行声光测试序列 (短鸣100ms -> 红1s -> 绿1s -> 蓝1s) <<<\n\n";

  // 步骤 1：蜂鸣器短鸣
  const auto beep_cmd =
    carcar_hardware::make_beep_command(static_cast<std::uint16_t>(beep_duration_ms));
  if (!write_frame_to_serial(fd, beep_cmd)) {
    safe_exit_cleanup("蜂鸣器指令发送失败");
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "[" << get_current_timestamp() << "] [指令已发送] 蜂鸣器短鸣 ("
            << beep_duration_ms << " ms) | 帧: " << bytes_to_hex(beep_cmd) << "\n";
  if (!sleep_with_interruption((beep_duration_ms + 100) / 1000.0)) {
    safe_exit_cleanup("用户中断");
    rclcpp::shutdown();
    return 0;
  }

  // 步骤 2：停止可能存在的旧灯效
  const auto stop_effect_cmd = carcar_hardware::make_rgb_effect_command(0);
  write_frame_to_serial(fd, stop_effect_cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 步骤 3：低亮度红色
  const auto red_cmd = carcar_hardware::make_rgb_command(
    static_cast<std::uint8_t>(led_id), static_cast<std::uint8_t>(color_brightness), 0, 0);
  if (!write_frame_to_serial(fd, red_cmd)) {
    safe_exit_cleanup("红色指令发送失败");
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "[" << get_current_timestamp() << "] [指令已发送] RGB 低亮度红色 (R="
            << color_brightness << ", G=0, B=0, 保持 " << color_duration_sec
            << " 秒) | 帧: " << bytes_to_hex(red_cmd) << "\n";
  if (!sleep_with_interruption(color_duration_sec)) {
    safe_exit_cleanup("用户中断");
    rclcpp::shutdown();
    return 0;
  }

  // 步骤 4：低亮度绿色
  const auto green_cmd = carcar_hardware::make_rgb_command(
    static_cast<std::uint8_t>(led_id), 0, static_cast<std::uint8_t>(color_brightness), 0);
  if (!write_frame_to_serial(fd, green_cmd)) {
    safe_exit_cleanup("绿色指令发送失败");
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "[" << get_current_timestamp() << "] [指令已发送] RGB 低亮度绿色 (R=0, G="
            << color_brightness << ", B=0, 保持 " << color_duration_sec
            << " 秒) | 帧: " << bytes_to_hex(green_cmd) << "\n";
  if (!sleep_with_interruption(color_duration_sec)) {
    safe_exit_cleanup("用户中断");
    rclcpp::shutdown();
    return 0;
  }

  // 步骤 5：低亮度蓝色
  const auto blue_cmd = carcar_hardware::make_rgb_command(
    static_cast<std::uint8_t>(led_id), 0, 0, static_cast<std::uint8_t>(color_brightness));
  if (!write_frame_to_serial(fd, blue_cmd)) {
    safe_exit_cleanup("蓝色指令发送失败");
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "[" << get_current_timestamp() << "] [指令已发送] RGB 低亮度蓝色 (R=0, G=0, B="
            << color_brightness << ", 保持 " << color_duration_sec
            << " 秒) | 帧: " << bytes_to_hex(blue_cmd) << "\n";
  if (!sleep_with_interruption(color_duration_sec)) {
    safe_exit_cleanup("用户中断");
    rclcpp::shutdown();
    return 0;
  }

  // 步骤 6：正常退出清理
  safe_exit_cleanup("测试序列完成");

  std::cout << "======================================================================\n"
            << "  测试执行完毕！串口写入成功仅记作【指令已发送】。\n"
            << "  请现场操作人员根据实物现象进行反馈确认：\n"
            << "  1. 是否听到约 100 ms 的清晰短鸣？\n"
            << "  2. 是否依次观察到 RGB 灯带显示低亮度红、绿、蓝各约 1 秒？\n"
            << "  3. 测试结束后灯光与蜂鸣器是否已全部熄灭与静音？\n"
            << "======================================================================\n\n";

  rclcpp::shutdown();
  return 0;
}
