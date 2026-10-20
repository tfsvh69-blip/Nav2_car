#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/serial_preflight.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <iostream>
using namespace carcar_mapping_console;
int main(int argc, char ** argv) {
  try {
    bool hardware = false, backend = false; std::string mode = "new", bundle, root = (std::filesystem::current_path()/"src/carcar_navigation/maps/mapping_sessions").string();
    std::string serial = "/dev/myserial", lidar = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0", slam_params;
    for (int i = 1; i < argc; ++i) {
      std::string key = argv[i];
      if (key == "--hardware") {hardware = true; continue;}
      if (key == "--backend-only") {backend = true; continue;}
      if (i + 1 >= argc) throw std::runtime_error("缺少参数值");
      const std::string value = argv[++i];
      if (key == "--mode") mode = value; else if (key == "--bundle") bundle = value;
      else if (key == "--map-root") root = value; else if (key == "--serial") serial = value;
      else if (key == "--lidar") lidar = value;
      else if (key == "--slam-params") slam_params = value;
      else throw std::runtime_error("未知参数 " + key);
    }
    if (mode != "new" && mode != "resume") throw std::runtime_error("mode 必须为 new 或 resume");
    if (!slam_params.empty()) {
      if (mode != "new") throw std::runtime_error("候选SLAM参数仅用于新建对照；旧位姿图含原传感器范围，请使用原入口续建");
      if (!std::filesystem::is_regular_file(slam_params)) throw std::runtime_error("候选SLAM参数文件不存在");
      slam_params = std::filesystem::canonical(slam_params).string();
    }
    root = std::filesystem::absolute(root).string(); if (mode == "resume") validate_bundle(root, bundle);
    rclcpp::init(0, nullptr); auto node = std::make_shared<rclcpp::Node>("mapping_preflight");
    const auto until = steady() + 1.0;
    while (steady() < until) {rclcpp::spin_some(node); usleep(20000);}
    for (const auto & topic : {"/mapping/cmd_vel", "/mapping/slam_scan", "/map"}) {
      const auto pubs = node->get_publishers_info_by_topic(topic);
      const bool own_relay = backend && std::string(topic) == "/mapping/slam_scan" && pubs.size() == 1 &&
        pubs.front().node_name() == "mapping_web_gateway" && pubs.front().node_namespace() == "/";
      if (!pubs.empty() && !own_relay) throw std::runtime_error(std::string("发现已有发布者，先正常关闭旧入口: ") + topic);
    }
    if (hardware) {
      for (const auto & port : {serial, lidar}) require_unused_serial(port);
      std::cout << "将启动底盘和雷达；底盘会发送 S1=94，相机支架可能旋转。速度初始为零。\n";
    } else std::cout << "仅软件页面：不启动底盘、雷达、相机。\n";
    node.reset(); rclcpp::shutdown();
    std::vector<std::string> arguments{"ros2", "launch", "carcar_mapping_console", "mapping.launch.xml",
      std::string("start_hardware:=") + (hardware ? "true" : "false"), "mode:="+mode,
      "map_root:="+root, "serial_port:="+serial, "lidar_port:="+lidar};
    if (!bundle.empty()) arguments.push_back("bundle_id:=" + bundle);
    if (!slam_params.empty()) arguments.push_back("slam_params_file:=" + slam_params);
    if (backend) {
      arguments.push_back("start_gateway:=false"); arguments.push_back("start_camera:=false");
      arguments.push_back("require_video:=false");
    }
    std::vector<char *> exec_arguments; for (auto & a : arguments) exec_arguments.push_back(a.data()); exec_arguments.push_back(nullptr);
    execvp(exec_arguments[0], exec_arguments.data()); throw std::runtime_error("无法执行 ros2，请加载环境");
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
