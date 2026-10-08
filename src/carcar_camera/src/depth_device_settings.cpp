// Copyright 2026 carcar maintainers; Apache-2.0.
// 显式手动入口：备份/恢复已有高级深度配置，不启动流、不切换高级模式、不复位USB。
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <librealsense2/rs_advanced_mode.hpp>

int main(int argc, char ** argv)
{
  if (argc != 4 || (std::string(argv[1]) != "save" && std::string(argv[1]) != "restore")) {
    std::cerr << "用法: depth_device_settings save|restore <精确序列号> <JSON路径>\n";
    return 2;
  }
  try {
    const std::string mode(argv[1]), serial(argv[2]);
    const std::filesystem::path path(argv[3]);
    if (serial.empty()) {throw std::invalid_argument("序列号不能为空");}
    std::string backup;
    if (mode == "save") {
      if (std::filesystem::exists(path)) {throw std::runtime_error("备份已存在，拒绝覆盖");}
    } else {
      if (!std::filesystem::is_regular_file(path) ||
        std::filesystem::file_size(path) == 0 || std::filesystem::file_size(path) > 1048576) {
        throw std::runtime_error("恢复文件不存在、为空或超过1MiB");
      }
      std::ifstream in(path);
      backup.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
      if (in.bad()) {throw std::runtime_error("无法读取恢复文件");}
    }
    rs2::context context;
    for (auto device : context.query_devices()) {
      if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ||
        serial != device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {continue;}
      rs400::advanced_mode advanced(device);
      if (!advanced || !advanced.is_enabled()) {
        throw std::runtime_error("当前设备未启用高级模式；本工具不会切换模式或复位");
      }
      if (mode == "save") {
        const auto json = advanced.serialize_json();
        std::ofstream out(path);
        out << json;
        out.close();
        if (!out) {throw std::runtime_error("备份写入失败");}
        std::cout << "已备份精确设备的高级测距配置；未启动流或修改设置：" << path << '\n';
      } else {
        advanced.load_json(backup);
        std::cout << "已恢复精确设备的高级测距配置；需重新核对ROS参数及数据健康\n";
      }
      return 0;
    }
    throw std::runtime_error("未找到指定序列号设备");
  } catch (const std::exception & error) {
    std::cerr << "深度配置操作失败：" << error.what() << '\n';
    return 1;
  }
}
