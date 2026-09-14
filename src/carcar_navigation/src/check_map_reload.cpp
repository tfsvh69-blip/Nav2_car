// NAV-003/NAV-004: C++ 地图单独重载与格式验证工具（扩展版）
// 遵循约定：新增可执行逻辑统一使用 C++；以 Transient Local 订阅 /map，不依赖 /map_metadata
//
// 新增功能（NAV-004）：
//   --ros-args -p expected_map_yaml:=<path>
//     指定后：读取所选地图 YAML 的 resolution / origin，与实际 /map 的尺寸、分辨率、
//     原点和栅格内容（空闲/占用/未知 > 0）进行比对，差异时说明原因并返回 2。
//   未指定：保留旧硬编码基线（nav002: 116×113, res=0.05, origin=[-2.66,-3.01]），
//     兼容已有测试入口。
// 返回值：0=通过，1=超时，2=验证失败

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// ── 简易 YAML 行解析（不引入 yaml-cpp 依赖） ──────────────────────────────
// 只处理顶层标量字段：key: value
// 对于 origin: [x, y, z] 形式单独处理
struct MapYamlInfo {
  bool valid{false};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  // width/height 从 /map 消息本身获取，不从 YAML 读
};

static std::string trim(const std::string & s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

static MapYamlInfo parse_map_yaml(const std::string & yaml_path) {
  MapYamlInfo info;
  std::ifstream f(yaml_path);
  if (!f.is_open()) {
    std::cerr << "错误：无法打开地图 YAML 文件：" << yaml_path << "\n";
    return info;
  }
  bool got_res = false, got_origin = false;
  std::string line;
  while (std::getline(f, line)) {
    // 去掉注释
    auto comment = line.find('#');
    if (comment != std::string::npos) line = line.substr(0, comment);
    std::string t = trim(line);
    if (t.empty()) continue;

    // resolution: 0.05
    if (t.rfind("resolution:", 0) == 0) {
      std::string val = trim(t.substr(11));
      try { info.resolution = std::stod(val); got_res = true; }
      catch (...) { std::cerr << "警告：无法解析 resolution 字段\n"; }
    }
    // origin: [-2.66, -3.01, 0]
    else if (t.rfind("origin:", 0) == 0) {
      std::string val = trim(t.substr(7));
      // 移除方括号
      if (!val.empty() && val.front() == '[') val = val.substr(1);
      if (!val.empty() && val.back() == ']') val.pop_back();
      std::istringstream ss(val);
      std::string tok;
      std::vector<double> nums;
      while (std::getline(ss, tok, ',')) {
        try { nums.push_back(std::stod(trim(tok))); }
        catch (...) {}
      }
      if (nums.size() >= 2) {
        info.origin_x = nums[0];
        info.origin_y = nums[1];
        got_origin = true;
      } else {
        std::cerr << "警告：无法解析 origin 字段（期望 [x, y, z]）\n";
      }
    }
  }
  if (got_res && got_origin) {
    info.valid = true;
  } else {
    std::cerr << "错误：YAML 文件缺少 resolution 或 origin 字段：" << yaml_path << "\n";
  }
  return info;
}

// ── 主节点 ─────────────────────────────────────────────────────────────────
class MapReloadChecker : public rclcpp::Node {
public:
  MapReloadChecker() : Node("map_reload_checker"), map_received_(false) {
    // 读取参数
    this->declare_parameter<std::string>("expected_map_yaml", "");
    expected_yaml_ = this->get_parameter("expected_map_yaml").as_string();

    if (!expected_yaml_.empty()) {
      yaml_info_ = parse_map_yaml(expected_yaml_);
      if (!yaml_info_.valid) {
        RCLCPP_ERROR(this->get_logger(),
                     "无法加载期望地图 YAML：%s，将使用旧基线检查", expected_yaml_.c_str());
        expected_yaml_.clear();  // 回退
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "期望地图 YAML：%s  resolution=%.4f  origin=[%.4f, %.4f]",
                    expected_yaml_.c_str(), yaml_info_.resolution,
                    yaml_info_.origin_x, yaml_info_.origin_y);
      }
    }

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&MapReloadChecker::map_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "正在订阅 /map (Transient Local QoS)，等待地图数据...");
  }

  bool has_received() const { return map_received_; }
  bool is_valid() const { return map_valid_; }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (map_received_) return;
    map_received_ = true;

    const auto & info = msg->info;
    double res = info.resolution;
    uint32_t width = info.width;
    uint32_t height = info.height;
    double ox = info.origin.position.x;
    double oy = info.origin.position.y;
    double oz = info.origin.position.z;

    size_t total_cells = msg->data.size();
    size_t free_cells = 0;
    size_t occupied_cells = 0;
    size_t unknown_cells = 0;

    for (int8_t cell : msg->data) {
      if (cell == 0) {
        ++free_cells;
      } else if (cell == 100) {
        ++occupied_cells;
      } else {
        ++unknown_cells;
      }
    }

    // 打印头部
    std::cout << "\n========================================\n";
    if (!expected_yaml_.empty()) {
      std::cout << "     NAV-004 地图重载验证报告（YAML 对照）  \n";
    } else {
      std::cout << "        NAV-003 地图重载验证报告         \n";
    }
    std::cout << "========================================\n";
    std::cout << "帧 ID (frame_id)      : " << msg->header.frame_id << "\n";
    std::cout << "分辨率 (resolution)   : " << res << " m/cell\n";
    std::cout << "尺寸 (width x height) : " << width << " x " << height << "\n";
    std::cout << "原点 (origin)         : [" << ox << ", " << oy << ", " << oz << "]\n";
    std::cout << "栅格统计              : 总计 " << total_cells
              << " 格 (空闲 " << free_cells << ", 占用 " << occupied_cells
              << ", 未探明 " << unknown_cells << ")\n";

    bool ok_res, ok_size, ok_origin, ok_content;

    if (!expected_yaml_.empty()) {
      // ── 动态基线（从 YAML 读取） ─────────────────────────────────────────
      // 分辨率
      ok_res = std::abs(res - yaml_info_.resolution) < 1e-4;
      // 期望尺寸：从 /map 消息本身获取（仅验证非零）
      ok_size = (width > 0 && height > 0);
      // 原点
      ok_origin = (std::abs(ox - yaml_info_.origin_x) < 0.01 &&
                   std::abs(oy - yaml_info_.origin_y) < 0.01);
      // 内容
      ok_content = (free_cells > 0 && occupied_cells > 0);

      std::cout << "--- 规则判定（YAML 对照）---\n";
      std::cout << "  分辨率 (" << yaml_info_.resolution << " m)  : "
                << (ok_res ? "PASS" : "FAIL") << "\n";
      if (!ok_res) {
        std::cout << "    ↳ 实际 " << res << " m，期望 " << yaml_info_.resolution
                  << " m（差 " << std::abs(res - yaml_info_.resolution) << " m）\n";
      }
      std::cout << "  尺寸非零 (" << width << " x " << height << ") : "
                << (ok_size ? "PASS" : "FAIL") << "\n";
      std::cout << "  原点 [" << yaml_info_.origin_x << ", " << yaml_info_.origin_y
                << "] : " << (ok_origin ? "PASS" : "FAIL") << "\n";
      if (!ok_origin) {
        std::cout << "    ↳ 实际 [" << ox << ", " << oy << "]，误差 ["
                  << std::abs(ox - yaml_info_.origin_x) << ", "
                  << std::abs(oy - yaml_info_.origin_y) << "]\n";
      }
      std::cout << "  占据与空闲非空       : " << (ok_content ? "PASS" : "FAIL") << "\n";
      if (!ok_content) {
        std::cout << "    ↳ free=" << free_cells << "，occupied=" << occupied_cells
                  << "（地图为空或未更新）\n";
      }
    } else {
      // ── 旧硬编码基线（nav002: 116×113, res=0.05, origin=[-2.66,-3.01]） ──
      ok_res    = std::abs(res - 0.05) < 1e-4;
      ok_size   = (width == 116 && height == 113);
      ok_origin = (std::abs(ox - (-2.66)) < 0.01 && std::abs(oy - (-3.01)) < 0.01);
      ok_content = (free_cells > 0 && occupied_cells > 0);

      std::cout << "--- 规则判定（nav002 基线）---\n";
      std::cout << "  分辨率 (0.05 m)    : " << (ok_res ? "PASS" : "FAIL") << "\n";
      std::cout << "  尺寸 (116 x 113)   : " << (ok_size ? "PASS" : "FAIL") << "\n";
      std::cout << "  原点 [-2.66,-3.01] : " << (ok_origin ? "PASS" : "FAIL") << "\n";
      std::cout << "  占据与空闲非空     : " << (ok_content ? "PASS" : "FAIL") << "\n";
    }
    std::cout << "========================================\n";

    map_valid_ = (ok_res && ok_size && ok_origin && ok_content);
    if (!map_valid_) {
      std::cerr << "地图验证失败，请核对地图服务加载的是正确文件，或使用 "
                   "--ros-args -p expected_map_yaml:=<path> 指定期望地图\n";
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  bool map_received_;
  bool map_valid_{false};
  std::string expected_yaml_;
  MapYamlInfo yaml_info_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapReloadChecker>();

  rclcpp::Rate loop_rate(10);
  auto start_time = std::chrono::steady_clock::now();
  const auto timeout = 5s;

  while (rclcpp::ok() && !node->has_received()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout) {
      std::cerr << "错误：5 秒超时未收到 /map 消息！请确认 map_server 是否处于 active 状态。\n";
      rclcpp::shutdown();
      return 1;
    }
  }

  bool passed = node->is_valid();
  rclcpp::shutdown();
  return passed ? 0 : 2;
}
