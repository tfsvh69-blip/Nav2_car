// NAV-003: C++ 地图单独重载与格式验证工具
// 遵循约定：新增可执行逻辑统一使用 C++；以 Transient Local 订阅 /map，不依赖 /map_metadata

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MapReloadChecker : public rclcpp::Node {
public:
  MapReloadChecker() : Node("map_reload_checker"), map_received_(false) {
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
    if (map_received_) {
      return;
    }
    map_received_ = true;

    const auto &info = msg->info;
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

    std::cout << "\n========================================\n";
    std::cout << "        NAV-003 地图重载验证报告         \n";
    std::cout << "========================================\n";
    std::cout << "帧 ID (frame_id)   : " << msg->header.frame_id << "\n";
    std::cout << "分辨率 (resolution): " << res << " m/cell\n";
    std::cout << "尺寸 (width x height): " << width << " x " << height << "\n";
    std::cout << "原点 (origin)      : [" << ox << ", " << oy << ", " << oz << "]\n";
    std::cout << "栅格统计           : 总计 " << total_cells
              << " 格 (空闲 " << free_cells << ", 占用 " << occupied_cells
              << ", 未探明 " << unknown_cells << ")\n";

    // 核验预期基准 (nav002: 116 x 113, res=0.05, origin=[-2.66, -3.01, 0])
    bool ok_res = std::abs(res - 0.05) < 1e-4;
    bool ok_size = (width == 116 && height == 113);
    bool ok_origin = (std::abs(ox - (-2.66)) < 0.01 && std::abs(oy - (-3.01)) < 0.01);
    bool ok_content = (free_cells > 0 && occupied_cells > 0);

    std::cout << "--- 规则判定 ---\n";
    std::cout << "  分辨率 (0.05 m)   : " << (ok_res ? "PASS" : "FAIL") << "\n";
    std::cout << "  尺寸 (116 x 113)  : " << (ok_size ? "PASS" : "FAIL") << "\n";
    std::cout << "  原点 [-2.66,-3.01]: " << (ok_origin ? "PASS" : "FAIL") << "\n";
    std::cout << "  占据与空闲非空    : " << (ok_content ? "PASS" : "FAIL") << "\n";
    std::cout << "========================================\n";

    map_valid_ = (ok_res && ok_size && ok_origin && ok_content);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  bool map_received_;
  bool map_valid_{false};
};

int main(int argc, char **argv) {
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

