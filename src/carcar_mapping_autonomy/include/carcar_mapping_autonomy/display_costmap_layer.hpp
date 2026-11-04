// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <mutex>
#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace carcar_mapping_autonomy {
// 放在普通插件末尾：只读过滤前栅格，不修改边界、代价、健康状态或安全过滤器。
class DisplayCostmapLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override;
  void updateBounds(double,double,double,double*,double*,double*,double*) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &,int,int,int,int) override;
  void reset() override;
  bool isClearable() override {return false;}
  void activate() override;
  void deactivate() override;
private:
  void publishPending();
  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr pending_;
  std::chrono::steady_clock::time_point last_{};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
