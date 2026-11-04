// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include "carcar_mapping_autonomy/display_costmap_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace carcar_mapping_autonomy {
void DisplayCostmapLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) throw std::runtime_error("显示出口缺少所属节点");
  enabled_ = true; current_ = true;
  publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "costmap_before_filters", rclcpp::QoS(1).reliable().transient_local());
  publisher_->on_activate();
  // DDS发送与地图更新分离，慢网页不会在代价计算持锁期间等待传输。
  timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]{publishPending();});
}
void DisplayCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D & map,int,int,int,int) {
  const auto at = std::chrono::steady_clock::now();
  if (at - last_ < std::chrono::milliseconds(400)) return;
  last_ = at;
  try {
    const uint64_t size = uint64_t(map.getSizeInCellsX()) * map.getSizeInCellsY();
    if (!size || size > 4000000) return;
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    msg->header.frame_id = layered_costmap_->getGlobalFrameID(); msg->header.stamp = clock_->now();
    msg->info.width = map.getSizeInCellsX(); msg->info.height = map.getSizeInCellsY();
    msg->info.resolution = map.getResolution();
    msg->info.origin.position.x = map.getOriginX(); msg->info.origin.position.y = map.getOriginY();
    msg->info.origin.orientation.w = 1;
    msg->data.resize(size);
    // 与Nav2发布的OccupancyGrid编码一致；读取真实值，不估计被禁入覆盖的梯度。
    const auto * raw = map.getCharMap();
    for (size_t i = 0; i < size; ++i) {
      const auto v = raw[i];
      msg->data[i] = v == 255 ? -1 : v == 254 ? 100 : v == 253 ? 99 : v == 0 ? 0 : 1 + 97 * (v - 1) / 251;
    }
    std::lock_guard<std::mutex> lock(mutex_); pending_ = std::move(msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "显示快照不可用，原导航计算不变：%s", e.what());
  }
}
void DisplayCostmapLayer::publishPending() {
  nav_msgs::msg::OccupancyGrid::SharedPtr msg;
  {std::lock_guard<std::mutex> lock(mutex_); msg.swap(pending_);}
  try {if (msg && publisher_->is_activated()) publisher_->publish(*msg);}
  catch (const std::exception & e) {RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "显示快照发送失败：%s", e.what());}
}
void DisplayCostmapLayer::reset() {std::lock_guard<std::mutex> lock(mutex_);pending_.reset();}
void DisplayCostmapLayer::activate() {publisher_->on_activate();}
void DisplayCostmapLayer::deactivate() {publisher_->on_deactivate();reset();}
}
PLUGINLIB_EXPORT_CLASS(carcar_mapping_autonomy::DisplayCostmapLayer, nav2_costmap_2d::Layer)
