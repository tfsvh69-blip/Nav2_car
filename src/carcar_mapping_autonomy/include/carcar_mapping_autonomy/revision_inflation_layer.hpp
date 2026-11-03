// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <memory>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <std_msgs/msg/string.hpp>

namespace carcar_mapping_autonomy {
class RevisionInflationLayer:public nav2_costmap_2d::InflationLayer {
public:
  void onInitialize()override;
  void updateBounds(double x,double y,double yaw,double*min_x,double*min_y,double*max_x,double*max_y)override;
  void updateCosts(nav2_costmap_2d::Costmap2D&master,int min_i,int min_j,int max_i,int max_j)override;
private:
  // 两阶段分别持上游锁，只有起止代次和值一致的完整重算才有资格回报。
  // 不跨方法持锁，避免上游因无更新范围跳过costs时遗留锁。
  int64_t revision_=0,cycle_revision_=0,reported_revision_=-1;double cycle_radius_=0,cycle_factor_=0;
  bool cycle_full_=false,cycle_started_=false;std::string node_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr revision_handler_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}
