#pragma once
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include "carcar_mapping_autonomy/subcell_departure.hpp"

namespace carcar_mapping_autonomy {
// 复用Humble Smac搜索，只在离散起点不可用时增加经过审查的连续短连接。
class SubcellSmacPlanner : public nav2_smac_planner::SmacPlanner2D {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) override;
private:
  nav_msgs::msg::Path checkedPlan(const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal);
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::chrono::steady_clock::time_point map_at_;
  std::mutex map_mutex_,planning_mutex_;
  double dynamic_radius_=0;
};
}  // namespace carcar_mapping_autonomy
