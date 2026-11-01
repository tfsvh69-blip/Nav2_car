#pragma once
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "carcar_mapping_autonomy/task.hpp"
namespace carcar_mapping_autonomy {
struct ExplorationPoint {Pose target,frontier;double gain=0,cost=0;std::vector<uint64_t> predicted_cells{};};
struct ExplorationVisit {Pose target;double gain=0,until=0;bool reached=false;std::vector<uint64_t> predicted_cells{};uint64_t revision=0;};
struct ExplorationRouteInput {
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  std::shared_ptr<const nav_msgs::msg::Path> active_path;
  Task task;Pose current;
  uint64_t revision=0,generation=0,navigation_generation=0,epoch=0;
  double now=0,speed=.10;
  std::vector<ExplorationPoint> prefix;
  std::vector<ExplorationVisit> visits;
};
struct ExplorationRouteResult {
  ExplorationRouteInput input;
  std::vector<ExplorationPoint> route;
  Json candidates=Json::array();
  Json observations=Json::array();double newly_known_m2=0;
  size_t frontiers=0;bool active_safe=true;std::string reason;
  double computation_ms=0;
};
// 纯计算：保留原前沿提取和0.22米净空；按可达距离／去重未知收益生成稳定两点路线。
// 调用方必须在独立线程执行，并按任务代次、导航坐标代次和地图版本接纳结果。
ExplorationRouteResult exploration_route(ExplorationRouteInput input);
}  // namespace carcar_mapping_autonomy
