#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "carcar_mapping_autonomy/exploration_route.hpp"
namespace carcar_mapping_autonomy {
// 监督器仍独占最终速度、健康和任务状态；本模块只拥有探索路线的规划／导航Action。
class ContinuousExploration {
public:
  struct Snapshot {
    ExplorationRouteInput route;
    uint64_t map_sequence=0;
    bool enabled=false,healthy=false,still=false;
    std::string task_id,request_id;
  };
  ContinuousExploration(rclcpp::Node & node,const std::string & behavior_tree);
  ~ContinuousExploration();
  void tick(Snapshot snapshot);
  void stop();
  bool terminal()const;
  bool motion()const;
  bool complete()const;
  std::string fault()const;
  Json status()const;
private:
  class Impl;std::unique_ptr<Impl> impl_;
};
}  // namespace carcar_mapping_autonomy
