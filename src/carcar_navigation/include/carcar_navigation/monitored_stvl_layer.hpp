#pragma once
#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp>

namespace carcar_navigation {
// 仅增加可观测接口；所有地图算法由固定版本 STVL 执行。
class MonitoredStvlLayer : public spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer {
public:
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void updateBounds(double x,double y,double yaw,double * min_x,double * min_y,
    double * max_x,double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master,int x0,int y0,int x1,int y1) override;
private:
  void installParameterGuard();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  std::chrono::steady_clock::time_point started_;
  bool sources_current_=false;
  uint64_t update_sequence_=0;
};
}  // namespace carcar_navigation
