#include "carcar_navigation/monitored_stvl_layer.hpp"
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace carcar_navigation {
void MonitoredStvlLayer::onInitialize() {
  SpatioTemporalVoxelLayer::onInitialize();
  auto node=node_.lock();
  diagnostics_=node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    name_+"/diagnostics",rclcpp::QoS(1).reliable());
  installParameterGuard();
}
void MonitoredStvlLayer::installParameterGuard() {
  auto node=node_.lock();
  parameter_guard_=node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result; result.successful=true;
      for (const auto & p:parameters) {
        if (p.get_name().rfind(name_+".",0)==0) {
          result.successful=false;
          result.reason="STVL 2.3.4 配置仅在启动时加载，请修改 YAML 后重启";
          break;
        }
      }
      return result;
    });
}
void MonitoredStvlLayer::activate() {
  parameter_guard_.reset();
  SpatioTemporalVoxelLayer::activate();
  // 最后注册，参数回调逆序执行；在上游产生订阅副作用前拒绝运行时修改。
  installParameterGuard();
}
void MonitoredStvlLayer::deactivate() {
  SpatioTemporalVoxelLayer::deactivate();
}
void MonitoredStvlLayer::updateBounds(double x,double y,double yaw,double * min_x,
  double * min_y,double * max_x,double * max_y) {
  started_=std::chrono::steady_clock::now();
  SpatioTemporalVoxelLayer::updateBounds(x,y,yaw,min_x,min_y,max_x,max_y);
  // updateCosts 的 reset 特例会临时置 current_=true，不能据此开放运动。
  sources_current_=current_;
}
void MonitoredStvlLayer::updateCosts(nav2_costmap_2d::Costmap2D & master,
  int x0,int y0,int x1,int y1) {
  SpatioTemporalVoxelLayer::updateCosts(master,x0,y0,x1,y1);
  auto node=node_.lock();
  if (!node || !diagnostics_) {return;}
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp=node->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name=name_; status.hardware_id="STVL-2.3.4";
  status.level=sources_current_ ? status.OK : status.ERROR;
  status.message=sources_current_ ? "CURRENT" : "INPUT_STALE";
  auto add=[&status](const std::string & key,const std::string & value) {
    diagnostic_msgs::msg::KeyValue item; item.key=key; item.value=value;
    status.values.push_back(item);
  };
  add("source_current",sources_current_ ? "true" : "false");
  add("update_seq",std::to_string(++update_sequence_));
  add("update_duration_ms",std::to_string(std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-started_).count()));
  array.status.push_back(status); diagnostics_->publish(array);
}
}  // namespace carcar_navigation
PLUGINLIB_EXPORT_CLASS(carcar_navigation::MonitoredStvlLayer,nav2_costmap_2d::Layer)
