// NAV-007/NAV-008: Nav2 实际加载参数只读快照。
// 不设置参数、不发布速度、不访问串口；输出可直接重定向到本次试验日志目录。

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
struct Target
{
  std::string node;
  std::vector<std::string> parameters;
};

void print_target(const rclcpp::Node::SharedPtr & node, const Target & target)
{
  std::cout << "[" << target.node << "]\n";
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, target.node);
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    std::cout << "  service=UNAVAILABLE\n";
    return;
  }
  try {
    const auto values = client->get_parameters(target.parameters, std::chrono::seconds(2));
    for (size_t index = 0; index < target.parameters.size(); ++index) {
      std::cout << "  " << target.parameters[index] << "=";
      if (index >= values.size() ||
        values[index].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        std::cout << "<NOT_SET>\n";
      } else {
        std::cout << values[index].value_to_string() << "\n";
      }
    }
  } catch (const std::exception & exception) {
    std::cout << "  query_error=" << exception.what() << "\n";
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav_runtime_snapshot");
  std::cout << "# NAV-007/NAV-008 actual runtime parameter snapshot\n";
  std::cout << "# This tool is read-only; UNAVAILABLE means the node/service was not active.\n";

  for (const auto & target : std::vector<Target>{
      {"/amcl", {"update_min_d", "update_min_a", "recovery_alpha_fast", "recovery_alpha_slow",
        "min_particles", "max_particles", "laser_max_range"}},
      {"/controller_server", {"failure_tolerance", "progress_checker_plugin",
        "progress_checker.required_movement_radius", "progress_checker.required_movement_angle",
        "progress_checker.movement_time_allowance", "FollowPath.min_vel_x", "FollowPath.max_vel_x",
        "FollowPath.sim_time", "FollowPath.critics", "FollowPath.publish_evaluation", "odom_topic"}},
      {"/planner_server", {"GridBased.plugin", "GridBased.tolerance", "GridBased.allow_unknown",
        "GridBased.max_planning_time", "GridBased.lattice_filepath",
        "GridBased.allow_reverse_expansion", "GridBased.smooth_path"}},
      {"/local_costmap/local_costmap", {"update_frequency", "publish_frequency", "width", "height",
        "resolution", "footprint", "footprint_padding", "inflation_layer.enabled",
        "inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor"}},
      {"/global_costmap/global_costmap", {"update_frequency", "publish_frequency", "resolution",
        "footprint", "footprint_padding", "inflation_layer.enabled", "inflation_layer.inflation_radius",
        "inflation_layer.cost_scaling_factor"}},
      {"/velocity_smoother", {"smoothing_frequency", "velocity_timeout", "max_velocity", "min_velocity",
        "max_accel", "max_decel"}},
      {"/behavior_server", {"cycle_frequency", "costmap_topic", "footprint_topic", "transform_tolerance",
        "simulate_ahead_time", "max_rotational_vel", "odom_topic"}},
      {"/bt_navigator", {"bt_loop_duration", "default_server_timeout", "default_nav_to_pose_bt_xml",
        "default_nav_through_poses_bt_xml", "plugin_lib_names", "odom_topic"}}})
  {
    print_target(node, target);
  }

  rclcpp::shutdown();
  return 0;
}
