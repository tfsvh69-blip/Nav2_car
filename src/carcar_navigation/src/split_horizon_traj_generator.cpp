#include "carcar_navigation/split_horizon.hpp"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace carcar_navigation
{

class SplitHorizonTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override
  {
    StandardTrajectoryGenerator::initialize(nh, plugin_name);
    nav2_util::declare_parameter_if_not_declared(
      nh, plugin_name + ".rotate_sim_time", rclcpp::ParameterValue(0.5));
    nh->get_parameter(plugin_name + ".rotate_sim_time", rotate_sim_time_);
    if (!std::isfinite(sim_time_) || sim_time_ <= 0.0 ||
      !std::isfinite(rotate_sim_time_) || rotate_sim_time_ <= 0.0 || rotate_sim_time_ > sim_time_) {
      throw std::invalid_argument("要求 0 < rotate_sim_time <= sim_time，且均为有限值");
    }
    forward_sim_time_ = sim_time_;
  }

  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const nav_2d_msgs::msg::Twist2D & cmd_vel) override
  {
    // 尚在平移时不能把制动段误当作纯原地转向而缩短视界。
    sim_time_ = std::hypot(start_vel.x, start_vel.y) > 1e-4 ? forward_sim_time_ : horizon_for_command(
      cmd_vel.x, cmd_vel.y, cmd_vel.theta, forward_sim_time_, rotate_sim_time_);
    struct Restore {double & value; double original; ~Restore() {value = original;}};
    Restore restore{sim_time_, forward_sim_time_};
    return StandardTrajectoryGenerator::generateTrajectory(start_pose, start_vel, cmd_vel);
  }

private:
  double rotate_sim_time_{0.5};
  double forward_sim_time_{1.0};
};

}  // namespace carcar_navigation

PLUGINLIB_EXPORT_CLASS(
  carcar_navigation::SplitHorizonTrajectoryGenerator, dwb_core::TrajectoryGenerator)
