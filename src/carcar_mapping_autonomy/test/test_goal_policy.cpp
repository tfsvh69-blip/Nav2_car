// 本机真实Nav2插件的软件回归，不启动执行器、不发布速度。
#include <cmath>
#include <algorithm>
#include <memory>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_controller/plugins/position_goal_checker.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_smac_planner/smac_planner_2d.hpp>

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2); p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return 2 * std::atan2(q.z, q.w);
}
rclcpp::NodeOptions options()
{
  return rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", NAV_BASE_CONFIG,
    "--params-file", MAPPING_CONFIG}).automatically_declare_parameters_from_overrides(true);
}
class MappingGoalPolicy : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(MappingGoalPolicy, ExplorationAcceptsOppositeYawAtPositionWhileReturnRequiresOriginYaw)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options());
  EXPECT_EQ(node->get_parameter("exploration_goal_checker.plugin").as_string(),
    "nav2_controller::PositionGoalChecker");
  nav2_controller::PositionGoalChecker exploration;
  nav2_controller::SimpleGoalChecker returning;
  exploration.initialize(node, "exploration_goal_checker", nullptr);
  returning.initialize(node, "general_goal_checker", nullptr);
  auto target = pose(1, 1, 0).pose;
  auto current = pose(.9, 1, std::acos(-1.0)).pose;
  geometry_msgs::msg::Twist stopped;
  EXPECT_TRUE(exploration.isGoalReached(current, target, stopped));
  EXPECT_FALSE(returning.isGoalReached(current, target, stopped));
  current.orientation = target.orientation;
  EXPECT_TRUE(returning.isGoalReached(current, target, stopped));
  exploration.reset(); returning.reset();
  current.position.x = .84;
  EXPECT_FALSE(exploration.isGoalReached(current, target, stopped));
  EXPECT_FALSE(returning.isGoalReached(current, target, stopped));
}

TEST_F(MappingGoalPolicy, ExplorationPlannerUsesApproachTangentAndReturnPreservesRequestedYaw)
{
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("goal_policy_map");
  for (const auto & result : map->set_parameters({
      {"plugins", std::vector<std::string>{}}, {"filters", std::vector<std::string>{}},
      {"global_frame", "map"}, {"robot_base_frame", "base_footprint"},
      {"rolling_window", false}, {"track_unknown_space", true}}))
  {ASSERT_TRUE(result.successful) << result.reason;}
  map->configure();
  ASSERT_EQ(map->get_current_state().label(), "inactive");
  map->getCostmap()->resizeMap(100, 100, .05, -2.5, -2.5);
  map->getCostmap()->resetMap(0, 0, 100, 100);
  std::fill_n(map->getCostmap()->getCharMap(), 10000, 0);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server", options());
  nav2_smac_planner::SmacPlanner2D exploration, returning;
  exploration.configure(node, "ExplorationGrid", map->getTfBuffer(), map);
  returning.configure(node, "GridBased", map->getTfBuffer(), map);
  // 目标在西边，却传入朝东；探索路径末端应朝西，返程继续尊重指定朝东。
  const auto start = pose(0, 0, 0), target = pose(-1, 0, 0);
  const auto outward = exploration.createPlan(start, target);
  const auto home = returning.createPlan(start, target);
  ASSERT_GT(outward.poses.size(), 2U); ASSERT_GT(home.poses.size(), 2U);
  EXPECT_LT(std::cos(yaw(outward.poses.back().pose.orientation)), -.99);
  EXPECT_NEAR(yaw(home.poses.back().pose.orientation), 0, 1e-9);
  // 同一栅格不应为了探索目标中的无关朝向而产生一次原地转向。
  const auto same = exploration.createPlan(start, pose(0, 0, std::acos(-1.0)));
  ASSERT_EQ(same.poses.size(), 1U);
  EXPECT_NEAR(yaw(same.poses.front().pose.orientation), 0, 1e-9);
  exploration.cleanup(); returning.cleanup(); map->cleanup();
}
}  // namespace
