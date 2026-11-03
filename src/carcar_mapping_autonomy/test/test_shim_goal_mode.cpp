#include <gtest/gtest.h>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include "carcar_mapping_autonomy/mapping_rotation_shim.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
class ShimGoalMode:public testing::Test {
protected:
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_F(ShimGoalMode,PositionGoalIgnoresTerminalYawWhileReturnCheckerStillRequiresIt){
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("goal_mode_local");
  for(const auto&r:local->set_parameters({{"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
      {"global_frame","map"},{"robot_base_frame","base_footprint"},{"rolling_window",false},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
  local->configure();local->getCostmap()->resizeMap(60,60,.05,-1.5,-1.5);local->getCostmap()->resetMap(0,0,60,60);
  rclcpp::NodeOptions options;options.automatically_declare_parameters_from_overrides(true).parameter_overrides({
    {"FollowPath.primary_controller",std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")},
    {"FollowPath.rotate_to_goal_heading",true},{"FollowPath.use_rotate_to_heading",false},
    {"FollowPath.desired_linear_vel",.1},{"FollowPath.rotate_to_heading_angular_vel",.22},{"FollowPath.max_angular_accel",3.2},
    {"FollowPath.angular_dist_threshold",.5},{"FollowPath.angular_disengage_threshold",.1},{"FollowPath.forward_sampling_distance",.3},
    {"position.xy_goal_tolerance",.15},{"simple.xy_goal_tolerance",.15},{"simple.yaw_goal_tolerance",.1},{"controller_frequency",10.}});
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_mode_controller",options);
  nav2_controller::PositionGoalChecker position;position.initialize(node,"position",local);
  nav2_controller::SimpleGoalChecker simple;simple.initialize(node,"simple",local);
  ca::MappingRotationShim controller;controller.configure(node,"FollowPath",local->getTfBuffer(),local);controller.activate();
  auto pose=[&](double x,double yaw){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.header.stamp=node->now();
    p.pose.position.x=x;p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);return p;};
  nav_msgs::msg::Path path;path.header.frame_id="map";path.poses={pose(0,0),pose(.1,M_PI/2)};
  auto query=pose(.04,0);geometry_msgs::msg::TransformStamped transform;transform.header=query.header;transform.child_frame_id="base_footprint";
  transform.transform.translation.x=.04;transform.transform.rotation.w=1;ASSERT_TRUE(local->getTfBuffer()->setTransform(transform,"goal_mode",true));
  geometry_msgs::msg::Twist stopped;controller.setPlan(path);
  const auto exploring=controller.computeVelocityCommands(query,stopped,&position).twist;
  EXPECT_NEAR(exploring.angular.z,0.,1e-9)<<"探索不得为了终点yaw增加末端旋转";
  EXPECT_TRUE(position.isGoalReached(query.pose,path.poses.back().pose,stopped));
  controller.setPlan(path);query.header.stamp=node->now();
  const auto returning=controller.computeVelocityCommands(query,stopped,&simple).twist;
  EXPECT_GT(returning.angular.z,.01)<<"普通返程仍须调整到起点朝向";
  EXPECT_FALSE(simple.isGoalReached(query.pose,path.poses.back().pose,stopped));
  controller.deactivate();controller.cleanup();local->cleanup();
}
}
