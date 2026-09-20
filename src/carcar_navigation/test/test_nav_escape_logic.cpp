#include <gtest/gtest.h>
#include "carcar_navigation/split_horizon.hpp"
#include "carcar_navigation/escape_policy.hpp"
#include "carcar_navigation/wheel_imu_ekf.hpp"
#include <pluginlib/class_loader.hpp>
#include <dwb_core/trajectory_generator.hpp>
#include <nav2_util/lifecycle_node.hpp>

TEST(SplitHorizon, LoadsConfiguredPluginAndPreservesBrakingHorizon)
{
  rclcpp::init(0, nullptr);
  {
    auto node=std::make_shared<nav2_util::LifecycleNode>("split_horizon_test");
    node->declare_parameter("FollowPath.sim_time",1.0);
    node->declare_parameter("FollowPath.rotate_sim_time",0.5);
    pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader("dwb_core","dwb_core::TrajectoryGenerator");
    auto generator=loader.createSharedInstance("carcar_navigation/SplitHorizonTrajectoryGenerator");
    generator->initialize(node,"FollowPath");
    geometry_msgs::msg::Pose2D pose;
    nav_2d_msgs::msg::Twist2D still,forward,rotate;
    forward.x=0.12;rotate.theta=0.3;
    auto horizon=[&](auto start,auto command) {
      auto result=generator->generateTrajectory(pose,start,command);
      EXPECT_FALSE(result.poses.empty());
      return rclcpp::Duration(result.time_offsets.back()).seconds();
    };
    EXPECT_NEAR(horizon(still,rotate),0.5,1e-8);
    EXPECT_NEAR(horizon(forward,rotate),1.0,1e-8);
    EXPECT_NEAR(horizon(still,forward),1.0,1e-8);
    EXPECT_NEAR(horizon(still,still),1.0,1e-8);
    node->set_parameter(rclcpp::Parameter("FollowPath.rotate_sim_time",-0.1));
    auto invalid=loader.createSharedInstance("carcar_navigation/SplitHorizonTrajectoryGenerator");
    EXPECT_THROW(invalid->initialize(node,"FollowPath"),std::invalid_argument);
  }
  rclcpp::shutdown();
}

TEST(SplitHorizon, UsesShorterHorizonForPureRotation)
{
  EXPECT_DOUBLE_EQ(carcar_navigation::horizon_for_command(0.12, 0, 0.2, 1.0, 0.5), 1.0);
  EXPECT_DOUBLE_EQ(carcar_navigation::horizon_for_command(0.0, 0.0, 0.3, 1.0, 0.5), 0.5);
  EXPECT_DOUBLE_EQ(carcar_navigation::horizon_for_command(0.0, 0.0, 0.0, 1.0, 0.5), 1.0);
}

TEST(EscapePolicy, PrefersBackupWhenAlreadyInCollision)
{
  using carcar_navigation::EscapeAction;
  EXPECT_EQ(carcar_navigation::choose_escape(false, true, true, true), EscapeAction::None);
  EXPECT_EQ(carcar_navigation::choose_escape(true, true, true, true), EscapeAction::Backup);
  EXPECT_EQ(carcar_navigation::choose_escape(true, false, true, false), EscapeAction::RotatePositive);
  EXPECT_EQ(carcar_navigation::choose_escape(true, false, false, true), EscapeAction::RotateNegative);
  EXPECT_EQ(carcar_navigation::choose_escape(true, false, false, false), EscapeAction::None);
}

TEST(WheelImuEkf, GyroYawDoesNotFollowWheelSlip)
{
  carcar_navigation::WheelImuEkf ekf;
  ekf.reset(0, 0, 0);
  for (int i = 0; i < 20; ++i) {
    ekf.predict(0.1, 0.0, 0.05, 0.02, 0.01);
    ekf.update_xy(0.1 * (i + 1) * 0.05, 0.0, 0.03);
  }
  EXPECT_NEAR(ekf.x, 0.1, 0.05);
  EXPECT_NEAR(ekf.yaw, 0.0, 0.05);

  carcar_navigation::WheelImuEkf turning;
  turning.reset(0, 0, 0);
  for (int i = 0; i < 10; ++i) {
    turning.predict(0.0, 0.5, 0.1, 0.02, 0.01);
  }
  EXPECT_NEAR(turning.yaw, 0.5, 0.05);
}
