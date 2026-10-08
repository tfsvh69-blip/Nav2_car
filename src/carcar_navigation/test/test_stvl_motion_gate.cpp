#include <gtest/gtest.h>
#include "carcar_navigation/motion_gate.hpp"

TEST(StvlMotionGate, StartupFaultAndReopeningRequireFreshPermitAndCommand) {
  carcar_navigation::MotionGate g(0.2,0.3,true,0.3);
  geometry_msgs::msg::Twist c; c.linear.x=0.1;
  g.permit(1,1); g.command(c,1); EXPECT_DOUBLE_EQ(g.output(1).linear.x,0);
  g.perception_heartbeat(true,1.01);
  g.command(c,1.02); EXPECT_DOUBLE_EQ(g.output(1.02).linear.x,0);
  g.permit(1,1.03); g.command(c,1.04); EXPECT_DOUBLE_EQ(g.output(1.04).linear.x,0.1);
  g.perception_heartbeat(false,1.05); EXPECT_DOUBLE_EQ(g.output(1.05).linear.x,0);
  g.permit(1,1.06); g.command(c,1.06);
  g.perception_heartbeat(true,1.07); EXPECT_DOUBLE_EQ(g.output(1.07).linear.x,0);
  g.permit(1,1.08); EXPECT_DOUBLE_EQ(g.output(1.08).linear.x,0);
  g.command(c,1.09); EXPECT_DOUBLE_EQ(g.output(1.09).linear.x,0.1);
}
TEST(StvlMotionGate, SupervisorLossDropsLeaseAndCachedVelocity) {
  carcar_navigation::MotionGate g(0.2,0.3,true,0.3);
  geometry_msgs::msg::Twist c; c.linear.x=0.1;
  g.perception_heartbeat(true,1);
  g.permit(2,1.29); g.command(c,1.29);
  EXPECT_DOUBLE_EQ(g.output(1.29).linear.x,0.1);
  EXPECT_DOUBLE_EQ(g.output(1.31).linear.x,0);
  g.perception_heartbeat(true,1.32); EXPECT_DOUBLE_EQ(g.output(1.32).linear.x,0);
}
TEST(StvlMotionGate, LegacyModeStillUsesExistingPerceptionSwitch) {
  carcar_navigation::MotionGate g;
  geometry_msgs::msg::Twist c; c.linear.x=0.1;
  g.permit(1,1); g.command(c,1); EXPECT_DOUBLE_EQ(g.output(1).linear.x,0.1);
  g.perception_allowed(false); EXPECT_DOUBLE_EQ(g.output(1.01).linear.x,0);
  g.perception_allowed(true); EXPECT_DOUBLE_EQ(g.output(1.02).linear.x,0);
}
