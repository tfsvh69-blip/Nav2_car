#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/handoff.hpp"
namespace ca=carcar_mapping_autonomy;
static nav_msgs::msg::Path route(std::initializer_list<ca::Pose>points){
  nav_msgs::msg::Path p;p.header.frame_id="map";
  for(const auto&v:points){geometry_msgs::msg::PoseStamped s;s.pose.position.x=v.x;s.pose.position.y=v.y;p.poses.push_back(s);}return p;
}
TEST(Handoff,UsesOrderedPathInsteadOfDistanceThroughWall){
  const auto p=route({{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,.1,0}});
  EXPECT_NEAR(ca::handoff_path_distance(p,{0,0,0},{0,.1,0}),3.9,1e-8);
}
TEST(Handoff,RejectsWrongFrameMissingPathAndStaleStart){
  auto p=route({{0,0,0},{1,0,0},{2,0,0}});
  EXPECT_FALSE(std::isfinite(ca::handoff_path_distance(p,{1.8,0,0},{2,0,0})));
  p.header.frame_id="odom";EXPECT_FALSE(std::isfinite(ca::handoff_path_distance(p,{0,0,0},{.1,0,0})));
  EXPECT_FALSE(std::isfinite(ca::handoff_path_distance({}, {}, {})));
}
TEST(Handoff,AcceptsNearIntermediateAndSmallAlreadyPassedPoint){
  const auto p=route({{0,0,0},{.5,0,0},{1,0,0}});
  EXPECT_NEAR(ca::handoff_path_distance(p,{.3,0,0},{.5,0,0}),.2,1e-8);
  EXPECT_EQ(ca::handoff_path_distance(p,{.4,0,0},{.3,0,0}),0);
  EXPECT_FALSE(std::isfinite(ca::handoff_path_distance(p,{.4,0,0},{.5,.5,0})));
}
