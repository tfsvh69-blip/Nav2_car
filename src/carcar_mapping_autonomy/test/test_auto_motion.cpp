#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/auto_motion.hpp"
using namespace carcar_mapping_autonomy;
namespace {
nav_msgs::msg::OccupancyGrid open_map(){
  nav_msgs::msg::OccupancyGrid map;map.header.frame_id="map";map.info.width=map.info.height=100;
  map.info.resolution=.05;map.info.origin.position.x=map.info.origin.position.y=-2.5;
  map.info.origin.orientation.w=1;map.data.assign(10000,0);return map;
}
geometry_msgs::msg::Twist forward(){geometry_msgs::msg::Twist v;v.linear.x=.1;return v;}
TEST(AutoMotion, EmptyCloudAndUnrelatedSideObstacleKeepForwardCommand){
  const auto map=open_map();const auto command=forward();
  EXPECT_DOUBLE_EQ(auto_motion_check(map,Task(),{}, {},command,command).scale,1);
  EXPECT_DOUBLE_EQ(auto_motion_check(map,Task(),{},{{0,.24,0},{-.4,0,0}},command,command).scale,1);
}
TEST(AutoMotion, OneFutureBeamSlowsBeforeDelayAndBrakingConsumeClearance){
  const auto map=open_map();const auto command=forward();
  const auto check=auto_motion_check(map,Task(),{},{{.34,0,0}},command,command);
  EXPECT_GT(check.scale,0);EXPECT_LT(check.scale,1);EXPECT_NE(check.reason.find("激光"),std::string::npos);
  const auto inside=auto_motion_check(map,Task(),{},{{.21,0,0}},command,command);
  EXPECT_DOUBLE_EQ(inside.velocity.linear.x,0);
}
TEST(AutoMotion, RawUnknownAndTaskBoundaryRemainHardEvenWithoutLaserReturn){
  auto map=open_map();const auto command=forward();
  for(int y=0;y<100;++y)map.data[y*100+56]=-1;
  const auto check=auto_motion_check(map,Task(),{}, {},command,command);
  EXPECT_LT(check.scale,1);EXPECT_NE(check.reason.find("原图"),std::string::npos);
  auto task=Task();task.anchored=true;task.origin={-19.94,0,0};
  EXPECT_LT(auto_motion_check(open_map(),task,{}, {},command,command).scale,1);
}
TEST(AutoMotion, PureTurnKeepsCircularClearanceAndInvalidMotionFailsClosed){
  auto command=forward();command.linear.x=0;command.angular.z=.36;
  EXPECT_DOUBLE_EQ(auto_motion_check(open_map(),Task(),{},{{.23,0,0}},command,{}).scale,1);
  command.linear.y=.01;EXPECT_DOUBLE_EQ(auto_motion_check(open_map(),Task(),{}, {},command,{}).scale,0);
  command.linear.y=0;command.linear.x=-.01;EXPECT_DOUBLE_EQ(auto_motion_check(open_map(),Task(),{}, {},command,{}).scale,0);
}
TEST(AutoMotion, PointTwentyKeepsPredictionAndBrakingProtection){
 auto command=forward();command.linear.x=.20;
 EXPECT_DOUBLE_EQ(auto_motion_check(open_map(),Task(),{}, {},command,command).scale,1);
 EXPECT_LT(auto_motion_check(open_map(),Task(),{},{{.34,0,0}},command,command).scale,1);
 EXPECT_DOUBLE_EQ(auto_motion_check(open_map(),Task(),{},{{.21,0,0}},command,command).velocity.linear.x,0);
 command.linear.x=.201;EXPECT_DOUBLE_EQ(auto_motion_check(open_map(),Task(),{}, {},command,{}).scale,0);
}
}
