#include <gtest/gtest.h>
#include <chrono>
#include "carcar_mapping_autonomy/manual_motion.hpp"
namespace ca=carcar_mapping_autonomy;
static sensor_msgs::msg::LaserScan::SharedPtr clear_scan(){
  auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();scan->angle_min=-M_PI;
  scan->angle_increment=2*M_PI/720;scan->angle_max=scan->angle_min+719*scan->angle_increment;
  scan->range_min=.05;scan->range_max=12;scan->ranges.assign(720,2);return scan;
}
TEST(ManualMotion, CoversForwardReverseAndTurningWithFreshFullObservation){
  const auto scan=clear_scan();
  for(double vx:{-.05,0.,.05})for(double wz:{-.10,0.,.10}){
    const auto check=ca::manual_motion_check({{scan,{0,0,0}}},vx,wz);
    EXPECT_TRUE(check.allowed)<<vx<<","<<wz<<":"<<check.reason;
  }
}
TEST(ManualMotion, MissingDirectionAndBrakingObstacleCannotBeAssumedClear){
  auto scan=clear_scan();scan->ranges[360]=NAN;
  EXPECT_FALSE(ca::manual_motion_check({{scan,{0,0,0}}},.05,0).allowed);
  scan=clear_scan();scan->ranges[360]=.20;
  EXPECT_FALSE(ca::manual_motion_check({{scan,{0,0,0}}},.05,0).allowed);
  // 后退的前缘仍在18厘米；前方20厘米障碍不应阻挡车身向后远离。
  EXPECT_TRUE(ca::manual_motion_check({{scan,{0,0,0}}},-.05,0).allowed);
  scan=clear_scan();scan->ranges[0]=.18;
  EXPECT_FALSE(ca::manual_motion_check({{scan,{0,0,0}}},-.05,0).allowed);
}
TEST(ManualMotion, SideObstacleOutsideMovingEnvelopeDoesNotStopForward){
  auto scan=clear_scan();scan->ranges[180]=.24;
  EXPECT_TRUE(ca::manual_motion_check({{scan,{0,0,0}}},.05,0).allowed);
  scan->ranges[180]=.14;
  EXPECT_FALSE(ca::manual_motion_check({{scan,{0,0,0}}},.05,0).allowed);
}
TEST(ManualMotion, RefusesInvalidBoundsAndPreservesZeroStop){
  EXPECT_FALSE(ca::manual_motion_check({},.05,0).allowed);
  EXPECT_TRUE(ca::manual_motion_check({},0,0).allowed);
  EXPECT_FALSE(ca::manual_motion_check({{clear_scan(),{0,0,0}}},.051,0).allowed);
  EXPECT_FALSE(ca::manual_motion_check({{clear_scan(),{0,0,0}}},0,NAN).allowed);
}
