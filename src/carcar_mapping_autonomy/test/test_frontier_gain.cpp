#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/frontier_gain.hpp"
using namespace carcar_mapping_autonomy;
TEST(FrontierGain, ThinKnownWallOccludesUnknownAreaInsteadOfRewardingThroughWall){
  nav_msgs::msg::OccupancyGrid map;map.header.frame_id="map";map.info.width=map.info.height=80;
  map.info.resolution=.05;map.info.origin.position.x=map.info.origin.position.y=-2;map.info.origin.orientation.w=1;
  map.data.assign(6400,0);
  for(int y=0;y<80;++y)for(int x=0;x<80;++x){
    if(!x||!y||x==79||y==79)map.data[y*80+x]=100;
    else if(x>50)map.data[y*80+x]=-1;
  }
  EXPECT_GT(visible_unknown_cells(map,{}).size(),10U);
  for(int y=0;y<80;++y)map.data[y*80+50]=100;
  EXPECT_TRUE(visible_unknown_cells(map,{}).empty());
}
TEST(FrontierGain, ActualDetourCanOutweighStraightLineDistanceAndTurnRevisitCostsMatter){
  // 左侧看似近但真实规划绕行4米，右侧直接通行1.5米且收益相同。
  EXPECT_GT(frontier_score(4.,2.,0,0),frontier_score(1.5,2.,0,0));
  EXPECT_GT(frontier_score(1.5,2.,M_PI,0),frontier_score(1.5,2.,0,0));
  EXPECT_GT(frontier_score(1.5,2.,0,2),frontier_score(1.5,2.,0,0));
  EXPECT_TRUE(std::isfinite(frontier_score(1.,0.,0,0)));
  EXPECT_FALSE(std::isfinite(frontier_score(NAN,2.,0,0)));
}

TEST(FrontierGain, UnknownDepthDoesNotRewardAnEntireUnseenRoomAsCertainGain){
  nav_msgs::msg::OccupancyGrid map;map.header.frame_id="map";map.info.width=map.info.height=120;
  map.info.resolution=.05;map.info.origin.position.x=map.info.origin.position.y=-3;map.info.origin.orientation.w=1;
  map.data.assign(14400,0);for(int y=0;y<120;++y)for(int x=80;x<120;++x)map.data[y*120+x]=-1;
  const auto bounded=visible_unknown_cells(map,{},.75),optimistic=visible_unknown_cells(map,{},3.);
  EXPECT_FALSE(bounded.empty());EXPECT_GT(optimistic.size(),bounded.size()*1.5);
  // 同一未知深度口径比较当前位置和候选位置，不能靠变更统计单位制造收益。
  EXPECT_EQ(bounded,visible_unknown_cells(map,{}));
}


// 复现v4旧区域中心与最新访问点不同步的实测例子，验证执行端与排序端禁区一致。
#include "../../third_party/explore_lite/include/explore/goal_memory.h"
TEST(FrontierObservationMemory, MergedRegionExportsOriginalCenterAndExactExpiry){
 explore::GoalMemory memory;geometry_msgs::msg::Point old_point,new_point,probe;
 old_point.x=-.198234150;old_point.y=.221183142;
 new_point.x=-.278224109;new_point.y=-.124987163;
 probe.x=-.128224104;probe.y=.575012847;
 memory.record(old_point,true,10.);memory.record(new_point,true,80.);
 ASSERT_GT(std::hypot(probe.x-new_point.x,probe.y-new_point.y),memory.radius);
 ASSERT_TRUE(memory.blocked(probe,81.));
 const auto regions=memory.blocked_regions(81.);ASSERT_EQ(regions.size(),1U);
 EXPECT_DOUBLE_EQ(regions[0].point.x,old_point.x);EXPECT_DOUBLE_EQ(regions[0].point.y,old_point.y);
 const carcar_mapping_autonomy::Json blocked=carcar_mapping_autonomy::Json::array({
   {{"center",{regions[0].point.x,regions[0].point.y}},{"radius",regions[0].radius},{"until",regions[0].until}}});
 EXPECT_TRUE(carcar_mapping_autonomy::observation_blocked_by_regions(blocked,{probe.x,probe.y,0},81.));
 EXPECT_FALSE(carcar_mapping_autonomy::observation_blocked_by_regions(blocked,{probe.x,probe.y,0},140.));
 EXPECT_TRUE(memory.blocked_regions(140.).empty());
}
TEST(FrontierObservationMemory, ActualFailureThresholdAndClearRemainUnchanged){
 explore::GoalMemory memory;geometry_msgs::msg::Point point;point.x=.5;
 memory.record(point,false,10.);EXPECT_TRUE(memory.blocked_regions(11.).empty());
 memory.record(point,false,12.);ASSERT_EQ(memory.blocked_regions(13.).size(),1U);
 EXPECT_FALSE(memory.blocked_regions(13.)[0].visited);
 memory.clear();EXPECT_TRUE(memory.blocked_regions(13.).empty());EXPECT_FALSE(memory.blocked(point,13.));
}
TEST(FrontierObservationMemory, MalformedBlockedRegionsFailClosed){
 using namespace carcar_mapping_autonomy;
 EXPECT_TRUE(observation_blocked_by_regions(Json::object(),{0,0,0},1.));
 EXPECT_TRUE(observation_blocked_by_regions(Json::array({{{"center",{0,0}},{"radius",-1},{"until",10}}}),{0,0,0},1.));
 EXPECT_FALSE(observation_blocked_by_regions(Json::array(),{0,0,0},1.));
}

TEST(FrontierPathTurn, GridCenterOffsetDoesNotInventAFullTurnOnStraightPath){
  std::vector<Pose> path;for(int i=0;i<=40;++i)path.push_back({.025,.025+i*.05,0});
  const double turn=path_turn_estimate(path,{.012,.021,M_PI/2},path.size()-1,.30);
  EXPECT_LT(turn,.20);EXPECT_GE(turn,0.);
}
TEST(FrontierPathTurn, StaircaseDiagonalUsesPhysicalHeadingRatherThanEveryGridCorner){
  std::vector<Pose> path;
  for(int i=1;i<=20;++i){path.push_back({i*.05,(i-1)*.05,0});path.push_back({i*.05,i*.05,0});}
  EXPECT_LT(path_turn_estimate(path,{0,0,M_PI/4},path.size()-1,.30),.10);
  EXPECT_NEAR(path_turn_estimate(path,{0,0,0},path.size()-1,.30),M_PI/4,.10);
}
TEST(FrontierPathTurn, RealNinetyDegreeAndReverseDirectionsRetainTheirCosts){
  std::vector<Pose> corner,reverse;
  for(int i=1;i<=20;++i){corner.push_back({i*.05,0,0});reverse.push_back({-i*.05,0,0});}
  for(int i=1;i<=20;++i)corner.push_back({1,i*.05,0});
  EXPECT_NEAR(path_turn_estimate(corner,{0,0,0},corner.size()-1,.30),M_PI/2,.10);
  EXPECT_NEAR(path_turn_estimate(reverse,{0,0,0},reverse.size()-1,.30),M_PI,.10);
  EXPECT_GT(frontier_score(1,2,M_PI,0,.10/.22),frontier_score(1,2,0,0,.10/.22));
}
TEST(FrontierPathTurn, InvalidGeometryDoesNotProduceAnExecutableRanking){
  const std::vector<Pose> path{{.5,0,0},{NAN,1,0}};
  EXPECT_FALSE(std::isfinite(path_turn_estimate(path,{0,0,0},1,.30)));
  EXPECT_FALSE(std::isfinite(path_turn_estimate(path,{0,0,0},2,.30)));
  EXPECT_FALSE(std::isfinite(path_turn_estimate(path,{0,0,0},0,0)));
  EXPECT_FALSE(std::isfinite(frontier_score(1,2,0,0,NAN)));
}
