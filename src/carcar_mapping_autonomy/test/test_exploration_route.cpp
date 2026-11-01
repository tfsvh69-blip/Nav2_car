#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "carcar_mapping_autonomy/exploration_route.hpp"
#include "carcar_mapping_autonomy/map_geometry.hpp"
namespace ca=carcar_mapping_autonomy;
static ca::ExplorationRouteInput scenario(bool closed=false){
  ca::ExplorationRouteInput j;auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  m->header.frame_id="map";m->info.width=m->info.height=100;m->info.resolution=.05;
  m->info.origin.position.x=m->info.origin.position.y=-2.5;m->info.origin.orientation.w=1;m->data.assign(10000,0);
  if(closed)for(int i=0;i<100;++i){m->data[i]=100;m->data[9900+i]=100;m->data[i*100]=100;m->data[i*100+99]=100;}
  j.map=m;j.task.anchored=true;j.task.phase="EXPLORING";j.generation=3;j.navigation_generation=1;j.revision=1;j.now=10;
  return j;
}
TEST(ExplorationRoute, CroppedOpenRoomProducesMultipleSafeDistinctWaypoints){
  auto input=scenario();const auto result=ca::exploration_route(input);
  EXPECT_GT(result.frontiers,0U);ASSERT_GE(result.route.size(),2U)<<result.reason;
  EXPECT_LE(result.route.size(),2U)<<"只承诺当前点与一个后续点，禁止固定长序列";
  for(const auto&p:result.route){EXPECT_TRUE(ca::free_footprint(*input.map,p.target));EXPECT_GT(p.gain,.1);}
  EXPECT_LT(result.computation_ms,2000);
}
TEST(ExplorationRoute, ObservedClosedWallsHaveNoFrontiers){
  const auto result=ca::exploration_route(scenario(true));EXPECT_EQ(result.frontiers,0U);EXPECT_TRUE(result.route.empty());
}
TEST(ExplorationRoute, NewlyObservedAreaInvalidatesOnlyCommitmentsWithoutRemainingGain){
  const auto initial=ca::exploration_route(scenario());ASSERT_EQ(initial.route.size(),2U);
  auto observed=scenario(true);observed.prefix=initial.route;
  const auto result=ca::exploration_route(observed);
  EXPECT_EQ(result.frontiers,0U);EXPECT_TRUE(result.route.empty())<<"旧收益已被新地图覆盖，不能继续绕行旧承诺点";
}
TEST(ExplorationRoute, InvalidMapNeverAuthorizesAnExistingRoute){
  auto input=scenario();input.map.reset();input.prefix.push_back({{1,0,0},{1.5,0,0},1,1});
  const auto result=ca::exploration_route(input);EXPECT_FALSE(result.active_safe);EXPECT_TRUE(result.route.empty());
}
TEST(ExplorationRoute, CurrentViewNeverEarnsAdditionalGainFromAnIdenticalCommitment){
 auto input=scenario();input.prefix.push_back({input.current,{2.5,0,0},5,1});
 const auto result=ca::exploration_route(input);ASSERT_GT(result.frontiers,0U);
 ASSERT_FALSE(result.route.empty());
 for(const auto&p:result.route)EXPECT_GT(std::hypot(p.target.x-input.current.x,p.target.y-input.current.y),.27);
}
TEST(ExplorationRoute, UnknownRobotFootprintIsUnsafeRatherThanExplorationComplete){
  auto input=scenario();auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*input.map);
  map->data[50*100+50]=-1;input.map=map;
  const auto result=ca::exploration_route(input);EXPECT_FALSE(result.active_safe);EXPECT_TRUE(result.route.empty());
}
TEST(ExplorationRoute, TwoCommittedPointsAreNeverReorderedOrExtended){
  auto input=scenario();auto first=ca::exploration_route(input);ASSERT_EQ(first.route.size(),2U);
  input.prefix=first.route;const auto next=ca::exploration_route(input);ASSERT_EQ(next.route.size(),2U);
  for(size_t i=0;i<2;++i){EXPECT_DOUBLE_EQ(next.route[i].target.x,first.route[i].target.x);EXPECT_DOUBLE_EQ(next.route[i].target.y,first.route[i].target.y);}
}
TEST(ExplorationRoute, FollowupGetsOnlyItsAdditionalObservableArea){
  const auto result=ca::exploration_route(scenario());ASSERT_EQ(result.route.size(),2U);
  std::vector<double> marginal;
  for(const auto&candidate:result.candidates)if(candidate.contains("marginal_unknown_m2"))marginal.push_back(candidate.at("marginal_unknown_m2").get<double>());
  ASSERT_EQ(marginal.size(),2U);EXPECT_GT(marginal[1],0);EXPECT_LE(marginal[1],result.route[1].gain);
  EXPECT_NE(result.route[0].target.x==result.route[1].target.x&&result.route[0].target.y==result.route[1].target.y,true);
  const auto&a=result.route[0].target;const auto&b=result.route[1].target;
  EXPECT_NEAR(a.yaw,std::atan2(a.y,a.x),1e-9);
  EXPECT_LE(1-std::cos(std::atan2(b.y-a.y,b.x-a.x)-a.yaw),1.5)<<"尚未到达首点时不得提前承诺大角度折返";
}
TEST(ExplorationRoute, MeasuresNewlyObservedCellsWithoutDoubleCountingRepeatedVisits){
  auto input=scenario();const auto first=ca::exploration_route(input);ASSERT_FALSE(first.route.empty());
  const auto&point=first.route.front();ASSERT_FALSE(point.predicted_cells.empty());
  input.visits.push_back({point.target,point.gain,0,true,point.predicted_cells});
  const auto before=ca::exploration_route(input);EXPECT_DOUBLE_EQ(before.newly_known_m2,0);
  auto expanded=std::make_shared<nav_msgs::msg::OccupancyGrid>(*input.map);
  expanded->info.width=160;expanded->data.assign(160*100,0);input.map=expanded;++input.revision;
  const auto after=ca::exploration_route(input);EXPECT_GT(after.newly_known_m2,0);
  input.visits.push_back(input.visits.front());const auto repeated=ca::exploration_route(input);
  EXPECT_DOUBLE_EQ(repeated.newly_known_m2,after.newly_known_m2);
}
TEST(ExplorationRoute, MaintainsCommittedPrefixAndDoesNotRevisitReachedRegionWithoutNewGain){
  auto input=scenario();auto first=ca::exploration_route(input);ASSERT_GE(first.route.size(),2U);
  input.prefix={first.route.front()};auto next=ca::exploration_route(input);ASSERT_FALSE(next.route.empty());
  EXPECT_DOUBLE_EQ(next.route.front().target.x,first.route.front().target.x);
  input.prefix.clear();for(const auto&p:first.route)input.visits.push_back({p.target,p.gain,0,true});
  input.now=1000;auto later=ca::exploration_route(input);
  for(const auto&p:later.route)for(const auto&visited:first.route)EXPECT_GT(std::hypot(p.target.x-visited.target.x,p.target.y-visited.target.y),.64);
}
TEST(ExplorationRoute, FailedRegionCanBeRevalidatedAfterMapChangeAndShortBackoff){
 auto input=scenario();const auto first=ca::exploration_route(input);ASSERT_FALSE(first.route.empty());
 const auto target=first.route.front().target;
 input.visits.push_back({target,first.route.front().gain,input.now+60,false,{},input.revision});
 auto absent=[&](const auto&r){for(const auto&p:r.route)if(std::hypot(p.target.x-target.x,p.target.y-target.y)<.65)return false;return true;};
 EXPECT_TRUE(absent(ca::exploration_route(input)));
 ++input.revision;input.now+=1;EXPECT_TRUE(absent(ca::exploration_route(input)));
 input.now+=5;const auto retry=ca::exploration_route(input);ASSERT_FALSE(retry.route.empty());
 EXPECT_NEAR(retry.route.front().target.x,target.x,1e-9);EXPECT_NEAR(retry.route.front().target.y,target.y,1e-9);
}
TEST(ExplorationRoute, ActualMapGrowthAllowsANewViewNearAProductiveVisit){
 auto input=scenario();const auto first=ca::exploration_route(input);ASSERT_FALSE(first.route.empty());
 const auto target=first.route.front().target;auto visited=target;visited.y+=.30;
 input.visits.push_back({visited,first.route.front().gain,0,true});
 const auto blocked=ca::exploration_route(input);
 for(const auto&p:blocked.route)EXPECT_GT(std::hypot(p.target.x-target.x,p.target.y-target.y),.20);
 // 模拟上次到访预计的11个未知格已经在当前地图变为已知；旧到访本身不是永久禁行圈。
 for(uint64_t x=0;x<11;++x)input.visits.front().predicted_cells.push_back(x<<32);
 const auto productive=ca::exploration_route(input);ASSERT_FALSE(productive.route.empty());
 EXPECT_GE(productive.newly_known_m2,.10);
 EXPECT_NEAR(productive.route.front().target.x,target.x,1e-9);EXPECT_NEAR(productive.route.front().target.y,target.y,1e-9);
}

TEST(ExplorationRoute, RecordedUnknownGapsWaitInsteadOfDrivingOutsideKnownSafeSpace){
  auto input=scenario();auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"data/exploration_corridor_start.json");
  const auto fixture=ca::Json::parse(file);map->header.frame_id="map";map->info.width=fixture.at("width");map->info.height=fixture.at("height");
  map->info.resolution=fixture.at("resolution");map->info.origin.position.x=fixture.at("origin").at(0);map->info.origin.position.y=fixture.at("origin").at(1);
  map->info.origin.orientation.w=1;map->data=fixture.at("data").get<std::vector<int8_t>>();input.map=map;
  const auto result=ca::exploration_route(input);ASSERT_GT(result.frontiers,0U);
  EXPECT_TRUE(result.route.empty());EXPECT_NE(result.reason.find("无安全观测点"),std::string::npos)<<result.reason;
}

TEST(ExplorationRoute, SymmetricStartupCanMakeAUsefulSafeStep){
  auto input=scenario();auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  std::ifstream file(std::filesystem::path(__FILE__).parent_path()/"data/exploration_symmetric_start.json");
  const auto fixture=ca::Json::parse(file);map->header.frame_id="map";map->info.width=fixture.at("width");map->info.height=fixture.at("height");
  map->info.resolution=fixture.at("resolution");map->info.origin.position.x=fixture.at("origin").at(0);map->info.origin.position.y=fixture.at("origin").at(1);
  map->info.origin.orientation.w=1;map->data=fixture.at("data").get<std::vector<int8_t>>();input.map=map;
  const auto result=ca::exploration_route(input);ASSERT_GT(result.frontiers,0U);
  EXPECT_FALSE(result.route.empty())<<result.reason;
}
