#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/observation_selector.hpp"
using namespace carcar_mapping_autonomy;

static ObservationSelector::Job doorway(double width){
  ObservationSelector::Job j;j.revision=1;j.operation=7;j.frontier={1.5,0,0};
  auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>();m->header.frame_id="map";
  m->info.width=m->info.height=100;m->info.resolution=.05;
  m->info.origin.position.x=m->info.origin.position.y=-2.5;m->info.origin.orientation.w=1;m->data.assign(10000,0);
  for(int y=0;y<100;++y)if(std::abs(cell_world(*m,63,y).y)>width/2)m->data[y*100+63]=100;
  j.map=m;return j;
}
TEST(ObservationSelector, VisibleFreeEndpointAcrossNarrowDoorIsNotReachable){
  auto j=doorway(.35);ASSERT_TRUE(safe_observation(*j.map,j.frontier,j.task));
  ObservationSelector::Search search;EXPECT_FALSE(search.select(j));
}
TEST(ObservationSelector, WideDoorKeepsOriginalObservationAndMap){
  auto j=doorway(1.2);auto original=j.map->data;auto expected=safe_observation(*j.map,j.frontier,j.task);
  ObservationSelector::Search search;auto target=search.select(j);ASSERT_TRUE(target);ASSERT_TRUE(expected);
  EXPECT_DOUBLE_EQ(target->x,expected->x);EXPECT_DOUBLE_EQ(target->y,expected->y);EXPECT_EQ(original,j.map->data);
}
TEST(ObservationSelector, AlreadyReachedNearestCandidateDoesNotHideOtherUsefulCandidates){
  auto j=doorway(1.2);j.current=*safe_observation(*j.map,j.frontier,j.task);
  ObservationSelector::Search search;const auto target=search.select(j);ASSERT_TRUE(target);
  EXPECT_GT(std::hypot(target->x-j.current.x,target->y-j.current.y),.25);
}
TEST(ObservationSelector, NewMapAndChangedStartInvalidateComponentCache){
  auto j=doorway(.35);ObservationSelector::Search search;EXPECT_FALSE(search.select(j));
  j.current={1.2,0,0};EXPECT_TRUE(search.select(j));
  j.current={0,0,0};EXPECT_FALSE(search.select(j));
  j.map=doorway(1.2).map;++j.revision;EXPECT_TRUE(search.select(j));
  j.map=doorway(.35).map;++j.revision;EXPECT_FALSE(search.select(j));
}
TEST(ObservationSelector, CanChooseReachableAlternativeWithinFrontierWindow){
  auto j=doorway(.35);j.frontier={.725,0,0};ObservationSelector::Search search;
  auto nearest=safe_observation(*j.map,j.frontier,j.task);ASSERT_TRUE(nearest);EXPECT_GT(nearest->x,.65);
  auto target=search.select(j);ASSERT_TRUE(target);EXPECT_LT(target->x,.65);
  EXPECT_TRUE(observation_visible(*j.map,*target,j.frontier));EXPECT_TRUE(planning_footprint(*j.map,*target));
}
TEST(ObservationSelector, InvalidAndUnsafeStartsFailClosed){
  auto j=doorway(1.2);ObservationSelector::Search search;
  j.current={1e100,0,0};EXPECT_FALSE(search.select(j));
  j.current={NAN,0,0};EXPECT_FALSE(search.select(j));
  j.current={.65,1,0};EXPECT_FALSE(search.select(j));
  j.map.reset();EXPECT_FALSE(search.select(j));
}
TEST(ObservationSelector, DepartureUsesExistingMarginRuleWithoutOpeningActualObstacles){
  auto j=doorway(1.2);auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(*j.map);
  m->data.assign(10000,0);for(int x=0;x<100;++x)m->data[62*100+x]=100;j.map=m;
  j.current={0,.34,0};j.frontier={1,0,0};ASSERT_TRUE(free_footprint(*m,j.current));ASSERT_FALSE(planning_footprint(*m,j.current));
  ObservationSelector::Search search;EXPECT_TRUE(search.select(j));
  j.current={0,.45,0};EXPECT_FALSE(search.select(j));
}
TEST(ObservationSelector, MapGrowthPreservesWorldSelectionAndWorkerTagsSnapshot){
  auto j=doorway(1.2);ObservationSelector::Search search;auto target=search.select(j);ASSERT_TRUE(target);
  auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(*j.map);m->info.origin.position.x-=.05;m->info.width=101;
  m->data.assign(10100,0);for(int y=0;y<100;++y)for(int x=0;x<100;++x)m->data[y*101+x+1]=j.map->data[y*100+x];
  j.map=m;++j.revision;auto grown=search.select(j);ASSERT_TRUE(grown);EXPECT_NEAR(target->x,grown->x,1e-7);EXPECT_NEAR(target->y,grown->y,1e-7);
  ObservationSelector worker;worker.submit(j);std::optional<ObservationSelector::Result> result;
  for(int i=0;i<500&&!result;++i){std::this_thread::sleep_for(std::chrono::milliseconds(2));result=worker.take();}
  ASSERT_TRUE(result);ASSERT_TRUE(result->target);EXPECT_EQ(result->job.operation,7U);EXPECT_EQ(result->job.revision,2U);
}
