#include <fstream>
#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/observation_selector.hpp"
#include "carcar_mapping_autonomy/return_selector.hpp"

namespace ca = carcar_mapping_autonomy;
static nav_msgs::msg::OccupancyGrid recorded_map() {
  std::ifstream input(SUBCELL_FIXTURE);
  const auto j = ca::Json::parse(input);
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = "map";
  m.info.width = j["info"]["width"];
  m.info.height = j["info"]["height"];
  m.info.resolution = j["info"]["resolution"];
  m.info.origin.position.x = j["info"]["origin"]["position"]["x"];
  m.info.origin.position.y = j["info"]["origin"]["position"]["y"];
  m.info.origin.orientation.w = 1;
  m.data = j["data"].get<std::vector<int8_t>>();
  return m;
}

TEST(SubcellDeparture, RecordedStoppedPoseCanSelectReachableObservationWithoutClearingUnknown) {
  // 现场停稳坐标与地图保持原值；格心不安全，实际0.22米包络仍安全。
  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>(recorded_map());
  ca::ObservationSelector::Job job;
  job.map = m; job.revision = 1;
  job.current = {.04239974986262743, -.012599852844464556, -.41358128558844526};
  job.frontier = {.227, .254, 0};
  ASSERT_TRUE(ca::free_footprint(*m, job.current));
  ASSERT_FALSE(ca::free_footprint(*m, ca::cell_world(*m, 34, 31)));
  const auto original = m->data;
  ca::ObservationSelector::Search search;
  const auto result = search.select(job);
  EXPECT_TRUE(result.has_value()) << "真实车位安全且可短距离衔接邻近安全格，不能因取整格心被困";
  EXPECT_EQ(m->data, original) << "不得填白未知格或修改现场地图";
}

TEST(SubcellDeparture, ContinuousConnectorRejectsCornerCrossingAndUnknownInterior) {
  auto m=recorded_map();
  const ca::Pose stopped{.04239974986262743,-.012599852844464556,0};
  const auto mask=ca::planning_keepout_mask(m);ASSERT_TRUE(mask);
  const auto entries=ca::departure_entries(m,stopped,*mask);ASSERT_FALSE(entries.empty());
  for(const auto&p:entries) {
    EXPECT_LE(std::hypot(p.x-stopped.x,p.y-stopped.y),.15);
    EXPECT_TRUE(ca::free_connector(m,stopped,p));
  }
  EXPECT_FALSE(ca::free_connector(m,stopped,ca::cell_world(m,34,31)));
  // 两端安全而中间穿过未知格的直线必须拒绝。
  m.info.width=m.info.height=40;m.info.resolution=.05;
  m.info.origin.position.x=m.info.origin.position.y=-1.;m.data.assign(1600,0);
  m.data[20*40+20]=-1;
  ASSERT_TRUE(ca::free_footprint(m,{-.5,0,0}));ASSERT_TRUE(ca::free_footprint(m,{.5,0,0}));
  EXPECT_FALSE(ca::free_connector(m,{-.5,0,0},{.5,0,0}));
  EXPECT_TRUE(ca::free_connector(m,{-.5,-.3,0},{.5,-.3,0}));
}

TEST(SubcellDeparture, UnsafeActualPositionCannotAcquireAReachableComponent) {
  auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(recorded_map());
  ca::ObservationSelector::Job job;job.map=m;job.revision=1;job.frontier={.227,.254,0};
  job.current={.04239974986262743,-.012599852844464556,0};
  ca::ObservationSelector::Search search;ASSERT_TRUE(search.select(job));
  // 不改变所在格，只移动到该格心；缓存不能沿用旧的实际位置连接。
  job.current=ca::cell_world(*m,34,31);EXPECT_FALSE(search.select(job));
}

TEST(SubcellDeparture, ReturnUsesTheSameCheckedSubcellConnectionAsExploration) {
  auto m=std::make_shared<nav_msgs::msg::OccupancyGrid>(recorded_map());
  ca::ReturnSelector::Job job;job.map=m;
  job.current={.04239974986262743,-.012599852844464556,0};
  job.task.origin={.47661976260876737,.0038987502426766074,0};
  const auto original=m->data;
  ASSERT_FALSE(ca::ReturnSelector::Search::select(job).candidates.empty());
  EXPECT_EQ(m->data,original);
  job.current=ca::cell_world(*m,34,31);
  EXPECT_TRUE(ca::ReturnSelector::Search::select(job).candidates.empty());
}
