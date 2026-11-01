#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <thread>
#include "carcar_mapping_autonomy/return_selector.hpp"

using namespace carcar_mapping_autonomy;

namespace {
ReturnSelector::Job open_map() {
  ReturnSelector::Job job;
  job.operation=7;job.generation=3;job.revision=11;job.request_id="return-offline-7";
  job.current={1,0,0};job.task.anchored=true;job.task.origin={.008,-.003,.47};
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id="map";map->info.width=map->info.height=100;map->info.resolution=.05;
  map->info.origin.position.x=map->info.origin.position.y=-2.5;map->info.origin.orientation.w=1;
  map->data.assign(10000,0);job.map=map;return job;
}

std::shared_ptr<nav_msgs::msg::OccupancyGrid> copy_map(ReturnSelector::Job & job) {
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*job.map);job.map=map;return map;
}

void block_origin(ReturnSelector::Job & job,int8_t value=100) {
  auto map=copy_map(job);map->data[50*map->info.width+50]=value;
}

double distance(const Pose & a,const Pose & b) {return std::hypot(a.x-b.x,a.y-b.y);}

std::optional<ReturnSelector::Result> wait_result(ReturnSelector & worker) {
  for(int i=0;i<2500;++i) {
    if(auto result=worker.take())return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return {};
}
}  // namespace

TEST(ReturnSelector, ExactOriginHasPriorityAndPreservesOriginalYawAndMap) {
  const auto job=open_map();const auto original=job.map->data;
  const auto result=ReturnSelector::Search::select(job);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_DOUBLE_EQ(result.candidates.front().x,job.task.origin.x);
  EXPECT_DOUBLE_EQ(result.candidates.front().y,job.task.origin.y);
  EXPECT_DOUBLE_EQ(result.candidates.front().yaw,job.task.origin.yaw);
  EXPECT_NE(result.reason.find("起点可用"),std::string::npos);
  EXPECT_EQ(original,job.map->data);
}

TEST(ReturnSelector, OccupiedOriginProducesSafeNearbyCandidatesWithoutClearingMap) {
  auto job=open_map();block_origin(job);const auto original=job.map->data;
  ASSERT_FALSE(free_footprint(*job.map,job.task.origin));
  const auto result=ReturnSelector::Search::select(job);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_NE(result.reason.find("起点不可用"),std::string::npos);
  for(const auto & target:result.candidates) {
    EXPECT_GT(distance(target,job.task.origin),.22);
    EXPECT_LE(distance(target,job.task.origin)+job.arrival_tolerance,job.radius+1e-9);
    EXPECT_TRUE(planning_footprint(*job.map,target));EXPECT_TRUE(job.task.inside(target));
    EXPECT_DOUBLE_EQ(target.yaw,job.task.origin.yaw);
  }
  EXPECT_EQ(original,job.map->data);
}

TEST(ReturnSelector, UnknownOriginDoesNotBecomeAFreeGoal) {
  auto job=open_map();block_origin(job,-1);
  const auto result=ReturnSelector::Search::select(job);
  ASSERT_FALSE(result.candidates.empty());
  for(const auto & target:result.candidates)EXPECT_TRUE(planning_footprint(*job.map,target));
  EXPECT_GT(distance(result.candidates.front(),job.task.origin),.22);
}

TEST(ReturnSelector, UnknownAreaAndMapBoundaryDoNotProvideUnsafeCandidates) {
  auto job=open_map();auto map=copy_map(job);
  map->data.assign(map->data.size(),-1);
  for(int y=35;y<65;++y)for(int x=65;x<95;++x)map->data[y*100+x]=0;
  job.current={1.5,0,0};
  const auto unknown=ReturnSelector::Search::select(job);
  EXPECT_TRUE(unknown.candidates.empty());
  EXPECT_NE(unknown.reason.find("没有"),std::string::npos);
  job=open_map();job.task.origin={-2.45,0,.2};job.current={-1.5,0,0};
  const auto edge=ReturnSelector::Search::select(job);
  ASSERT_FALSE(edge.candidates.empty());
  for(const auto & target:edge.candidates) {
    EXPECT_TRUE(planning_footprint(*job.map,target));
    EXPECT_GE(target.x,-2.5+.22+std::sqrt(2.0)*job.map->info.resolution);
  }
}

TEST(ReturnSelector, FreeOriginBehindAClosedWallIsNotReachable) {
  auto job=open_map();auto map=copy_map(job);
  for(int y=0;y<100;++y)map->data[y*100+50]=100;
  job.task.origin={-.75,0,.5};job.current={1,0,0};
  ASSERT_TRUE(free_footprint(*map,job.task.origin));
  const auto result=ReturnSelector::Search::select(job);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_NE(result.reason.find("起点不可用"),std::string::npos);
}

TEST(ReturnSelector, NearbyTargetsStayOnCurrentSideOfWall) {
  auto job=open_map();auto map=copy_map(job);
  for(int y=0;y<100;++y)map->data[y*100+50]=100;
  job.task.origin={-.15,0,.5};job.current={1,0,0};
  const auto result=ReturnSelector::Search::select(job);
  ASSERT_FALSE(result.candidates.empty());
  for(const auto & target:result.candidates)EXPECT_GT(target.x,.05);
}

TEST(ReturnSelector, ArrivalReserveAndTaskBoundaryLimitNearbyCenters) {
  auto job=open_map();block_origin(job);job.task.limits.radius=.5;
  job.radius=.6;job.arrival_tolerance=.2;
  const auto result=ReturnSelector::Search::select(job);
  ASSERT_FALSE(result.candidates.empty());
  for(const auto & target:result.candidates) {
    EXPECT_LE(distance(target,job.task.origin),.4+1e-9);
    EXPECT_TRUE(job.task.inside(target));
  }
  job.radius=.3;job.arrival_tolerance=.15;
  EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
}

TEST(ReturnSelector, CandidatesAreSortedSparseAndRespectCountLimit) {
  auto job=open_map();block_origin(job);job.max_candidates=4;
  const auto result=ReturnSelector::Search::select(job);
  ASSERT_EQ(result.candidates.size(),4U);
  for(size_t i=0;i<result.candidates.size();++i) {
    if(i) {EXPECT_LE(distance(result.candidates[i-1],job.task.origin),distance(result.candidates[i],job.task.origin)+1e-12);}
    for(size_t k=0;k<i;++k)EXPECT_GE(distance(result.candidates[i],result.candidates[k]),.12-1e-12);
  }
  job.max_candidates=1;EXPECT_EQ(ReturnSelector::Search::select(job).candidates.size(),1U);
}

TEST(ReturnSelector, DepartureRuleKeepsExistingFootprintAndDoesNotOpenWall) {
  auto job=open_map();auto map=copy_map(job);
  for(int x=0;x<100;++x)map->data[62*100+x]=100;
  job.current={1,.34,0};
  ASSERT_TRUE(free_footprint(*map,job.current));ASSERT_FALSE(planning_footprint(*map,job.current));
  EXPECT_FALSE(ReturnSelector::Search::select(job).candidates.empty());
  job.current={1,.45,0};EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
}

TEST(ReturnSelector, InvalidConfigurationAndUnsafeCurrentFailClosed) {
  const auto original=open_map();auto job=original;
  job.radius=std::numeric_limits<double>::quiet_NaN();EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.radius=job.arrival_tolerance;EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.arrival_tolerance=-.1;EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.max_candidates=0;EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.max_candidates=65;EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.current.x=100;EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.current.yaw=std::numeric_limits<double>::infinity();EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.task.origin.x=std::numeric_limits<double>::quiet_NaN();EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;job.map.reset();EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
  job=original;auto map=copy_map(job);map->info.origin.orientation.z=std::sin(.1);map->info.origin.orientation.w=std::cos(.1);
  EXPECT_TRUE(ReturnSelector::Search::select(job).candidates.empty());
}

TEST(ReturnSelector, StaticResultCarriesTheExactSubmittedSnapshotMetadata) {
  auto job=open_map();block_origin(job);const auto result=ReturnSelector::Search::select(job);
  EXPECT_EQ(result.job.operation,job.operation);EXPECT_EQ(result.job.generation,job.generation);
  EXPECT_EQ(result.job.revision,job.revision);EXPECT_EQ(result.job.request_id,job.request_id);
  EXPECT_EQ(result.job.map,job.map);EXPECT_DOUBLE_EQ(result.job.current.x,job.current.x);
}

TEST(ReturnSelector, WorkerKeepsOnlyLatestRequestAndReturnsItsMetadata) {
  auto old=open_map();ReturnSelector worker;worker.submit(old);
  auto latest=old;latest.operation=9;latest.generation=6;latest.revision=12;latest.request_id="return-latest-9";
  block_origin(latest);
  // 连续覆盖待算作业；旧快照的完成结果也不能冒充本次返程选择。
  for(int i=0;i<20;++i)worker.submit(i==19?latest:old);
  auto result=wait_result(worker);ASSERT_TRUE(result);ASSERT_FALSE(result->candidates.empty());
  EXPECT_EQ(result->job.operation,9U);EXPECT_EQ(result->job.generation,6U);EXPECT_EQ(result->job.revision,12U);
  EXPECT_EQ(result->job.request_id,"return-latest-9");EXPECT_EQ(result->job.map,latest.map);
  EXPECT_GT(distance(result->candidates.front(),latest.task.origin),.22);EXPECT_FALSE(worker.take());
}
