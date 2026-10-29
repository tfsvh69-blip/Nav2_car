#include <chrono>
#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/near_field.hpp"
#include "carcar_mapping_autonomy/task.hpp"
#include "carcar_mapping_autonomy/map_geometry.hpp"
#include "carcar_mapping_autonomy/startup_accumulation.hpp"
using namespace carcar_mapping_autonomy;
TEST(Task, StartupManualZeroOwnership){Task t;
EXPECT_EQ(t.phase,"MANUAL");
EXPECT_FALSE(t.automatic());
EXPECT_FALSE(t.anchored);}
TEST(Task, MustBeReadyAndStillBeforeAutomatic){Task t;
EXPECT_THROW(t.start(1,{},false,true),std::runtime_error);
EXPECT_THROW(t.start(1,{},true,false),std::runtime_error);}
TEST(Task, ResumeAndManualDoNotResetOriginOrBudget){Task t;t.start(1,{2,3,0},true,true);t.tick(11,{2,3,0},true);t.stop("MANUAL","接管",11);t.stopped(11.4,true,true,true);
ASSERT_EQ(t.phase,"MANUAL");t.start(100,{3,3,0},true,true);
EXPECT_EQ(t.origin.x,2);
EXPECT_EQ(t.elapsed,10);t.tick(105,{3,3,0},true);
EXPECT_EQ(t.elapsed,15);}
TEST(Task, CancelMustReachTerminalAndStandstill){Task t;t.start(1,{},true,true);t.stop("MANUAL","接管",2);t.stopped(2.4,false,true,true);
EXPECT_EQ(t.phase,"STOPPING");t.stopped(2.5,true,false,true);
EXPECT_EQ(t.phase,"STOPPING");t.stopped(2.6,true,true,true);
EXPECT_EQ(t.phase,"MANUAL");}
TEST(Task, CancellationTimeoutLocksRatherThanHandsOver){Task t;t.start(1,{},true,true);t.stop("MANUAL","接管",2);t.stopped(7.1,false,true,true);
EXPECT_EQ(t.phase,"FAULT");
EXPECT_FALSE(t.save_required);
EXPECT_THROW(t.start(8,{},true,true),std::runtime_error);}
TEST(Task, BudgetExhaustionReturnsThenRequiresReview){Task t;t.limits.explore_seconds=10;t.start(1,{},true,true);t.tick(12,{},true);
ASSERT_EQ(t.phase,"STOPPING");t.stopped(12.4,true,true,true);
ASSERT_EQ(t.phase,"RETURNING");t.returned(true,"");
EXPECT_TRUE(t.save_required);t.save_done();
EXPECT_EQ(t.phase,"REVIEW_REQUIRED");}
TEST(Task, RadiusLimitCannotBeResetByPause){Task t;t.start(1,{},true,true);t.tick(2,{21,0,0},true);
EXPECT_EQ(t.stop_target,"FINISH");t.stopped(2.4,true,true,true);
EXPECT_EQ(t.phase,"RETURNING");}
TEST(Task, FaultDoesNotAutomaticallyReturnAndCanSaveEvidence){Task t;t.start(1,{},true,true);t.tick(2,{},false);
EXPECT_EQ(t.stop_target,"FAULT");t.stopped(2.4,true,true,false);
EXPECT_EQ(t.phase,"FAULT");
EXPECT_TRUE(t.save_required);t.save_done();
EXPECT_EQ(t.phase,"FAULT");}
TEST(Task, ReturnTimeoutStopsBeforeSaving){Task t;t.start(1,{},true,true);t.stop("FINISH","完成",2);t.stopped(2.3,true,true,true);t.tick(603,{},true);
EXPECT_EQ(t.phase,"STOPPING");t.stopped(603.4,true,true,true);
EXPECT_EQ(t.phase,"SAVE_PENDING");}
TEST(Task, ReconnectCannotStartDuplicateTask){Task t;t.start(1,{},true,true);const auto epoch=t.generation;
EXPECT_THROW(t.start(2,{},true,true),std::runtime_error);
EXPECT_EQ(t.generation,epoch);}
TEST(Task, PartialEndingRemainsPartialAfterSuccessfulReturn){Task t;t.start(1,{},true,true);t.stop("FINISH","候选全部不可达",2,"partial_unreachable");t.stopped(2.4,true,true,true);t.returned(true,"");
EXPECT_EQ(t.outcome,"partial_unreachable");EXPECT_EQ(t.end_reason,"候选全部不可达");EXPECT_EQ(t.return_result,"succeeded");t.save_done();
EXPECT_THROW(t.start(3,{},true,true),std::runtime_error);}
TEST(Task, ReturnPauseAndManualKeepOriginalDeadline){Task t;t.limits.explore_seconds=1;t.start(1,{},true,true);t.tick(3,{},true);t.stopped(3.4,true,true,true);const auto deadline=t.return_started;
t.stop("PAUSED","暂停返程",4);t.stopped(4.4,true,true,true);t.stop("MANUAL","接管",5);t.stopped(5.4,true,true,true);t.start(10,{},true,true);
EXPECT_EQ(t.phase,"RETURNING");EXPECT_EQ(t.return_started,deadline);EXPECT_EQ(t.elapsed,2);}
TEST(Task, ExpiredPausedReturnSavesWithoutRestartingExploration){Task t;t.start(1,{},true,true);t.stop("FINISH","结束",2);t.stopped(2.4,true,true,true);t.stop("PAUSED","暂停",3);t.stopped(3.4,true,true,true);t.start(604,{},true,true);
EXPECT_EQ(t.phase,"SAVE_PENDING");EXPECT_EQ(t.return_result,"timeout");EXPECT_TRUE(t.save_required);}
static nav_msgs::msg::OccupancyGrid room(){nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.resolution=.05;m.info.width=m.info.height=100;m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;m.data.assign(10000,0);return m;}
TEST(MapGeometry, UnknownIsNotFreeAndFootprintMustFit){auto m=room();
EXPECT_TRUE(free_footprint(m,{}));m.data[50*100+50]=-1;
EXPECT_FALSE(free_footprint(m,{}));
EXPECT_FALSE(free_footprint(m,{2.49,0,0}));}
TEST(MapGeometry, CapturedStartupRayGapsDoNotProveOriginClear){
// 2026-10-27实车首帧地图的11x11起点窗口，保留影响足迹判定的3个未知格。
// 起点中心格已经是自由，但邻近射线缝仍未观测；不能因扫描/TF新鲜而清空它们。
nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.resolution=.05;
m.info.width=m.info.height=11;m.info.origin.position.x=m.info.origin.position.y=-.29;
m.info.origin.orientation.w=1;m.data.assign(121,0);
for(int y=0;y<=2;++y)m.data[y*11+6]=-1;
ASSERT_EQ(m.data[5*11+5],0);
EXPECT_FALSE(free_footprint(m,{}));
// 仅刷新消息时间戳不会增加任何几何观测，不得改变起点的安全判断。
m.header.stamp.sec=2000000000;
EXPECT_FALSE(free_footprint(m,{}));}
TEST(MapGeometry, NearbyThinObstacleIsRetainedInFootprint){auto m=room();m.data[50*100+53]=100;
EXPECT_FALSE(free_footprint(m,{}));}
TEST(MapGeometry, FrontierUnknownCentroidProjectsToKnownSafeCell){auto m=room();for(int y=0;y<100;++y)for(int x=60;x<100;++x)m.data[y*100+x]=-1;Task t;t.start(1,{},true,true);auto p=safe_observation(m,{.55,0,0},t);
ASSERT_TRUE(p);
EXPECT_TRUE(free_footprint(m,*p));
EXPECT_LT(p->x,.3);}
TEST(MapGeometry, OriginAndRotationAreRespected){auto m=room();m.info.origin.position.x=5;m.info.origin.position.y=6;m.info.origin.orientation.z=std::sin(M_PI/4);m.info.origin.orientation.w=std::cos(M_PI/4);const auto p=cell_world(m,50,50);
EXPECT_TRUE(free_footprint(m,p));}
TEST(MapGeometry, CorruptDimensionsAndNonPlanarOriginFailClosed){auto m=room();m.data.pop_back();
EXPECT_FALSE(grid_valid(m));m=room();m.info.origin.orientation.x=.1;
EXPECT_FALSE(free_footprint(m,{}));}
TEST(Acceptance, NormalLoopDoesNotCountAsRejectedScan){
EXPECT_FALSE(acceptance_stalled(true,true,10,10,1));
EXPECT_FALSE(acceptance_stalled(true,false,1,10,1));
EXPECT_FALSE(acceptance_stalled(false,false,10,10,1));
EXPECT_FALSE(acceptance_stalled(true,false,4,4,.1));
EXPECT_TRUE(acceptance_stalled(true,false,4,4,.2));}
TEST(MapGeometry, LargeMapGoalSearchStaysInsideObservationWindow){
nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.resolution=.05;m.info.width=m.info.height=1000;
m.info.origin.position.x=m.info.origin.position.y=-25;m.info.origin.orientation.w=1;m.data.assign(1000000,0);
Task t;t.start(1,{},true,true);
const auto began=std::chrono::steady_clock::now();
const auto pose=safe_observation(m,{0,0,0},t);
const auto elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
ASSERT_TRUE(pose);
EXPECT_LT(std::hypot(pose->x,pose->y),.08);
EXPECT_LT(elapsed_ms,150)<<"整图搜索会堵住50毫秒速度循环";}
TEST(StartupAccumulation, StationaryWindowAcceptsThreeScansThenCloses){
StartupAccumulation s;
EXPECT_EQ(consider_stationary_startup(s,true,false,0,0,10,.10,.30,3,2),StartupDecision::ignore);
EXPECT_TRUE(s.anchored);
EXPECT_EQ(consider_stationary_startup(s,false,false,0,0,10.10,.10,.30,3,2),StartupDecision::ignore);
EXPECT_EQ(s.extras,0);
EXPECT_EQ(consider_stationary_startup(s,false,false,0,0,10.30,.10,.30,3,2),StartupDecision::accept_extra);
EXPECT_EQ(consider_stationary_startup(s,false,false,0,0,10.60,.10,.30,3,2),StartupDecision::accept_extra);
EXPECT_EQ(consider_stationary_startup(s,false,false,0,0,10.90,.10,.30,3,2),StartupDecision::accept_extra);
EXPECT_EQ(s.extras,3);
EXPECT_EQ(consider_stationary_startup(s,false,false,0,0,12,.10,.30,3,2),StartupDecision::ignore);
EXPECT_TRUE(s.closed);
EXPECT_EQ(s.extras,3);}
TEST(StartupAccumulation, MotionAndPauseDoNotSpendTheStationaryBudget){
StartupAccumulation moving;
consider_stationary_startup(moving,true,false,0,0,0,.10,.30,3,2);
EXPECT_EQ(consider_stationary_startup(moving,false,false,.08,0,.30,.10,.30,3,2),StartupDecision::accept_extra);
EXPECT_EQ(consider_stationary_startup(moving,false,false,.10,0,.60,.10,.30,3,2),StartupDecision::ignore);
EXPECT_TRUE(moving.closed);
EXPECT_EQ(moving.extras,1);
StartupAccumulation held;
consider_stationary_startup(held,true,false,0,0,0,.10,.30,3,2);
EXPECT_EQ(consider_stationary_startup(held,false,true,0,0,.40,.10,.30,3,2),StartupDecision::ignore);
EXPECT_EQ(held.extras,0);
EXPECT_FALSE(held.closed);
StartupAccumulation disabled;
consider_stationary_startup(disabled,true,false,0,0,0,.10,.30,0,2);
EXPECT_EQ(consider_stationary_startup(disabled,false,false,0,0,.40,.10,.30,0,2),StartupDecision::ignore);
EXPECT_TRUE(disabled.closed);
EXPECT_EQ(disabled.extras,0);}
TEST(NearField, BodyBoxIgnoresReturnsOutsideTheNarrowChassis){
const double laser_x=-0.05;
const auto outside=assess_near_field({{0.155,85*M_PI/180}},laser_x,0,0);
EXPECT_TRUE(outside.valid);EXPECT_FALSE(outside.inside);EXPECT_EQ(outside.side,"左侧");
EXPECT_GT(outside.y,kBodyMaxY);
const auto inside=assess_near_field({{0.12,M_PI/2}},laser_x,0,0);
EXPECT_TRUE(inside.inside);EXPECT_EQ(inside.inside_count,1);EXPECT_EQ(inside.side,"左侧");
const auto note=motion_block_note(true,true,inside);
EXPECT_NE(note.find("激光进入车身停车框"),std::string::npos);
EXPECT_NE(note.find("左侧"),std::string::npos);
EXPECT_TRUE(motion_block_note(false,true,inside).empty());
EXPECT_NE(motion_block_note(true,true,outside).find("停车框外"),std::string::npos);
}
