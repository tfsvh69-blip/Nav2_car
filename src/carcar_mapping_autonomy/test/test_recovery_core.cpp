#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include "carcar_mapping_autonomy/recovery_core.hpp"
#include "carcar_mapping_autonomy/recovery_commit_guard.hpp"
#include "carcar_mapping_autonomy/recovery_artifact.hpp"
namespace ca=carcar_mapping_autonomy;
static nav_msgs::msg::OccupancyGrid room(bool corridor=false){
  nav_msgs::msg::OccupancyGrid m;m.info.resolution=.025;m.info.width=400;m.info.height=240;
  m.info.origin.position.x=-5;m.info.origin.position.y=-3;m.info.origin.orientation.w=1;
  m.data.assign(400*240,0);
  for(unsigned y=0;y<m.info.height;++y)for(unsigned x=0;x<m.info.width;++x){
    const double wx=-5+(x+.5)*.025,wy=-3+(y+.5)*.025;
    bool occupied=std::abs(wy-1.8)<.04||std::abs(wy+1.8)<.04;
    if(!corridor)occupied=occupied||std::abs(wx+2.5)<.04||std::abs(wx-3.2)<.04||
      (wx>.8&&wx<1.3&&wy>.3&&wy<.8)||(wx>-.9&&wx<-.5&&wy>1.0&&wy<1.4);
    if(occupied)m.data[y*m.info.width+x]=100;
  }
  return m;
}
static std::vector<ca::ReferenceMatcher::Ray> scans(const nav_msgs::msg::OccupancyGrid&m,double max_range=4){
  std::vector<ca::ReferenceMatcher::Ray> rays;
  for(int i=0;i<180;++i){const double a=i*2*M_PI/180;
    for(double r=.15;r<max_range;r+=.01){const double x=r*std::cos(a),y=r*std::sin(a);
      const int ix=(x-m.info.origin.position.x)/m.info.resolution,iy=(y-m.info.origin.position.y)/m.info.resolution;
      if(ix<0||iy<0||ix>=int(m.info.width)||iy>=int(m.info.height))break;
      if(m.data[iy*m.info.width+ix]>=65){rays.push_back({x,y,0,0});break;}
    }
  }return rays;
}
TEST(RecoveryImu, CalibratesOnlyWhileStillAndDoesNotUseQuaternion){
  ca::DisturbanceMonitor m;
  for(int i=0;i<80;++i)m.update({0,0,9.81},{0,0,0},false,i*.05);
  EXPECT_FALSE(m.ready);
  for(int i=80;i<180;++i)m.update({0,0,9.81},{0,0,0},true,i*.05);
  EXPECT_TRUE(m.ready);EXPECT_TRUE(m.settled(8.95));
  const std::array<double,3> lean{9.81*std::sin(.25),0,9.81*std::cos(.25)};
  m.update(lean,{0,0,0},false,9);
  EXPECT_FALSE(m.disturbed)<<"行驶中的比力偏转不单独闭门";
  m.update(lean,{0,0,0},true,9.05);
  EXPECT_FALSE(m.disturbed)<<"停稳后的单次倾斜采样不闭门";
  m.update(lean,{0,0,0},true,9.80);
  EXPECT_TRUE(m.disturbed);EXPECT_FALSE(m.settled(9.80));
  for(int i=0;i<80;++i)m.update({0,0,9.81},{0,0,0},true,10+i*.05);
  EXPECT_TRUE(m.settled(12));EXPECT_TRUE(m.disturbed);
  m.acknowledge();EXPECT_FALSE(m.disturbed);
}
TEST(RecoveryImu, SingleMovingShockDoesNotLatchUntilItPersists){
  ca::DisturbanceMonitor m;
  for(int i=0;i<80;++i)m.update({0,0,9.81},{0,0,0},true,i*.05);
  ASSERT_TRUE(m.ready);
  m.update({0,0,16.5},{0,0,0},false,4.1);
  EXPECT_FALSE(m.disturbed);
  m.update({0,0,16.5},{0,0,0},false,4.4);
  EXPECT_TRUE(m.disturbed);
}
TEST(RecoveryImu, StaleOrPersistentTiltNeverSettles){
  ca::DisturbanceMonitor m;
  for(int i=0;i<100;++i)m.update({0,0,9.81},{0,0,0},true,1+i*.05);
  EXPECT_FALSE(m.settled(7));
  for(int i=0;i<100;++i)m.update({2.5,0,9.5},{0,0,0},true,7+i*.05);
  EXPECT_FALSE(m.settled(11.95));
}
TEST(RecoveryImu, AGapOrInvalidSampleRestartsContinuousStabilityWindow){
  ca::DisturbanceMonitor m;
  for(int i=0;i<100;++i)m.update({0,0,9.81},{0,0,0},true,1+i*.05);
  ASSERT_TRUE(m.settled(5.95));
  m.update({0,0,9.81},{0,0,0},true,10);EXPECT_FALSE(m.settled(10));
  for(int i=1;i<=45;++i)m.update({0,0,9.81},{0,0,0},true,10+i*.05);
  ASSERT_TRUE(m.settled(12.25));
  m.update({NAN,0,9.81},{0,0,0},true,12.3);EXPECT_FALSE(m.settled(12.3));
  m.update({0,0,9.81},{0,0,0},true,12.35);EXPECT_FALSE(m.settled(12.35));
}
TEST(RecoveryGeometry, CorrectAsymmetricRoomPassesWrongPoseAndUnknownDoNot){
  const auto m=room();ca::ReferenceMatcher matcher(m);const auto rays=scans(m);
  ASSERT_GT(rays.size(),100u);
  const auto good=matcher.verify({},rays);
  EXPECT_TRUE(good.score.acceptable())<<good.score.mean<<' '<<good.score.free_conflict;
  EXPECT_TRUE(good.unique)<<good.ambiguity_margin;
  EXPECT_FALSE(matcher.score({.5,.3,.4},rays).acceptable());
  EXPECT_FALSE(matcher.score({20,20,0},rays).acceptable());
}
TEST(RecoveryGeometry, RepetitiveCorridorIsRejectedAsAmbiguous){
  const auto m=room(true);ca::ReferenceMatcher matcher(m);
  const auto candidate=matcher.verify({},scans(m));
  EXPECT_TRUE(candidate.score.acceptable());EXPECT_FALSE(candidate.unique);
}
TEST(RecoveryGeometry, SubcellWallEndpointsHaveOccupiedSupportWithoutFillingUnknown){
  nav_msgs::msg::OccupancyGrid m;m.info.resolution=.05;m.info.width=m.info.height=100;m.info.origin.orientation.w=1;m.data.assign(10000,0);
  for(unsigned y=0;y<100;++y)for(unsigned x=50;x<100;++x)m.data[y*100+x]=x==50?100:-1;
  ca::ReferenceMatcher matcher(m);std::vector<ca::ReferenceMatcher::Ray> rays;
  for(int i=0;i<80;++i)rays.push_back({2.525,1+i*.02,0,0});
  EXPECT_TRUE(matcher.score({},rays).acceptable());
  EXPECT_TRUE(matcher.score({.03,0,0},rays).acceptable());
  EXPECT_FALSE(matcher.score({.13,0,0},rays).acceptable());
  EXPECT_EQ(m.data[25*100+51],-1)<<"匹配支持不能修改未知格";
}
TEST(RecoveryGeometry, CroppedBoundarySupportsAtMostOneCellAndOnlyNearRealOccupiedCells){
  nav_msgs::msg::OccupancyGrid m;m.info.resolution=.05;m.info.width=m.info.height=100;m.info.origin.orientation.w=1;m.data.assign(10000,0);
  for(unsigned y=0;y<100;++y)m.data[y*100+99]=100;
  ca::ReferenceMatcher matcher(m);std::vector<ca::ReferenceMatcher::Ray> rays;
  for(int i=0;i<80;++i)rays.push_back({4.975,1+i*.02,0,0});
  EXPECT_TRUE(matcher.score({.03,0,0},rays).acceptable());
  EXPECT_FALSE(matcher.score({.09,0,0},rays).acceptable());
  std::fill(m.data.begin(),m.data.end(),-1);ca::ReferenceMatcher uncovered(m);
  EXPECT_FALSE(uncovered.score({},rays).acceptable());
}
TEST(RecoveryGeometry, RemoteRepeatedRoomFailsWholeMapCompetitorCheck){
  const auto original=room();auto m=original;m.info.width=720;m.data.assign(static_cast<size_t>(m.info.width)*m.info.height,0);
  for(unsigned y=0;y<original.info.height;++y)for(unsigned x=0;x<original.info.width;++x){
    // 只合并占据格；复制整个零填充画布会抹掉重叠边界，造成两房间本就不同。
    if(original.data[y*original.info.width+x]<65)continue;
    m.data[y*m.info.width+x]=100;m.data[y*m.info.width+x+280]=100;
  }
  ca::ReferenceMatcher matcher(m);const auto result=matcher.verify({},scans(m));
  EXPECT_GT(result.ambiguity_margin,.025);EXPECT_FALSE(result.unique);EXPECT_LT(result.global_margin,.025);
}
TEST(RecoveryFrames, DuplicateTimestampsAndPoseJumpsCannotSatisfyMultiFrameGate){
  ca::MultiFrameVerification frames;
  EXPECT_FALSE(frames.observe({},1,1,true));
  for(int i=0;i<20;++i)EXPECT_FALSE(frames.observe({},1,2+i*.2,true));
  EXPECT_EQ(frames.count,1u);
  EXPECT_FALSE(frames.observe({.3,0,0},2,3,true));EXPECT_EQ(frames.count,1u);
  bool passed=false;for(int i=3;i<9;++i)passed=frames.observe({.3,0,0},i,3+(i-2)*.21,true);
  EXPECT_TRUE(passed);EXPECT_FALSE(frames.observe({},9,5,false));EXPECT_EQ(frames.count,0u);
}
TEST(RecoveryCommit, DuplicateFramesCannotQualifyAndOneBadFrameRevokesWithoutPermanentFailure){
  ca::RecoveryCommitGuard guard;guard.reset(1);
  ca::ReferenceScore good;good.points=80;good.coverage=1;good.hit_ratio=1;good.mean=.02;good.free_conflict=.01;
  ASSERT_TRUE(good.acceptable());
  EXPECT_FALSE(guard.observe({},1,1,good));
  for(int i=0;i<20;++i)EXPECT_FALSE(guard.observe({},1,1.1+i*.01,good));
  EXPECT_EQ(guard.observations.count,1u);
  EXPECT_FALSE(guard.observe({},2,1.1,good));EXPECT_TRUE(guard.observe({},3,1.3,good));
  auto sparse=good;sparse.points=49;EXPECT_FALSE(guard.observe({},4,1.4,sparse));
  EXPECT_FALSE(guard.ready);EXPECT_FALSE(guard.expired(1.5));
  EXPECT_FALSE(guard.observe({},5,1.5,good));EXPECT_FALSE(guard.observe({},6,1.6,good));
  EXPECT_TRUE(guard.observe({},7,1.8,good));EXPECT_FALSE(guard.expired(10));
  auto wall=good;wall.free_conflict=.081;EXPECT_FALSE(guard.observe({},8,10,wall));
  EXPECT_TRUE(guard.expired(15.001));EXPECT_FALSE(guard.ready);
}
TEST(RecoveryCommit, OriginalPoseBoundsAndUnknownSparseOrWallScoresRemainClosed){
  EXPECT_TRUE(ca::RecoveryCommitGuard::agrees({.119,0,.099},{}));
  EXPECT_FALSE(ca::RecoveryCommitGuard::agrees({.12,0,0},{}));
  EXPECT_FALSE(ca::RecoveryCommitGuard::agrees({0,0,.10},{}));
  ca::ReferenceScore good;good.points=80;good.coverage=1;good.hit_ratio=1;good.mean=.02;good.free_conflict=.01;
  for(const std::string failure:{"unknown","sparse","wall","miss","residual"}){
    auto bad=good;
    if(failure=="unknown")bad.coverage=.79;
    if(failure=="sparse")bad.points=49;
    if(failure=="wall")bad.free_conflict=.081;
    if(failure=="miss")bad.hit_ratio=.69;
    if(failure=="residual")bad.mean=.101;
    ca::RecoveryCommitGuard guard;guard.reset(1);
    for(int i=0;i<60;++i)EXPECT_FALSE(guard.observe({},i+1,1+i*.1,bad))<<failure;
    EXPECT_TRUE(guard.expired(7))<<failure;
  }
}
TEST(RecoveryCommit, RecordedCurrentStationaryScansRevokeAndRegainQualificationAtSlamAlignedPose){
  // 19:56当前只读采样，与19:52历史故障帧区分；不改原地图、扫描、位姿或评分门槛。
  const auto path=std::filesystem::path(__FILE__).parent_path()/"data/recovery_commit_stationary_20261101.json";
  std::ifstream in(path);ASSERT_TRUE(in.good());const auto j=nlohmann::json::parse(in);const auto&m=j.at("map");
  nav_msgs::msg::OccupancyGrid map;map.info.width=m.at("width");map.info.height=m.at("height");map.info.resolution=m.at("resolution");
  map.info.origin.position.x=m.at("origin")[0];map.info.origin.position.y=m.at("origin")[1];map.info.origin.orientation.w=1;
  map.data=m.at("data").get<std::vector<int8_t>>();ca::ReferenceMatcher matcher(map);
  const auto&original=j.at("verified_pose");const ca::Pose verified{original[0],original[1],original[2]};
  ca::RecoveryCommitGuard guard;guard.reset(0);bool was_ready=false,revoked=false,regained=false,legacy_rejects_slam_accepts=false;size_t bad=0;
  const auto ns=[](const auto&s){return s.at("sec").template get<int64_t>()*1000000000LL+s.at("nanosec").template get<int64_t>();};
  const auto first=ns(j.at("frames").front().at("stamp"));
  for(const auto&s:j.at("frames")){
    const auto&t=s.at("sensor_in_base");const auto&q=t.at("rotation");
    ASSERT_DOUBLE_EQ(q[0],0);ASSERT_DOUBLE_EQ(q[1],0);ASSERT_DOUBLE_EQ(q[2],0);ASSERT_DOUBLE_EQ(q[3],1);
    const double ox=t.at("translation")[0],oy=t.at("translation")[1];const auto&ranges=s.at("ranges");
    std::vector<ca::ReferenceMatcher::Ray> rays;
    for(size_t i=0;i<ranges.size();i+=std::max<size_t>(1,ranges.size()/180)){
      if(ranges[i].is_null())continue;
      const double r=ranges[i];
      if(r<std::max(.15,s.at("range_min").get<double>())||r>std::min(3.,s.at("range_max").get<double>())-.05)continue;
      const double a=s.at("angle_min").get<double>()+i*s.at("angle_increment").get<double>();
      rays.push_back({ox+r*std::cos(a),oy+r*std::sin(a),ox,oy});
    }
    const auto&p=s.at("base_in_map_at_scan").at("pose2d");const ca::Pose aligned{p[0],p[1],p[2]};
    ASSERT_TRUE(ca::RecoveryCommitGuard::agrees(aligned,verified));
    const auto score=matcher.score(aligned,rays);const auto old=matcher.score(verified,rays);
    if(!old.acceptable()&&score.acceptable())legacy_rejects_slam_accepts=true;
    const auto stamp=ns(s.at("stamp"));const double now=double(stamp-first)/1e9;
    guard.observe(aligned,stamp,now,score);
    if(!score.acceptable()){++bad;if(was_ready)revoked=true;EXPECT_FALSE(guard.ready);}
    if(guard.ready){if(revoked)regained=true;was_ready=true;}
    EXPECT_FALSE(guard.expired(now));
  }
  EXPECT_GT(bad,0u);EXPECT_TRUE(revoked);EXPECT_TRUE(regained);EXPECT_TRUE(legacy_rejects_slam_accepts);
}
TEST(RecoveryArtifact, CorruptionIsRejectedWithoutOverwritingEvidence){
  char path[]="/tmp/carcar-recovery-artifact-XXXXXX";ASSERT_NE(mkdtemp(path),nullptr);const std::filesystem::path directory(path);
  ca::recovery_write_map(directory,room());
  {std::ofstream(directory/"graph.data")<<"raw scans";std::ofstream(directory/"graph.posegraph")<<"graph";}
  nlohmann::json manifest{{"complete",true},{"slam_version","2.6.10"},{"base_frame","base_footprint"}};
  for(const auto*name:{"graph.data","graph.posegraph","map.pgm","map.yaml"})manifest["sha256"][name]=ca::recovery_sha256(directory/name);
  {std::ofstream(directory/"manifest.json")<<manifest.dump();}
  EXPECT_NO_THROW(ca::recovery_validate_artifact(directory));
  {std::ofstream(directory/"graph.data",std::ios::app)<<"corruption";}
  EXPECT_THROW(ca::recovery_validate_artifact(directory),std::exception);
  EXPECT_EQ(std::filesystem::file_size(directory/"graph.data"),19u);
  std::filesystem::remove_all(directory);
}
