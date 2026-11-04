#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "carcar_mapping_autonomy/recovery_observation.hpp"

namespace ca=carcar_mapping_autonomy;
namespace {
sensor_msgs::msg::LaserScan scan(size_t count=720){
  sensor_msgs::msg::LaserScan result;result.angle_min=-M_PI;result.angle_increment=2*M_PI/720;
  result.range_min=.05;result.range_max=12;result.ranges.assign(count,NAN);return result;
}
sensor_msgs::msg::LaserScan from_json(const nlohmann::json&j){
  auto result=scan(0);result.angle_min=j.at("angle_min");result.angle_increment=j.at("angle_increment");
  result.range_min=j.at("range_min");result.range_max=j.at("range_max");
  for(const auto&r:j.at("ranges"))result.ranges.push_back(r.is_null()?NAN:r.get<float>());
  return result;
}
nav_msgs::msg::OccupancyGrid wall_map(){
  nav_msgs::msg::OccupancyGrid map;map.info.width=map.info.height=80;map.info.resolution=.05;
  map.info.origin.position.x=map.info.origin.position.y=-1;map.info.origin.orientation.w=1;map.data.assign(6400,0);
  for(size_t y=0;y<80;++y)map.data[y*80+40]=100;
  return map;
}
tf2::Transform identity(){return tf2::Transform::getIdentity();}
bool mentions_ambiguity(const std::vector<std::string>&reasons){
  return std::any_of(reasons.begin(),reasons.end(),[](const auto&reason){return reason.find("多解竞争")!=std::string::npos;});
}
}

TEST(RecoveryObservation, RecordedSparseNearRangeKeepsAll156RealMeasurements){
  std::ifstream input(std::string(RECOVERY_OBSERVATION_FIXTURE_DIR)+"/loop_guard_scan448_20261104.json");
  ASSERT_TRUE(input.good());const auto fixture=nlohmann::json::parse(input);const auto s=from_json(fixture);
  const auto rays=ca::reference_scan_rays(s,identity(),3);
  EXPECT_EQ(rays.size(),fixture.at("expected_valid_points").get<size_t>());
  EXPECT_GE(rays.size(),ca::ReferenceScore::minimum_points);
}

TEST(RecoveryObservation, CurrentRecordedFaultFramesReachGeometryUsingIndependentRealBeams){
  std::ifstream input(std::string(RECOVERY_OBSERVATION_FIXTURE_DIR)+"/recovery_scan_20261104.json");
  ASSERT_TRUE(input.good());const auto fixture=nlohmann::json::parse(input);const auto&source=fixture.at("map");
  auto map=wall_map();map.info.width=source.at("width");map.info.height=source.at("height");
  map.info.resolution=source.at("resolution");map.info.origin.position.x=source.at("origin")[0];
  map.info.origin.position.y=source.at("origin")[1];map.data=source.at("data").get<std::vector<int8_t>>();
  ca::ReferenceMatcher matcher(map);const auto&t=fixture.at("sensor_in_base");
  const auto&r=t.at("rotation");const auto&p=t.at("translation");
  const tf2::Transform transform(tf2::Quaternion(r[0],r[1],r[2],r[3]),tf2::Vector3(p[0],p[1],p[2]));
  const auto&initial=fixture.at("amcl_initial_pose");const ca::Pose candidate{initial[0],initial[1],initial[2]};
  for(const auto&frame:fixture.at("frames")){
    const auto s=from_json(frame.at("scan"));const auto rays=ca::reference_scan_rays(s,transform,fixture.at("mapping_range_m"));
    EXPECT_EQ(rays.size(),frame.at("eligible_points").get<size_t>());
    const auto geometry=matcher.score(candidate,rays);
    EXPECT_TRUE(geometry.acceptable())<<"points="<<geometry.points<<" coverage="<<geometry.coverage<<" hit="<<geometry.hit_ratio<<" mean="<<geometry.mean;
  }
  const auto result=matcher.verify(candidate,ca::reference_scan_rays(from_json(fixture.at("scan")),transform,3));
  EXPECT_TRUE(result.score.acceptable());EXPECT_TRUE(result.ambiguity_checked);EXPECT_GT(result.global_evaluations,0u);
  // 只报告实际歧义结论，不把单帧离线成功或多解拒绝改写为实车六帧恢复成功。
  std::cout<<"本轮原始故障帧离线重放 "<<nlohmann::json{{"points",result.score.points},{"coverage",result.score.coverage},
    {"hit_ratio",result.score.hit_ratio},{"mean_residual_m",result.score.mean},{"free_conflict",result.score.free_conflict},
    {"ambiguity_checked",result.ambiguity_checked},{"global_samples",result.global_evaluations},
    {"ambiguity_margin",result.ambiguity_margin},{"global_sample_margin",result.global_margin},{"unique",result.unique}}.dump()<<std::endl;
}

TEST(RecoveryObservation, PreservesOriginalBeamAnglesAndFullLaserTransformWhileLimitingTo180){
  auto s=scan();std::fill(s.ranges.begin(),s.ranges.end(),1);
  tf2::Quaternion rotation;rotation.setRPY(.04,-.07,.31);
  const tf2::Transform transform(rotation,tf2::Vector3(-.05,.03,.217));
  const auto rays=ca::reference_scan_rays(s,transform,3);ASSERT_EQ(rays.size(),180u);
  for(size_t selected=0;selected<rays.size();++selected){
    // 以原始束号计算独立期望值，禁止重编号、重复取点或丢掉有效扇区末端。
    const size_t index=selected*719/179;
    const double angle=s.angle_min+index*double(s.angle_increment);
    const auto expected=transform*tf2::Vector3(std::cos(angle),std::sin(angle),0);
    EXPECT_NEAR(rays[selected].x,expected.x(),1e-12);EXPECT_NEAR(rays[selected].y,expected.y(),1e-12);
    EXPECT_DOUBLE_EQ(rays[selected].ox,-.05);EXPECT_DOUBLE_EQ(rays[selected].oy,.03);
  }
}

TEST(RecoveryObservation, Keeps49And50PointBoundaryWithoutDuplicatingInsufficientMeasurements){
  ca::ReferenceMatcher matcher(wall_map());
  for(size_t count:{49u,50u}){
    auto s=scan();s.angle_min=-.11;s.angle_increment=.001;
    std::fill(s.ranges.begin()+100,s.ranges.begin()+100+count,1);
    const auto rays=ca::reference_scan_rays(s,identity(),3);ASSERT_EQ(rays.size(),count);
    const auto geometry=matcher.score({},rays);
    EXPECT_EQ(geometry.acceptable(),count>=50)<<geometry.mean<<"/"<<geometry.hit_ratio;
  }
}

TEST(RecoveryObservation, InvalidAndOutOfRangeBeamsCannotBecomeExtraEvidence){
  auto s=scan(8);s.ranges={.1499f,.15f,NAN,INFINITY,2.9499f,2.9501f,0,-INFINITY};
  EXPECT_EQ(ca::reference_scan_rays(s,identity(),3).size(),2u);
  s.range_min=.20;EXPECT_EQ(ca::reference_scan_rays(s,identity(),3).size(),1u);
  s.range_max=2;EXPECT_TRUE(ca::reference_scan_rays(s,identity(),3).empty());
  s.ranges.clear();EXPECT_TRUE(ca::reference_scan_rays(s,identity(),3).empty());
}

TEST(RecoveryObservation, FailedGeometryDoesNotClaimAnUnperformedAmbiguitySearch){
  ca::ReferenceMatcher matcher(wall_map());
  const std::vector<ca::ReferenceMatcher::Ray> sparse(49,{1,0,0,0});
  const auto result=matcher.verify({},sparse);
  EXPECT_FALSE(result.score.acceptable());EXPECT_FALSE(result.ambiguity_checked);EXPECT_EQ(result.global_evaluations,0u);
  const auto reasons=ca::ReferenceMatcher::verification_failure_reasons(result);
  ASSERT_EQ(reasons.size(),1u);EXPECT_EQ(reasons.front(),"有效匹配点不足50点");EXPECT_FALSE(mentions_ambiguity(reasons));
  const std::vector<ca::ReferenceMatcher::Ray> enough(50,{1,0,0,0});
  const auto wrong=matcher.verify({.8,0,0},enough);
  EXPECT_FALSE(wrong.score.acceptable());EXPECT_FALSE(wrong.ambiguity_checked);
  EXPECT_FALSE(mentions_ambiguity(ca::ReferenceMatcher::verification_failure_reasons(wrong)));
}

TEST(RecoveryObservation, APerformedAmbiguitySearchStillRejectsRepeatedGeometry){
  ca::ReferenceMatcher matcher(wall_map());
  const auto result=matcher.verify({},std::vector<ca::ReferenceMatcher::Ray>(50,{1,0,0,0}));
  ASSERT_TRUE(result.score.acceptable());EXPECT_TRUE(result.ambiguity_checked);EXPECT_FALSE(result.unique);
  EXPECT_TRUE(mentions_ambiguity(ca::ReferenceMatcher::verification_failure_reasons(result)));
}

TEST(RecoveryObservation, TimeoutSummaryDistinguishesFailedGeometryFromMissingCandidatesAndIncompleteFrames){
  const ca::Json sparse={{"points",43},{"ambiguity_checked",false},{"unique",false},
    {"failed_checks",ca::Json::array({"有效匹配点不足50点"})}};
  const auto insufficient=ca::recovery_verification_timeout_reason(sparse,0);
  EXPECT_NE(insufficient.find("已验证 0/6 帧"),std::string::npos);
  EXPECT_NE(insufficient.find("有效匹配点不足50点"),std::string::npos);
  EXPECT_NE(insufficient.find("多解检查尚未执行"),std::string::npos);
  EXPECT_EQ(insufficient.find("未通过重叠、残差、多解"),std::string::npos);
  const auto missing=ca::recovery_verification_timeout_reason(ca::Json::object(),0);
  EXPECT_NE(missing.find("未取得可验证的AMCL候选"),std::string::npos);
  EXPECT_NE(missing.find("扫描时间对齐"),std::string::npos);
  EXPECT_EQ(missing.find("多解竞争"),std::string::npos);
  const ca::Json one_frame={{"ambiguity_checked",true},{"unique",true},{"failed_checks",ca::Json::array()}};
  const auto frames=ca::recovery_verification_timeout_reason(one_frame,3);
  EXPECT_NE(frames.find("已验证 3/6 帧"),std::string::npos);
  EXPECT_NE(frames.find("尚未满足连续六帧"),std::string::npos);
  EXPECT_EQ(frames.find("多解检查尚未执行"),std::string::npos);
}
