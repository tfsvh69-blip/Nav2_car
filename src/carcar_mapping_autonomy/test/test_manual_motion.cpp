#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
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

TEST(ManualMotion, RecordedBlindDirectionsAreDiagnosedWithinBudgetAndSafeReverseIsPreserved){
  std::ifstream input(MANUAL_SCAN_FIXTURE);ASSERT_TRUE(input.good());nlohmann::json fixture;input>>fixture;
  std::vector<double> elapsed;
  for(const auto&sample:fixture.at("samples")){
    std::vector<ca::RetreatScan> observations;
    for(const auto&frame:sample.at("scans")){
      const auto&s=frame.at("scan");auto scan=clear_scan();
      scan->angle_min=s.at("angle_min");scan->angle_max=s.at("angle_max");scan->angle_increment=s.at("angle_increment");
      scan->range_min=s.at("range_min");scan->range_max=s.at("range_max");scan->ranges.clear();
      for(const auto&r:s.at("ranges"))scan->ranges.push_back(r.is_null()?NAN:r.get<float>());
      const auto&p=frame.at("laser_in_base");observations.push_back({scan,{p[0],p[1],p[2]}});
    }
    for(size_t repeat=0;repeat<40;++repeat){
      const auto begin=std::chrono::steady_clock::now();
      const auto result=ca::manual_motion_check(observations,sample.at("vx"),sample.at("wz"));
      elapsed.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
      EXPECT_FALSE(result.allowed);EXPECT_NE(result.reason.find("盲区"),std::string::npos)<<result.reason;
    }
    // 原始离线完整几何审查已确认这些扫描能覆盖后退；不能用计算优化封死安全方向。
    EXPECT_TRUE(ca::manual_motion_check(observations,-.05,0).allowed)<<sample.at("id");
  }
  std::sort(elapsed.begin(),elapsed.end());const double p99=elapsed.at((elapsed.size()-1)*99/100);
  std::cout<<"现场缺束多帧重放 "<<nlohmann::json{{"samples",elapsed.size()},{"p99_ms",p99},{"max_ms",elapsed.back()}}.dump()<<std::endl;
  EXPECT_LT(p99,4.)<<"8毫秒原预算应留出至少一半常态余量";
}

TEST(ManualMotion, ComplementaryFramesAndReversedAngularOrderPreserveCoverage){
  auto first=clear_scan(),second=clear_scan();first->ranges[360]=NAN;second->ranges[180]=NAN;
  EXPECT_TRUE(ca::manual_motion_check({{first,{0,0,0}},{second,{0,0,0}}},.05,0).allowed);
  // 负角增量仍沿原束号解释，首尾接缝不能被快速反例判定成新的盲区。
  auto reversed=clear_scan();std::reverse(reversed->ranges.begin(),reversed->ranges.end());
  reversed->angle_min=first->angle_max;reversed->angle_increment=-first->angle_increment;
  reversed->angle_max=reversed->angle_min+719*reversed->angle_increment;
  for(double speed:{-.05,.05})EXPECT_TRUE(ca::manual_motion_check({{reversed,{-.05,.01,.07}}},speed,0).allowed);
  auto boundary=clear_scan();boundary->ranges[0]=NAN;
  EXPECT_FALSE(ca::manual_motion_check({{boundary,{0,0,0}}},-.05,0).allowed);
}

TEST(ManualMotion, TinyAngularResolutionCannotInventAnUnsupportedPoint){
  auto scan=clear_scan();scan->angle_min=-1e-6;scan->angle_increment=1.1e-8;
  scan->angle_max=scan->angle_min+719*scan->angle_increment;
  std::fill(scan->ranges.begin(),scan->ranges.end(),NAN);scan->ranges[100]=2;
  ca::retreat_detail::Observation observation;std::string reason;ca::RetreatOptions options;
  ASSERT_TRUE(ca::retreat_detail::observe(*scan,{0,0,0},options,observation,reason));
  EXPECT_TRUE(ca::manual_detail::point_supported({.2,0},{observation}));
}
