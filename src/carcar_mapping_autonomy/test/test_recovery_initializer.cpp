#include <gtest/gtest.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include "carcar_mapping_autonomy/recovery_initializer.hpp"
using namespace carcar_mapping_autonomy;
namespace {
nav_msgs::msg::OccupancyGrid reference(){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.resolution=.05;
  m.info.width=m.info.height=100;m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;
  m.data.assign(10000,0);
  for(unsigned y=0;y<100;++y)for(unsigned x=0;x<100;++x){
    const double wx=-2.5+(x+.5)*.05,wy=-2.5+(y+.5)*.05;
    if(std::abs(wx)>2.3||std::abs(wy)>2.0||(wx>.8&&wx<1.3&&wy>.3&&wy<.8)||
      (wx>-.9&&wx<-.5&&wy>1.0&&wy<1.4))m.data[y*100+x]=100;
  }
  return m;
}
sensor_msgs::msg::LaserScan scan(const nav_msgs::msg::OccupancyGrid&m,const Pose&p){
  sensor_msgs::msg::LaserScan s;s.angle_min=-M_PI;s.angle_increment=2*M_PI/180;s.angle_max=M_PI;
  s.range_min=.05;s.range_max=8;s.ranges.assign(181,8);
  for(size_t i=0;i<s.ranges.size();++i){const double angle=p.yaw+s.angle_min+i*s.angle_increment;
    for(double r=.05;r<7.95;r+=.005){const int x=std::floor((p.x+r*std::cos(angle)+2.5)/.05),y=std::floor((p.y+r*std::sin(angle)+2.5)/.05);
      if(x<0||y<0||x>=100||y>=100||m.data[y*100+x]>=65){s.ranges[i]=r;break;}
    }
  }
  return s;
}
}
TEST(RecoveryInitializer, ExistingKartoMatcherCorrectsTranslationAndYawBeforeAmcl){
  const auto m=reference();const Pose truth{.30,-.10,.20};const auto s=scan(m,truth);
  const auto result=recovery_initial_pose(m,s,{0,0,0},{});
  EXPECT_NEAR(result.x,truth.x,.08);EXPECT_NEAR(result.y,truth.y,.08);EXPECT_NEAR(result.yaw,truth.yaw,.05);
}
TEST(RecoveryInitializer, MissingRangesCannotProduceArtificialBoundaryAndRegistrationIsReleased){
  const auto m=reference();auto s=scan(m,{});std::fill(s.ranges.begin(),s.ranges.end(),std::numeric_limits<float>::quiet_NaN());
  EXPECT_THROW(recovery_initial_pose(m,s,{},{}),std::runtime_error);
  EXPECT_NO_THROW(recovery_initial_pose(m,scan(m,{}),{},{}));
}
TEST(RecoveryInitializer, MissingBeamsDoNotCountAsContradictingMeasurements){
  const auto m=reference();const Pose truth{.30,-.10,.20};auto s=scan(m,truth);
  // 留下约三分之一的实测点，仍超过几何验收要求的50点；其余缺测不应成为反证。
  for(size_t i=0;i<s.ranges.size();++i)if(i%3)s.ranges[i]=std::numeric_limits<float>::quiet_NaN();
  Pose result;ASSERT_NO_THROW(result=recovery_initial_pose(m,s,{0,0,0},{}));
  EXPECT_NEAR(result.x,truth.x,.08);EXPECT_NEAR(result.y,truth.y,.08);EXPECT_NEAR(result.yaw,truth.yaw,.05);
  std::fill(s.ranges.begin(),s.ranges.end(),std::numeric_limits<float>::quiet_NaN());s.ranges[0]=1;
  EXPECT_THROW(recovery_initial_pose(m,s,{},{}),std::runtime_error);
}
TEST(RecoveryInitializer, RecordedTurnRequiresCurrentScanPrior){
  // 录制地图与激光保持原样，仅比较搜索中心；不把轮式推算当作实物真值。
  std::ifstream input(std::string(RECOVERY_FIXTURE_DIR)+"/recovery_turn_20261101.json");
  ASSERT_TRUE(input.good());const auto j=nlohmann::json::parse(input);
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";
  m.info.width=j["map"]["width"];m.info.height=j["map"]["height"];m.info.resolution=j["map"]["resolution"];
  m.info.origin.position.x=j["map"]["origin"][0];m.info.origin.position.y=j["map"]["origin"][1];m.info.origin.orientation.w=1;
  m.data=j["map"]["data"].get<std::vector<int8_t>>();
  sensor_msgs::msg::LaserScan s;s.angle_min=j["scan"]["angle_min"];s.angle_increment=j["scan"]["angle_increment"];
  s.range_min=j["scan"]["range_min"];s.range_max=j["scan"]["range_max"];
  for(const auto &r:j["scan"]["ranges"])s.ranges.push_back(r.is_null()?std::numeric_limits<float>::quiet_NaN():r.get<float>());
  const auto pose=[&](const char*key){return Pose{j[key][0],j[key][1],j[key][2]};};
  const auto prior=pose("prior"),aligned=pose("aligned"),offset=pose("laser_offset");
  EXPECT_GT(std::abs(std::remainder(aligned.yaw-prior.yaw,2*M_PI)),.60);
  EXPECT_THROW(recovery_initial_pose(m,s,prior,offset),std::runtime_error);
  Pose candidate=aligned;
  try{candidate=recovery_initial_pose(m,s,aligned,offset);}catch(const std::exception&e){ADD_FAILURE()<<e.what();}
  std::cout<<"录包对齐搜索候选 "<<candidate.x<<","<<candidate.y<<","<<candidate.yaw<<std::endl;
  std::vector<ReferenceMatcher::Ray> rays;
  for(size_t i=0;i<s.ranges.size();i+=std::max<size_t>(1,s.ranges.size()/180)){
    const double r=s.ranges[i];if(!std::isfinite(r)||r<std::max(.15,double(s.range_min))||r>s.range_max-.05)continue;
    const double a=s.angle_min+i*s.angle_increment+offset.yaw;
    rays.push_back({offset.x+r*std::cos(a),offset.y+r*std::sin(a),offset.x,offset.y});
  }
  ReferenceMatcher matcher(m);const auto score=matcher.score(candidate,rays);
  std::cout<<"几何检查 "<<score.points<<","<<score.mean<<","<<score.coverage<<","<<score.hit_ratio<<","<<score.free_conflict<<std::endl;
  EXPECT_TRUE(score.acceptable());
  const auto verification=matcher.verify(candidate,rays);
  std::cout<<"唯一性 "<<verification.unique<<","<<verification.ambiguity_margin<<","<<verification.global_margin<<std::endl;
  EXPECT_TRUE(verification.unique);
}

TEST(RecoveryInitializer, RecordedCheckpointFailureKeepsOldReferenceRejection){
  // 同一实车扫描分别对旧可信图与失败前生产栅格评分；后者仅作诊断，不能自动提升为可信图。
  std::ifstream input(std::string(RECOVERY_FIXTURE_DIR)+"/checkpoint_failure_20261101.json");
  ASSERT_TRUE(input.good());const auto j=nlohmann::json::parse(input);
  sensor_msgs::msg::LaserScan s;s.angle_min=j["scan"]["angle_min"];s.angle_increment=j["scan"]["angle_increment"];
  s.range_min=j["scan"]["range_min"];s.range_max=j["scan"]["range_max"];
  for(const auto&r:j["scan"]["ranges"])s.ranges.push_back(r.is_null()?NAN:r.get<float>());
  const Pose prior{j["aligned"][0],j["aligned"][1],j["aligned"][2]},offset{j["laser_offset"][0],j["laser_offset"][1],j["laser_offset"][2]};
  for(const auto*key:{"map","current_map"}){
    const auto&v=j[key];nav_msgs::msg::OccupancyGrid m;m.info.width=v["width"];m.info.height=v["height"];m.info.resolution=v["resolution"];
    m.info.origin.position.x=v["origin"][0];m.info.origin.position.y=v["origin"][1];m.info.origin.orientation.w=1;m.data=v["data"].get<std::vector<int8_t>>();
    try{
      const auto candidate=recovery_initial_pose(m,s,prior,offset);
      std::cout<<key<<" 候选="<<candidate.x<<","<<candidate.y<<","<<candidate.yaw<<std::endl;
      EXPECT_NE(std::string(key),"map")<<"旧可信图仍必须拒绝本次真实低分输入";
      std::vector<ReferenceMatcher::Ray> rays;
      for(size_t i=0;i<s.ranges.size();i+=std::max<size_t>(1,s.ranges.size()/180)){
        const double r=s.ranges[i];if(!std::isfinite(r)||r<std::max(.15,double(s.range_min))||r>s.range_max-.05)continue;
        const double a=s.angle_min+i*s.angle_increment+offset.yaw;
        rays.push_back({offset.x+r*std::cos(a),offset.y+r*std::sin(a),offset.x,offset.y});
      }
      ReferenceMatcher matcher(m);const auto score=matcher.verify(candidate,rays);
      std::cout<<key<<" 独立几何="<<score.score.mean<<","<<score.score.coverage<<","<<score.score.hit_ratio<<","<<score.score.free_conflict<<" 唯一性="<<score.unique<<std::endl;
      EXPECT_TRUE(score.score.acceptable());EXPECT_TRUE(score.unique);
    }catch(const std::exception&e){
      std::cout<<key<<" 拒绝="<<e.what()<<std::endl;
      if(std::string(key)=="map"){EXPECT_NE(std::string(e.what()).find("valid=0.194141"),std::string::npos);}
      else {ADD_FAILURE()<<e.what();}
    }
  }
}

TEST(RecoveryInitializer, RecordedMapAtCorrelationGridBorderDoesNotCrash){
  // 原包末尾缺3.024秒；边界复制发生在读取扫描匹配之前，保留原地图、先验和最近可用扫描。
  std::ifstream input(std::string(RECOVERY_FIXTURE_DIR)+"/recovery_grid_border_20261101.json");
  ASSERT_TRUE(input.good());const auto j=nlohmann::json::parse(input);const auto&v=j["map"];
  nav_msgs::msg::OccupancyGrid m;m.info.width=v["width"];m.info.height=v["height"];m.info.resolution=v["resolution"];
  m.info.origin.position.x=v["origin"][0];m.info.origin.position.y=v["origin"][1];m.info.origin.orientation.w=1;m.data=v["data"].get<std::vector<int8_t>>();
  sensor_msgs::msg::LaserScan s;s.angle_min=j["scan"]["angle_min"];s.angle_increment=j["scan"]["angle_increment"];
  s.range_min=j["scan"]["range_min"];s.range_max=j["scan"]["range_max"];
  for(const auto&r:j["scan"]["ranges"])s.ranges.push_back(r.is_null()?NAN:r.get<float>());
  const Pose prior{j["aligned"][0],j["aligned"][1],j["aligned"][2]},offset{j["laser_offset"][0],j["laser_offset"][1],j["laser_offset"][2]};
  try{
    const auto result=recovery_initial_pose(m,s,prior,offset);
    EXPECT_LT(std::hypot(result.x-prior.x,result.y-prior.y),.15);
    EXPECT_LT(std::abs(std::remainder(result.yaw-prior.yaw,2*M_PI)),.10);
  }catch(const karto::Exception&e){ADD_FAILURE()<<"Karto非标准异常越过边界："<<e.GetErrorMessage();}
  catch(const std::exception&e){ADD_FAILURE()<<e.what();}
}

TEST(RecoveryInitializer, KartoFailureBecomesStandardExceptionWithoutStealingRegistration){
  const karto::Name name("recovery_reference_laser");
  std::unique_ptr<karto::LaserRangeFinder> existing(karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,name));
  karto::SensorManager::GetInstance()->RegisterSensor(existing.get());
  const auto m=reference();
  EXPECT_THROW(recovery_initial_pose(m,scan(m,{}),{},{}),std::runtime_error);
  EXPECT_EQ(karto::SensorManager::GetInstance()->GetSensorByName(name),existing.get());
  karto::SensorManager::GetInstance()->UnregisterSensor(existing.get());
  EXPECT_NO_THROW(recovery_initial_pose(m,scan(m,{}),{},{}));
}
