#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include "carcar_mapping_autonomy/loop_guard.hpp"
#include "carcar_mapping_autonomy/frozen_scan_validation.hpp"
namespace ca=carcar_mapping_autonomy;
class GuardSolver:public karto::ScanSolver{
public:
  void Compute()override{}
  void Configure(rclcpp::Node::SharedPtr)override{}
  const IdPoseVector&GetCorrections()const override{return corrections;}
  void AddNode(karto::Vertex<karto::LocalizedRangeScan>*)override{++nodes;}
  void AddConstraint(karto::Edge<karto::LocalizedRangeScan>*)override{++edges;}
  void Reset()override{++resets;nodes=edges=0;}
  IdPoseVector corrections;size_t nodes=0,edges=0,resets=0;
};
class LoopGuardTest:public ::testing::Test{
protected:
  karto::Mapper mapper;karto::Dataset dataset;GuardSolver solver;
  void SetUp()override{
    auto*laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("loop-guard-test"));
    laser->SetMinimumRange(.05);laser->SetMaximumRange(12);laser->SetRangeThreshold(3);
    laser->SetMinimumAngle(-M_PI);laser->SetMaximumAngle(M_PI);laser->SetAngularResolution(M_PI/180);dataset.Add(laser,true);
    mapper.setParamMinimumTravelDistance(0);mapper.setParamMinimumTimeInterval(0);mapper.setParamDoLoopClosing(false);
    for(int k=0;k<8;++k){auto*s=new karto::LocalizedRangeScan(laser->GetName(),std::vector<double>(361,2));
      s->SetOdometricPose(karto::Pose2(0,0,0));s->SetCorrectedPose(karto::Pose2(0,0,0));s->SetTime(k+1);
      ASSERT_TRUE(mapper.Process(s));dataset.Add(s);}
  }
};
TEST_F(LoopGuardTest, ConsistentRigidCorrectionIsRetained){
  ca::LoopGuard guard;guard.begin(mapper);
  for(auto*s:mapper.GetAllProcessedScans())s->SetCorrectedPoseAndUpdate(karto::Pose2(.1,.1,0));
  EXPECT_TRUE(guard.finish(mapper,solver));EXPECT_EQ(solver.resets,0U);
  EXPECT_FALSE(guard.state()["rolled_back"]);EXPECT_NEAR(mapper.GetAllProcessedScans().back()->GetCorrectedPose().GetX(),.1,1e-9);
}
TEST_F(LoopGuardTest, InconsistentCorrectionRestoresPosesEdgesAndSolver){
  const auto scans=mapper.GetAllProcessedScans();const auto before=mapper.GetGraph()->GetEdges().size();
  const auto vertices=mapper.GetGraph()->GetVertices().begin()->second;
  auto*first=vertices.begin()->second;auto*last=vertices.rbegin()->second;
  const auto first_edges=first->GetEdges().size(),last_edges=last->GetEdges().size();
  ca::LoopGuard guard;guard.begin(mapper);
  auto*edge=new karto::Edge<karto::LocalizedRangeScan>(first,last);
  static_cast<karto::Graph<karto::LocalizedRangeScan>*>(mapper.GetGraph())->AddEdge(edge);
  scans.back()->SetCorrectedPoseAndUpdate(karto::Pose2(1,0,0));
  ASSERT_FALSE(guard.finish(mapper,solver));
  EXPECT_EQ(mapper.GetGraph()->GetEdges().size(),before);EXPECT_EQ(first->GetEdges().size(),first_edges);EXPECT_EQ(last->GetEdges().size(),last_edges);
  EXPECT_EQ(solver.resets,1U);EXPECT_EQ(solver.nodes,scans.size());EXPECT_EQ(solver.edges,before);
  EXPECT_TRUE(ca::LoopGuard::score(mapper,scans.back()).acceptable());
  for(auto*s:scans)EXPECT_NEAR(s->GetCorrectedPose().GetX(),0,1e-6);
  // 同一扫描的后续回环不得绕过拒绝；新的正常扫描不继承该限制。
  guard.begin(mapper);EXPECT_FALSE(guard.finish(mapper,solver));
  auto*next=new karto::LocalizedRangeScan(scans.back()->GetSensorName(),std::vector<double>(361,2));
  next->SetOdometricPose(karto::Pose2(0,0,0));next->SetCorrectedPose(karto::Pose2(0,0,0));next->SetTime(9);
  ASSERT_TRUE(mapper.Process(next));dataset.Add(next);
  guard.begin(mapper);EXPECT_TRUE(guard.finish(mapper,solver));EXPECT_EQ(guard.state()["rejections"],2);
}

TEST_F(LoopGuardTest, RejectedConstraintIsQuarantinedAcrossGraphRestoreWithoutCountingANewFailure){
  const auto vertices=mapper.GetGraph()->GetVertices().begin()->second;
  auto*first=vertices.begin()->second;auto*last=vertices.rbegin()->second;
  auto add=[&]{static_cast<karto::Graph<karto::LocalizedRangeScan>*>(mapper.GetGraph())->AddEdge(
    new karto::Edge<karto::LocalizedRangeScan>(first,last));};
  ca::LoopGuard guard;guard.begin(mapper);add();
  mapper.GetAllProcessedScans().back()->SetCorrectedPoseAndUpdate(karto::Pose2(1,0,0));
  ASSERT_FALSE(guard.finish(mapper,solver));const auto original=mapper.GetGraph()->GetEdges().size();
  // 恢复同一任务的检查点不能遗忘刚拒绝的约束；即使这次评分偶然好看也不能直接重纳。
  guard.reset_graph();guard.begin(mapper);add();
  EXPECT_FALSE(guard.finish(mapper,solver));
  EXPECT_EQ(mapper.GetGraph()->GetEdges().size(),original);
  EXPECT_EQ(guard.state().value("rejections",0),1);
  EXPECT_EQ(guard.state().value("quarantine_hits",0),1);
  EXPECT_TRUE(guard.state().value("quarantined",false));
}

TEST_F(LoopGuardTest, FrozenValidationRestoresNormalLoopClosingEvenWhenMatchingThrows){
  mapper.setParamDoLoopClosing(true);
  try{ca::FrozenScanValidation transaction(&mapper,true);EXPECT_FALSE(mapper.getParamDoLoopClosing());throw std::runtime_error("匹配失败");}
  catch(const std::runtime_error&){}
  EXPECT_TRUE(mapper.getParamDoLoopClosing());
  {ca::FrozenScanValidation normal(&mapper,false);EXPECT_TRUE(mapper.getParamDoLoopClosing());}
  mapper.setParamDoLoopClosing(false);
  {ca::FrozenScanValidation transaction(&mapper,true);}
  EXPECT_FALSE(mapper.getParamDoLoopClosing());
}


class SamplingLoopGuardTest:public ::testing::Test{
protected:
  karto::Mapper mapper;karto::Dataset dataset;GuardSolver solver;
  void populate(const std::vector<double>&ranges,double minimum_angle=-M_PI,
    double angular_resolution=M_PI/180,karto::Pose2 offset=karto::Pose2()){
    auto*laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("loop-sampling-test"));
    laser->SetMinimumRange(.05);laser->SetMaximumRange(12);laser->SetRangeThreshold(3);
    laser->SetMinimumAngle(minimum_angle);laser->SetMaximumAngle(minimum_angle+(ranges.size()-1)*angular_resolution);
    laser->SetAngularResolution(angular_resolution);laser->SetOffsetPose(offset);dataset.Add(laser,true);
    mapper.setParamMinimumTravelDistance(0);mapper.setParamMinimumTimeInterval(0);mapper.setParamDoLoopClosing(false);
    // 重复扫描仅建立可判读的几何夹具，不能代替现场回环正确性验证。
    for(int k=0;k<8;++k){auto*scan=new karto::LocalizedRangeScan(laser->GetName(),ranges);
      scan->SetOdometricPose(karto::Pose2());scan->SetCorrectedPose(karto::Pose2());scan->SetTime(k+1);
      ASSERT_TRUE(mapper.Process(scan));dataset.Add(scan);}
  }
  karto::LocalizedRangeScan*current(){return mapper.GetAllProcessedScans().back();}
};

TEST_F(SamplingLoopGuardTest, RecordedSparseScanKeepsDistinctValidBeams){
  const auto file=std::filesystem::path(__FILE__).parent_path()/"data/loop_guard_scan448_20261104.json";
  std::ifstream input(file);ASSERT_TRUE(input.is_open());const auto fixture=nlohmann::json::parse(input);
  std::vector<double>ranges;for(const auto&value:fixture["ranges"])ranges.push_back(value.is_null()?NAN:value.get<double>());
  populate(ranges,fixture["angle_min"],fixture["angle_increment"]);
  const auto score=ca::LoopGuard::score(mapper,current());
  EXPECT_EQ(score.points,fixture["expected_valid_points"].get<size_t>());EXPECT_TRUE(score.acceptable());
  ca::LoopGuard guard;guard.begin(mapper);EXPECT_TRUE(guard.finish(mapper,solver));EXPECT_EQ(solver.resets,0U);
}

TEST_F(SamplingLoopGuardTest, FortyNineDistinctPointsStillReject){
  std::vector<double>ranges(361,4.0);for(size_t i=0;i<49;++i)ranges[1+6*i]=2;
  populate(ranges);const auto score=ca::LoopGuard::score(mapper,current());
  EXPECT_EQ(score.points,49U);EXPECT_FALSE(score.acceptable());
  ca::LoopGuard guard;guard.begin(mapper);EXPECT_FALSE(guard.finish(mapper,solver));EXPECT_EQ(solver.resets,1U);
}

TEST_F(SamplingLoopGuardTest, FiftyDistinctPointsPassGeometryWithoutChangingThreshold){
  std::vector<double>ranges(361,4.0);for(size_t i=0;i<50;++i)ranges[1+6*i]=2;
  populate(ranges);const auto score=ca::LoopGuard::score(mapper,current());
  EXPECT_EQ(score.points,50U);EXPECT_TRUE(score.acceptable());
  ca::LoopGuard guard;guard.begin(mapper);EXPECT_TRUE(guard.finish(mapper,solver));EXPECT_EQ(solver.resets,0U);
}

TEST_F(SamplingLoopGuardTest, DenseWideScanRemainsBounded){
  std::vector<double>ranges(720,2.0);populate(ranges,-M_PI,2*M_PI/719);
  const auto score=ca::LoopGuard::score(mapper,current());EXPECT_EQ(score.points,180U);EXPECT_TRUE(score.acceptable());
}

TEST_F(SamplingLoopGuardTest, InvalidReturnsPreserveOriginalAnglesAndSensorOffset){
  const double angle_min=-M_PI,increment=2*M_PI/719;
  std::vector<double>ranges(720,4.0);
  for(size_t i=0;i<ranges.size();++i){switch(i%4){
    case 0:ranges[i]=std::numeric_limits<double>::infinity();break;
    case 1:ranges[i]=std::numeric_limits<double>::quiet_NaN();break;
    case 2:ranges[i]=.10;break;
    default:ranges[i]=-.5;break;}}
  for(size_t k=0;k<60;++k){const auto i=37+11*k;ranges[i]=1.5+.35*std::sin(5*(angle_min+i*increment));}
  populate(ranges,angle_min,increment,karto::Pose2(.11,-.07,.24));
  const auto score=ca::LoopGuard::score(mapper,current());EXPECT_EQ(score.points,60U);EXPECT_TRUE(score.acceptable());
}

TEST_F(SamplingLoopGuardTest, NoValidReturnsStillRollback){
  populate(std::vector<double>(361,2.0));auto*scan=current();
  scan->SetRangeReadings(std::vector<double>(361,std::numeric_limits<double>::infinity()));
  scan->SetCorrectedPoseAndUpdate(scan->GetCorrectedPose());
  const auto score=ca::LoopGuard::score(mapper,scan);EXPECT_EQ(score.points,0U);EXPECT_FALSE(score.acceptable());
  ca::LoopGuard guard;guard.begin(mapper);EXPECT_FALSE(guard.finish(mapper,solver));EXPECT_EQ(solver.resets,1U);
}
