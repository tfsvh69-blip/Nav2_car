#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/loop_guard.hpp"
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
