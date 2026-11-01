#include <gtest/gtest.h>
#include <thread>
#include <fstream>
#include "carcar_mapping_autonomy/snapshot_process.hpp"
#include "carcar_mapping_autonomy/recovery_artifact.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
std::pair<ca::GraphSnapshot,ca::SnapshotJson> sample(const std::string&id){
  karto::Mapper mapper;karto::Dataset dataset;mapper.setParamMinimumTravelDistance(.05);mapper.setParamDoLoopClosing(false);
  auto*laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("snapshot-worker-test"));
  laser->SetMinimumRange(.05);laser->SetMaximumRange(12);laser->SetRangeThreshold(8);
  laser->SetMinimumAngle(-M_PI);laser->SetMaximumAngle(M_PI);laser->SetAngularResolution(M_PI/180);dataset.Add(laser,true);
  karto::LocalizedRangeScan*last=nullptr;
  for(int i=0;i<12;++i){const double x=.12*i;std::vector<double>ranges;
    for(int k=0;k<=360;++k){const double angle=-M_PI+k*M_PI/180,dx=std::cos(angle),dy=std::sin(angle);double r=12;
      if(dx>1e-8)r=std::min(r,(4-x)/dx);else if(dx< -1e-8)r=std::min(r,(-2-x)/dx);
      if(dy>1e-8)r=std::min(r,2/dy);else if(dy< -1e-8)r=std::min(r,-2/dy);ranges.push_back(r);}
    auto*scan=new karto::LocalizedRangeScan(laser->GetName(),ranges);scan->SetOdometricPose({x,0,0});scan->SetCorrectedPose({x,0,0});
    if(mapper.Process(scan)){dataset.Add(scan);last=scan;}else delete scan;
  }
  if(!last)throw std::runtime_error("软件矩形场景没有接纳扫描");
  const auto p=last->GetCorrectedPose();
  ca::SnapshotJson metadata{{"task_id","snapshot-test"},{"request_id",id},{"generation",1},{"navigation_generation",1},{"origin",{0,0,0}},
    {"capture_stamp_ns",int64_t(1000000000)},{"capture_steady_seconds",1.0},{"scan_stamp_ns",int64_t(1000000000)},
    {"sequence",12},{"scan_id",last->GetUniqueId()},{"scan_pose",{p.GetX(),p.GetY(),p.GetHeading()}},
    {"resolution",.05},{"min_pass_through",mapper.getParamMinPassThrough()},{"occupancy_threshold",mapper.getParamOccupancyThreshold()},{"mapping_range_m",8.0}};
  auto prepared=ca::prepare_graph_snapshot();auto captured=ca::capture_graph(mapper,dataset,std::move(prepared));
  metadata["capture_ms"]=captured.capture_ms;return {std::move(captured),std::move(metadata)};
}
bool wait(ca::SnapshotProcess&process,double timeout=5){const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
  while(process.active()&&std::chrono::steady_clock::now()<end){process.poll();std::this_thread::sleep_for(std::chrono::milliseconds(2));}return !process.active();}
class SnapshotProcessTest:public ::testing::Test {
protected:
  std::filesystem::path root;
  void SetUp()override{char path[]="/tmp/carcar-online-snapshot-XXXXXX";ASSERT_NE(mkdtemp(path),nullptr);root=path;}
  void TearDown()override{std::filesystem::remove_all(root);}
};
}
TEST_F(SnapshotProcessTest, IndependentWorkerPublishesCompletePairAndRefusesOverwrite){
  ca::SnapshotProcess process;auto input=sample("first");
  process.start(SNAPSHOT_WORKER,root,input.second,ca::snapshot_process_inputs(std::move(input.first)));
  ASSERT_TRUE(wait(process));ASSERT_EQ(process.state().value("phase",""),"READY")<<process.state();
  const auto path=std::filesystem::path(process.state().at("path").get<std::string>());
  const auto manifest=ca::recovery_validate_artifact(path);EXPECT_TRUE(manifest.at("complete"));EXPECT_EQ(manifest.at("request_id"),"first");
  const auto hash=ca::recovery_sha256(path/"graph.posegraph");
  auto duplicate=sample("first");process.start(SNAPSHOT_WORKER,root,duplicate.second,ca::snapshot_process_inputs(std::move(duplicate.first)));
  ASSERT_TRUE(wait(process));EXPECT_EQ(process.state().value("phase",""),"FAILED");EXPECT_EQ(ca::recovery_sha256(path/"graph.posegraph"),hash);
}
TEST_F(SnapshotProcessTest, BadDestinationAndMismatchedMetadataDoNotPublish){
  ca::SnapshotProcess process;const auto blocked=root/"not-a-directory";std::ofstream(blocked)<<"preserved";
  auto input=sample("bad-disk");process.start(SNAPSHOT_WORKER,blocked,input.second,ca::snapshot_process_inputs(std::move(input.first)));
  ASSERT_TRUE(wait(process));EXPECT_EQ(process.state().value("phase",""),"FAILED");
  input=sample("bad-pose");input.second["scan_pose"]={20,0,0};process.start(SNAPSHOT_WORKER,root,input.second,ca::snapshot_process_inputs(std::move(input.first)));
  ASSERT_TRUE(wait(process));EXPECT_EQ(process.state().value("phase",""),"FAILED");EXPECT_FALSE(std::filesystem::exists(root/"snapshot-test/bad-pose"));
}
TEST_F(SnapshotProcessTest, TimeoutKillsOnlyWorkerAndIsReportedWithoutBlockingPoll){
  ca::SnapshotProcess process;auto input=sample("slow-worker");
  process.start(STALLED_SNAPSHOT_WORKER,root,input.second,ca::snapshot_process_inputs(std::move(input.first)),.15);
  double maximum=0;const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(process.active()&&std::chrono::steady_clock::now()<end){const auto begin=std::chrono::steady_clock::now();process.poll();
    maximum=std::max(maximum,std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count());std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  EXPECT_FALSE(process.active());EXPECT_EQ(process.state().value("phase",""),"FAILED");EXPECT_LT(maximum,.05);
}
TEST_F(SnapshotProcessTest, WorkerExitWithoutFinalMessageIsNeverPromoted){
  ca::SnapshotProcess process;auto input=sample("worker-exit");
  process.start("/bin/false",root,input.second,ca::snapshot_process_inputs(std::move(input.first)));
  ASSERT_TRUE(wait(process));EXPECT_EQ(process.state().value("phase",""),"FAILED");
  EXPECT_FALSE(std::filesystem::exists(root/"snapshot-test/worker-exit"));
}
TEST_F(SnapshotProcessTest, BusyParentDescriptorTableCannotOverwriteArchiveInputs){
 auto input=sample("busy-descriptors");
 struct Descriptors{std::vector<int>fds;~Descriptors(){for(int fd:fds)close(fd);}} held;
 for(;;){const int fd=open("/dev/null",O_RDONLY|O_CLOEXEC);ASSERT_GE(fd,0);held.fds.push_back(fd);if(fd>=196)break;}
 auto archives=ca::snapshot_process_inputs(std::move(input.first));
 ASSERT_EQ(archives.graph,197);ASSERT_EQ(archives.data,198);
 ca::SnapshotProcess process;process.start(SNAPSHOT_WORKER,root,input.second,std::move(archives));
 ASSERT_TRUE(wait(process));EXPECT_EQ(process.state().value("phase",""),"READY")<<process.state();
 EXPECT_TRUE(ca::recovery_validate_artifact(root/"snapshot-test/busy-descriptors").at("complete"));
}
TEST(SnapshotProtocol, RejectsUnsafeIdentifiersAndSupersededGeneration){
  ca::SnapshotJson request{{"task_id","task"},{"request_id","request"},{"generation",1},{"navigation_generation",2},{"origin",{0,0,0}}};
  EXPECT_NO_THROW(ca::validate_snapshot_request(request));auto other=request;other["navigation_generation"]=3;
  EXPECT_FALSE(ca::snapshot_same_generation(request,other));other=request;other["task_id"]="../old-map";
  EXPECT_THROW(ca::validate_snapshot_request(other),std::runtime_error);
}
