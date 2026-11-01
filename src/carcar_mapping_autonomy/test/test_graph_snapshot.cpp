#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "carcar_mapping_autonomy/graph_snapshot.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
std::string read_file(const std::string &path){std::ifstream in(path,std::ios::binary);return {std::istreambuf_iterator<char>(in),{}};}
}
TEST(GraphSnapshot, OfficialArchivesRemainCompatibleAndImmutableAfterFurtherMapping){
  karto::Mapper mapper;karto::Dataset dataset;
  auto *laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("snapshot-test"));
  laser->SetMinimumRange(.05);laser->SetMaximumRange(12.);laser->SetRangeThreshold(3.);
  laser->SetMinimumAngle(-M_PI);laser->SetMaximumAngle(M_PI);laser->SetAngularResolution(M_PI/180.);
  dataset.Add(laser,true);
  auto append=[&](double x){auto *scan=new karto::LocalizedRangeScan(laser->GetName(),std::vector<double>(361,2.));
    scan->SetOdometricPose(karto::Pose2(x,0,0));scan->SetCorrectedPose(karto::Pose2(x,0,0));
    if(mapper.Process(scan)){dataset.Add(scan);return true;}delete scan;return false;};
  ASSERT_TRUE(append(0));
  auto prepared=ca::prepare_graph_snapshot();
  auto snapshot=ca::capture_graph(mapper,dataset,std::move(prepared));
  ASSERT_TRUE(snapshot.graph->sealed());ASSERT_TRUE(snapshot.data->sealed());
  const auto graph=snapshot.graph->read(),data=snapshot.data->read();
  char directory[]="/tmp/carcar-snapshot-test-XXXXXX";ASSERT_NE(mkdtemp(directory),nullptr);
  const std::string prefix=std::string(directory)+"/official";
  mapper.SaveToFile(prefix+".posegraph");dataset.SaveToFile(prefix+".data");
  EXPECT_EQ(graph,read_file(prefix+".posegraph"));EXPECT_EQ(data,read_file(prefix+".data"));
  ASSERT_TRUE(append(.4));EXPECT_EQ(snapshot.graph->read(),graph);EXPECT_EQ(snapshot.data->read(),data);
  EXPECT_EQ(snapshot.graph->buffer(),nullptr);
  // 只清理由本测试创建的独立临时目录，原工程从未写入。
  std::filesystem::remove_all(directory);
}
TEST(GraphSnapshot, EmptyArchiveCannotBeSealed){
  ca::SnapshotMemoryBuffer file;EXPECT_THROW(file.seal(),std::runtime_error);
}
TEST(GraphSnapshot, UnpreparedAndOverflowingBuffersAreRejected){
  ca::SnapshotMemoryBuffer file;EXPECT_THROW(ca::SnapshotOutputBuffer{file},std::runtime_error);
  file.prepare(8);ca::SnapshotOutputBuffer buffer(file);
  EXPECT_EQ(buffer.sputn("12345678",8),8);EXPECT_EQ(buffer.sputn("x",1),0);
  EXPECT_EQ(buffer.size(),8U);file.finish(buffer.size());EXPECT_EQ(file.read(),"12345678");
  EXPECT_THROW(file.prepare(16),std::runtime_error);
}
TEST(GraphSnapshotDeathTest, ReadOnlyCaptureRejectsWritesInKernel){
  ca::SnapshotMemoryBuffer file;file.prepare(4096);file.buffer()[0]='a';file.finish(1);
  ASSERT_DEATH({*const_cast<volatile char*>(file.data())='b';},"");
  EXPECT_EQ(file.read(),"a");
}
