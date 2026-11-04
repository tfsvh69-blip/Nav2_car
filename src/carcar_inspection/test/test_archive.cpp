#include <gtest/gtest.h>
#include "carcar_inspection/archive.hpp"
using namespace carcar_inspection;
namespace {
struct Temporary {
  std::filesystem::path root;
  Temporary(){char name[]="/tmp/carcar-archive-XXXXXX";root=mkdtemp(name);std::filesystem::create_directory(root/"media");}
  ~Temporary(){std::filesystem::remove_all(root);}
};
json record(const std::filesystem::path&root){
  std::ofstream(root/"media/photo1.jpg",std::ios::binary)<<"原始测试图像";
  return {{"id","run1"},{"status","SUCCEEDED"},{"snapshot",{{"points",json::array({{{"id","p1"}}})}}},
    {"captures",{{"p1",{{"id","photo1"},{"sha256",carcar_mapping_console::sha256(root/"media/photo1.jpg")}}}}}};
}
}
TEST(Archive, PreservesOriginalAndIncludesManifest){
  Temporary t;auto run=record(t.root);const auto original=carcar_mapping_console::sha256(t.root/"media/photo1.jpg");
  const auto bytes=run_archive(t.root,run);ASSERT_GT(bytes.size(),4U);EXPECT_EQ(bytes[0],'P');EXPECT_EQ(bytes[1],'K');
  zip_error_t error;zip_error_init(&error);auto *source=zip_source_buffer_create(bytes.data(),bytes.size(),0,&error);
  ASSERT_NE(source,nullptr);auto *archive=zip_open_from_source(source,ZIP_RDONLY,&error);ASSERT_NE(archive,nullptr);
  EXPECT_EQ(zip_get_num_entries(archive,0),2);EXPECT_GE(zip_name_locate(archive,"manifest.json",0),0);EXPECT_GE(zip_name_locate(archive,"photos/photo1.jpg",0),0);
  auto *manifest_file=zip_fopen(archive,"manifest.json",0);ASSERT_NE(manifest_file,nullptr);
  char buffer[8192];auto count=zip_fread(manifest_file,buffer,sizeof(buffer));ASSERT_GT(count,0);
  EXPECT_EQ(json::parse(std::string(buffer,count)),run);zip_fclose(manifest_file);
  auto *photo_file=zip_fopen(archive,"photos/photo1.jpg",0);ASSERT_NE(photo_file,nullptr);
  count=zip_fread(photo_file,buffer,sizeof(buffer));EXPECT_EQ(std::string(buffer,count),"原始测试图像");zip_fclose(photo_file);
  zip_close(archive);zip_error_fini(&error);
  EXPECT_EQ(carcar_mapping_console::sha256(t.root/"media/photo1.jpg"),original);
  for(const auto &entry:std::filesystem::directory_iterator(t.root))EXPECT_EQ(entry.path().filename(),"media");
}
TEST(Archive, RejectsRunningMissingAndCorruptedPhotos){
  Temporary t;auto run=record(t.root);run["status"]="RUNNING";EXPECT_THROW(run_archive(t.root,run),std::runtime_error);
  run["status"]="FAILED";std::ofstream(t.root/"media/photo1.jpg")<<"已损坏";EXPECT_THROW(run_archive(t.root,run),std::runtime_error);
  std::filesystem::remove(t.root/"media/photo1.jpg");EXPECT_THROW(run_archive(t.root,run),std::runtime_error);
}
TEST(Archive, FailedRunWithNoPhotosStillExportsItsResult){
  Temporary t;json run{{"id","failed"},{"status","FAILED"},{"captures",json::object()},{"error","导航失败"}};
  EXPECT_FALSE(run_archive(t.root,run).empty());
}
