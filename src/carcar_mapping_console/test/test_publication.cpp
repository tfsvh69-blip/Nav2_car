#include <gtest/gtest.h>
#include "carcar_mapping_console/publication.hpp"
using namespace carcar_mapping_console;
namespace fs=std::filesystem;
class Publication:public testing::Test{
protected:fs::path root,dir;
void SetUp()override{root=fs::temp_directory_path()/("carcar-publication-"+nonce());dir=root/"bundle";fs::create_directories(dir);json manifest{{"complete",true},{"slam_version","2.6.10"},{"base_frame","base_footprint"},{"review_required",true}};
for(const auto&name:{"map.yaml","map.pgm","graph.posegraph","graph.data"}){std::ofstream(dir/name)<<"fixture";manifest["sha256"][name]=sha256(dir/name);}atomic_json(dir/"manifest.json",manifest);}
void TearDown()override{fs::remove_all(root);}
};
TEST(PublicationPolicy, AutomaticAndDiagnosticSaveStayFrozen){
EXPECT_FALSE(resume_scan_forwarding(false,true,true));
EXPECT_FALSE(resume_scan_forwarding(false,false,false));
EXPECT_FALSE(resume_scan_forwarding(true,false,true));
EXPECT_TRUE(resume_scan_forwarding(false,false,true));}
TEST_F(Publication, CompleteSaveWithoutReviewSidecarIsStillHidden){EXPECT_FALSE(navigation_map_visible(dir/"map.yaml"));}
TEST_F(Publication, ApprovedAssetBecomesVisibleButTamperingHidesIt){auto state=review_bundle(root,"bundle",true,"现场墙线核对通过");
EXPECT_EQ(state["status"],"published");
EXPECT_TRUE(navigation_map_visible(dir/"map.yaml"));std::ofstream(dir/"map.pgm")<<"changed";
EXPECT_FALSE(navigation_map_visible(dir/"map.yaml"));}
TEST_F(Publication, RejectionPreservesAllAssets){const auto hash=sha256(dir/"graph.data");review_bundle(root,"bundle",false,"走廊错位，重建");
EXPECT_FALSE(navigation_map_visible(dir/"map.yaml"));
EXPECT_EQ(sha256(dir/"graph.data"),hash);}
TEST_F(Publication, DiagnosticCannotBePublished){auto j=json::parse(read_file(dir/"manifest.json"));j["diagnostic_only"]=true;atomic_json(dir/"manifest.json",j);
EXPECT_THROW(review_bundle(root,"bundle",true,"确认"),std::runtime_error);}
TEST_F(Publication, LegacyRemainsReadableWithoutBeingAutomaticallyActivated){auto j=json::parse(read_file(dir/"manifest.json"));j.erase("review_required");atomic_json(dir/"manifest.json",j);
EXPECT_TRUE(navigation_map_visible(dir/"map.yaml"));
EXPECT_EQ(bundle_publication(dir)["status"],"legacy");}
TEST_F(Publication, TraversalAndEmptyReviewAreRejected){EXPECT_THROW(review_bundle(root,"../bundle",true,"确认"),std::runtime_error);
EXPECT_THROW(review_bundle(root,"bundle",true,""),std::runtime_error);}
TEST_F(Publication, ResumeRejectsDifferentRangeLossAndDiagnostic){auto j=json::parse(read_file(dir/"manifest.json"));j["range_m"]=3.;j["resolution"]=.05;atomic_json(dir/"manifest.json",j);
EXPECT_EQ(validate_autonomy_bundle(root,"bundle"),dir);
j["runtime_parameters"]={{"max_laser_range","6.0"}};atomic_json(dir/"manifest.json",j);
EXPECT_THROW(validate_autonomy_bundle(root,"bundle"),std::runtime_error);
j["runtime_parameters"]={{"max_laser_range","3.0"},{"ceres_loss_function","CauchyLoss"}};atomic_json(dir/"manifest.json",j);
EXPECT_THROW(validate_autonomy_bundle(root,"bundle"),std::runtime_error);
j.erase("runtime_parameters");j["diagnostic_only"]=true;atomic_json(dir/"manifest.json",j);
EXPECT_THROW(validate_autonomy_bundle(root,"bundle"),std::runtime_error);}
