#include <gtest/gtest.h>
#include "carcar_mapping_console/publication.hpp"
#include "carcar_mapping_console/bundle_catalog.hpp"
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
TEST_F(Publication, CompleteSaveIsVisibleAndUsableWithoutReview){
  EXPECT_TRUE(saved_map_visible(dir/"map.yaml"));EXPECT_TRUE(saved_map_navigation_allowed(dir/"map.yaml"));
  for(const auto & status:{"candidate","saved","published","legacy"}){
    atomic_json(dir/"publication.json",{{"status",status},{"asset_sha256",json::parse(read_file(dir/"manifest.json")).at("sha256")}});
    const auto state=saved_map_status(dir/"map.yaml");EXPECT_TRUE(state["saved"]);EXPECT_TRUE(state["navigation_allowed"]);
    EXPECT_EQ(state["quality"],"normal");
  }
}
TEST_F(Publication, DiagnosticAndRejectedAssetsRemainVisibleWithoutNavigation){
  for(const auto & status:{"diagnostic","rejected"}){
    atomic_json(dir/"publication.json",{{"status",status},{"asset_sha256",json::parse(read_file(dir/"manifest.json")).at("sha256")}});
    EXPECT_TRUE(saved_map_visible(dir/"map.yaml"));EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));
    EXPECT_EQ(saved_map_status(dir/"map.yaml")["quality"],status);
  }
  auto manifest=json::parse(read_file(dir/"manifest.json"));manifest["diagnostic_only"]=true;atomic_json(dir/"manifest.json",manifest);
  atomic_json(dir/"publication.json",{{"status","published"},{"asset_sha256",manifest.at("sha256")}});
  EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));EXPECT_EQ(saved_map_status(dir/"map.yaml")["quality"],"diagnostic");
}
TEST_F(Publication, BrokenStatusIsVisibleWithAnExplicitErrorAndCannotNavigate){
  std::ofstream(dir/"publication.json")<<"not json";
  const auto state=saved_map_status(dir/"map.yaml");EXPECT_TRUE(state["saved"]);EXPECT_FALSE(state["navigation_allowed"]);
  EXPECT_EQ(state["quality"],"invalid");EXPECT_FALSE(state["quality_reason"].get<std::string>().empty());
  atomic_json(dir/"publication.json",{{"status","saved"},{"asset_sha256",json::object()}});
  EXPECT_TRUE(saved_map_visible(dir/"map.yaml"));EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));
  atomic_json(dir/"publication.json",{{"status","saved"}});
  EXPECT_TRUE(saved_map_visible(dir/"map.yaml"));EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));
}
TEST_F(Publication, InvalidOrTamperedAssetsCannotBeListedAsSaved){
  std::ofstream(dir/"map.pgm")<<"changed";
  EXPECT_FALSE(saved_map_visible(dir/"map.yaml"));EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));
}
TEST_F(Publication, LegacyYamlWithoutManifestStaysReadable){
  fs::remove(dir/"manifest.json");EXPECT_TRUE(saved_map_visible(dir/"map.yaml"));EXPECT_TRUE(saved_map_navigation_allowed(dir/"map.yaml"));
}
TEST_F(Publication, SessionWithoutManifestCannotBecomeLegacyEvenWithoutPublication){
  const auto sessions=root/"mapping_sessions";fs::create_directories(sessions);
  fs::rename(dir,sessions/"bundle");dir=sessions/"bundle";fs::remove(dir/"manifest.json");
  EXPECT_FALSE(saved_map_visible(dir/"map.yaml"));EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));
  for(const auto & status:{"legacy","candidate","saved","published","diagnostic","rejected"}){
    atomic_json(dir/"publication.json",{{"status",status},{"asset_sha256",json::object()}});
    EXPECT_FALSE(saved_map_visible(dir/"map.yaml"))<<status;EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"))<<status;
  }
}
TEST_F(Publication, NonLegacySidecarCannotReplaceAMissingManifest){
  fs::remove(dir/"manifest.json");atomic_json(dir/"publication.json",{{"status","saved"},{"asset_sha256",json::object()}});
  EXPECT_FALSE(saved_map_visible(dir/"map.yaml"));EXPECT_FALSE(saved_map_navigation_allowed(dir/"map.yaml"));
}
TEST_F(Publication, BackgroundCatalogCachesUnchangedAssetsAndDetectsEveryProtectedFileChange){
  SavedBundleCatalog catalog(root,std::chrono::milliseconds(20));
  auto wait_for=[&](auto predicate){const auto until=steady()+3;while(steady()<until){if(predicate(catalog.snapshot()))return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return false;};
  ASSERT_TRUE(wait_for([](const json&value){return !value.value("loading",true)&&value["bundles"].size()==1;}));
  auto value=catalog.snapshot();ASSERT_TRUE(value["bundles"][0]["saved"]);const auto initial=value["validations"];
  for(int i=0;i<100;++i)EXPECT_EQ(catalog.snapshot()["validations"],initial);
  std::this_thread::sleep_for(std::chrono::milliseconds(70));EXPECT_EQ(catalog.snapshot()["validations"],initial);
  for(const auto & name:{"map.yaml","map.pgm","graph.posegraph","graph.data","manifest.json","publication.json"}){
    const auto previous=catalog.snapshot().at("validations").get<uint64_t>();
    const auto file=dir/name;const bool existed=fs::exists(file);const auto original=existed?read_file(file):std::string();
    std::ofstream(file,std::ios::app)<<"changed";
    ASSERT_TRUE(wait_for([&](const json&next){return next.at("validations").get<uint64_t>()>previous;}))<<name;
    const auto changed=catalog.snapshot().at("validations").get<uint64_t>();
    if(existed)std::ofstream(file,std::ios::trunc)<<original;else fs::remove(file);
    ASSERT_TRUE(wait_for([&](const json&next){return next.at("validations").get<uint64_t>()>changed&&next["bundles"][0].value("saved",false);} ))<<name;
  }
}
TEST_F(Publication, BackgroundCatalogReportsBadStatusAndMissingManifestWithoutLosingOtherMaps){
  SavedBundleCatalog catalog(root,std::chrono::milliseconds(20));
  auto wait_for=[&](auto predicate){const auto until=steady()+3;while(steady()<until){if(predicate(catalog.snapshot()))return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return false;};
  ASSERT_TRUE(wait_for([](const json&value){return !value.value("loading",true)&&value["bundles"].size()==1;}));
  std::ofstream(dir/"publication.json")<<"invalid";
  ASSERT_TRUE(wait_for([](const json&value){return value["bundles"][0].value("quality","")=="invalid";}));
  EXPECT_TRUE(catalog.snapshot()["bundles"][0]["saved"]);EXPECT_FALSE(catalog.snapshot()["bundles"][0]["navigation_allowed"]);
  fs::remove(dir/"manifest.json");
  ASSERT_TRUE(wait_for([](const json&value){return !value["bundles"][0].value("saved",true);}));
  EXPECT_FALSE(catalog.snapshot()["bundles"][0]["valid"]);
}
