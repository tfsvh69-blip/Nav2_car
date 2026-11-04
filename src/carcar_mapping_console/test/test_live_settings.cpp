#include <gtest/gtest.h>
#include <limits>
#include <sys/stat.h>
#include "carcar_mapping_console/live_settings.hpp"
using namespace carcar_mapping_console;
class LiveSettings : public ::testing::Test {
protected:
  std::filesystem::path directory,path;
  void SetUp()override{directory=std::filesystem::temp_directory_path()/("carcar-live-settings-"+nonce());std::filesystem::create_directories(directory);path=directory/"state"/"defaults.json";}
  void TearDown()override{std::filesystem::remove_all(directory);}
};
TEST_F(LiveSettings, MissingFileUsesBaselineWithoutCreatingAnything){
  EXPECT_EQ(load_tuning_defaults(path),baseline_live_defaults());
  EXPECT_FALSE(std::filesystem::exists(path.parent_path()));
}
TEST_F(LiveSettings, SaveAndReloadOnlyCompleteCostsAndHandoff){
  auto config=baseline_live_defaults();config["values"]["global_inflation_radius"]=.65;config["handoff_distance_m"]=1.5;
  save_tuning_defaults(path,config);
  EXPECT_EQ(load_tuning_defaults(path),config);
  auto invalid=config;invalid["handoff_distance_m"]=1.51;
  EXPECT_THROW(validate_tuning_defaults(invalid),std::exception);
  EXPECT_THROW(save_tuning_defaults(path,invalid),std::exception);
  EXPECT_EQ(load_tuning_defaults(path),config);
  struct stat metadata{};
  ASSERT_EQ(::stat(path.c_str(),&metadata),0);
  EXPECT_EQ(metadata.st_mode&0777,0600);
  config["values"]["local_cost_scaling_factor"]=7.;save_tuning_defaults(path,config);
  EXPECT_EQ(load_tuning_defaults(path),config);
  size_t files=0;for(const auto&entry:std::filesystem::directory_iterator(path.parent_path())){(void)entry;++files;}EXPECT_EQ(files,1u);
}
TEST_F(LiveSettings, InvalidOrIncompatibleOriginalIsPreserved){
  std::filesystem::create_directories(path.parent_path());std::ofstream(path)<<"{broken";
  EXPECT_THROW(load_tuning_defaults(path),std::exception);
  EXPECT_THROW(save_tuning_defaults(path,baseline_live_defaults()),std::exception);
  EXPECT_EQ(read_file(path),"{broken");
  auto unsupported=baseline_live_defaults();unsupported["version"]=2;std::ofstream(path)<<unsupported.dump();
  EXPECT_THROW(save_tuning_defaults(path,baseline_live_defaults()),std::exception);
  EXPECT_EQ(json::parse(read_file(path))["version"],2);
}
TEST_F(LiveSettings, SymbolicFileAndParentNeverFollowOrOverwrite){
  std::filesystem::create_directories(path.parent_path());const auto original=directory/"original.json";std::ofstream(original)<<baseline_live_defaults().dump();
  std::filesystem::create_symlink(original,path);
  EXPECT_THROW(load_tuning_defaults(path),std::exception);
  EXPECT_THROW(save_tuning_defaults(path,baseline_live_defaults()),std::exception);
  EXPECT_TRUE(std::filesystem::is_symlink(path));
  const auto linked=directory/"linked";std::filesystem::create_directory_symlink(path.parent_path(),linked);
  EXPECT_THROW(load_tuning_defaults(linked/"other.json"),std::exception);
}
TEST_F(LiveSettings, NonRegularAndOversizedFilesRejectWithoutBlocking){
  std::filesystem::create_directories(path.parent_path());
  ASSERT_EQ(::mkfifo(path.c_str(),0600),0);
  EXPECT_THROW(load_tuning_defaults(path),std::exception);std::filesystem::remove(path);
  std::ofstream(path)<<std::string(16385,' ');
  EXPECT_THROW(load_tuning_defaults(path),std::exception);
}
TEST_F(LiveSettings, SavingReadsConfirmedValuesAndNeverWritesWalls){
  json state={{"task_id","settings-task"},{"tuning",{{"task_id","settings-task"},{"ready",true},{"age_seconds",.1},{"phase","READY"},{"pending",false},{"requested",{{"global_inflation_radius",1.2}}},{"confirmed",baseline_tuning_values()}}},
    {"handoff",{{"phase","READY"},{"distance_m",.4}}},{"walls",{{"items",json::array({{{"id","ephemeral"},{"points",{{0,0},{1,0}}}}})}}}};
  const auto result=stable_tuning_defaults(state);
  EXPECT_EQ(result["values"]["global_inflation_radius"],.5);
  EXPECT_EQ(result["handoff_distance_m"],.4);
  EXPECT_FALSE(result.contains("walls"));
  save_tuning_defaults(path,result);
  EXPECT_FALSE(read_file(path).find("ephemeral")!=std::string::npos);
  state["tuning"]["pending"]=true;
  EXPECT_THROW(stable_tuning_defaults(state),std::exception);state["tuning"]["pending"]=false;state["tuning"]["phase"]="FAILED";
  EXPECT_THROW(stable_tuning_defaults(state),std::exception);
}
TEST_F(LiveSettings, SavingRejectsCachedReadyMissingAgeAndWrongTask){
  const json valid={{"task_id","current"},{"tuning",{{"task_id","current"},{"ready",true},{"age_seconds",.05},{"phase","READY"},{"pending",false},{"confirmed",baseline_tuning_values()}}},{"handoff",{{"phase","READY"},{"distance_m",.25}}}};
  EXPECT_NO_THROW(stable_tuning_defaults(valid));
  auto invalid=valid;invalid["tuning"]["ready"]=false;EXPECT_THROW(stable_tuning_defaults(invalid),std::exception);
  invalid=valid;invalid["tuning"].erase("age_seconds");EXPECT_THROW(stable_tuning_defaults(invalid),std::exception);
  for(const double age:{-1.,2.01,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){invalid=valid;invalid["tuning"]["age_seconds"]=age;EXPECT_THROW(stable_tuning_defaults(invalid),std::exception);}
  invalid=valid;invalid["tuning"]["task_id"]="old-task";EXPECT_THROW(stable_tuning_defaults(invalid),std::exception);
  invalid=valid;invalid.erase("task_id");EXPECT_THROW(stable_tuning_defaults(invalid),std::exception);
}
TEST_F(LiveSettings, WhitelistNumbersAndCompleteDefaults){
  EXPECT_THROW(validate_tuning_values({{"global_inflation_radius",.1}}),std::exception);
  EXPECT_THROW(validate_tuning_values({{"global_cost_scaling_factor",std::numeric_limits<double>::infinity()}}),std::exception);
  EXPECT_THROW(validate_tuning_values({{"local_cost_scaling_factor",true}}),std::exception);
  EXPECT_THROW(validate_tuning_values({{"watchdog",1}}),std::exception);
  EXPECT_THROW(validate_tuning_values({{"global_inflation_radius",.5}},true),std::exception);
  auto config=baseline_live_defaults();config["walls"]=json::array();
  EXPECT_THROW(validate_tuning_defaults(config),std::exception);
}
TEST_F(LiveSettings, RequestTypesAndFieldsAreStrict){
  EXPECT_EQ(validate_live_request("set_tuning",{{"values",{{"global_inflation_radius",.6}}}})["values"]["global_inflation_radius"],.6);
  EXPECT_THROW(validate_live_request("set_tuning",{{"values",{{"global_inflation_radius",.6}}},{"speed_mps",.18}}),std::exception);
  EXPECT_THROW(validate_live_request("set_escape_enabled",{{"enabled",1}}),std::exception);
  EXPECT_EQ(validate_live_request("set_handoff",{{"distance_m",1.5}})["distance_m"],1.5);
  EXPECT_THROW(validate_live_request("set_handoff",{{"distance_m",1.51}}),std::exception);
  EXPECT_THROW(validate_live_request("set_handoff",{{"distance_m",.1}}),std::exception);
  EXPECT_THROW(validate_live_request("set_handoff",{{"distance_m",.3},{"walls",json::array()}}),std::exception);
  const auto input=json::parse(R"({"walls":[{"id":"line","points":[[0,0],[1,0]]}],"wall_revision":0,"navigation_generation":2})");
  EXPECT_EQ(validate_live_request("set_walls",input)["walls"].size(),1u);
  auto duplicate=input;duplicate["walls"].push_back(input["walls"][0]);
  EXPECT_THROW(validate_live_request("set_walls",duplicate),std::exception);
  auto unknown=input;unknown["walls"][0]["persistent"]=true;
  EXPECT_THROW(validate_live_request("set_walls",unknown),std::exception);
  auto nan=input;nan["walls"][0]["points"][0][0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_live_request("set_walls",nan),std::exception);
  auto repeated=input;repeated["walls"][0]["points"][1]=repeated["walls"][0]["points"][0];
  EXPECT_THROW(validate_live_request("set_walls",repeated),std::exception);
}
TEST_F(LiveSettings, OnlineCostsDoNotStopAndWallsDoStopManualInput){
  for(const auto&action:{"set_tuning","restore_tuning","save_tuning","set_handoff","set_escape_enabled"})EXPECT_TRUE(online_live_action(action));
  for(const auto&action:{"set_walls","confirm_walls","set_wall_bypass"}){EXPECT_TRUE(live_action(action));
  EXPECT_FALSE(online_live_action(action));}
}

TEST_F(LiveSettings, SupervisedTrialIsExplicitBooleanAndOnlyForEscape){
 const json request={{"enabled",true},{"supervised_trial",true}};
 EXPECT_EQ(validate_live_request("set_escape_enabled",request).at("supervised_trial"),true);
 EXPECT_THROW(validate_live_request("set_escape_enabled",{{"enabled",true},{"supervised_trial","true"}}),std::exception);
 EXPECT_THROW(validate_live_request("set_wall_bypass",request),std::exception);
}
