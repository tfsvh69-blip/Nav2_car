#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
class SnapshotSupervisorIntegration:public SupervisorIntegration {
protected:
  void SetUp()override{recovery_mode=true;online_snapshot_mode=true;slam_sequence=4;recovery_state["checkpoint_age_seconds"]=0.;SupervisorIntegration::SetUp();}
};
TEST_F(SnapshotSupervisorIntegration, NinetySecondMaintenanceAndOneFailureDoNotStopNavigation){
  start_goal();recovery_state["checkpoint_age_seconds"]=91.;
  ASSERT_TRUE(until([this]{return std::find(recovery_actions.begin(),recovery_actions.end(),"snapshot")!=recovery_actions.end();},2));
  until([]{return false;},.3);EXPECT_EQ(state.value("phase",""),"EXPLORING");EXPECT_GT(last_velocity,0);
  until([]{return false;},10.1);const auto requests_before_failure=recovery_actions.size();
  recovery_state["snapshot"]["phase"]="FAILED";recovery_state["snapshot"]["in_flight"]=false;recovery_state["snapshot"]["reason"]="模拟磁盘空间不足";
  until([]{return false;},1.0);EXPECT_EQ(state.value("phase",""),"EXPLORING");EXPECT_GT(last_velocity,0);
  EXPECT_EQ(recovery_actions.size(),requests_before_failure)<<"失败后必须再等10秒，不能从请求开始计退避";
  EXPECT_FALSE(state.value("recovery_pending",true));
  recovery_state["checkpoint_age_seconds"]=121.;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2))<<state;
  EXPECT_EQ(last_velocity,0);EXPECT_FALSE(state.value("checkpoint_usable",true));EXPECT_NE(state.value("reason","").find("120秒"),std::string::npos);
}
TEST_F(SnapshotSupervisorIntegration, LatePublicationCannotRefreshAnExpiredCapture){
  start_goal();recovery_state["checkpoint_id"]="newly-finished-old-capture";recovery_state["checkpoint_age_seconds"]=125.;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2))<<state;
  request("start","resume-expired");until([]{return false;},.4);
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(last_velocity,0);EXPECT_FALSE(state.value("checkpoint_usable",true));
}
class InitialSnapshotSupervisorIntegration:public SupervisorIntegration {
protected:
  void SetUp()override{recovery_mode=true;online_snapshot_mode=true;automatic_on_boot=true;slam_sequence=4;
    recovery_state["checkpoint_id"]="";recovery_state["checkpoint_age_seconds"]=-1.;SupervisorIntegration::SetUp();}
};
TEST_F(InitialSnapshotSupervisorIntegration, FirstFailureKeepsVehicleStoppedWithoutFaultAndRetries){
  ASSERT_TRUE(until([this]{return !recovery_actions.empty();},3));
  recovery_state["snapshot"]["phase"]="FAILED";recovery_state["snapshot"]["in_flight"]=false;
  const auto requests=recovery_actions.size();until([]{return false;},.3);
  EXPECT_EQ(state.value("phase",""),"MANUAL");EXPECT_TRUE(state.value("auto_start_pending",false));EXPECT_EQ(last_velocity,0);
  ASSERT_TRUE(until([this,requests]{return recovery_actions.size()>requests;},11));
  EXPECT_EQ(nav_requests,0U);EXPECT_EQ(last_velocity,0);
  recovery_state["checkpoint_id"]="first-trusted";recovery_state["checkpoint_age_seconds"]=0.;
  recovery_state["snapshot"]["phase"]="READY";recovery_state["snapshot"]["in_flight"]=false;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));
}
