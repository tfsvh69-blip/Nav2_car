#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"

class LiveSupervisorIntegration:public SupervisorIntegration {
protected:
  void SetUp()override{live_mode=recovery_mode=restricted_manual_enabled=true;slam_sequence=4;SupervisorIntegration::SetUp();}
  Json walls(double x=1.5){return Json::array({{{"id","glass"},{"points",Json::array({Json::array({x,-1.0}),Json::array({x,1.0})})}}});}
  void wall_request(const std::string&action,const std::string&id,Json items=Json()){
    Json data={{"navigation_generation",navigation_generation},{"wall_revision",live_scene->state().value("revision",uint64_t{})}};
    if(action=="set_walls")data["walls"]=items;
    request(action,id,data);
  }
  void install_wall(double x=1.5){wall_request("set_walls","initial-wall",walls(x));
    // 场景已更新不能代替监督器收到同版本ACK；旧空墙的READY不得提前结束等待。
    ASSERT_TRUE(until([this]{return live_scene->state().value("revision",0)==1&&
      state["walls"].value("applied_revision",0)==1&&state["walls"].value("navigation_ready",false)&&
      !state["walls"].value("pending",false)&&result("initial-wall","accepted")&&state.value("phase","")=="MANUAL";},4))<<state.dump();}
  bool result(const std::string&id,const std::string&value){
    for(const auto&r:state.value("request_results",Json::array()))if(r.value("request_id","")==id)return r.value("request_result","")==value;
    return false;}
};

class LiveAutomaticStartupIntegration:public LiveSupervisorIntegration {
protected:
  void SetUp()override{
    automatic_on_boot=true;
    tuning_state.update({{"ready",false},{"pending",true},{"requires_stop",false}});
    LiveSupervisorIntegration::SetUp();
  }
  void tuning_ready(){tuning_state.update({{"ready",true},{"pending",false},{"requires_stop",false}});}
  void cancel_waiting_start(const std::string&action,const std::string&target){
    ASSERT_TRUE(state.value("auto_start_pending",false))<<state.dump();
    request(action,"cancel-startup-"+action);
    ASSERT_TRUE(until([&]{return result("cancel-startup-"+action,"accepted")&&state.value("phase","")==target;},3))<<state.dump();
    EXPECT_FALSE(state.value("auto_start_pending",true));EXPECT_DOUBLE_EQ(last_velocity,0);
    const auto stopped_outputs=nonzero;
    if(action=="manual"){
      auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);double desired=.05;
      auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist velocity;velocity.linear.x=desired;manual->publish(velocity);});
      // 允许取消自动等待，不等于允许参数尚未核验时遥控行驶。
      until([]{return false;},.7);EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,stopped_outputs);
      desired=0;until([]{return false;},.2);heartbeat.reset();
    }
    // READY只能解除初始化等待，不能覆盖操作者已撤销的自动启动请求。
    tuning_ready();until([]{return false;},.7);
    EXPECT_EQ(state.value("phase",""),target);EXPECT_FALSE(state.value("auto_start_pending",true));
    EXPECT_EQ(nonzero,stopped_outputs);EXPECT_DOUBLE_EQ(last_velocity,0);
    EXPECT_EQ(nav_requests,0U);EXPECT_FALSE(state["fault"].value("active",false));
  }
};

TEST_F(LiveAutomaticStartupIntegration, DelayedTuningAndSceneKeepBootRequestUntilBothAreReady){
  until([]{return false;},.3);
  ASSERT_TRUE(state.value("auto_start_pending",false))<<state.dump();
  EXPECT_EQ(state.value("phase",""),"MANUAL");EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_NE(state.value("automatic_start_reason","").find("动态参数"),std::string::npos);
  // 参数即将就绪时再制造真实跨话题地图同步等待，验证两道门槛均不会消费请求。
  live_state=live_mask=false;++grid_width;grid_origin-=.05;
  ASSERT_TRUE(until([this]{return !state["walls"].value("navigation_ready",true);},1));
  tuning_ready();
  ASSERT_TRUE(until([this]{return state.value("automatic_start_reason","").find("人工墙")!=std::string::npos;},1))<<state.dump();
  EXPECT_TRUE(state.value("auto_start_pending",false));EXPECT_EQ(state.value("phase",""),"MANUAL");
  EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,0U);EXPECT_EQ(nav_requests,0U);
  live_state=live_mask=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3))<<state.dump();
  EXPECT_FALSE(state.value("auto_start_pending",true));EXPECT_TRUE(state["walls"].value("navigation_ready",false));
  EXPECT_TRUE(state.value("healthy",false));EXPECT_FALSE(state["fault"].value("active",false));
}

TEST_F(LiveAutomaticStartupIntegration, HaltDuringStartupWaitCannotBeOverriddenByReady){
  cancel_waiting_start("halt","PAUSED");
}

TEST_F(LiveAutomaticStartupIntegration, PauseDuringStartupWaitCannotBeOverriddenByReady){
  EXPECT_TRUE(state["actions"]["pause"].value("enabled",false));
  cancel_waiting_start("pause","PAUSED");
}

TEST_F(LiveAutomaticStartupIntegration, ManualDuringStartupWaitCancelsBootWithoutBypassingReadiness){
  EXPECT_TRUE(state["actions"]["manual"].value("enabled",false));
  cancel_waiting_start("manual","MANUAL");
}

TEST_F(LiveSupervisorIntegration, NormalMapReorderingHoldsOutputWithoutFaultAndRecovers){
  start_goal();const auto goal=active;
  live_state=false;live_mask=false;++grid_width;grid_origin-=.05;
  ASSERT_TRUE(until([this]{return !state["walls"].value("navigation_ready",true)&&last_velocity==0;},1));
  until([]{return false;},.45);EXPECT_EQ(state["phase"],"EXPLORING");EXPECT_FALSE(state["fault"].value("active",false));
  live_state=live_mask=true;ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false)&&last_velocity>0;},2));
  EXPECT_EQ(active,goal);EXPECT_EQ(nav_requests,1U);
}

TEST_F(LiveSupervisorIntegration, WallAckCannotResumeBeforeDerivedMapAndMaskMatch){
  start_goal();live_map=live_mask=false;wall_request("set_walls","replace-wait-map",walls());
  ASSERT_TRUE(until([this]{return live_scene->state().value("revision",0)==1;},3));
  until([]{return false;},.4);EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_FALSE(state["walls"].value("navigation_ready",true));EXPECT_TRUE(state["walls"].value("pending",false));
  live_map=live_mask=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING"&&state["walls"].value("navigation_ready",false);},3))<<state.dump();
  EXPECT_TRUE(result("replace-wait-map","accepted"));
}

TEST_F(LiveSupervisorIntegration, HumanStopCancelsWallAutomaticResumeAndRejectedUpdatePreservesOldWalls){
  install_wall();start_goal();live_requests=false;
  wall_request("set_walls","stopped-change",walls(1.6));
  ASSERT_TRUE(until([this]{return !wall_commands.empty()&&wall_commands.back().value("request_id","")=="stopped-change";},3));
  request("halt","human-stop");ASSERT_TRUE(until([this]{return result("human-stop","accepted");},2));
  live_scene->request(wall_commands.back());live_requests=true;
  ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false)&&!state["walls"].value("pending",false);},3));
  EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
  live_reject=true;const auto old=live_scene->state()["walls"];wall_request("set_walls","rejected-change",walls(1.8));
  ASSERT_TRUE(until([this]{return result("rejected-change","rejected");},3))<<state.dump();
  EXPECT_EQ(live_scene->state()["walls"],old);EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, ParameterRollbackPausesWithoutLocalizationFaultAndHumanStopWins){
  start_goal();tuning_state.update({{"ready",false},{"requires_stop",true},{"pending",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));EXPECT_FALSE(state["fault"].value("active",false));
  request("halt","stop-during-rollback");ASSERT_TRUE(until([this]{return result("stop-during-rollback","accepted");},2));
  tuning_state.update({{"ready",true},{"requires_stop",false},{"pending",false}});
  until([]{return false;},.7);EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
  request("start","after-rollback");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));
  tuning_state.update({{"ready",false},{"requires_stop",true},{"pending",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
  tuning_state.update({{"ready",true},{"requires_stop",false},{"pending",false}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));EXPECT_FALSE(state["fault"].value("active",false));
}

TEST_F(LiveSupervisorIntegration, ManualSweptFootprintRejectsWallDirectionAndBypassExitStops){
  install_wall(.26);
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);double desired=.08;
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist velocity;velocity.linear.x=desired;manual->publish(velocity);});
  until([]{return false;},.7);EXPECT_DOUBLE_EQ(last_velocity,0);desired=-.08;
  ASSERT_TRUE(until([this]{return last_velocity<-.01;},2))<<state.dump();
  desired=0;ASSERT_TRUE(until([this]{return last_velocity==0;},1));
  request("set_wall_bypass","explicit-bypass",{{"enabled",true},{"bypass_owner_id",manual_owner_id}});
  ASSERT_TRUE(until([this]{return state["restricted_manual"].value("ready",false)&&state["walls"].value("bypass_enabled",false);},3))<<state.dump();
  desired=.12;ASSERT_TRUE(until([this]{return last_velocity>.01;},3))<<state.dump();EXPECT_LE(last_velocity,.05);
  request("set_wall_bypass","exit-bypass",{{"enabled",false}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED"&&last_velocity==0;},3));
  EXPECT_TRUE(state["walls"].value("requires_recheck",false));EXPECT_FALSE(state["walls"].value("bypass_enabled",true));
  request("start","cannot-skip-recheck");ASSERT_TRUE(until([this]{return result("cannot-skip-recheck","rejected");},2));
  heartbeat.reset();
}

TEST_F(LiveSupervisorIntegration, RecoveryUsesRawMapThenWallReviewThenNavigationCommit){
  install_wall();start_goal();const auto origin=state["origin"];
  recovery_state.update({{"phase","DISTURBED"},{"requires_stop",true},{"gate_write",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3))<<state.dump();
  ASSERT_TRUE(until([this]{return recovery_state["phase"]=="WAITING_STABLE";},2));
  recovery_state.update({{"phase","VERIFY_READY"},{"commit_ready",true}});
  ASSERT_TRUE(until([this]{return state["walls"].value("review_allowed",false);},2))<<state.dump();
  EXPECT_FALSE(state["walls"].value("navigation_ready",true));EXPECT_DOUBLE_EQ(last_velocity,0);
  until([]{return false;},.5);EXPECT_EQ(state["phase"],"RECOVERING");
  EXPECT_EQ(std::find(recovery_actions.begin(),recovery_actions.end(),"commit_verified"),recovery_actions.end());
  wall_request("confirm_walls","review-after-restore");
  ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false);},3))<<state.dump();
  const auto reviewed_generation=navigation_generation;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},6))<<state.dump();
  EXPECT_EQ(navigation_generation,reviewed_generation);EXPECT_EQ(state["origin"],origin);
  EXPECT_EQ(live_scene->state()["phase"],"READY");EXPECT_EQ(nav_requests,1U);
  EXPECT_NE(std::find(recovery_actions.begin(),recovery_actions.end(),"commit_verified"),recovery_actions.end());
}

TEST_F(LiveSupervisorIntegration,BypassRequiresCurrentOwnerAndHandoffRevokesItWithoutReusingCommands){
  install_wall(.26);
  request("set_wall_bypass","legacy-no-owner",{{"enabled",true}});
  ASSERT_TRUE(until([this]{return result("legacy-no-owner","rejected");},2));
  request("set_wall_bypass","wrong-owner",{{"enabled",true},{"bypass_owner_id","different-owner"}});
  ASSERT_TRUE(until([this]{return result("wrong-owner","rejected");},2));
  request("set_wall_bypass","owner-bypass",{{"enabled",true},{"bypass_owner_id",manual_owner_id}});
  ASSERT_TRUE(until([this]{return state["restricted_manual"].value("ready",false)&&state["walls"].value("bypass_enabled",false);},3));
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist velocity;velocity.linear.x=.05;manual->publish(velocity);});
  ASSERT_TRUE(until([this]{return last_velocity>.01;},3))<<state.dump();
  manual_owner_id.clear();ASSERT_TRUE(until([this]{return !state["walls"].value("bypass_enabled",true)&&state.value("phase","")=="PAUSED"&&last_velocity==0;},3));
  manual_owner_id="replacement-owner";until([]{return false;},.4);
  EXPECT_TRUE(state["walls"].value("requires_recheck",false));EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_TRUE(state["walls"].value("bypass_owner_id",std::string()).empty());heartbeat.reset();
}
