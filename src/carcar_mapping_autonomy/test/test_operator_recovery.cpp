#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <filesystem>
#include <future>
// 真实监督器进程；隔离域，无底盘、串口及实车速度出口。
class OperatorRecovery:public SupervisorIntegration {
protected:
  void SetUp()override{recovery_mode=true;slam_sequence=4;SupervisorIntegration::SetUp();}
  void fail_scan(){feed_scan=false;ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));session_ready=false;}
};
TEST_F(OperatorRecovery, ScanRecoversButMotionWaitsForExplicitOperatorChoice){
  start_goal();const auto origin=state["origin"];fail_scan();
  ASSERT_TRUE(state.contains("fault"));EXPECT_NE(state["fault"].value("reason","").find("/scan"),std::string::npos);
  const auto first=state["fault"];feed_scan=true;
  ASSERT_TRUE(until([this]{return state.value("actions",Json::object()).value("recheck",Json::object()).value("enabled",false);},3))<<state;
  request("recheck","operator-check");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="CHECKING";},3))<<state;
  EXPECT_EQ(last_velocity,0);
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},5))<<state;
  EXPECT_EQ(state["origin"],origin);EXPECT_EQ(state["fault"]["reason"],first["reason"]);
  EXPECT_EQ(state["operator_check"]["phase"],"READY");EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
  until([]{return false;},.5);EXPECT_EQ(last_velocity,0);EXPECT_EQ(state["phase"],"PAUSED");
  session_ready=true;ASSERT_TRUE(until([this]{return state.value("healthy",false);},2));
  request("return","operator-return");ASSERT_TRUE(until([this]{return state.value("phase","")=="RETURNING";},3))<<state;
}
TEST_F(OperatorRecovery, PersistentFaultRejectsCheckAndHaltIsIdempotent){
  start_goal();fail_scan();request("recheck","still-stale");
  ASSERT_TRUE(until([this]{return state.value("request_id","")=="still-stale";},1));
  EXPECT_EQ(state["request_result"],"rejected");EXPECT_EQ(state["phase"],"FAULT");EXPECT_EQ(last_velocity,0);
  request("halt","stop-fault");ASSERT_TRUE(until([this]{return state.value("request_id","")=="stop-fault";},1));
  EXPECT_EQ(state["request_result"],"accepted");EXPECT_EQ(state["phase"],"FAULT");
}
TEST_F(OperatorRecovery, PausedFrozenMapCanRecheckWithStaleNavigationButCannotDrive){
  start_goal();request("pause","operator-pause");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2));
  freeze_nav_pose=true;recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true},{"reason","检查点几何校验失败"}});
  until([]{return false;},1.8);
  EXPECT_FALSE(state.value("healthy",true));
  ASSERT_TRUE(state["actions"]["recheck"].value("enabled",false))<<state;
  EXPECT_FALSE(state["actions"]["resume"].value("enabled",true));
  request("recheck","stale-nav-check");
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3))<<state;
  EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
  recovery_state["phase"]="VERIFY_READY";recovery_state["commit_ready"]=true;until([]{return false;},1.3);
  EXPECT_EQ(state.value("phase",""),"RECOVERING");EXPECT_EQ(last_velocity,0);
  freeze_nav_pose=false;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},5))<<state;
  EXPECT_EQ(state["operator_check"]["phase"],"READY");EXPECT_EQ(nav_requests,1U);
}
TEST_F(OperatorRecovery, FrozenObservationDoesNotBecomeMotionHealthyFromFreshTf){
  start_goal();request("pause","hold-before-failure");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2));
  recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});
  until([]{return false;},.4);
  EXPECT_FALSE(state.value("healthy",true));
  EXPECT_FALSE(state["actions"]["manual"].value("enabled",true));
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, PausedCheckpointFailureKeepsOperatorHoldAndRecoveryEntry){
  start_goal();request("pause","pause-before-checkpoint");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2));
  fail_checkpoints=true;recovery_state["checkpoint_age_seconds"]=31.;
  ASSERT_TRUE(until([this]{return std::find(recovery_actions.begin(),recovery_actions.end(),"checkpoint")!=recovery_actions.end();},3));
  ASSERT_TRUE(until([this]{return state.value("reason","").find("检查点生成失败")!=std::string::npos;},2))<<state;
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(last_velocity,0);
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  EXPECT_FALSE(state.value("recovery_pending",true));
}
TEST_F(OperatorRecovery, OlderReferenceAllowsExplicitRevalidationButNotMotion){
  start_goal();request("pause","long-pause");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2));
  recovery_state.update({{"checkpoint_age_seconds",180.},{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});
  ASSERT_TRUE(until([this]{return !state.value("healthy",true)&&state["actions"]["recheck"].value("enabled",false);},2));
  request("recheck","revalidate-old-reference");
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
  EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
}
class AutomaticRecoveryLoop:public OperatorRecovery {
protected:
  void SetUp()override{fast_selection_timeout=true;observation_wall=true;OperatorRecovery::SetUp();}
  void selection_pause(){
    request("start","start-with-blocked-frontier");
    ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
    ASSERT_TRUE(proxy->wait_for_action_server(std::chrono::seconds(2)));
    Nav::Goal goal;goal.pose.header.frame_id="map";goal.pose.pose.position.x=.725;goal.pose.pose.orientation.w=1;
    auto sent=proxy->async_send_goal(goal);
    ASSERT_TRUE(until([&]{return sent.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));
    ASSERT_TRUE(sent.get());auto result=proxy->async_get_result(sent.get());
    ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},3));
    ASSERT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);
    ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},4))<<state;
  }
};
TEST_F(OperatorRecovery, RejectedLoopStopsAndVerifiesBeforeAutomaticContinuation){
  start_goal();const auto origin=state["origin"];++loop_rejections;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3))<<state;
  ASSERT_TRUE(until([this]{return std::find(recovery_actions.begin(),recovery_actions.end(),"recover_rollback")!=recovery_actions.end();},2));
  EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
  recovery_state["phase"]="VERIFY_READY";recovery_state["commit_ready"]=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},5))<<state;
  EXPECT_EQ(state["origin"],origin);EXPECT_EQ(state["automatic_recovery"]["attempts"],1);
  EXPECT_NE(std::find(recovery_actions.begin(),recovery_actions.end(),"commit_verified"),recovery_actions.end());
}
TEST_F(OperatorRecovery, RejectedLoopDuringOperatorPauseCannotRestart){
  start_goal();request("pause","hold-before-loop");ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
  ++loop_rejections;until([]{return false;},1.2);
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(last_velocity,0);
  EXPECT_EQ(std::count(recovery_actions.begin(),recovery_actions.end(),"recover_rollback"),0);
}
TEST_F(OperatorRecovery, HaltDuringLoopValidationIgnoresLateSuccess){
  start_goal();++loop_rejections;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3));
  request("halt","halt-loop-validation");ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));
  recovery_state.update({{"phase","SUCCEEDED"},{"gate_write",false},{"requires_stop",false}});
  until([]{return false;},1);EXPECT_EQ(state.value("phase",""),"FAULT");EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, FailedLoopValidationRemainsStopped){
  start_goal();++loop_rejections;
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
  recovery_state.update({{"phase","FAILED"},{"reason","回环撤销后多帧几何不通过"}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));
  EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
}
TEST_F(OperatorRecovery, AnotherRejectedLoopInvalidatesInFlightRecovery){
  start_goal();++loop_rejections;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3));
  ++loop_rejections;ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));
  recovery_state.update({{"phase","SUCCEEDED"},{"gate_write",false},{"requires_stop",false}});
  until([]{return false;},1);EXPECT_EQ(state.value("phase",""),"FAULT");EXPECT_EQ(last_velocity,0);
}
TEST_F(AutomaticRecoveryLoop, SelectionPauseThenCheckpointFailureRecoversOriginalTask){
  selection_pause();ASSERT_EQ(state.value("phase",""),"PAUSED");
  const auto origin=state["origin"];const double elapsed=state["elapsed_seconds"];
  recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true},{"reason","检查点几何校验失败"}});
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3))<<state;
  EXPECT_EQ(last_velocity,0);recovery_state["phase"]="VERIFY_READY";recovery_state["commit_ready"]=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},5))<<state;
  EXPECT_EQ(state["origin"],origin);EXPECT_GE(state["elapsed_seconds"].get<double>(),elapsed);
}
TEST_F(AutomaticRecoveryLoop, HaltDuringAutomaticRecoveryRejectsLateSuccess){
  selection_pause();ASSERT_EQ(state.value("phase",""),"PAUSED");
  recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3));
  request("halt","cancel-automatic-recovery");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));
  recovery_state.update({{"phase","SUCCEEDED"},{"gate_write",false},{"requires_stop",false}});
  until([]{return false;},1);EXPECT_EQ(state.value("phase",""),"FAULT");EXPECT_EQ(last_velocity,0);
}
TEST_F(AutomaticRecoveryLoop, RepeatedFailuresExhaustBudgetWithoutLosingRecheck){
  selection_pause();ASSERT_EQ(state.value("phase",""),"PAUSED");
  for(unsigned attempt=1;attempt<=2;++attempt){
    recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});
    ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3))<<state;
    recovery_state["phase"]="VERIFY_READY";recovery_state["commit_ready"]=true;
    // DDS状态可能仍是上一轮EXPLORING；必须等待本轮次数和提交结果同时到达。
    ASSERT_TRUE(until([this,attempt]{return state.value("phase","")=="EXPLORING"&&
      state["automatic_recovery"].value("attempts",0U)==attempt&&recovery_state.value("phase","")=="SUCCEEDED";},5))<<state;
    EXPECT_EQ(state["automatic_recovery"]["attempts"],attempt);
  }
  recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  EXPECT_EQ(state["automatic_recovery"]["attempts"],2);
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  until([]{return false;},1);EXPECT_EQ(last_velocity,0);EXPECT_EQ(state.value("phase",""),"PAUSED");
}
// 达到恢复边界时，只有质量故障、尚未闭门的路径也必须明确冻结。
TEST_F(AutomaticRecoveryLoop, QualityFaultAfterBudgetExhaustionStillFreezesObservation){
  selection_pause();ASSERT_EQ(state.value("phase",""),"PAUSED");
  for(unsigned attempt=1;attempt<=2;++attempt){
    recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});
    ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
    recovery_state["phase"]="VERIFY_READY";recovery_state["commit_ready"]=true;
    ASSERT_TRUE(until([this,attempt]{return state.value("phase","")=="EXPLORING"&&
      state["automatic_recovery"].value("attempts",0U)==attempt&&recovery_state.value("phase","")=="SUCCEEDED";},5));
  }
  ASSERT_FALSE(recovery_state.value("gate_write",true));
  const auto freezes=std::count(recovery_actions.begin(),recovery_actions.end(),"freeze");
  pose_x=.8;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED"&&!state.value("quality_fault","").empty();},3))<<state;
  ASSERT_TRUE(until([this,freezes]{return std::count(recovery_actions.begin(),recovery_actions.end(),"freeze")==freezes+1&&
    recovery_state.value("gate_write",false);},2))<<state;
  EXPECT_EQ(state["automatic_recovery"]["attempts"],2);EXPECT_EQ(last_velocity,0);
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  until([]{return false;},.6);EXPECT_EQ(state.value("phase",""),"PAUSED");
  request("recheck","check-after-quality-budget");
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
  EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, QualityFaultWithoutReferenceStillClosesAdmission){
  start_goal();recovery_state["checkpoint_id"]="";pose_x=std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  ASSERT_TRUE(until([this]{return recovery_state.value("gate_write",false);},2))<<state;
  pose_x=0;until([]{return false;},.3);
  EXPECT_EQ(state["automatic_recovery"]["attempts"],0);EXPECT_EQ(last_velocity,0);
  EXPECT_FALSE(state["actions"]["recheck"].value("enabled",true));
  EXPECT_NE(state["actions"]["recheck"].value("reason","").find("可信检查点"),std::string::npos);
}
TEST_F(OperatorRecovery, QualityFaultWithExpiredReferenceFreezesButAllowsExplicitCheck){
  start_goal();recovery_state["checkpoint_age_seconds"]=180.;pose_x=std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  ASSERT_TRUE(until([this]{return recovery_state.value("gate_write",false);},2))<<state;
  pose_x=0;until([]{return false;},.3);
  EXPECT_EQ(state["automatic_recovery"]["attempts"],0);EXPECT_EQ(last_velocity,0);
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  request("recheck","check-after-expired-quality");
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
  EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, QualityFaultDuringOperatorPauseFreezesWithoutAutomaticRecovery){
  start_goal();request("pause","hold-before-quality-fault");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2));
  pose_x=.8;
  ASSERT_TRUE(until([this]{return !state.value("quality_fault","").empty();},3));
  ASSERT_TRUE(until([this]{return recovery_state.value("gate_write",false);},2))<<state;
  until([]{return false;},.6);
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(last_velocity,0);
  EXPECT_TRUE(state["automatic_recovery"].value("operator_hold",false));
  EXPECT_EQ(state["automatic_recovery"]["attempts"],0);
  EXPECT_EQ(std::count(recovery_actions.begin(),recovery_actions.end(),"freeze"),1);
  EXPECT_EQ(std::count(recovery_actions.begin(),recovery_actions.end(),"recover"),0);
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  request("recheck","check-held-quality-fault");
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
  EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, QualityFaultDuringManualModeFreezesWithoutAutomaticRecovery){
  start_goal();request("manual","manual-before-quality-fault");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL";},2));
  pose_x=.8;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));
  ASSERT_TRUE(until([this]{return recovery_state.value("gate_write",false);},2))<<state;
  until([]{return false;},.6);
  EXPECT_EQ(state.value("phase",""),"FAULT");EXPECT_EQ(last_velocity,0);
  EXPECT_EQ(state["automatic_recovery"]["attempts"],0);
  EXPECT_EQ(std::count(recovery_actions.begin(),recovery_actions.end(),"freeze"),1);
  EXPECT_EQ(std::count(recovery_actions.begin(),recovery_actions.end(),"recover"),0);
  EXPECT_TRUE(state["actions"]["recheck"].value("enabled",false));
  request("recheck","check-manual-quality-fault");
  ASSERT_TRUE(until([this]{return recovery_state.value("phase","")=="WAITING_STABLE";},3));
  EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, CancelCheckInvalidatesLateSuccess){
  start_goal();fail_scan();feed_scan=true;until([]{return false;},.8);request("recheck","check-cancel");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="CHECKING";},3));
  request("halt","cancel-check");ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3));
  recovery_state.update({{"phase","SUCCEEDED"},{"gate_write",false},{"requires_stop",false}});
  until([]{return false;},1);EXPECT_EQ(state["phase"],"FAULT");EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
}
TEST_F(OperatorRecovery, RecheckCannotBypassObstacleOrLowDisk){
  start_goal();fail_scan();feed_scan=true;near_obstacle=true;until([]{return false;},.8);
  request("recheck","near-obstacle");ASSERT_TRUE(until([this]{return state.value("request_id","")=="near-obstacle";},1));
  EXPECT_EQ(state["request_result"],"rejected");EXPECT_EQ(state["phase"],"FAULT");
  near_obstacle=false;storage_ready=false;until([]{return false;},.5);
  request("recheck","low-disk");ASSERT_TRUE(until([this]{return state.value("request_id","")=="low-disk";},1));
  EXPECT_EQ(state["request_result"],"rejected");EXPECT_EQ(state["phase"],"FAULT");
  EXPECT_NE(state["error"].get<std::string>().find("磁盘"),std::string::npos);EXPECT_EQ(last_velocity,0);
}
TEST_F(OperatorRecovery, MovedRobotRequiresCheckpointCommitThenRemainsPaused){
  start_goal();const auto origin=state["origin"];fail_scan();
  pose_x=odom_x=.1;feed_scan=true;until([]{return false;},.8);
  request("recheck","moved-check");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3))<<state;
  ASSERT_TRUE(until([this]{return recovery_state["phase"]=="WAITING_STABLE";},2));
  EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
  recovery_state["phase"]="VERIFY_READY";recovery_state["commit_ready"]=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},6))<<state;
  EXPECT_EQ(state["origin"],origin);EXPECT_EQ(state["operator_check"]["phase"],"READY");
  EXPECT_NE(std::find(recovery_actions.begin(),recovery_actions.end(),"commit_verified"),recovery_actions.end());
  until([]{return false;},.5);EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
}
TEST_F(OperatorRecovery, RealHttpGatewayAndSupervisorCompleteRecheckWithoutMotion){
  namespace asio=boost::asio;namespace beast=boost::beast;namespace http=beast::http;
  using tcp=asio::ip::tcp;
  const char* executable=std::getenv("MAPPING_CHECK_GATEWAY");ASSERT_NE(executable,nullptr);
  ASSERT_TRUE(std::filesystem::exists(executable));
  // 复现现场“故障候选已经保存”的入口；保存算法另有独立网关回归。
  start_goal();fail_scan();request("save_done","fixture-saved-fault");
  ASSERT_TRUE(until([this]{return state.value("request_id","")=="fixture-saved-fault"&&!state.value("save_required",true);},2));
  char temporary[]="/tmp/carcar-r27-http-XXXXXX";ASSERT_NE(mkdtemp(temporary),nullptr);
  struct Child{pid_t pid=-1;~Child(){if(pid>0){kill(pid,SIGINT);for(int i=0;i<100;i++){if(waitpid(pid,nullptr,WNOHANG)==pid)return;usleep(20000);}kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}}gateway;
  std::vector<std::string> args{executable,"--ros-args","-p","bind_address:=127.0.0.1","-p","port:=18232",
    "-p","autonomy_enabled:=true","-p","task_id:="+task_id,"-p",std::string("map_root:=")+temporary};
  std::vector<char*> argv;for(auto&a:args)argv.push_back(a.data());argv.push_back(nullptr);
  ASSERT_EQ(posix_spawn(&gateway.pid,executable,nullptr,nullptr,argv.data(),environ),0);
  std::string session;
  auto http_request=[&](const std::string&path,Json body=Json()){
    asio::io_context io;beast::tcp_stream socket(io);socket.expires_after(std::chrono::seconds(2));
    socket.connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18232));
    http::request<http::string_body> req(body.is_null()?http::verb::get:http::verb::post,path,11);req.set("Host","127.0.0.1:18232");
    if(!body.is_null()){body["session"]=session;req.set("X-Mapping-Console","1");req.body()=body.dump();req.prepare_payload();}
    http::write(socket,req);beast::flat_buffer buffer;http::response<http::string_body> res;http::read(socket,buffer,res);
    auto result=Json::parse(res.body());result["http_status"]=res.result_int();return result;
  };
  ASSERT_TRUE(until([&]{try{return http_request("/api/status").value("phase","")=="FAILED";}catch(...){return false;}},6));
  publish_session=false;ready.reset();
  asio::io_context io;beast::websocket::stream<tcp::socket> socket(io);
  socket.next_layer().connect(tcp::endpoint(asio::ip::make_address("127.0.0.1"),18232));socket.handshake("127.0.0.1:18232","/ws/control");
  beast::flat_buffer welcome;socket.read(welcome);session=Json::parse(beast::buffers_to_string(welcome.data())).at("session");
  ASSERT_TRUE(until([&]{return http_request("/api/status").value("phase","")=="FAILED";},2));
  feed_scan=true;ASSERT_TRUE(until([this]{return state.value("actions",Json::object()).value("recheck",Json::object()).value("enabled",false);},3));
  EXPECT_EQ(http_request("/api/autonomy",{{"action","recheck"},{"request_id","http-recheck"}})["http_status"],202);
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},4))<<state;
  ASSERT_TRUE(until([&]{return http_request("/api/status").value("phase","")=="CURRENT"&&state.value("healthy",false);},3));
  EXPECT_EQ(state["operator_check"]["request_id"],"http-recheck");EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,1U);
  socket.next_layer().close();
}
