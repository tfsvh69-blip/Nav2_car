#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
#include <filesystem>
#include <fstream>
#include "carcar_mapping_autonomy/supervisor_output.hpp"
class SlowTelemetryIntegration:public SupervisorIntegration {
protected:
 std::string marker;
 virtual const char* delay_mode()const{return "file";}
 void SetUp()override{
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"205");
  marker="/tmp/carcar-telemetry-"+std::to_string(getpid())+"-"+delay_mode()+".arm";
  std::filesystem::remove(marker);std::filesystem::remove(marker+".fired");
  setenv("LD_PRELOAD",SLOW_TELEMETRY_LIBRARY,1);setenv("CARCAR_TEST_TELEMETRY_DELAY",delay_mode(),1);setenv("CARCAR_TEST_TELEMETRY_ARM",marker.c_str(),1);
  SupervisorIntegration::SetUp();unsetenv("LD_PRELOAD");unsetenv("CARCAR_TEST_TELEMETRY_DELAY");unsetenv("CARCAR_TEST_TELEMETRY_ARM");
 }
 void TearDown()override{SupervisorIntegration::TearDown();std::filesystem::remove(marker);std::filesystem::remove(marker+".fired");}
 void arm(){
  max_output_gap=0;std::ofstream(marker)<<"软件夹具已开始真实导航\n";
  auto pub=n->create_publisher<explore_lite_msgs::msg::ExploreStatus>("/explore/status",rclcpp::QoS(10).reliable().transient_local());
  explore_lite_msgs::msg::ExploreStatus status;status.status="exploration_waiting_candidates";pub->publish(status);
  ASSERT_TRUE(until([this]{return std::filesystem::exists(marker+".fired");},2))<<state.dump();
 }
 void verify_slow_output_does_not_fault(){
  start_goal();arm();until([]{return false;},1.0);
  EXPECT_EQ(state.value("phase",""),"EXPLORING")<<state.dump();
  EXPECT_TRUE(state.value("fault",Json::object()).empty())<<"扫描持续由独立夹具发布，慢输出不得误报："<<state.dump();
  EXPECT_LT(max_output_gap,.20)<<"慢状态或日志阻塞了50毫秒控制循环";
  const auto publication=state.value("publication",Json::object());
  RecordProperty("max_output_gap_ms",std::to_string(max_output_gap*1000));
  RecordProperty("max_control_tick_work_ms",std::to_string(state.value("control_max_tick_work_ms",0.)));
  RecordProperty("max_publish_ms",std::to_string(publication.value("max_publish_ms",0.)));
  RecordProperty("max_log_ms",std::to_string(publication.value("max_log_ms",0.)));
  RecordProperty("state_replaced",std::to_string(publication.value("state_replaced",uint64_t{0})));
  EXPECT_GE(publication.value(std::string(delay_mode())=="file"?"max_log_ms":"max_publish_ms",0.),590.);
  EXPECT_GE(publication.value("state_replaced",uint64_t{0}),3U);
  EXPECT_EQ(publication.value("events_dropped",uint64_t{1}),0U);
 }
};
class SlowStateTelemetryIntegration:public SlowTelemetryIntegration {
 const char* delay_mode()const override{return "state";}
};
TEST_F(SlowTelemetryIntegration, SlowFileLogCannotInventStaleScan){verify_slow_output_does_not_fault();}
TEST_F(SlowStateTelemetryIntegration, SlowStatePublicationCannotInventStaleScan){verify_slow_output_does_not_fault();}
TEST_F(SlowTelemetryIntegration, PauseDuringSlowLogStillStopsAndCancelsPromptly){
 start_goal();arm();const auto began=std::chrono::steady_clock::now();request("pause","slow-log-pause");
 ASSERT_TRUE(until([this]{return last_velocity==0;},.25));
 EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.25);
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},1.4))<<state.dump();
 EXPECT_LT(max_output_gap,.20);EXPECT_TRUE(state.value("fault",Json::object()).empty());
}
TEST_F(SlowTelemetryIntegration, HaltDuringSlowLogStillStopsAndCancelsPromptly){
 start_goal();arm();request("halt","slow-log-halt");
 ASSERT_TRUE(until([this]{return last_velocity==0;},.25));
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},1.4))<<state.dump();
 EXPECT_TRUE(state.value("navigation_terminal",false));EXPECT_LT(max_output_gap,.20);
 EXPECT_TRUE(state.value("fault",Json::object()).empty());
}
TEST_F(SlowStateTelemetryIntegration, RealMissingScanStillCancelsAndLocksAfterSlowOutput){
 start_goal();arm();until([]{return false;},.8);feed_scan=false;
 ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},2.))<<state.dump();
 EXPECT_EQ(last_velocity,0);EXPECT_NE(state["fault"].value("reason",std::string()).find("原始 /scan"),std::string::npos);
 EXPECT_TRUE(state.value("navigation_terminal",false));EXPECT_LT(max_output_gap,.20);
}

TEST(SupervisorOutput, SlowSinkKeepsLatestStateAndSeparateCriticalEvent){
 std::mutex mutex;std::condition_variable cv;bool entered=false,released=false;
 std::vector<int>samples;std::vector<std::string>events;
 carcar_mapping_autonomy::SupervisorOutput output([&](const std::string&body){
  const auto data=Json::parse(body);samples.push_back(data.at("sample"));
  if(samples.size()==1){std::unique_lock<std::mutex>lock(mutex);entered=true;cv.notify_all();cv.wait(lock,[&]{return released;});}
 },[&](int,const std::string&body){events.push_back(body);});
 output.submit({{"sample",0}},false);
 {std::unique_lock<std::mutex>lock(mutex);ASSERT_TRUE(cv.wait_for(lock,std::chrono::seconds(2),[&]{return entered;}));}
 const auto began=std::chrono::steady_clock::now();for(int i=1;i<=200;++i)output.submit({{"sample",i}},false);
 output.event(30,"停车请求已保留");EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.1);
 EXPECT_EQ(output.diagnostics()["state_replaced"],199);EXPECT_EQ(output.diagnostics()["event_pending"],1);
 {std::lock_guard<std::mutex>lock(mutex);released=true;}cv.notify_all();output.close();
 ASSERT_EQ(samples.size(),2U);EXPECT_EQ(samples.back(),200);ASSERT_EQ(events.size(),1U);EXPECT_EQ(events.front(),"停车请求已保留");
}
TEST(SupervisorOutput, EventOverflowIsBoundedAndDoesNotReplaceRequestRejectionWithHeartbeat){
 std::mutex mutex;std::condition_variable cv;bool entered=false,released=false;std::vector<std::string>events;
 carcar_mapping_autonomy::SupervisorOutput output([&](const std::string&){
  std::unique_lock<std::mutex>lock(mutex);if(!entered){entered=true;cv.notify_all();cv.wait(lock,[&]{return released;});}
 },[&](int,const std::string&body){events.push_back(body);});
 output.submit({{"sample",0}},false);
 {std::unique_lock<std::mutex>lock(mutex);ASSERT_TRUE(cv.wait_for(lock,std::chrono::seconds(2),[&]{return entered;}));}
 for(int i=1;i<=100;++i)output.submit({{"sample",i}},true);
 output.event(30,"请求被拒绝，保持停车");
 auto diagnostics=output.diagnostics();EXPECT_EQ(diagnostics["event_pending"],64);EXPECT_EQ(diagnostics["events_dropped"],37);
 {std::lock_guard<std::mutex>lock(mutex);released=true;}cv.notify_all();output.close();
 EXPECT_EQ(events.size(),64U);EXPECT_EQ(events.back(),"请求被拒绝，保持停车");EXPECT_EQ(output.diagnostics()["event_pending"],0);
}
TEST(SupervisorOutput, OutputExceptionsAreObservableAndDoNotTerminateWorker){
 std::atomic_int publications{0},events{0};
 carcar_mapping_autonomy::SupervisorOutput output([&](const std::string&){if(publications++==0)throw std::runtime_error("隔离发布异常");},
  [&](int,const std::string&){if(events++==0)throw std::runtime_error("隔离日志异常");});
 output.submit({{"sample",1}},true);
 const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
 while((publications<1||events<1)&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(2));
 output.submit({{"sample",2}},true);output.close();const auto diagnostics=output.diagnostics();
 EXPECT_EQ(diagnostics["publish_failures"],1);EXPECT_EQ(diagnostics["log_failures"],1);EXPECT_EQ(diagnostics["state_published"],1);EXPECT_EQ(diagnostics["events_logged"],1);
}
