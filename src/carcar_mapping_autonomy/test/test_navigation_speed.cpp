// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/navigation_speed.hpp"
#include <thread>

using carcar_mapping_autonomy::MappingNavigationSpeed;
class MappingNavigationSpeedTest:public ::testing::Test {
protected:
  static void SetUpTestSuite() {if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}}
  void SetUp() override {
    owner=std::make_shared<rclcpp::Node>("mapping_speed_test");
    server=std::make_shared<rclcpp::Node>("speed_parameter_fixture",rclcpp::NodeOptions().start_parameter_services(false));
    set=server->create_service<MappingNavigationSpeed::Set>("/controller_server/set_parameters_atomically",
      [this](MappingNavigationSpeed::Set::Request::SharedPtr request,MappingNavigationSpeed::Set::Response::SharedPtr response){
        writes++;EXPECT_EQ(request->parameters.size(),1u);
        EXPECT_EQ(request->parameters[0].name,"FollowPath.desired_linear_vel");
        response->result.successful=!reject;response->result.reason=reject?"fixture rejects":"";
        if(!reject)value=request->parameters[0].value.double_value;
      });
    get=server->create_service<MappingNavigationSpeed::Get>("/controller_server/get_parameters",
      [this](MappingNavigationSpeed::Get::Request::SharedPtr request,MappingNavigationSpeed::Get::Response::SharedPtr response){
        EXPECT_EQ(request->names,std::vector<std::string>({"FollowPath.desired_linear_vel"}));
        rcl_interfaces::msg::ParameterValue result;
        result.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        result.double_value=mismatch?.07:value;response->values={result};
      });
    speed=std::make_unique<MappingNavigationSpeed>(*owner);
    executor.add_node(owner);executor.add_node(server);
    // DDS discovery is asynchronous; no controller service wait occurs inside tick.
    for(int i=0;i<300&&speed->state()["phase"]!="READY";i++)step();
    ASSERT_EQ(speed->state()["phase"],"READY");ASSERT_DOUBLE_EQ(speed->limit(),.1);
  }
  void TearDown() override {executor.remove_node(owner);executor.remove_node(server);speed.reset();}
  void step() {executor.spin_some();speed->tick(now);now+=.005;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  void finish() {for(int i=0;i<300&&speed->pending();i++)step();ASSERT_FALSE(speed->pending());}
  rclcpp::Node::SharedPtr owner,server;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Service<MappingNavigationSpeed::Set>::SharedPtr set;
  rclcpp::Service<MappingNavigationSpeed::Get>::SharedPtr get;
  std::unique_ptr<MappingNavigationSpeed> speed;
  double now=0,value=.18;int writes=0;bool reject=false,mismatch=false;
};
TEST_F(MappingNavigationSpeedTest,StartupResetsRetainedControllerSpeedAndReadsBack) {
  EXPECT_DOUBLE_EQ(value,.1);EXPECT_DOUBLE_EQ(speed->state()["confirmed_mps"].get<double>(),.1);
  EXPECT_EQ(writes,1);
}
TEST_F(MappingNavigationSpeedTest,RaiseWaitsForControllerReadback) {
  ASSERT_TRUE(speed->select(.18,now,"raise"));EXPECT_DOUBLE_EQ(speed->limit(),.1);
  EXPECT_TRUE(speed->state()["pending"]);finish();EXPECT_DOUBLE_EQ(speed->limit(),.18);
  EXPECT_EQ(speed->state()["request_id"],"raise");
}
TEST_F(MappingNavigationSpeedTest,LowerTakesEffectBeforeServiceAndFailureRetainsLowerCap) {
  reject=true;ASSERT_TRUE(speed->select(.05,now,"lower"));EXPECT_DOUBLE_EQ(speed->limit(),.05);
  finish();EXPECT_EQ(speed->state()["phase"],"FAILED");EXPECT_DOUBLE_EQ(speed->limit(),.05);
  EXPECT_DOUBLE_EQ(speed->state()["confirmed_mps"].get<double>(),.1);
}
TEST_F(MappingNavigationSpeedTest,RejectedRaiseNeverRaisesCap) {
  reject=true;ASSERT_TRUE(speed->select(.18,now,"raise-rejected"));finish();EXPECT_DOUBLE_EQ(speed->limit(),.1);
  EXPECT_FALSE(speed->state()["error"].get<std::string>().empty());
}
TEST_F(MappingNavigationSpeedTest,WrongReadbackNeverRaisesCap) {
  mismatch=true;ASSERT_TRUE(speed->select(.18,now,"mismatch"));finish();
  EXPECT_EQ(speed->state()["phase"],"FAILED");EXPECT_DOUBLE_EQ(speed->limit(),.1);
}
TEST_F(MappingNavigationSpeedTest,TimeoutIgnoresLateResponse) {
  ASSERT_TRUE(speed->select(.18,now,"timeout"));speed->tick(now+1.51);
  EXPECT_EQ(speed->state()["phase"],"FAILED");
  for(int i=0;i<30;i++)step();
  EXPECT_DOUBLE_EQ(speed->limit(),.1);
  ASSERT_TRUE(speed->select(.06,now,"retry"));finish();EXPECT_DOUBLE_EQ(speed->limit(),.06);
}
TEST_F(MappingNavigationSpeedTest,AlreadyArrivedResponseAfterDeadlineCannotRaiseCap) {
  ASSERT_TRUE(speed->select(.18,now,"late"));
  for(int i=0;i<20;i++){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  speed->tick(now+1.51);EXPECT_EQ(speed->state()["phase"],"FAILED");EXPECT_DOUBLE_EQ(speed->limit(),.1);
}
TEST_F(MappingNavigationSpeedTest,DuplicatePendingAndInvalidValuesCannotChangeRequest) {
  for(double invalid:{0.0,.049,.181,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
    EXPECT_FALSE(speed->select(invalid,now,"invalid"));
  ASSERT_TRUE(speed->select(.12,now,"first"));EXPECT_FALSE(speed->select(.18,now,"second"));
  EXPECT_EQ(speed->state()["request_id"],"first");finish();EXPECT_DOUBLE_EQ(speed->limit(),.12);
}
TEST_F(MappingNavigationSpeedTest,TaskRejectionHasSeparateRequestIdWithoutChangingCap) {
  EXPECT_FALSE(speed->reject("phase-rejection","当前正在恢复，调速未下发"));
  EXPECT_EQ(speed->state()["request_id"],"phase-rejection");EXPECT_EQ(speed->state()["phase"],"FAILED");
  EXPECT_DOUBLE_EQ(speed->limit(),.1);EXPECT_DOUBLE_EQ(speed->state()["confirmed_mps"].get<double>(),.1);
}
TEST_F(MappingNavigationSpeedTest,TaskRejectionCannotOverwritePendingTransaction) {
  ASSERT_TRUE(speed->select(.18,now,"first"));
  EXPECT_FALSE(speed->reject("late-phase-rejection","当前正在停车"));
  EXPECT_EQ(speed->state()["request_id"],"first");EXPECT_TRUE(speed->pending());
  finish();EXPECT_DOUBLE_EQ(speed->limit(),.18);
}
