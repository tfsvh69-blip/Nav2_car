#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"

class PlannerDiagnostics:public SupervisorIntegration {
protected:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reports;
  void install_failure(bool stale=false) {
    reports=n->create_publisher<std_msgs::msg::String>("/mapping/planner_diagnostic",10);
    plan_server.reset();
    plan_server=rclcpp_action::create_server<Plan>(n,"/compute_path_to_pose",
      [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this,stale](auto h){
        ++plan_requests;const auto&g=*h->get_goal();std_msgs::msg::String report;
        report.data=Json{{"planner_id",g.planner_id},
          {"goal_stamp_ns",rclcpp::Time(g.goal.header.stamp).nanoseconds()+(stale?1:0)},
          {"target",{g.goal.pose.position.x,g.goal.pose.position.y}},
          {"code","START_BLOCKED"},{"reason","规划未执行：实际车位存在动态障碍"}}.dump();
        reports->publish(report);h->abort(std::make_shared<Plan::Result>());
      });
  }
  void begin(){request("start","diagnostic-start");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));}
};
TEST_F(PlannerDiagnostics, StartBlockedReturnPausesWithoutBurningTwelveDestinations) {
  begin();install_failure();request("return","blocked-start-return");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  EXPECT_EQ(plan_requests,1U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
  EXPECT_EQ(state["return_destination"]["phase"],"BLOCKED");
  EXPECT_NE(state.value("reason","").find("实际车位存在动态障碍"),std::string::npos);
  EXPECT_FALSE(state.value("save_required",true));
}
TEST_F(PlannerDiagnostics, StaleReportCannotBeAttachedToCurrentAttempt) {
  begin();install_failure(true);request("return","stale-report-return");
  ASSERT_TRUE(until([this]{return plan_requests>=2;},3))<<state;
  EXPECT_NE(state.value("phase",""),"PAUSED");
  EXPECT_EQ(state.value("navigation_detail","").find("实际车位存在动态障碍"),std::string::npos);
  request("pause","stop-stale-test");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
  EXPECT_EQ(nonzero,0U);
}
TEST_F(PlannerDiagnostics, StartBlockedExplorationReportsSpecificCauseImmediately) {
  begin();install_failure();Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=1.2;g.pose.pose.orientation.w=1;
  proxy->async_send_goal(g);
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  EXPECT_EQ(plan_requests,1U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
  EXPECT_NE(state.value("reason","").find("实际车位存在动态障碍"),std::string::npos);
}
