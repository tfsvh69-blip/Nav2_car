#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"

// 每个用例只运行隔离 DDS 域内的真实监督器和软件接口，不连接底盘、串口或电机。
class ReturnNearbyEdges : public SupervisorIntegration {
protected:
  void leave_origin() {
    request("start","return-edge-start");
    ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
    // 同步推进轮式反馈与地图位姿，避免把人为跳点伪装成真实运动。
    for(int i=1;i<=24;++i) {
      pose_x=odom_x=i*.05;
      ++slam_sequence;
      until([]{return false;},.055);
    }
    ASSERT_TRUE(until([this]{return state.value("current_pose",Json::array({0,0,0}))[0].get<double>()>1.15;},2));
  }

  std::shared_ptr<Plan::Result> straight_path(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Plan>> & goal) {
    auto result=std::make_shared<Plan::Result>();result->path.header.frame_id="map";
    geometry_msgs::msg::PoseStamped start;start.header.frame_id="map";start.pose.orientation.w=1;
    start.pose.position.x=pose_x;start.pose.position.y=pose_y;
    result->path.poses={start,goal->get_goal()->goal};return result;
  }
};

TEST_F(ReturnNearbyEdges, PausePreventsLateReturnPlanFromDispatching) {
  leave_origin();stall_plan=true;request("return","return-late-plan");
  ASSERT_TRUE(until([this]{return bool(stalled_plan);},3))<<state;
  const auto late=stalled_plan;request("pause","pause-late-return-plan");
  ASSERT_TRUE(until([&]{return late->is_canceling();},2))<<state;
  // 规划器在收到取消后才送出已算完的成功结果，监督器仍必须服从先到的暂停。
  late->succeed(straight_path(late));
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  until([]{return false;},.5);
  EXPECT_EQ(nav_requests,0U);EXPECT_EQ(plan_requests,1U);EXPECT_EQ(nonzero,0U);
  EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_TRUE(state.value("navigation_terminal",false));
  EXPECT_FALSE(state.value("save_required",true));EXPECT_EQ(state["return_result"],"running");
}

TEST_F(ReturnNearbyEdges, ResumePreservesOriginalOriginBudgetAndAttemptCount) {
  leave_origin();request("return","return-budget-before-pause");
  ASSERT_TRUE(until([this]{return active&&state.value("phase","")=="RETURNING";},3))<<state;
  const auto original=state["origin"];
  const auto attempts=state["return_destination"]["attempts"].get<size_t>();
  const double remaining=state["return_remaining_seconds"].get<double>();
  ASSERT_EQ(attempts,1U);
  request("pause","pause-return-budget");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state;
  until([]{return false;},.6);
  request("start","resume-return-budget");
  ASSERT_TRUE(until([this]{return nav_requests==2&&state.value("phase","")=="RETURNING"&&state["return_destination"].value("attempts",size_t{0})==2;},3))<<state;
  EXPECT_EQ(state["origin"],original);
  EXPECT_EQ(state["return_destination"]["attempts"],attempts+1);
  EXPECT_LT(state["return_remaining_seconds"].get<double>(),remaining-.35);
  EXPECT_EQ(state["return_result"],"running");EXPECT_FALSE(state.value("save_required",true));
  ASSERT_EQ(nav_poses.size(),2U);
  EXPECT_DOUBLE_EQ(nav_poses[0].pose.position.x,nav_poses[1].pose.position.x);
  EXPECT_DOUBLE_EQ(nav_poses[0].pose.position.y,nav_poses[1].pose.position.y);
}

TEST_F(ReturnNearbyEdges, TwelveFailedCandidatesStopWithoutAnyNavigationGoal) {
  leave_origin();plan_failures=1000;request("return","return-all-plans-fail");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="SAVE_PENDING";},10))<<state;
  EXPECT_EQ(plan_requests,12U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
  EXPECT_EQ(state["return_destination"]["attempts"],12U);
  EXPECT_EQ(state["return_destination"]["max_candidates"],12U);
  EXPECT_EQ(state["return_destination"]["phase"],"FAILED");
  EXPECT_EQ(state["return_result"],"failed");EXPECT_TRUE(state.value("save_required",false));
  EXPECT_TRUE(state.value("navigation_terminal",false));EXPECT_DOUBLE_EQ(last_velocity,0);
  until([]{return false;},.6);EXPECT_EQ(plan_requests,12U);EXPECT_EQ(nav_requests,0U);
}

TEST_F(ReturnNearbyEdges, LatePathUsesLatestMapAndUnsafeRouteIsNeverDispatched) {
  leave_origin();stall_plan=true;request("return","return-map-changes-during-plan");
  ASSERT_TRUE(until([this]{return bool(stalled_plan);},3))<<state;
  const auto late=stalled_plan;
  const auto revision=state.value("map_revision",uint64_t{0});block_route=true;
  ASSERT_TRUE(until([&]{return state.value("map_revision",uint64_t{0})>revision;},2))<<state;
  late->succeed(straight_path(late));
  ASSERT_TRUE(until([this]{return state.value("return_retries",size_t{0})>=1;},3))<<state;
  EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_NE(state.value("navigation_detail","").find("净空"),std::string::npos);
  EXPECT_EQ(state["return_result"],"running");EXPECT_FALSE(state.value("save_required",true));
}

TEST_F(ReturnNearbyEdges, PauseDuringArrivalVerificationCannotCompleteReturnLater) {
  leave_origin();origin_obstacle=true;until([]{return false;},.2);
  fake_return_success=true;succeed_any_return=true;
  request("return","return-nav-reports-success-far-away");
  ASSERT_TRUE(until([this]{return state.contains("return_destination")&&state["return_destination"].value("phase","")=="VERIFYING";},3))<<state;
  EXPECT_GT(state["current_pose"][0].get<double>(),1.15);
  EXPECT_EQ(state["return_result"],"running");EXPECT_FALSE(state.value("save_required",true));
  succeed_any_return=false;request("pause","pause-return-verification");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2))<<state;
  // 超过原核验超时后，旧成功反馈仍不得改变暂停结果或自动发起下一次目标。
  until([]{return false;},3.3);
  EXPECT_EQ(state["phase"],"PAUSED");EXPECT_EQ(state["return_result"],"running");
  EXPECT_EQ(nav_requests,1U);EXPECT_FALSE(state.value("save_required",true));
  EXPECT_TRUE(state.value("navigation_terminal",false));EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(ReturnNearbyEdges, SidewaysWheelFeedbackDoesNotCountAsStill) {
  sideways_speed=.08;
  ASSERT_TRUE(until([this]{return !state.value("still",true);},.8))<<state;
  EXPECT_DOUBLE_EQ(last_velocity,0);
  sideways_speed=0;
  ASSERT_TRUE(until([this]{return state.value("still",false);},1));
}

TEST_F(ReturnNearbyEdges, ExhaustedCandidatesWaitForAllWheelMotionBeforeSaving) {
  leave_origin();plan_failures=11;stall_after_failures=true;
  request("return","return-failed-needs-still");
  ASSERT_TRUE(until([this]{return bool(stalled_plan);},10))<<state;
  ASSERT_EQ(plan_requests,12U);
  sideways_speed=.08;
  ASSERT_TRUE(until([this]{return !state.value("still",true);},1));
  stalled_plan->abort(std::make_shared<Plan::Result>());
  ASSERT_TRUE(until([this]{return state.value("phase","")=="STOPPING";},2))<<state;
  EXPECT_FALSE(state.value("save_required",true));EXPECT_EQ(state["return_result"],"failed");
  EXPECT_DOUBLE_EQ(last_velocity,0);
  sideways_speed=0;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="SAVE_PENDING";},2))<<state;
  EXPECT_TRUE(state.value("still",false));EXPECT_TRUE(state.value("save_required",false));
}
