#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"

// 只启动隔离DDS域内的真实监督器与软件接口，无底盘或串口节点。
class ReturnNearby : public SupervisorIntegration {
protected:
  void leave_origin() {
    request("start", "nearby-start");
    ASSERT_TRUE(until([this] { return state.value("phase", "") == "EXPLORING"; }, 3));
    for (int i = 1; i <= 24; ++i) {
      pose_x = odom_x = i * .05;
      ++slam_sequence;
      until([] { return false; }, .055);
    }
    ASSERT_TRUE(until([this] { return state.value("current_pose", Json::array({0, 0, 0}))[0].get<double>() > 1.15; }, 2));
  }
};

TEST_F(ReturnNearby, BlockedOriginPlansNearbyInsteadOfSavingFarAway) {
  leave_origin();
  origin_obstacle = true;
  until([] { return false; }, .2);
  request("return", "origin-now-blocked");
  ASSERT_TRUE(until([this] { return nav_requests > 0; }, 3)) << state;
  ASSERT_FALSE(nav_poses.empty());
  const auto &target = nav_poses.back().pose.position;
  EXPECT_GT(std::hypot(target.x, target.y), .22);
  EXPECT_LE(std::hypot(target.x, target.y), .65 + 1e-6);
  EXPECT_EQ(state["return_destination"]["kind"], "nearby");
  EXPECT_FALSE(state.value("save_required", false));
}

TEST_F(ReturnNearby, NearbySuccessNeedsActualArrivalAndStillFeedback) {
  leave_origin();
  origin_obstacle = true;
  until([] { return false; }, .2);
  succeed_any_return = true;
  request("return", "nearby-success");
  ASSERT_TRUE(until([this] { return state.value("return_result", "") == "nearby"; }, 5)) << state;
  EXPECT_TRUE(state.value("still", false));
  EXPECT_TRUE(state.value("navigation_terminal", false));
  EXPECT_DOUBLE_EQ(last_velocity, 0);
  EXPECT_TRUE(state.value("save_required", false));
  EXPECT_EQ(state["return_destination"]["phase"], "COMPLETED");
  EXPECT_LE(state["return_destination"]["distance_to_origin_m"].get<double>(), .8);
  EXPECT_EQ(state["origin"], Json::array({0., 0., 0.}));
}

TEST_F(ReturnNearby, Nav2SuccessFarFromOriginNeverClaimsReturnSuccess) {
  leave_origin();
  origin_obstacle = true;
  until([] { return false; }, .2);
  fake_return_success = true;
  succeed_any_return = true;
  request("return", "false-nearby-success");
  ASSERT_TRUE(until([this] { return nav_requests > 0; }, 3)) << state;
  until([] { return false; }, 1);
  EXPECT_NE(state.value("return_result", ""), "succeeded");
  EXPECT_NE(state.value("return_result", ""), "nearby");
  EXPECT_GT(state["current_pose"][0].get<double>(), 1.15);
}

TEST_F(ReturnNearby, NoSafeNearbyStopSavesWithExplicitReasonAndNoGoal) {
  leave_origin();
  fill_return_neighborhood = true;
  until([] { return false; }, .2);
  request("return", "all-nearby-blocked");
  ASSERT_TRUE(until([this] { return state.value("return_result", "") == "failed"; }, 4)) << state;
  EXPECT_EQ(nav_requests, 0U);
  EXPECT_EQ(plan_requests, 0U);
  EXPECT_EQ(state["return_destination"]["phase"], "FAILED");
  EXPECT_NE(state["return_destination"]["reason"].get<std::string>().find("附近"), std::string::npos);
  EXPECT_TRUE(state.value("save_required", false));
  EXPECT_DOUBLE_EQ(last_velocity, 0);
}

TEST_F(ReturnNearby, ExactOriginRemainsPreferred) {
  leave_origin();
  succeed_any_return = true;
  request("return", "exact-origin-success");
  ASSERT_TRUE(until([this] { return state.value("return_result", "") == "succeeded"; }, 4)) << state;
  ASSERT_FALSE(nav_poses.empty());
  EXPECT_DOUBLE_EQ(nav_poses.front().pose.position.x, 0);
  EXPECT_DOUBLE_EQ(nav_poses.front().pose.position.y, 0);
  EXPECT_EQ(state["return_destination"]["kind"], "origin");
  EXPECT_DOUBLE_EQ(last_velocity, 0);
}

TEST_F(ReturnNearby, RealPlannerFailureTriesAnotherSafeTarget) {
  leave_origin();
  plan_failures = 1;
  succeed_any_return = true;
  request("return", "planner-blocks-exact");
  ASSERT_TRUE(until([this] { return state.value("return_result", "") == "nearby"; }, 6)) << state;
  EXPECT_GE(plan_requests, 2U);
  EXPECT_EQ(nav_requests, 1U);
  EXPECT_EQ(state["return_destination"]["kind"], "nearby");
}
