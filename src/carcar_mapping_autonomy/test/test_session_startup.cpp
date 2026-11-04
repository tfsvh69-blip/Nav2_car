#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
#include <filesystem>

// 只替代传感器与恢复服务；会话状态由真实网关产生，并交给真实监督器消费。
class GatewaySessionStartupIntegration : public SupervisorIntegration {
protected:
  pid_t gateway_pid_ = -1;
  std::filesystem::path gateway_root_;
  Json gateway_session_ = Json::object();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gateway_session_sub_;

  void SetUp() override {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "228");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    publish_session = false;
    live_mode = recovery_mode = online_snapshot_mode = restricted_manual_enabled = true;
    automatic_on_boot = true;
    slam_sequence = 13;
    recovery_state.update({{"phase", "WARMING"}, {"checkpoint_id", ""}});
    gateway_root_ = std::filesystem::temp_directory_path() /
      ("carcar-session-startup-" + std::to_string(getpid()));
    std::filesystem::create_directories(gateway_root_);
    const char * override_gateway = std::getenv("CARCAR_SESSION_GATEWAY_EXE");
    std::vector<std::string> arguments{
      override_gateway ? override_gateway : CARCAR_STARTUP_GATEWAY_EXE,
      "--ros-args", "-p", "bind_address:=127.0.0.1", "-p", "port:=18228",
      "-p", "mode:=new", "-p", "autonomy_enabled:=true",
      "-p", "backend_hardware:=false", "-p", "manage_inspection:=false",
      "-p", "task_id:=live-supervisor-fixture", "-p", "control_mode:=automatic",
      "-p", std::string("web_root:=") + CARCAR_STARTUP_WEB_ROOT,
      "-p", "map_root:=" + gateway_root_.string(),
      "-p", "log_root:=" + (gateway_root_ / "logs").string()};
    std::vector<char *> raw;
    for (auto & argument : arguments) raw.push_back(argument.data());
    raw.push_back(nullptr);
    ASSERT_EQ(posix_spawn(&gateway_pid_, raw.front(), nullptr, nullptr, raw.data(), environ), 0);
    SupervisorIntegration::SetUp();
    if (HasFatalFailure()) return;
    gateway_session_sub_ = n->create_subscription<std_msgs::msg::String>(
      "/mapping/session/state", 1, [this](const std_msgs::msg::String & message) {
        gateway_session_ = Json::parse(message.data);
      });
  }

  void TearDown() override {
    gateway_session_sub_.reset();
    // 先退出真实监督器，再退出网关，测试结束期间也不留下会话发布者。
    SupervisorIntegration::TearDown();
    if (gateway_pid_ > 0) {
      kill(gateway_pid_, SIGINT);
      bool exited = false;
      for (int i = 0; i < 100; ++i) {
        if (waitpid(gateway_pid_, nullptr, WNOHANG) == gateway_pid_) {exited = true; break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!exited) {kill(gateway_pid_, SIGKILL); waitpid(gateway_pid_, nullptr, 0);}
      gateway_pid_ = -1;
    }
    if (!gateway_root_.empty()) std::filesystem::remove_all(gateway_root_);
  }
};

TEST_F(GatewaySessionStartupIntegration, PendingAutomaticStartupCreatesCheckpointWithoutManualMotionPermission) {
  ASSERT_TRUE(until([this] {
    return state.value("auto_start_pending", false) && state.contains("manual_policy") &&
      gateway_session_.contains("mapping_ready") &&
      gateway_session_.value("mapping_ready", false) &&
      !gateway_session_.value("ready", true);
  }, 3)) << "真实网关应分别发布地图会话就绪与手动运动许可：" << gateway_session_.dump()
         << "；车端：" << state.dump();
  // 不能只观察启动时尚无手动策略的短暂心跳；稳定闭环后仍须保持健康。
  until([] {return false;}, .7);
  ASSERT_TRUE(state.value("healthy", false)) << state.dump();
  EXPECT_TRUE(recovery_state.value("checkpoint_id", std::string()).empty());
  EXPECT_FALSE(state.value("checkpoint_usable", true));
  EXPECT_EQ(state.value("phase", std::string()), "MANUAL");
  EXPECT_EQ(nonzero, 0U);
  EXPECT_EQ(nav_requests, 0U);

  ASSERT_TRUE(until([this] {
    return std::find(recovery_actions.begin(), recovery_actions.end(), "snapshot") != recovery_actions.end();
  }, 3)) << "首检查点请求必须从真实监督器发出：" << state.dump();
  ASSERT_TRUE(recovery_state.contains("snapshot"));
  ASSERT_FALSE(recovery_state["snapshot"].value("request_id", std::string()).empty());
  // 收到真实请求以后才模拟恢复模块提交检查点，禁止以预置检查点掩盖启动循环。
  recovery_state["checkpoint_id"] = "session-startup-verified";
  recovery_state["phase"] = "CHECKPOINT_READY";
  recovery_state["snapshot"]["phase"] = "READY";
  recovery_state["snapshot"]["in_flight"] = false;
  ASSERT_TRUE(until([this] {
    return state.value("phase", std::string()) == "EXPLORING" &&
      !state.value("auto_start_pending", true) && state.value("checkpoint_usable", false);
  }, 4)) << state.dump();
  EXPECT_TRUE(state.value("healthy", false));
  EXPECT_TRUE(state.value("anchored", false));
  EXPECT_EQ(nav_requests, 0U) << "进入探索阶段不等于已下发导航目标";
  EXPECT_EQ(nonzero, 0U) << "本用例没有任何运动目标或遥控手势";
}
