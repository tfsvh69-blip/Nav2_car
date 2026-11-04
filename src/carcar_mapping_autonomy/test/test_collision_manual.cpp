#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
#include <filesystem>
#include <fstream>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

// 使用真实Humble近场监视器；全部速度只存在于域227的虚拟传感器链路。
class RealCollisionManualIntegration : public SupervisorIntegration {
protected:
  pid_t monitor_pid_ = -1;
  std::filesystem::path monitor_root_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr monitored_output_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_input_;
  rclcpp::TimerBase::SharedPtr manual_timer_;
  double desired_manual_ = 0;
  size_t monitor_messages_ = 0, monitor_nonzero_ = 0;

  void SetUp() override {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "227");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    recovery_mode = restricted_manual_enabled = true;
    slam_sequence = 13;
    SupervisorIntegration::SetUp();
    if (HasFatalFailure()) return;
    // 移除夹具的逐条回显，否则会掩盖官方监视器停止重复零输出的行为。
    input.reset();
    collision.reset();
    monitored_output_ = n->create_subscription<geometry_msgs::msg::Twist>(
      "/mapping/collision_output", 1, [this](const geometry_msgs::msg::Twist & velocity) {
        ++monitor_messages_;
        if (std::abs(velocity.linear.x) + std::abs(velocity.angular.z) > 1e-5) ++monitor_nonzero_;
      });
    manual_input_ = n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel", 1);
    manual_timer_ = n->create_wall_timer(std::chrono::milliseconds(50), [this] {
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = desired_manual_;
      manual_input_->publish(velocity);
    });
    monitor_root_ = std::filesystem::temp_directory_path() /
      ("carcar-collision-manual-" + std::to_string(getpid()));
    std::filesystem::create_directories(monitor_root_);
    const auto config = monitor_root_ / "collision.yaml";
    std::ofstream(config) << R"(collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: /mapping/collision_input
    cmd_vel_out_topic: /mapping/collision_output
    transform_tolerance: 0.1
    source_timeout: 0.5
    base_shift_correction: true
    stop_pub_timeout: 0.0
    polygons: [Stop]
    Stop:
      type: polygon
      points: [0.14, 0.13, 0.14, -0.13, -0.14, -0.13, -0.14, 0.13]
      action_type: stop
      max_points: 0
      visualize: true
      polygon_pub_topic: /mapping/collision_stop_polygon
      enabled: true
    observation_sources: [scan]
    scan:
      type: scan
      topic: /scan
      enabled: true
)";
    std::vector<std::string> arguments{
      "/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor",
      "--ros-args", "--params-file", config.string()};
    std::vector<char *> raw;
    for (auto & argument : arguments) raw.push_back(argument.data());
    raw.push_back(nullptr);
    ASSERT_EQ(posix_spawn(&monitor_pid_, raw.front(), nullptr, nullptr, raw.data(), environ), 0);
    transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (HasFatalFailure()) return;
    transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (HasFatalFailure()) return;
    // 替换发布者期间可按原保护切入冻结的本地接管；测试要求真实手动候选许可，
    // 不把全局地图重新健康当作移车前提，最终速度仍须等真实近场反馈。
    ASSERT_TRUE(until([this] {
      return state.value("phase", std::string()) == "MANUAL" &&
        state.value("manual_policy", Json::object()).value("motion_allowed", false);
    }, 4)) << state.dump();
  }

  void transition(uint8_t id) {
    auto client = n->create_client<lifecycle_msgs::srv::ChangeState>("/collision_monitor/change_state");
    ASSERT_TRUE(until([&] {return client->service_is_ready();}, 5));
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = id;
    auto future = client->async_send_request(request);
    ASSERT_TRUE(until([&] {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;}, 4));
    ASSERT_TRUE(future.get()->success) << "隔离近场监视器生命周期转换失败，transition=" << unsigned(id);
  }

  void TearDown() override {
    manual_timer_.reset();
    manual_input_.reset();
    monitored_output_.reset();
    if (monitor_pid_ > 0) {
      // 测试可能停在SIGSTOP故障注入阶段，先解除暂停才能正常接收退出信号。
      kill(monitor_pid_, SIGCONT);
      kill(monitor_pid_, SIGINT);
      bool exited = false;
      for (int i = 0; i < 100; ++i) {
        if (waitpid(monitor_pid_, nullptr, WNOHANG) == monitor_pid_) {exited = true; break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!exited) {kill(monitor_pid_, SIGKILL); waitpid(monitor_pid_, nullptr, 0);}
      monitor_pid_ = -1;
    }
    SupervisorIntegration::TearDown();
    if (!monitor_root_.empty()) std::filesystem::remove_all(monitor_root_);
  }
};

TEST_F(RealCollisionManualIntegration, RestartsAfterSuppressedZeroAndStopsWithoutFreshMonitorOrScan) {
  // 给零输出抑制和旧反馈过期留出足够时间，随后确认输出话题确实不再回显零。
  until([] {return false;}, .7);
  const auto stopped_messages = monitor_messages_;
  until([] {return false;}, .6);
  ASSERT_EQ(monitor_messages_, stopped_messages);
  ASSERT_EQ(last_velocity, 0);
  ASSERT_EQ(nonzero, 0U);
  ASSERT_EQ(monitor_nonzero_, 0U);

  desired_manual_ = .05;
  ASSERT_TRUE(until([this] {return monitor_nonzero_ > 0 && last_velocity > .01;}, 3))
    << "静止后手动候选应经过真实近场监视器重新取得安全反馈：" << state.dump();
  EXPECT_LE(last_velocity, .05);
  const auto messages_before_failure = monitor_messages_;
  ASSERT_EQ(kill(monitor_pid_, SIGSTOP), 0);
  ASSERT_TRUE(until([this] {return std::abs(last_velocity) < 1e-8;}, .7))
    << "监视器不回应时旧的非零反馈不得继续驱动最终输出";
  until([] {return false;}, .3);
  EXPECT_LE(monitor_messages_, messages_before_failure + 1);
  EXPECT_EQ(last_velocity, 0);

  // 反馈故障须锁存；监视器恢复不能把故障前一直按住的手势当作新授权。
  const auto held_nonzero = nonzero;
  ASSERT_EQ(kill(monitor_pid_, SIGCONT), 0);
  until([] {return false;}, .6);
  EXPECT_EQ(last_velocity, 0) << "近场反馈恢复后，旧的持续按住手势必须保持停车";
  EXPECT_EQ(nonzero, held_nonzero) << "持续按住期间不得出现短暂的非零输出";
  desired_manual_ = 0;
  until([] {return false;}, .4);
  desired_manual_ = .05;
  ASSERT_TRUE(until([this] {return last_velocity > .01;}, 3)) << state.dump();
  feed_scan = false;
  ASSERT_TRUE(until([this] {return std::abs(last_velocity) < 1e-8;}, .9))
    << "真实扫描失鲜时监督器必须停车";
  until([] {return false;}, .3);
  EXPECT_EQ(last_velocity, 0);
}
