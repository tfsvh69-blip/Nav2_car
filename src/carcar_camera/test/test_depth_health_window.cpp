#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <spawn.h>
#include <string>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

extern char ** environ;

// 使用合成消息和隔离域，验证真实监控进程能锁存异常，且中断不成为通过。
TEST(DepthHealthWindow, CompleteHealthyGapDiagnosticAndInterruptedWindows)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("health_fixture");
  auto raw = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/health_fixture/points", rclcpp::SensorDataQoS());
  auto diag = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/health_fixture/diagnostics", 100);
  for (int scenario = 0; scenario < 4; ++scenario) {
    const auto dir = std::filesystem::temp_directory_path() /
      ("carcar_health_fixture_" + std::to_string(getpid()) + "_" + std::to_string(scenario));
    std::filesystem::create_directories(dir);
    const std::string output = "output_dir:=" + (dir / "window").string();
    const char * arguments[] = {HEALTH_WINDOW_EXECUTABLE, "--ros-args", "-p", output.c_str(),
      "-p", "duration_s:=10.0", "-p", "min_hz:=5.0", "-p", "max_gap_s:=0.5",
      "-r", "/camera/depth/points:=/health_fixture/points",
      "-r", "/camera/navigation/diagnostics:=/health_fixture/diagnostics", nullptr};
    posix_spawn_file_actions_t actions;
    ASSERT_EQ(posix_spawn_file_actions_init(&actions), 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO,
      (dir / "process.log").c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0600);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    pid_t child = 0;
    ASSERT_EQ(posix_spawn(&child, HEALTH_WINDOW_EXECUTABLE, &actions, nullptr,
      const_cast<char **>(arguments), environ), 0);
    posix_spawn_file_actions_destroy(&actions);
    const auto start = std::chrono::steady_clock::now();
    bool interrupted = false, exited = false;
    int status = 0;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(25)) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (!(scenario == 1 && elapsed >= 6 && elapsed < 7)) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = node->now(); cloud.width = cloud.height = 1;
        raw->publish(cloud);
      }
      diagnostic_msgs::msg::DiagnosticArray health;
      health.header.stamp = node->now();
      diagnostic_msgs::msg::DiagnosticStatus value;
      value.name = "D435DepthFilter";
      value.level = scenario == 2 && elapsed >= 6 && elapsed < 7 ? 2 : 0;
      diagnostic_msgs::msg::KeyValue reason;
      reason.key = "reason_code";
      reason.value = value.level ? "POINTCLOUD_TIMEOUT" : "NONE";
      value.values.push_back(reason); health.status.push_back(value); diag->publish(health);
      if (scenario == 3 && elapsed >= 6 && !interrupted) {
        kill(child, SIGINT); interrupted = true;
      }
      rclcpp::spin_some(node);
      if (waitpid(child, &status, WNOHANG) == child) {exited = true; break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!exited) {kill(child, SIGTERM); waitpid(child, &status, 0);}
    EXPECT_TRUE(exited) << dir;
    EXPECT_TRUE(WIFEXITED(status)) << dir;
    EXPECT_EQ(WEXITSTATUS(status), scenario == 0 ? 0 : 1) << dir;
    std::ifstream input(dir / "window" / "summary.txt");
    const std::string summary((std::istreambuf_iterator<char>(input)), {});
    EXPECT_NE(summary.find(scenario == 0 ? "verdict=PASS" : "verdict=FAIL"), std::string::npos)
      << summary << dir;
    EXPECT_NE(summary.find(scenario == 3 ? "completed=0" : "completed=1"), std::string::npos)
      << summary << dir;
  }
  rclcpp::shutdown();
}
