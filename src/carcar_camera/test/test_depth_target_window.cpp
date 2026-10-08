#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <limits>
#include <spawn.h>
#include <sstream>
#include <string>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

extern char ** environ;

namespace {
double value(const std::string & text, const std::string & key)
{
  const auto offset = text.find(key + "=");
  if (offset == std::string::npos) {return NAN;}
  return std::stod(text.substr(offset + key.size() + 1));
}
}

// 合成消息验证真实进程：原始/过滤独立计数，过期锁存，无TF不能形成有效取证。
TEST(DepthTargetWindow, CountsFilteredLossRejectsStaleAndMissingTransform)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("target_fixture");
  auto raw = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/target_fixture/raw", rclcpp::SensorDataQoS());
  auto filtered = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/target_fixture/filtered", rclcpp::SensorDataQoS());
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "base_footprint";
  tf.child_frame_id = "target_fixture_camera";
  tf.transform.rotation.w = 1;
  tf.header.stamp = node->now();
  broadcaster.sendTransform(tf);
  for (int scenario = 0; scenario < 3; ++scenario) {
    const auto dir = std::filesystem::temp_directory_path() /
      ("carcar_target_fixture_" + std::to_string(getpid()) + "_" + std::to_string(scenario));
    std::filesystem::create_directories(dir);
    const std::string output = "output_dir:=" + (dir / "window").string();
    const char * args[] = {TARGET_WINDOW_EXECUTABLE, "--ros-args", "-p", output.c_str(),
      "-p", "duration_s:=10.0", "-p", "center_x:=0.65", "-p", "center_y:=0.0",
      "-r", "/camera/depth/points:=/target_fixture/raw",
      "-r", "/camera/navigation/obstacles:=/target_fixture/filtered", nullptr};
    posix_spawn_file_actions_t actions;
    ASSERT_EQ(posix_spawn_file_actions_init(&actions), 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO,
      (dir / "process.log").c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0600);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    pid_t child = 0;
    ASSERT_EQ(posix_spawn(&child, TARGET_WINDOW_EXECUTABLE, &actions, nullptr,
      const_cast<char **>(args), environ), 0);
    posix_spawn_file_actions_destroy(&actions);
    const auto start = std::chrono::steady_clock::now();
    int status = 0;
    bool exited = false;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(25)) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = scenario == 2 ? "missing_target_frame" : tf.child_frame_id;
      cloud.header.stamp = node->now();
      if (scenario == 1 && elapsed > 5 && elapsed < 6) {
        cloud.header.stamp = node->now() - rclcpp::Duration::from_seconds(1);
      }
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(6);
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (int i = 0; i < 6; ++i, ++x, ++y, ++z) {
        *x = i == 3 ? 0.9F : 0.65F;
        *y = 0;
        *z = i == 4 ? 0.2F : i == 5 ? std::numeric_limits<float>::quiet_NaN() : 0.02F;
      }
      raw->publish(cloud);
      // 保持过滤流频率，后半窗为空：不能把有消息等同于有目标。
      if (scenario == 0 && elapsed > 5) {modifier.resize(0);}
      filtered->publish(cloud);
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
    EXPECT_EQ(value(summary, "measurement_valid"), scenario == 0 ? 1 : 0) << summary;
    EXPECT_EQ(value(summary, "completed"), scenario == 2 ? 0 : 1) << summary;
    if (scenario == 0) {
      EXPECT_EQ(value(summary, "raw_fraction_ge3"), 1) << summary;
      EXPECT_GT(value(summary, "filtered_fraction_ge3"), 0.3) << summary;
      EXPECT_LT(value(summary, "filtered_fraction_ge3"), 0.7) << summary;
      EXPECT_GT(value(summary, "filtered_longest_without_ge3_s"), 4) << summary;
      std::ifstream frames(dir / "window" / "frames.csv");
      std::string line;
      size_t checked = 0;
      while (std::getline(frames, line)) {
        if (line.rfind("RUN,raw,", 0) != 0) {continue;}
        std::istringstream row(line);
        std::string cell;
        for (int column = 0; column <= 8; ++column) {std::getline(row, cell, ',');}
        EXPECT_EQ(std::stoi(cell), 3) << line;
        ++checked;
      }
      EXPECT_GT(checked, 50U);
    }
    if (scenario == 1) {EXPECT_GT(value(summary, "raw_unusable"), 0) << summary;}
  }
  rclcpp::shutdown();
}
