#include <chrono>
#include <csignal>
#include <spawn.h>
#include <thread>
#include <vector>
#include <sys/wait.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

extern char ** environ;

namespace {
struct PreviewProcess
{
  pid_t pid{0};
  ~PreviewProcess()
  {
    if (pid > 0) {kill(pid, SIGINT); waitpid(pid, nullptr, 0);}
    rclcpp::shutdown();
  }
};
}

// 真实进程回归：收到非空帧后必须转发空帧，避免显示残留；畸形缓冲仍拒绝。
TEST(DepthCloudPreview, DeliversEmptyUpdateAfterOccupiedCloud)
{
  rclcpp::init(0, nullptr);
  PreviewProcess process;
  auto node = std::make_shared<rclcpp::Node>("preview_fixture");
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/preview_fixture/input", rclcpp::SensorDataQoS());
  std::vector<sensor_msgs::msg::PointCloud2> received;
  auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/preview_fixture/output", rclcpp::SensorDataQoS(),
    [&received](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {received.push_back(*msg);});
  const char * args[] = {CLOUD_PREVIEW_EXECUTABLE, "--ros-args",
    "-p", "input_topic:=/preview_fixture/input", "-p", "output_topic:=/preview_fixture/output",
    "-p", "max_points:=2", "-p", "max_fps:=10.0", nullptr};
  ASSERT_EQ(posix_spawn(&process.pid, CLOUD_PREVIEW_EXECUTABLE, nullptr, nullptr,
    const_cast<char **>(args), environ), 0);
  const auto spin_until = [&node](const auto & ready, double seconds) {
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        if (ready()) {return true;}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return ready();
    };
  ASSERT_TRUE(spin_until([&]() {
    return publisher->get_subscription_count() > 0 && subscription->get_publisher_count() > 0;
  }, 5));
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "fixture_frame";
  cloud.header.stamp = node->now();
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(5);
  publisher->publish(cloud);
  ASSERT_TRUE(spin_until([&]() {return received.size() == 1;}, 2));
  EXPECT_EQ(received.back().width, 2U);
  EXPECT_EQ(received.back().header, cloud.header);
  EXPECT_EQ(received.back().fields, cloud.fields);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  modifier.resize(0);
  cloud.header.stamp = node->now();
  publisher->publish(cloud);
  EXPECT_TRUE(spin_until([&]() {return received.size() == 2;}, 2));
  if (received.size() == 2) {
    EXPECT_EQ(received.back().width, 0U);
    EXPECT_TRUE(received.back().data.empty());
    EXPECT_EQ(received.back().header, cloud.header);
    EXPECT_EQ(received.back().fields, cloud.fields);
  }
  const auto valid_count = received.size();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  modifier.resize(5);
  cloud.data.clear();
  publisher->publish(cloud);
  EXPECT_FALSE(spin_until([&]() {return received.size() > valid_count;}, 0.5));
}
