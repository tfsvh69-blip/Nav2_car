#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include "carcar_camera/depth_cloud_filter_factory.hpp"

namespace {
class DepthFilterComponentTest : public testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

sensor_msgs::msg::PointCloud2::UniquePtr cloud(rclcpp::Node & node, bool obstacles = true)
{
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = "fixture_frame";
  msg->header.stamp = node.now();
  msg->height = 1;
  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
  // 前两点属于同一格；第三点为地面，不得混入质心。
  for (int i = 0; i < 3; ++i, ++x, ++y, ++z) {
    *x = 0.501F + 0.001F * i; *y = 0.001F;
    *z = obstacles && i < 2 ? 0.101F : 0.001F;
  }
  return msg;
}

TEST_F(DepthFilterComponentTest, IpcDownsamplesOnlyObstaclesAndKeepsClearingAndHeaders)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true).parameter_overrides({
    rclcpp::Parameter("base_frame", "fixture_frame"),
    rclcpp::Parameter("input_topic", "/fixture/input"),
    rclcpp::Parameter("temporal_confirm_enable", true),
    rclcpp::Parameter("voxel_grid_enable", true)});
  auto filter = carcar_camera::make_depth_cloud_filter(options);
  auto fixture = std::make_shared<rclcpp::Node>("filter_fixture",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  auto publisher = fixture->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/fixture/input", rclcpp::SensorDataQoS());
  std::vector<sensor_msgs::msg::PointCloud2> obstacles, clearing;
  auto obstacle_sub = fixture->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/navigation/obstacles", rclcpp::SensorDataQoS(),
    [&obstacles](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {obstacles.push_back(*msg);});
  auto clearing_sub = fixture->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/navigation/clearing", rclcpp::SensorDataQoS(),
    [&clearing](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {clearing.push_back(*msg);});
  // 迟加入的健康订阅必须仍收到状态；该低带宽话题刻意走DDS持久化。
  rclcpp::SubscriptionOptions health_options;
  health_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  bool healthy = false;
  auto health_sub = fixture->create_subscription<std_msgs::msg::Bool>(
    "/camera/navigation/healthy", rclcpp::QoS(1).reliable().transient_local(),
    [&healthy](std_msgs::msg::Bool::ConstSharedPtr msg) {healthy = msg->data;}, health_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter); executor.add_node(fixture);
  const auto spin_until = [&executor](const std::function<bool()> & ready, double seconds) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      while (std::chrono::steady_clock::now() < end) {
        executor.spin_some();
        if (ready()) {return true;}
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      return ready();
    };
  ASSERT_TRUE(spin_until([&]() {return publisher->get_intra_process_subscription_count() == 1;}, 3));
  auto msg = cloud(*fixture);
  publisher->publish(std::move(msg));
  ASSERT_TRUE(spin_until([&]() {return obstacles.size() == 1 && clearing.size() == 1;}, 2));
  EXPECT_EQ(obstacles.back().width, 0U);  // 同帧重复点不得冒充跨帧确认。
  auto confirmed = cloud(*fixture);
  const auto header = confirmed->header;
  publisher->publish(std::move(confirmed));
  ASSERT_TRUE(spin_until([&]() {return obstacles.size() == 2 && clearing.size() == 2;}, 2));
  EXPECT_EQ(obstacles.back().width, 1U);
  EXPECT_EQ(clearing.back().width, 3U);
  EXPECT_EQ(obstacles.back().header, header);
  EXPECT_EQ(clearing.back().header, header);
  sensor_msgs::PointCloud2ConstIterator<float> z(obstacles.back(), "z");
  EXPECT_NEAR(*z, 0.101F, 1e-6);
  ASSERT_TRUE(spin_until([&]() {return healthy;}, 0.4));
  publisher->publish(cloud(*fixture, false));
  ASSERT_TRUE(spin_until([&]() {return obstacles.size() == 3 && clearing.size() == 3;}, 1));
  EXPECT_EQ(obstacles.back().width, 0U);
  EXPECT_EQ(clearing.back().width, 3U);
  // 过期数据不得续发标障，输入停止后必须撤销健康。
  auto expired = cloud(*fixture);
  expired->header.stamp.sec -= 2;
  publisher->publish(std::move(expired));
  ASSERT_TRUE(spin_until([&]() {return !healthy;}, 1));
  EXPECT_EQ(obstacles.size(), 3U);
}

TEST_F(DepthFilterComponentTest, StandaloneDefaultDoesNotEnableVoxelGrid)
{
  auto filter = carcar_camera::make_depth_cloud_filter(rclcpp::NodeOptions{});
  EXPECT_FALSE(filter->get_parameter("voxel_grid_enable").as_bool());
  EXPECT_FALSE(filter->get_node_options().use_intra_process_comms());
}
}  // namespace
