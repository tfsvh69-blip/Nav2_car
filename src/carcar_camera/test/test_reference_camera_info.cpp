#include <chrono>
#include <csignal>
#include <cstring>
#include <spawn.h>
#include <thread>
#include <sys/wait.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "carcar_camera/reference_camera_info.hpp"
extern char ** environ;

// 通过发行版配准进程验证目标网格/已知3D点投影，避免跨OpenCV ABI调用。
TEST(ReferenceCameraInfo, ActualRegistrationUsesRGBPreviewGrid)
{
  rclcpp::init(0, nullptr);
  struct Process {
    pid_t pid{0};
    ~Process() {if (pid > 0) {kill(pid, SIGINT); waitpid(pid, nullptr, 0);} rclcpp::shutdown();}
  } process;
  auto node = std::make_shared<rclcpp::Node>("registration_fixture");
  const auto qos = rclcpp::QoS(5).reliable();
  auto depth_pub = node->create_publisher<sensor_msgs::msg::Image>("/fixture/depth/image_rect", qos);
  auto depth_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("/fixture/depth/camera_info", qos);
  auto rgb_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("/fixture/rgb/camera_info", qos);
  auto tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).transient_local());
  sensor_msgs::msg::Image::ConstSharedPtr received;
  auto sub = node->create_subscription<sensor_msgs::msg::Image>("/fixture/output/image_rect", qos,
    [&received](sensor_msgs::msg::Image::ConstSharedPtr msg) {received = msg;});
  const char * args[] = {REGISTER_EXECUTABLE, "--ros-args", "-r", "__node:=registration_under_test",
    "-r", "depth/image_rect:=/fixture/depth/image_rect", "-r", "depth/camera_info:=/fixture/depth/camera_info",
    "-r", "rgb/camera_info:=/fixture/rgb/camera_info", "-r", "depth_registered/image_rect:=/fixture/output/image_rect",
    "-r", "depth_registered/camera_info:=/fixture/output/camera_info", nullptr};
  ASSERT_EQ(posix_spawn(&process.pid, REGISTER_EXECUTABLE, nullptr, nullptr, const_cast<char **>(args), environ), 0);
  sensor_msgs::msg::CameraInfo depth_info;
  depth_info.width = 640; depth_info.height = 480;
  depth_info.header.frame_id = "fixture_depth";
  depth_info.distortion_model = "plumb_bob"; depth_info.d.assign(5, 0);
  depth_info.k = {400, 0, 320, 0, 400, 240, 0, 0, 1};
  depth_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  depth_info.p = {400, 0, 320, 0, 0, 400, 240, 0, 0, 0, 1, 0};
  auto rgb_info = depth_info;
  rgb_info.header.frame_id = "fixture_color";
  rgb_info.k[0] = rgb_info.k[4] = rgb_info.p[0] = rgb_info.p[5] = 600;
  auto target_info = carcar_camera::reference_camera_info(rgb_info, 320);
  EXPECT_EQ(rgb_info.binning_x, 0U);
  EXPECT_EQ(target_info.header.frame_id, rgb_info.header.frame_id);
  EXPECT_THROW(carcar_camera::reference_camera_info(rgb_info, 400), std::invalid_argument);
  sensor_msgs::msg::Image depth;
  depth.header.frame_id = "fixture_depth";
  depth.width = 640; depth.height = 480; depth.step = 1280; depth.encoding = "16UC1";
  depth.data.assign(depth.step * depth.height, 0);
  const uint16_t millimetres = 1000;
  std::memcpy(depth.data.data() + 240 * depth.step + 320 * 2, &millimetres, 2);
  tf2_msgs::msg::TFMessage transform;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "fixture_color"; tf.child_frame_id = "fixture_depth";
  tf.transform.translation.x = .02; tf.transform.rotation.w = 1;
  transform.transforms.push_back(tf); tf_pub->publish(transform);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(7);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    depth.header.stamp = node->now(); depth_info.header.stamp = target_info.header.stamp = depth.header.stamp;
    depth_info_pub->publish(depth_info); rgb_info_pub->publish(target_info); depth_pub->publish(depth);
    rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_TRUE(received);
  ASSERT_EQ(received->width, 320U); ASSERT_EQ(received->height, 240U);
  EXPECT_EQ(received->header.frame_id, "fixture_color");
  // x=0.02m,z=1m -> (600*0.02+320)/2=166，v=240/2=120。
  uint16_t actual = 0;
  std::memcpy(&actual, received->data.data() + 120 * received->step + 166 * 2, 2);
  EXPECT_EQ(actual, millimetres);
}
