#include <chrono>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
extern char ** environ;
// 真实 mapping_slam 进程：静止时必须超过首帧，并在补帧上限后停止增长；移动后原门槛仍能再接纳。
TEST(StartupSlam, StationaryProcessAcceptsBoundedScansThenMotionStillPasses){
  ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : ""), "207");
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("startup_slam_fixture");
  tf2_ros::TransformBroadcaster tf(*node);
  tf2_ros::StaticTransformBroadcaster fixed(*node);
  geometry_msgs::msg::TransformStamped laser;
  laser.header.frame_id = "base_footprint";
  laser.child_frame_id = "laser_frame";
  laser.transform.translation.z = .217;
  laser.transform.rotation.w = 1;
  fixed.sendTransform(laser);
  auto scans = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
  nlohmann::json status = nlohmann::json::object();
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/mapping/slam/status", rclcpp::QoS(1).reliable().transient_local(),
    [&](const std_msgs::msg::String & message){status = nlohmann::json::parse(message.data);});
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  auto map_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),
    [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){grid=m;});
  std::filesystem::create_directories("log/nav018_startup_accumulation");
  const int log_fd = open("log/nav018_startup_accumulation/slam.log", O_CREAT | O_TRUNC | O_WRONLY, 0644);
  ASSERT_GE(log_fd, 0);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, log_fd, STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, log_fd, STDERR_FILENO);
  char executable[] = SLAM_EXE;
  char ros[] = "--ros-args", parameter[] = "-p";
  char saver[] = "use_map_saver:=false", travel[] = "minimum_travel_distance:=0.10";
  char interval[] = "minimum_time_interval:=0.30", extras[] = "startup_stationary_scans:=3";
  char window[] = "startup_stationary_window:=2.0", range[] = "max_laser_range:=8.0";
  char minimum[] = "min_laser_range:=0.05";
  char map_period[] = "map_update_interval:=0.20";
  char * args[] = {executable, ros, parameter, saver, parameter, travel, parameter, interval,
    parameter, extras, parameter, window, parameter, range, parameter, minimum, parameter, map_period, nullptr};
  pid_t pid = -1;
  ASSERT_EQ(posix_spawn(&pid, executable, &actions, nullptr, args, environ), 0);
  posix_spawn_file_actions_destroy(&actions);
  close(log_fd);
  double x = 0;
  int tick = 0;
  bool moved = false;
  const auto began = std::chrono::steady_clock::now();
  auto elapsed = [&]{return std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();};
  uint64_t settled = 0;
  double settled_at = -1;
  auto spin_until = [&](auto done, double timeout){
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::steady_clock::now() < end) {
      const auto now = node->now();
      geometry_msgs::msg::TransformStamped odom;
      odom.header.stamp = now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.transform.translation.x = x;
      odom.transform.rotation.w = 1;
      tf.sendTransform(odom);
      if (tick % 2 == 0) {
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now;
        scan.header.frame_id = "laser_frame";
        scan.angle_min = -M_PI;
        scan.angle_increment = 2 * M_PI / 360;
        scan.angle_max = scan.angle_min + 360 * scan.angle_increment;
        scan.range_min = .05f;
        scan.range_max = 12.f;
        scan.ranges.assign(361, 2.f);
        scan.ranges[20] = 1.7f + .02f * (tick % 5);
        scans->publish(scan);
      }
      ++tick;
      rclcpp::spin_some(node);
      if (done()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    rclcpp::spin_some(node);
    return done();
  };
  const bool accumulated = spin_until([&]{
    const auto sequence = status.value("sequence", uint64_t{0});
    if (sequence >= 4 && settled_at < 0) {settled = sequence; settled_at = elapsed();}
    return settled_at >= 0 && elapsed() - settled_at > 1.2;
  }, 8);
  const auto after_still = status;
  const bool map_ready=spin_until([&]{return static_cast<bool>(grid);},3.5);
  const unsigned stationary_width=grid?grid->info.width:0,stationary_height=grid?grid->info.height:0;
  const bool advanced = spin_until([&]{
    if (!moved && elapsed() > settled_at + 1.2) {x = .25; moved = true;}
    return moved && status.value("sequence", uint64_t{0}) >= settled + 1;
  }, 4);
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  for (int i = 0; i < 8; ++i) {rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  const auto after_motion = status;
  const auto before_micro=status.value("sequence",uint64_t{0});
  x=.345;spin_until([]{return false;},.7);
  const auto after_rejected_micro=status;
  x=.355;
  const bool micro_admitted=spin_until([&]{return status.value("sequence",uint64_t{0})>before_micro;},1.5);
  const auto after_micro=status;
  if (pid > 0) {
    kill(pid, SIGINT);
    for (int i = 0; i < 50 && waitpid(pid, nullptr, WNOHANG) == 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (waitpid(pid, nullptr, WNOHANG) == 0) {kill(pid, SIGKILL); waitpid(pid, nullptr, 0);}
  }
  std::ifstream log("log/nav018_startup_accumulation/slam.log");
  std::string tail((std::istreambuf_iterator<char>(log)), std::istreambuf_iterator<char>());
  if (tail.size() > 1200) tail.erase(0, tail.size() - 1200);
  EXPECT_TRUE(accumulated) << after_still.dump() << '\n' << tail;
  EXPECT_TRUE(map_ready);EXPECT_GE(stationary_width,81U)<<"2m半径在0.05m栅格中必须容纳最大端点索引80";
  EXPECT_GE(stationary_height,81U)<<"真实mapping_slam发布不能漏掉最大y端点";
  EXPECT_EQ(after_still.value("sequence", uint64_t{0}), 4u) << after_still.dump();
  EXPECT_EQ(after_still.value("startup_extras", -1), 3);
  EXPECT_TRUE(after_still.value("startup_closed", false));
  EXPECT_TRUE(advanced) << after_motion.dump() << '\n' << tail;
  EXPECT_GE(after_motion.value("sequence", uint64_t{0}), 5u);
  EXPECT_LE(after_motion.value("sequence", uint64_t{0}), 6u) << after_motion.dump();
  EXPECT_EQ(after_rejected_micro.value("sequence",uint64_t{0}),before_micro)<<after_rejected_micro.dump();
  EXPECT_TRUE(micro_admitted)<<"外层0.095米尝试被Karto拒绝后，达到0.105米应按最后接纳基准入图："<<after_micro.dump();
  rclcpp::shutdown();
}
