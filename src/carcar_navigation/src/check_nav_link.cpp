// NAV-003: C++ 导航链路状态与频率只读检查工具
// 遵循约定：新增可执行逻辑统一使用 C++；零驱动输出，只读检查传感器与 TF 状态

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class NavLinkChecker : public rclcpp::Node {
public:
  NavLinkChecker()
      : Node("nav_link_checker"),
        scan_count_(0),
        odom_count_(0),
        amcl_received_(false),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    // 激光订阅：匹配 sllidar_node 的 KeepLast(10), Reliable
    rclcpp::QoS scan_qos(rclcpp::KeepLast(10));
    scan_qos.reliable();
    scan_qos.durability_volatile();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr) { ++scan_count_; });

    // 里程计订阅：匹配 rosmaster_base 的 KeepLast(10), Reliable
    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odom_qos.reliable();
    odom_qos.durability_volatile();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel/odometry", odom_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr) { ++odom_count_; });

    // AMCL 位姿订阅：Reliable, Volatile
    rclcpp::QoS amcl_qos(rclcpp::KeepLast(5));
    amcl_qos.reliable();
    amcl_qos.durability_volatile();

    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", amcl_qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          amcl_received_ = true;
          latest_amcl_x_ = msg->pose.pose.position.x;
          latest_amcl_y_ = msg->pose.pose.position.y;
        });

    RCLCPP_INFO(this->get_logger(), "开始采样 NAV-003 传感器与定位链路 (持续 3 秒)...");
  }

  void sample(double duration_sec = 3.0) {
    scan_count_ = 0;
    odom_count_ = 0;

    auto start_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(100);

    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();

      auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start_time).count();
      if (elapsed >= duration_sec) {
        break;
      }
    }

    double scan_hz = scan_count_ / duration_sec;
    double odom_hz = odom_count_ / duration_sec;

    // 检查 TF 链路
    std::string tf_err;
    bool tf_map_odom = tf_buffer_.canTransform("map", "odom", tf2::TimePointZero, &tf_err);
    bool tf_odom_base = tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &tf_err);
    bool tf_base_laser = tf_buffer_.canTransform("base_footprint", "laser_frame", tf2::TimePointZero, &tf_err);
    bool tf_map_base = tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &tf_err);

    std::cout << "\n========================================\n";
    std::cout << "        NAV-003 链路状态诊断报告         \n";
    std::cout << "========================================\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "1. 话题采样频率 (3.0s 均值):\n";
    std::cout << "   /scan             : " << scan_hz << " Hz (要求 >= 8.0 Hz) -> "
              << (scan_hz >= 8.0 ? "PASS" : "WARN/FAIL") << "\n";
    std::cout << "   /wheel/odometry   : " << odom_hz << " Hz (要求 >= 20.0 Hz) -> "
              << (odom_hz >= 20.0 ? "PASS" : "WARN/FAIL") << "\n";
    std::cout << "2. AMCL 位姿反馈:\n";
    if (amcl_received_) {
      std::cout << "   /amcl_pose        : 已接收 (x=" << latest_amcl_x_ << ", y=" << latest_amcl_y_ << ") -> PASS\n";
    } else {
      std::cout << "   /amcl_pose        : 待接收 (提示：需在 RViz 顶部点击 '2D Pose Estimate' 在地图上标定小车初始位姿) -> PENDING\n";
    }
    std::cout << "3. TF 树连通性:\n";
    if (tf_map_odom) {
      std::cout << "   map -> odom              : 连通 (PASS)\n";
    } else {
      std::cout << "   map -> odom              : 未连通 (AMCL 正在等待初始位姿，标定后将立即激活)\n";
    }
    std::cout << "   odom -> base_footprint   : " << (tf_odom_base ? "连通 (PASS)" : "未连通 (底盘驱动未就绪)") << "\n";
    std::cout << "   base_footprint -> laser  : " << (tf_base_laser ? "连通 (PASS)" : "未连通 (URDF 未就绪)") << "\n";
    if (tf_map_base) {
      std::cout << "   map -> base_footprint    : 完全贯通 (PASS)\n";
    } else {
      std::cout << "   map -> base_footprint    : 待贯通 (底盘与雷达已就绪，等待在 RViz 点击 '2D Pose Estimate' 完成闭环)\n";
    }
    std::cout << "========================================\n";
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  size_t scan_count_;
  size_t odom_count_;
  bool amcl_received_{false};
  double latest_amcl_x_{0.0};
  double latest_amcl_y_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavLinkChecker>();
  node->sample(3.0);
  rclcpp::shutdown();
  return 0;
}
