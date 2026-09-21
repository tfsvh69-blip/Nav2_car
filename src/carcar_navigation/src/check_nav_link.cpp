// NAV-003/NAV-005/NAV-009: C++ 导航链路状态、频率、生命周期与 Groot 桥接只读检查工具
// 遵循约定：新增可执行逻辑统一使用 C++；零驱动输出，只读检查传感器、TF、生命周期与门控状态
//
// NAV-009 扩展内容：
//   6. 导航生命周期状态：查询 controller_server, velocity_smoother, planner_server, behavior_server, bt_navigator
//      精确区分“节点存在”与“导航已激活（active）”。
//   7. 定位健康门控状态：/localization_monitor/ready, /localization_monitor/recovery_allowed 及 diagnostics。
//   8. Groot 行为树监视桥接状态：/bt_monitor/diagnostics, ZMQ 端口与拓扑节点数。
//   9. 综合就绪判定：明确区分“节点进程在线”与“导航全链路真正就绪”。

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

struct LifecycleInfo {
  bool node_exists{false};
  uint8_t state_id{0};
  std::string state_label{"未运行"};
};

// ── 主节点 ─────────────────────────────────────────────────────────────────
class NavLinkChecker : public rclcpp::Node {
public:
  NavLinkChecker()
      : Node("nav_link_checker"),
        scan_count_(0),
        odom_count_(0),
        amcl_received_(false),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    // 激光订阅：匹配 sllidar_node 的 KeepLast(10), Reliable
    rclcpp::QoS scan_qos(rclcpp::KeepLast(10));
    scan_qos.reliable();
    scan_qos.durability_volatile();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          ++scan_count_;
          double stamp_sec = msg->header.stamp.sec +
                             msg->header.stamp.nanosec * 1e-9;
          scan_stamps_sec_.push_back(stamp_sec);
          scan_frames_.push_back(msg->header.frame_id);
        });

    // 里程计订阅：匹配 rosmaster_base 的 KeepLast(10), Reliable
    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odom_qos.reliable();
    odom_qos.durability_volatile();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
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

    // 定位健康门控话题订阅（Latching QoS）
    rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
    latching_qos.reliable();
    latching_qos.transient_local();

    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/localization_monitor/ready", latching_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          have_ready_ = true;
          ready_val_ = msg->data;
          last_ready_stamp_ = this->now();
        });

    recovery_allowed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/localization_monitor/recovery_allowed", latching_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          have_recovery_allowed_ = true;
          recovery_allowed_val_ = msg->data;
          last_recovery_allowed_stamp_ = this->now();
        });

    loc_diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/localization_monitor/diagnostics", rclcpp::QoS(5).reliable(),
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
          if (!msg->status.empty()) {
            have_loc_diag_ = true;
            loc_diag_level_ = msg->status[0].level;
            loc_diag_msg_ = msg->status[0].message;
          }
        });

    // Groot 行为树监视桥接诊断订阅
    bt_diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/bt_monitor/diagnostics", rclcpp::QoS(5).reliable(),
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
          if (!msg->status.empty()) {
            have_bt_diag_ = true;
            bt_diag_level_ = msg->status[0].level;
            bt_diag_msg_ = msg->status[0].message;
            for (const auto & kv : msg->status[0].values) {
              bt_diag_values_[kv.key] = kv.value;
            }
          }
        });

    // 创建导航节点生命周期客户端
    nav_lifecycle_nodes_ = {
      "controller_server",
      "velocity_smoother",
      "planner_server",
      "behavior_server",
      "bt_navigator"
    };
    for (const auto & name : nav_lifecycle_nodes_) {
      lifecycle_clients_[name] = this->create_client<lifecycle_msgs::srv::GetState>(
          "/" + name + "/get_state");
    }

    RCLCPP_INFO(this->get_logger(), "开始采样 NAV-009 导航全链路状态 (持续 3 秒)...");
  }

  void sample(double duration_sec = 3.0) {
    scan_count_ = 0;
    odom_count_ = 0;
    scan_stamps_sec_.clear();
    scan_frames_.clear();

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

    // ── 1. 基础 TF 连通性（最新时刻）────────────────────────────────────
    std::string tf_err;
    bool tf_map_odom  = tf_buffer_.canTransform("map", "odom", tf2::TimePointZero, &tf_err);
    bool tf_odom_base = tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &tf_err);
    bool tf_base_laser = tf_buffer_.canTransform("base_footprint", "laser_frame", tf2::TimePointZero, &tf_err);
    bool tf_map_base  = tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &tf_err);

    // ── 2. 扫描时间戳与消息年龄 ─────────────────────────────────────────
    double now_sec = this->now().seconds();
    std::vector<double> ages;
    size_t tf_matched_at_scan = 0;

    for (size_t i = 0; i < scan_stamps_sec_.size(); ++i) {
      double age = now_sec - scan_stamps_sec_[i];
      ages.push_back(age);

      if (!scan_frames_.empty()) {
        builtin_interfaces::msg::Time ros_stamp;
        ros_stamp.sec    = static_cast<int32_t>(scan_stamps_sec_[i]);
        ros_stamp.nanosec = static_cast<uint32_t>(
            (scan_stamps_sec_[i] - ros_stamp.sec) * 1e9);
        rclcpp::Time rclcpp_stamp(ros_stamp);
        tf2::TimePoint tp = tf2::TimePoint(
            std::chrono::nanoseconds(rclcpp_stamp.nanoseconds()));
        try {
          if (tf_buffer_.canTransform("map", scan_frames_[i], tp,
                                      tf2::durationFromSec(0.0), &tf_err)) {
            ++tf_matched_at_scan;
          }
        } catch (...) {}
      }
    }

    double age_max = 0.0, age_mean = 0.0;
    if (!ages.empty()) {
      for (double a : ages) {
        age_mean += a;
        if (a > age_max) age_max = a;
      }
      age_mean /= ages.size();
    }
    double tf_match_rate = ages.empty() ? 0.0 :
        (100.0 * tf_matched_at_scan / ages.size());

    // ── 3. TF 变化统计（map->odom 和 odom->base_footprint）─────────────
    std::vector<double> mo_trans, mo_rot;
    std::vector<double> ob_trans, ob_rot;

    if (tf_map_odom) {
      try {
        auto t0 = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
        rclcpp::sleep_for(100ms);
        rclcpp::spin_some(this->get_node_base_interface());
        auto t1 = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
        double dx = t1.transform.translation.x - t0.transform.translation.x;
        double dy = t1.transform.translation.y - t0.transform.translation.y;
        mo_trans.push_back(std::sqrt(dx * dx + dy * dy));
        double dw = std::abs(t1.transform.rotation.w - t0.transform.rotation.w);
        mo_rot.push_back(dw * 2.0);
      } catch (...) {}
    }

    if (tf_odom_base) {
      try {
        auto t0 = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        rclcpp::sleep_for(100ms);
        rclcpp::spin_some(this->get_node_base_interface());
        auto t1 = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        double dx = t1.transform.translation.x - t0.transform.translation.x;
        double dy = t1.transform.translation.y - t0.transform.translation.y;
        ob_trans.push_back(std::sqrt(dx * dx + dy * dy));
        double dw = std::abs(t1.transform.rotation.w - t0.transform.rotation.w);
        ob_rot.push_back(dw * 2.0);
      } catch (...) {}
    }

    // ── 4. 查询导航节点生命周期状态 ─────────────────────────────────────
    std::map<std::string, LifecycleInfo> lifecycle_results;
    for (const auto & name : nav_lifecycle_nodes_) {
      LifecycleInfo info;
      auto client = lifecycle_clients_[name];
      if (client->wait_for_service(50ms)) {
        info.node_exists = true;
        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 200ms) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          auto res = future.get();
          info.state_id = res->current_state.id;
          info.state_label = res->current_state.label;
        } else {
          info.state_label = "响应超时";
        }
      } else {
        info.node_exists = false;
        info.state_label = "未运行 (服务未注册)";
      }
      lifecycle_results[name] = info;
    }

    // ── 输出诊断报告 ─────────────────────────────────────────────────────
    std::cout << "\n========================================\n";
    std::cout << "        NAV-009 导航全链路诊断报告       \n";
    std::cout << "========================================\n";
    std::cout << std::fixed << std::setprecision(1);

    std::cout << "1. 话题采样频率 (" << duration_sec << " s 均值):\n";
    std::cout << "   /scan             : " << scan_hz << " Hz (要求 >= 8.0 Hz) -> "
              << (scan_hz >= 8.0 ? "PASS" : "WARN/FAIL") << "\n";
    std::cout << "   " << odom_topic_ << "   : " << odom_hz << " Hz (要求 >= 20.0 Hz) -> "
              << (odom_hz >= 20.0 ? "PASS" : "WARN/FAIL") << "\n";

    std::cout << "2. AMCL 位姿反馈:\n";
    if (amcl_received_) {
      std::cout << "   /amcl_pose        : 已接收 (x=" << latest_amcl_x_
                << ", y=" << latest_amcl_y_ << ") -> PASS\n";
    } else {
      std::cout << "   /amcl_pose        : 待接收 (提示：需在 RViz 点击 '2D Pose Estimate' "
                   "标定初始位姿) -> PENDING\n";
    }

    std::cout << "3. TF 树连通性（最新时刻）:\n";
    if (tf_map_odom) {
      std::cout << "   map -> odom              : 连通 (PASS)\n";
    } else {
      std::cout << "   map -> odom              : 未连通 (AMCL 正在等待初始位姿，标定后将立即激活)\n";
    }
    std::cout << "   odom -> base_footprint   : "
              << (tf_odom_base ? "连通 (PASS)" : "未连通 (底盘驱动未就绪)") << "\n";
    std::cout << "   base_footprint -> laser  : "
              << (tf_base_laser ? "连通 (PASS)" : "未连通 (URDF 未就绪)") << "\n";
    if (tf_map_base) {
      std::cout << "   map -> base_footprint    : 完全贯通 (PASS)\n";
    } else {
      std::cout << "   map -> base_footprint    : 待贯通 (等待在 RViz 设置初始位姿)\n";
    }

    std::cout << "4. 扫描时间戳与消息年龄 (" << ages.size() << " 帧):\n";
    if (ages.empty()) {
      std::cout << "   未收到任何 /scan 消息 -> WARN\n";
    } else {
      std::cout << std::setprecision(3);
      std::cout << "   消息年龄 最大值 : " << age_max << " s"
                << (age_max > 0.5 ? " -> WARN（超过 0.5 s，可能时钟不同步）" : " -> PASS") << "\n";
      std::cout << "   消息年龄 均值   : " << age_mean << " s\n";
      std::cout << "   TF 匹配率（扫描帧时间戳）: " << std::setprecision(1)
                << tf_match_rate << "%"
                << (tf_match_rate < 80.0 ? " -> WARN（匹配率不足，可能有 TF 延迟）" : " -> PASS") << "\n";
    }

    std::cout << "5. TF 变化统计（采样窗口内 ~0.1 s 差值）:\n";
    if (!mo_trans.empty()) {
      std::cout << std::setprecision(4);
      std::cout << "   map->odom 平移变化 : " << mo_trans[0] << " m"
                << (mo_trans[0] > 0.05 ? " -> WARN" : " -> OK") << "\n";
      std::cout << "   map->odom 旋转变化 : " << mo_rot[0] << " rad"
                << (mo_rot[0] > 0.05 ? " -> WARN" : " -> OK") << "\n";
    } else {
      std::cout << "   map->odom 变换不可用（AMCL 未就绪）\n";
    }
    if (!ob_trans.empty()) {
      std::cout << "   odom->base_footprint 平移变化 : " << ob_trans[0] << " m"
                << (ob_trans[0] > 0.05 ? " -> WARN" : " -> OK") << "\n";
      std::cout << "   odom->base_footprint 旋转变化 : " << ob_rot[0] << " rad"
                << (ob_rot[0] > 0.05 ? " -> WARN" : " -> OK") << "\n";
    } else {
      std::cout << "   odom->base_footprint 变换不可用\n";
    }

    // ── 6. 导航节点生命周期状态 ─────────────────────────────────────────
    std::cout << "6. 导航节点生命周期状态:\n";
    int active_count = 0;
    int exist_count = 0;
    for (const auto & name : nav_lifecycle_nodes_) {
      const auto & info = lifecycle_results[name];
      std::cout << "   " << std::left << std::setw(25) << name << ": ";
      if (!info.node_exists) {
        std::cout << "未运行 (服务未注册) -> OFF\n";
      } else {
        ++exist_count;
        if (info.state_label == "active") {
          ++active_count;
          std::cout << "active -> PASS\n";
        } else if (info.state_label == "inactive") {
          std::cout << "inactive -> WARN (节点已启动但未激活，可能行为树解析失败或处于待配置状态)\n";
        } else {
          std::cout << info.state_label << " -> PENDING\n";
        }
      }
    }

    // ── 7. 定位健康与门控状态 ───────────────────────────────────────────
    std::cout << "7. 定位健康与门控状态:\n";
    if (have_ready_) {
      double ready_age = (this->now() - last_ready_stamp_).seconds();
      std::cout << "   /localization_monitor/ready            : "
                << (ready_val_ ? "TRUE (健康放行)" : "FALSE (未就绪/受阻)")
                << " (新鲜度: " << std::setprecision(2) << ready_age << " s) -> "
                << (ready_val_ && ready_age <= 1.0 ? "PASS" : "WARN") << "\n";
    } else {
      std::cout << "   /localization_monitor/ready            : 未收到 (监控节点可能未启动) -> PENDING\n";
    }
    if (have_recovery_allowed_) {
      std::cout << "   /localization_monitor/recovery_allowed : "
                << (recovery_allowed_val_ ? "TRUE (允许 AMCL 重定位)" : "FALSE (输入异常禁止重定位)")
                << " -> " << (recovery_allowed_val_ ? "PASS" : "WARN") << "\n";
    } else {
      std::cout << "   /localization_monitor/recovery_allowed : 未收到 -> PENDING\n";
    }
    if (have_loc_diag_) {
      std::cout << "   定位监控诊断详情                       : " << loc_diag_msg_ << "\n";
    }

    // ── 8. Groot 行为树监视桥接状态 ─────────────────────────────────────
    std::cout << "8. Groot 行为树监视桥接状态:\n";
    if (have_bt_diag_) {
      std::cout << "   桥接节点 (bt_monitor)                  : 运行中 (" << bt_diag_msg_ << ") -> PASS\n";
      auto it_xml = bt_diag_values_.find("bt_xml_path");
      if (it_xml != bt_diag_values_.end()) {
        std::cout << "   加载行为树                             : " << it_xml->second << "\n";
      }
      auto it_pub = bt_diag_values_.find("zmq_publisher_port");
      auto it_srv = bt_diag_values_.find("zmq_server_port");
      if (it_pub != bt_diag_values_.end() && it_srv != bt_diag_values_.end()) {
        std::cout << "   ZMQ 监视端口                           : 127.0.0.1:" << it_pub->second << "/" << it_srv->second << "\n";
      }
      auto it_nodes = bt_diag_values_.find("named_nodes");
      if (it_nodes != bt_diag_values_.end()) {
        std::cout << "   已注册镜像节点数                       : " << it_nodes->second << "\n";
      }
    } else {
      std::cout << "   桥接节点 (bt_monitor)                  : 未运行或未收到诊断 -> OFF/WAIT\n";
      std::cout << "   提示：第四终端运行 navigation_experimental.launch.xml 将自动带起桥接与 Groot 窗口。\n";
    }

    // ── 9. 综合就绪判定（区分“节点存在”与“导航已就绪”） ─────────────────
    std::cout << "9. 综合就绪判定:\n";
    std::cout << "   - 节点在线情况 : " << exist_count << " / " << nav_lifecycle_nodes_.size() << " 个节点在线\n";
    std::cout << "   - 导航激活状态 : " << active_count << " / " << nav_lifecycle_nodes_.size() << " 个节点已激活 (active)\n";
    std::cout << "   - 定位健康状态 : " << (have_ready_ && ready_val_ ? "就绪 (PASS)" : "未就绪 (WAIT)") << "\n";
    std::cout << "   - 桥接监视状态 : " << (have_bt_diag_ ? "就绪 (PASS)" : "未就绪 (WAIT)") << "\n";

    bool nav_ready = (exist_count == static_cast<int>(nav_lifecycle_nodes_.size())) &&
                     (active_count == static_cast<int>(nav_lifecycle_nodes_.size())) &&
                     (have_ready_ && ready_val_);

    if (nav_ready) {
      std::cout << "   >>> 综合结论: 【导航全链路已就绪】所有组件 active，定位健康放行，可开始发送导航目标 (READY) <<<\n";
    } else if (exist_count > 0 && active_count < exist_count) {
      std::cout << "   >>> 综合结论: 【导航未就绪（节点已启动但未全激活）】\n";
      auto it_bt = lifecycle_results.find("bt_navigator");
      if (it_bt != lifecycle_results.end() && it_bt->second.state_label == "inactive") {
        std::cout << "       关键提示: bt_navigator 处于 inactive！请检查行为树 XML 是否加载成功或存在未实现接口。\n";
      }
      std::cout << "   <<<\n";
    } else if (exist_count == 0) {
      std::cout << "   >>> 综合结论: 【导航节点未启动】请先在终端 4 启动 navigation_experimental.launch.xml <<<\n";
    } else if (!have_ready_ || !ready_val_) {
      std::cout << "   >>> 综合结论: 【导航节点已激活，等待定位健康】请在 RViz 标定 2D Pose Estimate 直至 /localization_monitor/ready 为 TRUE <<<\n";
    }
    std::cout << "========================================\n";
  }

private:
  std::string odom_topic_{"/wheel/odometry"};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_allowed_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr loc_diag_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr bt_diag_sub_;

  std::vector<std::string> nav_lifecycle_nodes_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_clients_;

  size_t scan_count_;
  size_t odom_count_;
  bool amcl_received_{false};
  double latest_amcl_x_{0.0};
  double latest_amcl_y_{0.0};

  bool have_ready_{false};
  bool ready_val_{false};
  rclcpp::Time last_ready_stamp_;

  bool have_recovery_allowed_{false};
  bool recovery_allowed_val_{false};
  rclcpp::Time last_recovery_allowed_stamp_;

  bool have_loc_diag_{false};
  uint8_t loc_diag_level_{0};
  std::string loc_diag_msg_;

  bool have_bt_diag_{false};
  uint8_t bt_diag_level_{0};
  std::string bt_diag_msg_;
  std::map<std::string, std::string> bt_diag_values_;

  std::vector<double> scan_stamps_sec_;
  std::vector<std::string> scan_frames_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavLinkChecker>();
  node->sample(3.0);
  rclcpp::shutdown();
  return 0;
}
