// Copyright 2026 carcar maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// NAV-008/NAV-011/NAV-013: 导航事件监控、终端状态摘要、只读诊断与多维度会话录包系统
// 遵循约定：全 C++ 实现；只读监听各话题与 Action 状态；零驱动输出，绝不发布运动指令。
// 启动时自动建立 log/nav_sessions/<时间>_<PID>/ 会话目录，保存参数快照、配置、行为树 XML、
// 结构化 events.jsonl、中文 events.log、终端摘要记录 terminal_summary.log，并后台自动分段录制 rosbag2。
// 终端每秒输出状态摘要，并在阶段变化或异常时即时输出；高频重复告警与周期重规划自动合并计数；
// 发布只读状态话题 /navigation/status (diagnostic_msgs/DiagnosticArray)。

#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "carcar_navigation/goal_status_policy.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/record_options.hpp>

namespace
{
std::string get_iso_timestamp(const std::chrono::system_clock::time_point & tp = std::chrono::system_clock::now())
{
  const auto in_time_t = std::chrono::system_clock::to_time_t(tp);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    tp.time_since_epoch()) % 1000;
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S")
     << "." << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

std::string get_session_timestamp_id()
{
  const auto now = std::chrono::system_clock::now();
  const auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  return ss.str();
}

std::string get_time_only_str(const std::chrono::system_clock::time_point & tp = std::chrono::system_clock::now())
{
  const auto in_time_t = std::chrono::system_clock::to_time_t(tp);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%H:%M:%S");
  return ss.str();
}

std::string uuid_to_hex(const std::array<uint8_t, 16> & uuid)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (size_t i = 0; i < 16; ++i) {
    ss << std::setw(2) << static_cast<int>(uuid[i]);
  }
  return ss.str();
}

std::string uuid_short(const std::string & uuid)
{
  if (uuid.empty() || uuid == "NONE" || uuid == "UNASSOCIATED") return uuid;
  if (uuid.size() <= 8) return uuid;
  return uuid.substr(0, 8);
}

struct RobotPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id{"map"};
  bool valid{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct ObstacleStats
{
  double min_dist{99.0};
  double min_front{99.0};
  double min_rear{99.0};
  bool valid{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

enum class NavStage
{
  WAITING_FOR_LOCALIZATION,
  PLANNING,
  PATH_FOLLOWING,
  PATH_FOLLOWING_REPLANNING,
  RECOVERY_WAITING,
  SPINNING,
  REAR_CHECK,
  BACKING_UP,
  PARK_AND_OBSERVE,
  RECOVERING_LOCALIZATION,
  SUCCEEDED,
  CANCELED,
  FAILED,
  IDLE
};

std::string stage_to_string(NavStage stage)
{
  switch (stage) {
    case NavStage::WAITING_FOR_LOCALIZATION: return "等待定位";
    case NavStage::PLANNING: return "规划";
    case NavStage::PATH_FOLLOWING: return "路径跟随";
    case NavStage::PATH_FOLLOWING_REPLANNING: return "路径跟随 (正在重规划)";
    case NavStage::RECOVERY_WAITING: return "恢复等待";
    case NavStage::SPINNING: return "受控转向";
    case NavStage::REAR_CHECK: return "后方检查";
    case NavStage::BACKING_UP: return "执行倒车";
    case NavStage::PARK_AND_OBSERVE: return "停车观察";
    case NavStage::RECOVERING_LOCALIZATION: return "定位恢复中";
    case NavStage::SUCCEEDED: return "到达";
    case NavStage::CANCELED: return "取消";
    case NavStage::FAILED: return "失败";
    case NavStage::IDLE: return "待命";
    default: return "未知";
  }
}

std::string stage_to_code(NavStage stage)
{
  switch (stage) {
    case NavStage::WAITING_FOR_LOCALIZATION: return "LOC_WAIT";
    case NavStage::PLANNING: return "PLANNING";
    case NavStage::PATH_FOLLOWING: return "FOLLOWING";
    case NavStage::PATH_FOLLOWING_REPLANNING: return "REPLANNING";
    case NavStage::RECOVERY_WAITING: return "RECOVERY_WAIT";
    case NavStage::SPINNING: return "SPINNING";
    case NavStage::REAR_CHECK: return "REAR_CHECK";
    case NavStage::BACKING_UP: return "BACKING";
    case NavStage::PARK_AND_OBSERVE: return "PARK_WAIT";
    case NavStage::RECOVERING_LOCALIZATION: return "LOC_RECOVER";
    case NavStage::SUCCEEDED: return "SUCCEEDED";
    case NavStage::CANCELED: return "CANCELED";
    case NavStage::FAILED: return "FAILED";
    case NavStage::IDLE: return "IDLE";
    default: return "UNKNOWN";
  }
}

struct GoalTask
{
  std::string goal_id;
  std::string uuid;
  std::string action_type{"NavigateToPose"};
  std::string start_time;
  std::string end_time;
  double target_x{0.0};
  double target_y{0.0};
  double target_yaw{0.0};
  std::string frame_id{"map"};
  RobotPose start_pose;
  double initial_distance{0.0};
  double remaining_distance{-1.0};
  rclcpp::Time last_distance_update{0, 0, RCL_ROS_TIME};
  int8_t action_status{-1};
  bool is_active{false};
  bool is_terminal{false};
};

struct StateSnapshot
{
  std::string timestamp_str;
  double t_sec{0.0};
  std::string goal_uuid{"NONE"};
  std::string stage{"待命"};
  std::string motion_feedback{"静止"};
  double cmd_vel_nav_vx{0.0};
  double cmd_vel_nav_wz{0.0};
  double cmd_vel_vx{0.0};
  double cmd_vel_wz{0.0};
  double odom_vx{0.0};
  double odom_wz{0.0};
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_yaw{0.0};
  double min_scan_dist{-1.0};
  bool loc_ready{false};
  bool chassis_watchdog_stopped{false};
  double scan_age_s{-1.0};
  double odom_age_s{-1.0};
  std::string recent_anomaly;
};

struct StopRecord
{
  std::string timestamp;
  std::string goal_uuid{"NONE"};
  std::string action_type{"None"};
  std::string stop_category{"CAUSE_UNDETERMINED"};
  std::string cause_code{"NONE"};
  std::string trigger_reason{"未知"};
  std::string evidence_status{"INSUFFICIENT"};  // CERTAIN, CORRELATED, INSUFFICIENT
  uint64_t stop_event_id{0};
  std::string stage{"待命"};
  std::string motion_feedback{"静止"};
  std::string related_anomalies{"无"};
  std::string recovery_or_terminal{"无"};
  double cmd_vel_nav_vx{0.0};
  double cmd_vel_nav_wz{0.0};
  double cmd_vel_vx{0.0};
  double cmd_vel_wz{0.0};
  double odom_vx{0.0};
  double odom_wz{0.0};
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_yaw{0.0};
  std::vector<std::string> clues;
  nlohmann::json evidence_window{nlohmann::json::array()};
};

struct ActiveEvidenceWindow
{
  bool is_active{false};
  double window_start_time{0.0};
  double stop_event_time{0.0};
  double window_end_time{0.0};
  StopRecord stop_record;
  std::vector<StateSnapshot> pre_snapshots;
  std::vector<StateSnapshot> post_snapshots;
};

struct AlertAggregator
{
  std::string last_msg;
  std::string node;
  std::string msg;
  rclcpp::Time first_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
  int count{0};
};

}  // namespace

class NavEventLogger : public rclcpp::Node
{
public:
  NavEventLogger()
  : Node("nav_event_logger"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 参数声明
    sessions_base_dir_ = this->declare_parameter<std::string>(
      "sessions_base_dir", "/home/jetson/luhao/my_nav_carcar/log/nav_sessions");
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    base_params_file_ = this->declare_parameter<std::string>("base_params_file", "");
    experiment_params_file_ = this->declare_parameter<std::string>("experiment_params_file", "");
    default_bt_xml_ = this->declare_parameter<std::string>("default_bt_xml", "");
    default_nav_through_poses_bt_xml_ = this->declare_parameter<std::string>("default_nav_through_poses_bt_xml", "");

    full_event_display_ = this->declare_parameter<bool>("full_event_display", false);
    motion_linear_still_threshold_ = this->declare_parameter<double>("motion_linear_still_threshold", 0.01);
    motion_angular_still_threshold_ = this->declare_parameter<double>("motion_angular_still_threshold", 0.02);
    motion_still_duration_ = this->declare_parameter<double>("motion_still_duration", 1.0);
    motion_data_timeout_ = this->declare_parameter<double>("motion_data_timeout", 0.6);

    record_raw_bag_ = this->declare_parameter<bool>("record_raw_bag", false);
    raw_bag_max_duration_s_ = this->declare_parameter<double>("raw_bag_max_duration_s", 120.0);
    raw_bag_max_bytes_ = this->declare_parameter<int64_t>("raw_bag_max_bytes", 268435456LL);  // 256 MiB
    full_bt_debug_events_ = this->declare_parameter<bool>("full_bt_debug_events", false);
    replanning_summary_interval_s_ = this->declare_parameter<double>("replanning_summary_interval_s", 30.0);
    evidence_window_pre_s_ = this->declare_parameter<double>("evidence_window_pre_s", 10.0);
    evidence_window_post_s_ = this->declare_parameter<double>("evidence_window_post_s", 5.0);

    // 解析行为树 XML 文件，加载节点名称到节点类型的映射表
    load_bt_xml_mappings(default_bt_xml_);
    load_bt_xml_mappings(default_nav_through_poses_bt_xml_);

    // 初始化会话目录
    init_session();

    // 状态发布者：只读发布 /navigation/status 供声光提示与全局监控复用
    nav_status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/status", rclcpp::QoS(10).reliable());

    // 订阅目标下发
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_goal_pose, this, std::placeholders::_1));

    // 订阅行为树事件流（精细捕获节点流转）
    bt_log_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      "/behavior_tree_log", rclcpp::QoS(100).reliable(),
      std::bind(&NavEventLogger::on_bt_log, this, std::placeholders::_1));

    // 订阅 NavigateToPose Action 状态与反馈
    nav_to_pose_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10).reliable(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        on_action_status("NavigateToPose", msg);
      });

    nav_to_pose_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
      "/navigate_to_pose/_action/feedback", rclcpp::QoS(10).reliable(),
      [this](const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const std::string uuid = uuid_to_hex(msg->goal_id.uuid);
        if (goals_by_uuid_.find(uuid) != goals_by_uuid_.end()) {
          goals_by_uuid_[uuid].remaining_distance = msg->feedback.distance_remaining;
          goals_by_uuid_[uuid].last_distance_update = this->now();
        }
      });

    // 订阅 NavigateThroughPoses Action 状态与反馈
    nav_through_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_through_poses/_action/status", rclcpp::QoS(10).reliable(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        on_action_status("NavigateThroughPoses", msg);
      });

    nav_through_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>(
      "/navigate_through_poses/_action/feedback", rclcpp::QoS(10).reliable(),
      [this](const nav2_msgs::action::NavigateThroughPoses_FeedbackMessage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const std::string uuid = uuid_to_hex(msg->goal_id.uuid);
        if (goals_by_uuid_.find(uuid) != goals_by_uuid_.end()) {
          goals_by_uuid_[uuid].remaining_distance = msg->feedback.distance_remaining;
          goals_by_uuid_[uuid].last_distance_update = this->now();
        }
      });

    // 订阅激光扫描
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&NavEventLogger::on_scan, this, std::placeholders::_1));

    // 订阅定位就绪与门控状态
    loc_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/localization_monitor/ready", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&NavEventLogger::on_loc_ready, this, std::placeholders::_1));

    // 订阅定位诊断
    loc_diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/localization_monitor/diagnostics", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_loc_diag, this, std::placeholders::_1));

    // 订阅两级控制速度：/cmd_vel_nav (Nav2 原始输出) 与 /cmd_vel (平滑后输出)
    cmd_vel_nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_cmd_vel_nav_ = *msg;
        cmd_vel_nav_stamp_ = this->now();
      });

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_cmd_vel_ = *msg;
        cmd_vel_stamp_ = this->now();
      });

    // 订阅纯轮式里程计
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odometry", rclcpp::QoS(10),
      std::bind(&NavEventLogger::on_odom, this, std::placeholders::_1));

    // 订阅底盘状态与硬件看门狗诊断
    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      std::bind(&NavEventLogger::on_chassis_diagnostics, this, std::placeholders::_1));

    // 订阅 ProgressGuard 门控诊断
    progress_guard_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/progress_guard/status", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_progress_guard_status, this, std::placeholders::_1));

    recovery_status_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/recovery/status", rclcpp::QoS(10),
      [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        recovery_fields_.clear();
        for (const auto & kv : msg->values) {recovery_fields_[kv.key] = kv.value;}
        recovery_received_ = std::chrono::steady_clock::now();
        record_event_locked("RECOVERY_DIAGNOSTIC", active_uuid_, msg->name, msg->message, recovery_fields_);
        if (recovery_fields_.value("reason_code","")=="GOAL_REPLACED") {
          record_event_locked("GOAL_REPLACED",recovery_fields_.value("previous_navigation_uuid",""),
            msg->name,"监督节点观测到导航 UUID 切换，关闭许可并取消旧动作",recovery_fields_);
        }
      });
    permit_status_sub_=this->create_subscription<std_msgs::msg::UInt64>(
      "/navigation/motion_permit",rclcpp::QoS(1),[this](std_msgs::msg::UInt64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        permit_received_=std::chrono::steady_clock::now();permit_token_=msg->data;
      });
    // 订阅 RearClear 结构化状态
    rear_clear_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/rear_clear/status", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_rear_clear_status, this, std::placeholders::_1));

    // 订阅 BackUp Action 状态与反馈
    backup_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/backup/_action/status", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_backup_action_status, this, std::placeholders::_1));

    backup_feedback_sub_ = this->create_subscription<nav2_msgs::action::BackUp_FeedbackMessage>(
      "/backup/_action/feedback", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_backup_feedback, this, std::placeholders::_1));

    // 订阅 rosout 以汇总 DWB 轨迹及系统告警
    rosout_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100),
      std::bind(&NavEventLogger::on_rosout, this, std::placeholders::_1));

    // 启动后台录包
    start_rosbag_recorder();

    // 异步位姿缓存定时器 (10 Hz)，避免每条事件处理时同步阻塞查询 TF
    timer_pose_cache_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NavEventLogger::update_pose_cache, this));

    // 1 Hz 终端摘要刷新定时器
    timer_1hz_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NavEventLogger::publish_terminal_summary, this));

    // 2 Hz 统一状态诊断发布定时器
    timer_2hz_status_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        publish_navigation_status_locked(this->now());
      });

    // 2 秒检查一次磁盘空间与录包容量
    // 5 Hz 内存滑动状态采样与停车证据窗口定时器
    timer_5hz_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&NavEventLogger::on_5hz_timer, this));

    // 1 秒检查一次磁盘空间与录包限额
    timer_check_limits_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NavEventLogger::check_storage_limits, this));

    RCLCPP_INFO(this->get_logger(),
      "NAV-013 导航事件监控与会话录包系统已就绪: 会话目录=%s", session_dir_.c_str());
  }

  ~NavEventLogger() override
  {
    close_session();
  }

private:
  void load_bt_xml_mappings(const std::string & xml_path)
  {
    if (xml_path.empty() || !std::filesystem::exists(xml_path)) {
      return;
    }
    std::ifstream in(xml_path);
    std::string line;
    std::regex re(R"xml(<([A-Za-z0-9_]+)[^>]*\bname="([^"]+)")xml");
    while (std::getline(in, line)) {
      std::smatch match;
      if (std::regex_search(line, match, re) && match.size() > 2) {
        const std::string tag = match[1].str();
        const std::string name = match[2].str();
        bt_node_name_to_type_[name] = tag;
      }
    }
  }

  std::string resolve_bt_node_type(const std::string & node_name)
  {
    const auto it = bt_node_name_to_type_.find(node_name);
    if (it != bt_node_name_to_type_.end()) {
      const auto & type = it->second;
      if (type == "SafeFollowPath") return "FollowPath";
      if (type == "SafeBackUp") return "BackUp";
      if (type == "SafeComputePathToPose") return "ComputePathToPose";
      if (type == "SafeComputePathThroughPoses") return "ComputePathThroughPoses";
      return type;
    }
    if (node_name.find("ProgressGuard") != std::string::npos) return "ProgressGuard";
    if (node_name.find("ControlledSpin") != std::string::npos) return "ControlledSpin";
    if (node_name.find("ParkAndObserve") != std::string::npos) return "ParkAndObserve";
    if (node_name.find("RearClear") != std::string::npos) return "RearClear";
    if (node_name.find("BackUp") != std::string::npos) return "BackUp";
    if (node_name.find("ComputePathToPose") != std::string::npos) return "ComputePathToPose";
    if (node_name.find("ComputePathThroughPoses") != std::string::npos) return "ComputePathThroughPoses";
    if (node_name.find("FollowPath") != std::string::npos) return "FollowPath";
    if (node_name.find("WaitForLocalizationStatus") != std::string::npos) return "WaitForLocalizationStatus";
    if (node_name.find("RecoverLocalization") != std::string::npos) return "RecoverLocalization";
    if (node_name.find("Spin") != std::string::npos) return "Spin";
    if (node_name.find("Wait") != std::string::npos) return "Wait";
    return node_name;
  }

  void init_session()
  {
    const auto pid = getpid();
    session_id_ = get_session_timestamp_id() + "_" + std::to_string(pid);
    session_dir_ = sessions_base_dir_ + "/" + session_id_;
    std::filesystem::create_directories(session_dir_);

    jsonl_path_ = session_dir_ + "/events.jsonl";
    text_log_path_ = session_dir_ + "/events.log";
    summary_log_path_ = session_dir_ + "/terminal_summary.log";
    session_info_path_ = session_dir_ + "/session_info.json";
    session_state_path_ = session_dir_ + "/session_state.json";

    // 保存 session_info.json
    nlohmann::json info;
    info["session_id"] = session_id_;
    info["pid"] = pid;
    info["start_time"] = get_iso_timestamp();
    info["ros_distro"] = "humble";
    info["base_params_file"] = base_params_file_;
    info["experiment_params_file"] = experiment_params_file_;
    info["default_bt_xml"] = default_bt_xml_;
    info["default_nav_through_poses_bt_xml"] = default_nav_through_poses_bt_xml_;

    std::error_code ec;
    auto space = std::filesystem::space(session_dir_, ec);
    if (!ec) {
      info["disk_available_bytes_at_start"] = space.available;
      info["disk_total_bytes"] = space.capacity;
    }

    std::ofstream info_file(session_info_path_);
    if (info_file.is_open()) {
      info_file << info.dump(2) << "\n";
    }

    // 备份行为树 XML
    std::filesystem::create_directories(session_dir_ + "/behavior_trees");
    if (!default_bt_xml_.empty() && std::filesystem::exists(default_bt_xml_)) {
      std::filesystem::copy_file(
        default_bt_xml_, session_dir_ + "/behavior_trees/navigate_to_pose_active.xml",
        std::filesystem::copy_options::overwrite_existing, ec);
    }
    if (!default_nav_through_poses_bt_xml_.empty() && std::filesystem::exists(default_nav_through_poses_bt_xml_)) {
      std::filesystem::copy_file(
        default_nav_through_poses_bt_xml_, session_dir_ + "/behavior_trees/navigate_through_poses_active.xml",
        std::filesystem::copy_options::overwrite_existing, ec);
    }

    // 备份配置文件
    if (!experiment_params_file_.empty() && std::filesystem::exists(experiment_params_file_)) {
      std::filesystem::copy_file(
        experiment_params_file_, session_dir_ + "/active_config.yaml",
        std::filesystem::copy_options::overwrite_existing, ec);
    }

    write_session_state("ACTIVE");

    record_event_locked("SESSION_STARTED", "NONE", "nav_event_logger",
      "导航记录会话启动 (PID=" + std::to_string(pid) + ")");
  }

  void write_session_state(const std::string & status)
  {
    nlohmann::json s;
    s["status"] = status;
    s["session_id"] = session_id_;
    s["record_raw_bag"] = record_raw_bag_;
    s["session_type"] = record_raw_bag_ ? "RAW_BAG" : "LIGHTWEIGHT";
    s["raw_bag_cleaned"] = false;
    s["is_recording_bag"] = is_recording_bag_;
    s["evidence_incomplete"] = evidence_incomplete_;
    s["evidence_incomplete_reason"] = incomplete_reason_;
    s["total_bag_bytes"] = current_bag_bytes_;
    s["total_events"] = event_counter_;
    s["total_goals"] = goals_by_uuid_.size();
    s["total_stops"] = stop_records_.size();

    std::ofstream f(session_state_path_);
    if (f.is_open()) {
      f << s.dump(2) << "\n";
    }
  }

  void start_rosbag_recorder()
  {
    if (!record_raw_bag_) {
      is_recording_bag_ = false;
      RCLCPP_INFO(this->get_logger(),
        "【轻量日志模式】record_raw_bag=false，不创建原始 rosbag，专注记录事件、状态摘要与停车证据窗口。");
      return;
    }

    try {
      auto writer = std::make_shared<rosbag2_cpp::Writer>();
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = session_dir_ + "/bag";
      storage_options.storage_id = "sqlite3";
      storage_options.max_bagfile_size = 1073741824ULL;  // 1 GiB 分卷
      storage_options.max_bagfile_size = static_cast<uint64_t>(raw_bag_max_bytes_);

      rosbag2_transport::RecordOptions record_options;
      record_options.include_hidden_topics = true;
      record_options.include_unpublished_topics = true;
      // 停止默认保存 /evaluation 与 /trajectories 等高频轨迹明细，仅保留关键状态与感知数据
      record_options.topics = {
        "/scan",
        "/map",
        "/map_metadata",
        "/tf",
        "/tf_static",
        "/wheel/odometry",
        "/odom",
        "/amcl_pose",
        "/particlecloud",
        "/particle_cloud",
        "/localization_monitor/ready",
        "/localization_monitor/recovery_allowed",
        "/localization_monitor/diagnostics",
        "/plan",
        "/transformed_global_plan",
        "/local_plan",
        "/global_costmap/costmap",
        "/global_costmap/costmap_raw",
        "/local_costmap/costmap",
        "/local_costmap/costmap_raw",
        "/global_costmap/published_footprint",
        "/local_costmap/published_footprint",
        "/evaluation",
        "/trajectories",
        "/cmd_vel",
        "/cmd_vel_nav",
        "/diagnostics",
        "/behavior_tree_log",
        "/progress_guard/status",
        "/rear_clear/status",
        "/navigation/status",
        "/navigate_to_pose/_action/status",
        "/navigate_to_pose/_action/feedback",
        "/navigate_through_poses/_action/status",
        "/navigate_through_poses/_action/feedback",
        "/backup/_action/status",
        "/backup/_action/feedback",
        "/spin/_action/status",
        "/spin/_action/feedback",
        "/wait/_action/status",
        "/wait/_action/feedback",
        "/parameter_events",
        "/rosout"
      };

      bag_recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        writer, storage_options, record_options, "nav_bag_recorder");
      bag_recorder_->record();

      bag_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      bag_executor_->add_node(bag_recorder_);
      bag_thread_ = std::thread([this]() {
        bag_executor_->spin();
      });

      is_recording_bag_ = true;
      bag_start_time_ = this->now();
      record_event_locked("ROSBAG_RECORDING_STARTED", "NONE", "rosbag2",
        "按需原始录包已启动 (限额: 120s / 256 MiB, 存储: sqlite3)");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "启动 rosbag2 自动录包失败: %s", e.what());
      is_recording_bag_ = false;
      evidence_incomplete_ = true;
      incomplete_reason_ = std::string("rosbag2启动异常: ") + e.what();
    }
  }

  void stop_rosbag_recorder(const std::string & reason)
  {
    if (!is_recording_bag_) {
      return;
    }
    is_recording_bag_ = false;
    if (bag_recorder_) {
      try {
        bag_recorder_->stop();
      } catch (...) {}
    }
    if (bag_executor_) {
      try {
        bag_executor_->cancel();
      } catch (...) {}
    }
    if (bag_thread_.joinable()) {
      bag_thread_.join();
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    record_event_locked("ROSBAG_RECORDING_STOPPED", active_uuid_, "rosbag2",
      "原始录包已停止 (" + reason + ")，轻量化事件与状态摘要继续运行");
    std::cout << "\n[" << get_time_only_str() << "] [录包控制] 原始 rosbag 录包已安全停止: "
              << reason << " (轻量日志继续运行)\n" << std::endl;
    write_session_state("ACTIVE");
  }

  void check_storage_limits()
  {
    uint64_t bag_size = 0;
    const auto bag_path = std::filesystem::path(session_dir_) / "bag";
    if (std::filesystem::exists(bag_path)) {
      std::error_code ec;
      for (const auto & entry : std::filesystem::recursive_directory_iterator(bag_path, ec)) {
        if (std::filesystem::is_regular_file(entry, ec)) {
          bag_size += std::filesystem::file_size(entry, ec);
        }
      }
    }
    current_bag_bytes_ = bag_size;

    if (is_recording_bag_) {
      const double duration_s = (this->now() - bag_start_time_).seconds();
      if (duration_s >= raw_bag_max_duration_s_) {
        std::ostringstream ss;
        ss << "达到录包时长上限 (已录制 " << std::fixed << std::setprecision(1)
           << duration_s << "s / 上限 " << raw_bag_max_duration_s_ << "s)";
        stop_rosbag_recorder(ss.str());
        return;
      }

      if (bag_size >= static_cast<uint64_t>(raw_bag_max_bytes_)) {
        const double mb = bag_size / (1024.0 * 1024.0);
        std::ostringstream ss;
        ss << "达到录包大小上限 (已占用 " << std::fixed << std::setprecision(1)
           << mb << " MB / 上限 " << (raw_bag_max_bytes_ / (1024 * 1024)) << " MB)";
        stop_rosbag_recorder(ss.str());
        return;
      }

      std::error_code ec;
      auto space = std::filesystem::space(session_dir_, ec);
      if (!ec && space.available < 2147483648ULL) {
        evidence_incomplete_ = true;
        incomplete_reason_ = "磁盘剩余空间低于 2 GiB (" + std::to_string(space.available / (1024 * 1024)) + " MB)";
        std::cout << "\n[告警] 磁盘剩余空间低于 2 GiB，停止原始录包，继续记录状态摘要与结构化事件 (证据标记为不完整)\n" << std::endl;
        stop_rosbag_recorder(incomplete_reason_);
        return;
      } else if (bag_size >= 10737418240ULL) {  // 单次达到 10 GiB
        evidence_incomplete_ = true;
        incomplete_reason_ = "单次会话录包达到 10 GiB 保护上限";
        std::cout << "\n[告警] 单次会话录包达到 10 GiB 保护上限，停止原始录包，继续记录状态摘要与结构化事件 (证据标记为不完整)\n" << std::endl;
        stop_rosbag_recorder(incomplete_reason_);
        return;
      }
    }
  }

  void close_session()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (active_window_.is_active) {
        flush_active_evidence_window_locked();
      }
    }
    if (is_recording_bag_ && bag_recorder_) {
      bag_recorder_->stop();
      try {
        bag_recorder_->stop();
      } catch (...) {}
      is_recording_bag_ = false;
    }
    if (bag_executor_) {
      bag_executor_->cancel();
      try {
        bag_executor_->cancel();
      } catch (...) {}
    }
    if (bag_thread_.joinable()) {
      bag_thread_.join();
    }
    write_session_state("ENDED");
    std::cout << "[会话日志: 已结束] [原始 rosbag: "
              << (record_raw_bag_?"已停止":"未启用") << "] " << session_id_ << std::endl;
  }

  void on_5hz_timer()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = this->now();
    const double now_sec = now.seconds();

    StateSnapshot snap;
    snap.timestamp_str = get_iso_timestamp();
    snap.t_sec = now_sec;
    snap.goal_uuid = active_uuid_.empty() ? "NONE" : active_uuid_;
    snap.stage = stage_to_string(current_stage_);
    snap.motion_feedback = motion_feedback_str_;
    snap.cmd_vel_nav_vx = latest_cmd_vel_nav_.linear.x;
    snap.cmd_vel_nav_wz = latest_cmd_vel_nav_.angular.z;
    snap.cmd_vel_vx = latest_cmd_vel_.linear.x;
    snap.cmd_vel_wz = latest_cmd_vel_.angular.z;
    snap.odom_vx = latest_odom_.twist.twist.linear.x;
    snap.odom_wz = latest_odom_.twist.twist.angular.z;

    const auto robot = get_current_pose();
    snap.robot_x = robot.x;
    snap.robot_y = robot.y;
    snap.robot_yaw = robot.yaw;
    snap.scan_age_s = latest_obstacles_.stamp.nanoseconds() > 0 ? (now - latest_obstacles_.stamp).seconds() : -1.0;
    snap.odom_age_s = odom_stamp_.nanoseconds() > 0 ? (now - odom_stamp_).seconds() : -1.0;
    snap.recent_anomaly = latest_anomaly_;

    // 环形缓冲保留最多 100 帧 (~20s 数据，覆盖 10s pre 证据)
    ring_buffer_.push_back(snap);
    while (ring_buffer_.size() > 100) {
      ring_buffer_.pop_front();
    }

    if (active_window_.is_active) {
      if (now_sec <= active_window_.window_end_time) {
        active_window_.post_snapshots.push_back(snap);
      } else {
        flush_active_evidence_window_locked();
      }
    }
  }

  void trigger_stop_window_locked(const StopRecord & rec)
  {
    const double now_sec = this->now().seconds();

    if (active_window_.is_active) {
      // 合并重叠停车窗口：延长窗口结束时间，合并线索，不重复刷屏开窗
      active_window_.window_end_time = std::max(active_window_.window_end_time, now_sec + evidence_window_post_s_);
      for (const auto & c : rec.clues) {
        if (std::find(active_window_.stop_record.clues.begin(), active_window_.stop_record.clues.end(), c) ==
            active_window_.stop_record.clues.end()) {
          active_window_.stop_record.clues.push_back(c);
        }
      }
      if (active_window_.stop_record.stop_category == "CAUSE_UNDETERMINED" && rec.stop_category != "CAUSE_UNDETERMINED") {
        active_window_.stop_record.stop_category = rec.stop_category;
        active_window_.stop_record.trigger_reason = rec.trigger_reason;
        active_window_.stop_record.evidence_status = rec.evidence_status;
      }
      if (!rec.recovery_or_terminal.empty() && rec.recovery_or_terminal != "无") {
        active_window_.stop_record.recovery_or_terminal = rec.recovery_or_terminal;
      }
      return;
    }

    active_window_.is_active = true;
    active_window_.stop_event_time = now_sec;
    active_window_.window_start_time = now_sec - evidence_window_pre_s_;
    active_window_.window_end_time = now_sec + evidence_window_post_s_;
    active_window_.stop_record = rec;
    active_window_.pre_snapshots.clear();
    active_window_.post_snapshots.clear();

    for (const auto & s : ring_buffer_) {
      if (s.t_sec >= active_window_.window_start_time && s.t_sec <= active_window_.stop_event_time) {
        active_window_.pre_snapshots.push_back(s);
      }
    }
  }

  void flush_active_evidence_window_locked()
  {
    if (!active_window_.is_active) return;
    active_window_.is_active = false;

    // 组装证据窗口帧数组
    nlohmann::json win_arr = nlohmann::json::array();
    auto append_snaps = [&](const std::vector<StateSnapshot> & vec) {
      for (const auto & s : vec) {
        nlohmann::json item;
        item["dt_s"] = s.t_sec - active_window_.stop_event_time;
        item["timestamp"] = s.timestamp_str;
        item["stage"] = s.stage;
        item["feedback"] = s.motion_feedback;
        item["cmd_vel_nav"] = {{"vx", s.cmd_vel_nav_vx}, {"wz", s.cmd_vel_nav_wz}};
        item["cmd_vel"] = {{"vx", s.cmd_vel_vx}, {"wz", s.cmd_vel_wz}};
        item["odom"] = {{"vx", s.odom_vx}, {"wz", s.odom_wz}};
        item["pose"] = {{"x", s.robot_x}, {"y", s.robot_y}, {"yaw", s.robot_yaw}};
        item["recent_anomaly"] = s.recent_anomaly;
        win_arr.push_back(item);
      }
    };
    append_snaps(active_window_.pre_snapshots);
    append_snaps(active_window_.post_snapshots);

    active_window_.stop_record.evidence_window = win_arr;

    // 记录 STOP_EVIDENCE_ATTACHED 事件
    nlohmann::json ev;
    ev["timestamp"] = get_iso_timestamp();
    ev["event_index"] = ++event_counter_;
    ev["event_type"] = "STOP_EVIDENCE_ATTACHED";
    ev["goal_uuid"] = active_window_.stop_record.goal_uuid;
    ev["stop_category"] = active_window_.stop_record.stop_category;
    ev["trigger_reason"] = active_window_.stop_record.trigger_reason;
    ev["evidence_status"] = active_window_.stop_record.evidence_status;
    ev["stage"] = active_window_.stop_record.stage;
    ev["related_anomalies"] = active_window_.stop_record.related_anomalies;
    ev["recovery_or_terminal"] = active_window_.stop_record.recovery_or_terminal;
    ev["evidence_window_frames"] = win_arr.size();
    ev["evidence_window"] = win_arr;

    std::ofstream jsonl(jsonl_path_, std::ios::app);
    if (jsonl.is_open()) {
      jsonl << ev.dump() << "\n";
    }

    if (!stop_records_.empty()) {
      stop_records_.back().evidence_window = win_arr;
      stop_records_.back().stop_category = active_window_.stop_record.stop_category;
      stop_records_.back().related_anomalies = active_window_.stop_record.related_anomalies;
      stop_records_.back().recovery_or_terminal = active_window_.stop_record.recovery_or_terminal;
    }
  }

  void update_pose_cache()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.01));
      std::lock_guard<std::mutex> lock(pose_mutex_);
      cached_robot_pose_.x = transform.transform.translation.x;
      cached_robot_pose_.y = transform.transform.translation.y;
      cached_robot_pose_.yaw = tf2::getYaw(transform.transform.rotation);
      cached_robot_pose_.valid = true;
      cached_robot_pose_.stamp = transform.header.stamp;
      cached_robot_pose_.frame_id = global_frame_;
    } catch (const tf2::TransformException &) {
      // 保持已有缓存，避免高频日志阶段每条同步阻塞
    }
  }

  RobotPose get_current_pose()
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return cached_robot_pose_;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_obstacles_.min_dist = 99.0;
    latest_obstacles_.min_front = 99.0;
    latest_obstacles_.min_rear = 99.0;
    latest_obstacles_.valid = true;
    latest_obstacles_.stamp = msg->header.stamp;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (r < latest_obstacles_.min_dist) {
        latest_obstacles_.min_dist = r;
      }
      // 前方扇区：±45° (±0.785 rad)
      if (std::abs(angle) <= 0.785) {
        if (r < latest_obstacles_.min_front) latest_obstacles_.min_front = r;
      }
      // 后方扇区：> 135° (2.356 rad)
      if (std::abs(angle) >= 2.356) {
        if (r < latest_obstacles_.min_rear) latest_obstacles_.min_rear = r;
      }
    }
  }

  void on_loc_ready(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    loc_ready_ = msg->data;
    loc_ready_stamp_ = this->now();
  }

  void on_loc_diag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    loc_diag_stamp_ = msg->header.stamp;
    for (const auto & status : msg->status) {
      if (status.name == "localization_monitor") {
        latest_loc_diag_summary_ = status.message;
      }
    }
  }

  void on_chassis_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & s : msg->status) {
      if (s.name == "carcar/rosmaster") {
        chassis_diag_stamp_ = this->now();
        for (const auto & kv : s.values) {
          if (kv.key == "watchdog_stopped") {
            chassis_watchdog_stopped_ = (kv.value == "True" || kv.value == "true");
          } else if (kv.key == "last_cmd_age_s") {
            try { chassis_last_cmd_age_s_ = std::stod(kv.value); } catch (...) {}
          } else if (kv.key == "battery_voltage") {
            try { chassis_battery_voltage_ = std::stod(kv.value); } catch (...) {}
          }
        }
      }
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = *msg;
    odom_stamp_ = this->now();

    const double vx = msg->twist.twist.linear.x;
    const double wz = msg->twist.twist.angular.z;

    // 倒车运动状态判定与位移统计
    if (current_stage_ == NavStage::BACKING_UP) {
      if (vx < -0.01) {
        if (!backup_odom_moving_) {
          backup_odom_moving_ = true;
          record_event_locked("BACKUP_ODOM_FEEDBACK_MOVING", active_uuid_, "ntpe_BackUp",
            "里程计反馈产生后退位移 (vx=" + std::to_string(vx) + " m/s)");
        }
      }
    }

    // 轮式里程计运动反馈与静止判据：|vx| < 0.01 且 |wz| < 0.02 持续 1.0 秒判定为静止
    const bool within_still_bounds = (std::abs(vx) < motion_linear_still_threshold_ &&
                                      std::abs(wz) < motion_angular_still_threshold_);
    if (within_still_bounds) {
      if (still_start_stamp_.nanoseconds() == 0) {
        still_start_stamp_ = this->now();
      }
      if ((this->now() - still_start_stamp_).seconds() >= motion_still_duration_) {
        if (!is_still_) {
          is_still_ = true;
          motion_feedback_str_ = "静止";
          check_stop_trigger_locked("MOTION_STILL", "底盘里程计速度持续归零，车体进入物理静止");
        } else {
          motion_feedback_str_ = "静止";
        }
      } else {
        motion_feedback_str_ = "减速中";
      }
    } else {
      still_start_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      is_still_ = false;
      if (vx > motion_linear_still_threshold_) {
        motion_feedback_str_ = "前进";
      } else if (vx < -motion_linear_still_threshold_) {
        motion_feedback_str_ = "后退";
      } else if (std::abs(wz) >= motion_angular_still_threshold_) {
        motion_feedback_str_ = "自转";
      }
    }
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto robot_pose = get_current_pose();
    const std::string goal_id = "goal_" + std::to_string(++goal_counter_);

    unassociated_goal_ = GoalTask();
    unassociated_goal_.goal_id = goal_id;
    unassociated_goal_.uuid = "PENDING_ACTION_UUID";
    unassociated_goal_.action_type = "NavigateToPose";
    unassociated_goal_.start_time = get_iso_timestamp();
    unassociated_goal_.target_x = msg->pose.position.x;
    unassociated_goal_.target_y = msg->pose.position.y;
    unassociated_goal_.target_yaw = tf2::getYaw(msg->pose.orientation);
    unassociated_goal_.frame_id = msg->header.frame_id;
    unassociated_goal_.start_pose = robot_pose;
    unassociated_goal_.is_active = true;

    record_event_locked("GOAL_POSE_RECEIVED", "NONE", "RViz",
      "收到 /goal_pose 目标点: 目标=(" + std::to_string(unassociated_goal_.target_x) +
      ", " + std::to_string(unassociated_goal_.target_y) + ") [待关联 Action UUID]");
  }

  void on_action_status(const std::string & action_type, const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::string selected = goal_status_policy_.observe(*msg);
    const bool switching = !selected.empty() && selected != active_uuid_;
    const std::string previous = active_uuid_;
    if (switching) {
      active_uuid_ = selected;
      active_bt_nodes_.clear(); active_bt_node_types_.clear();
      latest_anomaly_.clear();
      latest_stop_record_ = StopRecord{};
      latest_stop_record_.cause_code = "NONE";
      latest_stop_record_.evidence_status = "NONE";
      transition_stage_locked(NavStage::PLANNING, "新的 EXECUTING 导航目标激活");
      if (!previous.empty()) {
        record_event_locked("CURRENT_GOAL_CHANGED",selected,"bt_navigator",
          "当前任务切换：旧 UUID="+previous+"；不据此推断旧目标 ABORTED 的原因");
      }
    }
    for (const auto & s : msg->status_list) {
      const std::string uuid = uuid_to_hex(s.goal_info.goal_id.uuid);

      // 若为新 UUID，注册独立目标记录（旧目标终态不得覆盖新目标）
      if (goals_by_uuid_.find(uuid) == goals_by_uuid_.end()) {
        GoalTask task;
        task.uuid = uuid;
        task.action_type = action_type;
        task.goal_id = "action_" + uuid_short(uuid);
        task.start_time = get_iso_timestamp();
        task.is_active = s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;

        if (unassociated_goal_.is_active && uuid == selected) {
          task.target_x = unassociated_goal_.target_x;
          task.target_y = unassociated_goal_.target_y;
          task.target_yaw = unassociated_goal_.target_yaw;
          task.frame_id = unassociated_goal_.frame_id;
          task.start_pose = unassociated_goal_.start_pose;
          unassociated_goal_.is_active = false;
        } else {
          task.start_pose = get_current_pose();
        }
        goals_by_uuid_[uuid] = task;
        if (s.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || task.is_active) {
          record_event_locked("ACTION_GOAL_ACCEPTED", uuid, "bt_navigator",
            "导航目标已被接受 (" + action_type + ")");
        }
      }

      GoalTask & task = goals_by_uuid_[uuid];
      if (task.action_status == s.status) {
        continue;
      }
      task.action_status = s.status;

      if (s.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        task.is_active = false;
        task.is_terminal = true;
        task.end_time = get_iso_timestamp();
        if (active_uuid_ == uuid) {
          active_bt_nodes_.clear();
          active_bt_node_types_.clear();
          transition_stage_locked(NavStage::SUCCEEDED, "导航成功抵达目标点");
        }
        record_event_locked("NAVIGATION_SUCCEEDED", uuid, "bt_navigator", "导航目标成功抵达 (SUCCEEDED)");
        if (goal_status_policy_.owns_terminal(uuid)) {
          check_stop_trigger_locked("SUCCEEDED", "导航目标正常抵达达成", "GOAL_REACHED");
        }
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
        task.is_active = false;
        task.is_terminal = true;
        task.end_time = get_iso_timestamp();
        if (active_uuid_ == uuid) {
          active_bt_nodes_.clear();
          active_bt_node_types_.clear();
          transition_stage_locked(NavStage::FAILED, "导航任务中止失败 (ABORTED)");
        }
        record_event_locked("NAVIGATION_ABORTED", uuid, "bt_navigator", "导航任务异常中止 (ABORTED)");
        if (goal_status_policy_.owns_terminal(uuid)) {
          check_stop_trigger_locked("ABORTED", "导航任务异常中止 (ABORTED)", "CAUSE_UNDETERMINED");
        }
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        task.is_active = false;
        task.is_terminal = true;
        task.end_time = get_iso_timestamp();
        if (active_uuid_ == uuid) {
          active_bt_nodes_.clear();
          active_bt_node_types_.clear();
          transition_stage_locked(NavStage::CANCELED, "导航任务被取消 (CANCELED)");
        }
        record_event_locked("NAVIGATION_CANCELED", uuid, "bt_navigator", "导航任务已被取消 (CANCELED)");
        if (goal_status_policy_.owns_terminal(uuid)) {
          check_stop_trigger_locked("CANCELED", "导航任务已被取消 (CANCELED)", "USER_CANCELLED");
        }
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        task.is_active = uuid == selected;
      }
    }
  }

  void on_bt_log(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & event : msg->event_log) {
      const std::string & node = event.node_name;
      const std::string & prev = event.previous_status;
      const std::string & curr = event.current_status;

      if (prev == curr) continue;

      const std::string btype = resolve_bt_node_type(node);
      record_bt_event_locked(node, btype, prev, curr, event.timestamp);

      if (curr == "RUNNING") {
        active_bt_nodes_.insert(node);
        active_bt_node_types_[node] = btype;
      } else {
        active_bt_nodes_.erase(node);
        active_bt_node_types_.erase(node);
      }

      // 1. 叶节点：WaitForLocalizationStatus
      if (btype == "WaitForLocalizationStatus") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::WAITING_FOR_LOCALIZATION, "正在等待定位健康就绪");
        }
      }
      // 2. 叶节点：ComputePathToPose / ComputePathThroughPoses
      else if (btype == "ComputePathToPose" || btype == "ComputePathThroughPoses") {
        if (curr == "RUNNING") {
          is_replanning_ = true;
          replan_counter_++;
          if (has_active_node_type("FollowPath")) {
            transition_stage_locked(NavStage::PATH_FOLLOWING_REPLANNING, "周期重规划触发 (路径跟随中)");
          } else {
            transition_stage_locked(NavStage::PLANNING, "全局路径规划中");
          }
        } else {
          is_replanning_ = false;
          if (curr == "SUCCESS") {
            successful_replan_count_++;
            const auto now = this->now();
            if (last_replan_summary_time_.nanoseconds() == 0) {
              last_replan_summary_time_ = now;
            }
            if ((now - last_replan_summary_time_).seconds() >= replanning_summary_interval_s_) {
              std::cout << "\n[" << get_time_only_str() << "] [重规划汇总] 过去 "
                        << static_cast<int>(replanning_summary_interval_s_) << "s 内成功重规划 "
                        << successful_replan_count_ << " 次" << std::endl;
              record_event_locked("REPLANNING_SUMMARY", active_uuid_, "planner_server",
                "重规划汇总: 过去 " + std::to_string(static_cast<int>(replanning_summary_interval_s_)) +
                "s 内成功重规划 " + std::to_string(successful_replan_count_) + " 次");
              successful_replan_count_ = 0;
              last_replan_summary_time_ = now;
            }
            if (has_active_node_type("FollowPath")) {
              transition_stage_locked(NavStage::PATH_FOLLOWING, "全局重规划成功完成，继续跟随");
            }
          } else if (curr == "FAILURE") {
            record_event_locked("PLAN_FAILED", active_uuid_, node, "全局规划失败 (无通行路径)");
            check_stop_trigger_locked("PLAN_FAILED", "全局路径规划失败无通行路径", "PLAN_FAILED");
          }
        }
      }
      // 3. 叶节点：FollowPath
      else if (btype == "FollowPath") {
        if (curr == "RUNNING") {
          if (has_active_node_type("ComputePathToPose") || has_active_node_type("ComputePathThroughPoses")) {
            transition_stage_locked(NavStage::PATH_FOLLOWING_REPLANNING, "开始执行路径跟随 (并行重规划中)");
          } else {
            transition_stage_locked(NavStage::PATH_FOLLOWING, "开始执行路径跟随");
          }
        } else if (curr == "FAILURE") {
          record_event_locked("CONTROL_FAILED", active_uuid_, node, "局部控制器失败 (受阻或偏离)");
          check_stop_trigger_locked("CONTROL_FAILED", "局部控制器失败 (受阻或偏离)", "CONTROL_FAILED");
        }
      }
      // 4. 叶节点：Wait
      else if (btype == "Wait") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::RECOVERY_WAITING, "触发恢复等待 (" + node + ")");
        }
      }
      // 5. 叶节点：ControlledSpin / Spin
      else if (btype == "ControlledSpin" || btype == "Spin") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::SPINNING, "触发受控转向恢复 (" + node + ")");
        }
      }
      // 6. 叶节点：RearClear (后方净空检查)
      else if (btype == "RearClear") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::REAR_CHECK, "执行倒车后方净空检查");
        }
      }
      // 7. 叶节点：BackUp (真实倒车动作，NAV-012 参数更新：限额 0.10m, 0.05m/s, 最长 4.0s)
      else if (btype == "BackUp") {
        if (curr == "RUNNING") {
          backup_odom_moving_ = false;
          transition_stage_locked(NavStage::BACKING_UP, "执行受限倒车恢复 (限额 0.10m, 0.05m/s, 最长 4.0s)");
          record_event_locked("BACKUP_ACTION_STARTED", active_uuid_, node, "倒车 Action 开始执行");
        } else if (curr == "SUCCESS") {
          record_event_locked("BACKUP_ACTION_SUCCESS", active_uuid_, node, "倒车 Action 完成");
        } else if (curr == "FAILURE") {
          record_event_locked("BACKUP_ACTION_FAILURE", active_uuid_, node, "倒车 Action 失败中止");
          check_stop_trigger_locked("BACKUP_ACTION_FAILURE", "倒车 Action 失败中止", "CONTROL_FAILED");
        }
      }
      // 8. 叶节点：ParkAndObserve (停车观察 30 秒)
      else if (btype == "ParkAndObserve") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::PARK_AND_OBSERVE, "执行停车观察 (预算 30s，每秒轮询通道与定位)");
          check_stop_trigger_locked("PARK_AND_OBSERVE", "触发停车观察脱困 (30s 周期轮询)", "PARK_AND_OBSERVE");
        }
      }
      // 9. 叶节点：RecoverLocalization
      else if (btype == "RecoverLocalization") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::RECOVERING_LOCALIZATION, "定位恢复中 (静止更新/全局重定位)");
        }
      }
    }
  }

  void on_progress_guard_status(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    progress_guard_stamp_ = this->now();
    latest_progress_guard_msg_ = msg->message;
    std::string reason_code = "PROGRESS_STAGNATION";
    std::string detail;
    for (const auto & kv : msg->values) {
      if (kv.key == "reason_code") reason_code = kv.value;
      if (kv.key == "detail") detail = kv.value;
    }

    if (msg->level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      record_event_locked("PROGRESS_GUARD_STAGNATION", active_uuid_, "ProgressGuard",
        "进展守卫警告: " + msg->message + " (" + detail + ")");
      check_stop_trigger_locked("PROGRESS_GUARD_STAGNATION", "进展停滞判定: " + msg->message + " (" + detail + ")", reason_code);
    }
  }

  void on_rear_clear_status(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool passed = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
    const std::string reason = msg->message;
    std::string reason_code = "COSTMAP_OBSTACLE";
    std::string detail;
    for (const auto & kv : msg->values) {
      if (kv.key == "reason_code") reason_code = kv.value;
      if (kv.key == "detail") detail = kv.value;
    }

    if (passed) {
      record_event_locked("REAR_CLEAR_PASSED", active_uuid_, "RearClear",
        "后方检查通过: 净空正常，允许倒车");
    } else {
      record_event_locked("REAR_CLEAR_DENIED", active_uuid_, "RearClear",
        "后方检查拒绝: 原因=" + reason + " (" + detail + ")");
      check_stop_trigger_locked("REAR_CLEAR_DENIED", "后方检查拒绝倒车: " + reason + " (" + detail + ")", reason_code);
      std::cout << "\n[" << get_time_only_str() << "] [告警] 后方检查拒绝倒车: 原因="
                << reason << " (" << detail << ")\n" << std::endl;
    }
  }

  void on_backup_action_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & s : msg->status_list) {
      if (s.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        record_event_locked("BACKUP_STATUS_SUCCEEDED", active_uuid_, "behavior_server",
          "BackUp Action 状态: SUCCEEDED (已完成倒车)");
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
        record_event_locked("BACKUP_STATUS_ABORTED", active_uuid_, "behavior_server",
          "BackUp Action 状态: ABORTED (倒车中止)");
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        record_event_locked("BACKUP_STATUS_CANCELED", active_uuid_, "behavior_server",
          "BackUp Action 状态: CANCELED (倒车取消)");
      }
    }
  }

  void on_backup_feedback(const nav2_msgs::action::BackUp_FeedbackMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_backup_feedback_dist_ = msg->feedback.distance_traveled;
    last_backup_feedback_stamp_ = this->now();
  }

  void on_rosout(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    if (msg->level < rcl_interfaces::msg::Log::WARN) {
      return;
    }

    const std::string text = msg->msg;
    std::string key_fault;

    if (text.find("No valid trajectories") != std::string::npos) {
      key_fault = "DWB局部规划无可行轨迹 (障碍过近或包络碰撞)";
    } else if (text.find("Failed to make progress") != std::string::npos) {
      key_fault = "小车进展超时 (progress checker timeout)";
    } else if (text.find("Oscillation") != std::string::npos) {
      key_fault = "控制器振荡淘汰";
    } else if (text.find("Controller patience exceeded") != std::string::npos) {
      key_fault = "控制器计算超时 (patience exceeded)";
    }

    // 监测关键节点告警/错误 (controller, planner, bt_navigator, amcl, rosmaster, etc.)
    std::string node_lower = msg->name;
    std::transform(node_lower.begin(), node_lower.end(), node_lower.begin(), ::tolower);
    const bool is_key_node = (node_lower.find("controller") != std::string::npos ||
                              node_lower.find("planner") != std::string::npos ||
                              node_lower.find("bt_navigator") != std::string::npos ||
                              node_lower.find("amcl") != std::string::npos ||
                              node_lower.find("rosmaster") != std::string::npos ||
                              node_lower.find("behavior") != std::string::npos ||
                              node_lower.find("localization") != std::string::npos);

    if (!key_fault.empty()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_anomaly_ = key_fault;
      record_event_locked("CONTROLLER_FAULT", active_uuid_, msg->name, key_fault + ": " + text);

      if (text.find("No valid trajectories") != std::string::npos) {
        check_stop_trigger_locked("DWB_NO_TRAJECTORY", key_fault);
      }

      // 高频重复告警合并与计数 (维护首次、末次与计数)
      const auto now = this->now();
      auto & agg = alert_aggregators_[key_fault];
      if (agg.count == 0) {
        agg.count = 1;
        agg.first_time = now;
        agg.last_time = now;
        agg.node = msg->name;
        agg.msg = text;
        std::cout << "\n[" << get_time_only_str() << "] [告警] " << key_fault << std::endl;
      } else {
        agg.count++;
        const double gap = (now - agg.last_time).seconds();
        agg.last_time = now;
        if (gap > 2.0 || agg.count % 5 == 0) {
          std::cout << "[" << get_time_only_str() << "] [告警合并] " << key_fault
                    << " (连续触发 " << agg.count << " 次, 节点: " << msg->name << ")" << std::endl;
        }
      }
    } else if (is_key_node) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_anomaly_ = msg->name + ": " + text;
      if (msg->level >= rcl_interfaces::msg::Log::ERROR) {
        record_event_locked("NODE_ERROR", active_uuid_, msg->name, text);
      }
    }
  }

  void transition_stage_locked(NavStage next_stage, const std::string & reason)
  {
    if (current_stage_ == next_stage) {
      return;
    }
    const auto prev_stage = current_stage_;
    current_stage_ = next_stage;
    stage_start_time_ = std::chrono::steady_clock::now();

    const std::string prev_str = stage_to_string(prev_stage);
    const std::string next_str = stage_to_string(next_stage);

    record_event_locked("STAGE_TRANSITION", active_uuid_, "NavigationState",
      "阶段切换: [" + prev_str + "] -> [" + next_str + "] (" + reason + ")");

    // 阶段变化输出：周期重规划在终端内合并计数，不每秒刷屏；其余关键流转即时输出
    const bool is_replan_transition = (
      (prev_stage == NavStage::PATH_FOLLOWING && next_stage == NavStage::PATH_FOLLOWING_REPLANNING) ||
      (prev_stage == NavStage::PATH_FOLLOWING_REPLANNING && next_stage == NavStage::PATH_FOLLOWING));

    if (!is_replan_transition || full_event_display_) {
      std::cout << "\n[" << get_time_only_str() << "] >>> 阶段切换: [" << prev_str
                << "] -> [" << next_str << "] (" << reason << ")" << std::endl;
    }

    publish_navigation_status_locked(this->now());
  }

  void check_stop_trigger_locked(
    const std::string & trigger_source,
    const std::string & detail,
    const std::string & reason_code = "")
  {
    StopRecord rec;
    rec.timestamp = get_iso_timestamp();
    rec.stop_event_id = ++stop_event_counter_;
    rec.goal_uuid = active_uuid_.empty() ? "UNASSOCIATED" : active_uuid_;
    rec.stage = stage_to_string(current_stage_);
    rec.motion_feedback = motion_feedback_str_;
    rec.related_anomalies = latest_anomaly_;

    if (!active_uuid_.empty() && goals_by_uuid_.find(active_uuid_) != goals_by_uuid_.end()) {
      rec.action_type = goals_by_uuid_[active_uuid_].action_type;
    } else {
      rec.action_type = "None";
    }

    const auto robot = get_current_pose();
    rec.robot_x = robot.x;
    rec.robot_y = robot.y;
    rec.robot_yaw = robot.yaw;
    rec.cmd_vel_nav_vx = latest_cmd_vel_nav_.linear.x;
    rec.cmd_vel_nav_wz = latest_cmd_vel_nav_.angular.z;
    rec.cmd_vel_vx = latest_cmd_vel_.linear.x;
    rec.cmd_vel_wz = latest_cmd_vel_.angular.z;
    rec.odom_vx = latest_odom_.twist.twist.linear.x;
    rec.odom_wz = latest_odom_.twist.twist.angular.z;

    // 判定原因与证据置信度
    // 判定停车大类、触发原因与恢复/任务终态
    if (trigger_source == "SUCCEEDED") {
      rec.stop_category = "GOAL_REACHED";
      rec.cause_code = "GOAL_REACHED";
      rec.trigger_reason = "目标正常抵达达成 (SUCCEEDED)";
      rec.evidence_status = "CERTAIN";
      rec.recovery_or_terminal = "正常到达";
    } else if (trigger_source == "CANCELED") {
      rec.stop_category = "USER_CANCELLED";
      rec.cause_code = "USER_CANCELLED";
      rec.trigger_reason = "导航任务被取消 (CANCELED)";
      rec.evidence_status = "CERTAIN";
      rec.recovery_or_terminal = "用户取消";
    } else if (trigger_source == "ABORTED") {
      rec.stop_category = "NAVIGATION_FAILED";
      rec.trigger_reason = "导航任务异常中止 (ABORTED)";
      rec.evidence_status = "CERTAIN";
      rec.recovery_or_terminal = "导航中止失败";
      if (!latest_stop_record_.cause_code.empty() &&
          latest_stop_record_.cause_code != "NONE" &&
          latest_stop_record_.cause_code != "GOAL_REACHED") {
        rec.cause_code = latest_stop_record_.cause_code;
      } else {
        rec.cause_code = "CAUSE_UNDETERMINED";
      }
    } else if (trigger_source == "PARK_AND_OBSERVE") {
      rec.stop_category = "ACTIVE_WAIT";
      rec.cause_code = "PARK_AND_OBSERVE";
      rec.trigger_reason = "触发停车观察脱困 (PARK_AND_OBSERVE, 预算30s)";
      rec.evidence_status = "CERTAIN";
      rec.recovery_or_terminal = "停车观察脱困中";
    } else if (trigger_source == "REAR_CLEAR_DENIED") {
      rec.stop_category = "NAVIGATION_FAILED";
      rec.trigger_reason = "后方检查拒绝倒车 (REAR_CLEAR_DENIED)";
      rec.evidence_status = "CERTAIN";
      rec.recovery_or_terminal = "倒车受阻等待";
      if (!reason_code.empty()) {
        if (reason_code == "COSTMAP_OUT_OF_BOUNDS" || reason_code == "COSTMAP_UNKNOWN") {
          rec.cause_code = "COSTMAP_OBSTACLE";
        } else {
          rec.cause_code = reason_code;
        }
      } else {
        rec.cause_code = "COSTMAP_OBSTACLE";
      }
    } else if (trigger_source == "PROGRESS_GUARD_STAGNATION") {
      rec.stop_category = "ACTIVE_WAIT";
      rec.cause_code = reason_code.empty() ? "PROGRESS_STAGNATION" : reason_code;
      rec.trigger_reason = "进展停滞守卫触发 (PROGRESS_GUARD: " + rec.cause_code + ")";
      rec.evidence_status = "CERTAIN";
      rec.recovery_or_terminal = "触发恢复行为";
    } else if (trigger_source == "DWB_NO_TRAJECTORY") {
      rec.stop_category = "ACTIVE_WAIT";
      rec.cause_code = "DWB_NO_TRAJECTORY";
      rec.trigger_reason = "DWB局部规划无可行轨迹 (障碍过近或包络碰撞)";
      rec.evidence_status = "CORRELATED";
      rec.recovery_or_terminal = "等待规划恢复";
    } else if (trigger_source == "PLAN_FAILED") {
      rec.stop_category = "NAVIGATION_FAILED";
      rec.cause_code = "PLAN_FAILED";
      rec.trigger_reason = "全局路径规划失败 (无通行路径)";
      rec.evidence_status = "CORRELATED";
      rec.recovery_or_terminal = "规划重试";
    } else if (trigger_source == "CONTROL_FAILED") {
      rec.stop_category = "NAVIGATION_FAILED";
      rec.cause_code = "CONTROL_FAILED";
      rec.trigger_reason = "局部控制器执行失败 (受阻或偏离)";
      rec.evidence_status = "CORRELATED";
      rec.recovery_or_terminal = "触发恢复行为";
    } else if (trigger_source == "BACKUP_ACTION_FAILURE") {
      rec.stop_category = "NAVIGATION_FAILED";
      rec.cause_code = "CONTROL_FAILED";
      rec.trigger_reason = "受限倒车动作失败中止";
      rec.evidence_status = "CORRELATED";
      rec.recovery_or_terminal = "倒车失败中止";
    } else if (trigger_source == "MOTION_STILL") {
      // 普通归零检查：绝不单凭零速度或看门狗判定为唯一根因
      // 绝不将零速度、单次规划失败或 watchdog_stopped=true 粗暴定为唯一根因，仅作为线索
      const bool zero_cmd = (std::abs(latest_cmd_vel_.linear.x) < 0.001 &&
                             std::abs(latest_cmd_vel_.angular.z) < 0.001);
      const auto now = this->now();
      if (current_stage_ == NavStage::SUCCEEDED) {
        rec.stop_category = "GOAL_REACHED";
        rec.cause_code = "GOAL_REACHED";
        rec.trigger_reason = "到达目标点后正常受控停车";
        rec.evidence_status = "CERTAIN";
        rec.recovery_or_terminal = "正常到达";
      } else if (current_stage_ == NavStage::IDLE) {
        rec.stop_category = "ACTIVE_WAIT";
        rec.cause_code = "NONE";
        rec.trigger_reason = "底盘处于待命状态";
        rec.evidence_status = "NONE";
        rec.recovery_or_terminal = "待命";
      } else if (current_stage_ == NavStage::PATH_FOLLOWING_REPLANNING) {
        rec.stop_category = "ACTIVE_WAIT";
        rec.cause_code = "NONE";
        rec.trigger_reason = "重规划期间减速等待";
        rec.evidence_status = "NONE";
        rec.recovery_or_terminal = "继续跟随";
      } else if (chassis_watchdog_stopped_ && chassis_last_cmd_age_s_ > 0.3 &&
                 (current_stage_ == NavStage::PATH_FOLLOWING || current_stage_ == NavStage::BACKING_UP || current_stage_ == NavStage::SPINNING)) {
        rec.stop_category = "UPSTREAM_COMMAND_STALE";
        rec.cause_code = "CAUSE_UNDETERMINED";
        rec.trigger_reason = "上游速度已过期且底盘处于停车状态；watchdog_stopped 也可能由零速置位，不能据此认定超时触发";
        rec.evidence_status = "CORRELATED";
        rec.recovery_or_terminal = "等待上游刷新指令";
      } else if (odom_stamp_.nanoseconds() > 0 && (now - odom_stamp_).seconds() > motion_data_timeout_) {
        rec.stop_category = "DATA_TIMEOUT";
        rec.cause_code = "DATA_EXPIRED";
        rec.trigger_reason = "传感器/里程计数据超时 (超过 " + std::to_string(motion_data_timeout_) + "s 未刷新)";
        rec.evidence_status = "CORRELATED";
        rec.recovery_or_terminal = "等待传感器数据恢复";
      } else if (zero_cmd) {
        rec.stop_category = "ACTIVE_WAIT";
        rec.cause_code = "NONE";
        rec.trigger_reason = "受控停车/平滑减速归零";
        rec.evidence_status = "NONE";
        rec.recovery_or_terminal = "受控状态";
      } else {
        rec.stop_category = "CAUSE_UNDETERMINED";
        rec.cause_code = "CAUSE_UNDETERMINED";
        rec.trigger_reason = "指令非零但车体静止 (疑似受阻或打滑)";
        rec.evidence_status = "CORRELATED";
        rec.recovery_or_terminal = "原因待定";
      }
    } else {
      rec.stop_category = "CAUSE_UNDETERMINED";
      rec.cause_code = reason_code.empty() ? "CAUSE_UNDETERMINED" : reason_code;
      rec.trigger_reason = detail;
      rec.evidence_status = "INSUFFICIENT";
      rec.recovery_or_terminal = "原因未确定";
    }

    // 收集多维度线索
    std::ostringstream ss_cmd;
    ss_cmd << "两级指令: nav(vx=" << std::fixed << std::setprecision(2) << latest_cmd_vel_nav_.linear.x
           << ",wz=" << latest_cmd_vel_nav_.angular.z << ") cmd(vx="
           << latest_cmd_vel_.linear.x << ",wz=" << latest_cmd_vel_.angular.z << ")";
    rec.clues.push_back(ss_cmd.str());

    std::ostringstream ss_odom;
    ss_odom << "里程计: vx=" << std::fixed << std::setprecision(2) << latest_odom_.twist.twist.linear.x
            << ",wz=" << latest_odom_.twist.twist.angular.z << " (反馈: " << motion_feedback_str_ << ")";
    rec.clues.push_back(ss_odom.str());

    std::ostringstream ss_diag;
    ss_diag << "底盘诊断: 看门狗停车=" << (chassis_watchdog_stopped_ ? "true" : "false")
            << ",上游延时=" << std::fixed << std::setprecision(3) << chassis_last_cmd_age_s_ << "s"
            << ",电压=" << std::setprecision(2) << chassis_battery_voltage_ << "V";
    rec.clues.push_back(ss_diag.str());

    if (latest_obstacles_.valid) {
      std::ostringstream ss_obs;
      ss_obs << "障碍特征: 最近=" << std::fixed << std::setprecision(2) << latest_obstacles_.min_dist
             << "m(前:" << latest_obstacles_.min_front << "m,后:" << latest_obstacles_.min_rear << "m)";
      rec.clues.push_back(ss_obs.str());
    }

    if (!loc_ready_) {
      rec.clues.push_back("定位未就绪/丢失");
    }

    stop_records_.push_back(rec);
    latest_stop_record_ = rec;

    // 触发或合并 5Hz 停车证据滑动窗口
    trigger_stop_window_locked(rec);

    // 写入 JSONL
    nlohmann::json item;
    item["timestamp"] = rec.timestamp;
    item["event_index"] = ++event_counter_;
    item["event_type"] = "NAVIGATION_STOP";
    item["goal_uuid"] = rec.goal_uuid;
    item["action_type"] = rec.action_type;
    item["stop_category"] = rec.stop_category;
    item["cause_code"] = rec.cause_code;
    item["stop_event_id"] = rec.stop_event_id;
    item["trigger_reason"] = rec.trigger_reason;
    item["evidence_status"] = rec.evidence_status;
    item["stage"] = rec.stage;
    item["motion_feedback"] = rec.motion_feedback;
    item["related_anomalies"] = rec.related_anomalies;
    item["recovery_or_terminal"] = rec.recovery_or_terminal;
    item["cmd_vel_nav"] = {{"vx", rec.cmd_vel_nav_vx}, {"wz", rec.cmd_vel_nav_wz}};
    item["cmd_vel"] = {{"vx", rec.cmd_vel_vx}, {"wz", rec.cmd_vel_wz}};
    item["odom_vel"] = {{"vx", rec.odom_vx}, {"wz", rec.odom_wz}};
    item["robot_pose"] = {{"valid", robot.valid}, {"x", robot.x}, {"y", robot.y}, {"yaw", robot.yaw}};
    item["obstacles"] = {{"valid", latest_obstacles_.valid}, {"min_dist", latest_obstacles_.min_dist},
                         {"min_front", latest_obstacles_.min_front}, {"min_rear", latest_obstacles_.min_rear}};
    item["clues"] = rec.clues;

    std::ofstream jsonl(jsonl_path_, std::ios::app);
    if (jsonl.is_open()) {
      jsonl << item.dump() << "\n";
    }

    // 写入 events.log
    std::ofstream txt(text_log_path_, std::ios::app);
    if (txt.is_open()) {
      txt << "[" << rec.timestamp << "] [NAVIGATION_STOP       ] [" << uuid_short(rec.goal_uuid)
          << "] [" << rec.stage << "] 大类=" << rec.stop_category
          << " 触发原因=" << rec.trigger_reason
          << " [判定: " << rec.evidence_status << "] 异常=" << rec.related_anomalies
          << " 终态=" << rec.recovery_or_terminal << " | " << ss_cmd.str()
          << " | " << ss_odom.str() << " | " << ss_diag.str() << "\n";
    }

    publish_navigation_status_locked(this->now());
  }

  void record_bt_event_locked(
    const std::string & raw_node, const std::string & behavior_type,
    const std::string & prev, const std::string & curr,
    const builtin_interfaces::msg::Time & bt_stamp)
  {
    if (!full_bt_debug_events_) {
      // 仅记录关键节点流转或失败事件，避免高频控制流节点充斥日志
      const bool is_key_node = (
        behavior_type == "ComputePathToPose" || behavior_type == "ComputePathThroughPoses" ||
        behavior_type == "FollowPath" || behavior_type == "BackUp" ||
        behavior_type == "ParkAndObserve" || behavior_type == "ControlledSpin" ||
        behavior_type == "Spin" || behavior_type == "Wait" ||
        behavior_type == "RearClear" || behavior_type == "RecoverLocalization" ||
        behavior_type == "WaitForLocalizationStatus");
      const bool is_failure = (curr == "FAILURE");
      if (!is_key_node && !is_failure) {
        return;
      }
    }

    event_counter_++;
    const auto iso_time = get_iso_timestamp();

    std::ostringstream ss_bt_time;
    ss_bt_time << bt_stamp.sec << "." << std::setfill('0') << std::setw(9) << bt_stamp.nanosec;

    nlohmann::json item;
    item["timestamp"] = iso_time;
    item["event_index"] = event_counter_;
    item["event_type"] = "BT_NODE_TRANSITION";
    item["goal_uuid"] = active_uuid_.empty() ? "NONE" : active_uuid_;
    item["raw_node"] = raw_node;
    item["behavior_type"] = behavior_type;
    item["previous_status"] = prev;
    item["current_status"] = curr;
    item["event_time"] = ss_bt_time.str();
    item["stage"] = stage_to_string(current_stage_);

    std::ofstream jsonl(jsonl_path_, std::ios::app);
    if (jsonl.is_open()) {
      jsonl << item.dump() << "\n";
    }
  }

  void record_event_locked(
    const std::string & event_type, const std::string & uuid,
    const std::string & node_name, const std::string & description,
    const nlohmann::json & extra = nlohmann::json::object())
  {
    event_counter_++;
    const auto iso_time = get_iso_timestamp();
    const auto robot = get_current_pose();
    const auto obs = latest_obstacles_;

    // 1. 追加写 JSONL 流
    nlohmann::json item;
    item["timestamp"] = iso_time;
    item["event_index"] = event_counter_;
    item["event_type"] = event_type;
    item["goal_uuid"] = uuid.empty() ? "NONE" : uuid;
    item["node"] = node_name;
    item["stage"] = stage_to_string(current_stage_);
    item["description"] = description;
    if (!extra.empty()) {item["recovery"] = extra;}
    item["robot_pose"] = {
      {"valid", robot.valid},
      {"x", robot.x},
      {"y", robot.y},
      {"yaw", robot.yaw}
    };
    item["obstacles"] = {
      {"valid", obs.valid},
      {"min_dist", obs.min_dist},
      {"min_front", obs.min_front},
      {"min_rear", obs.min_rear}
    };

    std::ofstream jsonl(jsonl_path_, std::ios::app);
    if (jsonl.is_open()) {
      jsonl << item.dump() << "\n";
    }

    // 2. 追加写中文可读日志
    std::ofstream txt(text_log_path_, std::ios::app);
    if (txt.is_open()) {
      txt << "[" << iso_time << "] [" << std::left << std::setw(22) << event_type << "] ["
          << uuid_short(uuid) << "] [" << stage_to_string(current_stage_) << "] "
          << "Node=" << node_name << " | " << description
          << " | 车体=(" << std::fixed << std::setprecision(2) << robot.x << "," << robot.y << ")"
          << " 障最近=" << obs.min_dist << "m(前:" << obs.min_front << "m,后:" << obs.min_rear << "m)\n";
    }
  }

  void publish_terminal_summary()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = this->now();

    // 计算当前阶段持续时间
    const auto stage_duration_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - stage_start_time_).count() / 1000.0;

    // 目标标识简写
    std::string uuid_display = "无活动目标";
    std::string dist_str = "缺失";
    std::string action_type_display = "None";

    if (!active_uuid_.empty() && goals_by_uuid_.find(active_uuid_) != goals_by_uuid_.end()) {
      const auto & task = goals_by_uuid_[active_uuid_];
      uuid_display = uuid_short(active_uuid_);
      action_type_display = task.action_type;
      if (task.remaining_distance >= 0.0) {
        const double age = (now - task.last_distance_update).seconds();
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << task.remaining_distance << "m";
        if (age > 2.0) {
          ss << "(过期)";
        }
        dist_str = ss.str();
      }
    }

    // 两级指令速度检查
    std::string cmd_str = "缺失";
    const double cmd_age = (now - cmd_vel_stamp_).seconds();
    const double cmd_nav_age = (now - cmd_vel_nav_stamp_).seconds();
    if (cmd_vel_stamp_.nanoseconds() > 0 || cmd_vel_nav_stamp_.nanoseconds() > 0) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << "nav(" << latest_cmd_vel_nav_.linear.x << "," << latest_cmd_vel_nav_.angular.z << ") "
         << "cmd(" << latest_cmd_vel_.linear.x << "," << latest_cmd_vel_.angular.z << ")";
      if (cmd_age > 0.6 || cmd_nav_age > 0.6) {
        ss << "(过期)";
      }
      cmd_str = ss.str();
    }

    // 里程计速度检查与运动反馈
    std::string odom_str = "缺失";
    const double odom_age = (now - odom_stamp_).seconds();
    if (odom_stamp_.nanoseconds() > 0) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2) << "vx=" << latest_odom_.twist.twist.linear.x
         << " wz=" << latest_odom_.twist.twist.angular.z << " (" << motion_feedback_str_ << ")";
      if (odom_age > motion_data_timeout_) {
        ss << "(过期)";
      }
      odom_str = ss.str();
    }

    // 定位健康检查
    std::string loc_str = "未就绪";
    const double loc_age = (now - loc_ready_stamp_).seconds();
    if (loc_ready_stamp_.nanoseconds() > 0) {
      if (loc_ready_) {
        loc_str = (loc_age > 1.5) ? "就绪(过期)" : "正常就绪";
      } else {
        loc_str = (loc_age > 1.5) ? "未就绪(过期)" : "定位未就绪";
      }
    }

    // 录包状态
    std::string bag_str;
    if (is_recording_bag_) {
      const double mb = current_bag_bytes_ / (1024.0 * 1024.0);
      std::ostringstream ss;
      ss << "录制中(" << std::fixed << std::setprecision(1) << mb << "MB)";
      bag_str = ss.str();
    } else {
      bag_str = !record_raw_bag_ ? "未启用" : (evidence_incomplete_ ? "已停止(证据不完整)" : "已停止");
    }

    // 倒车特殊显示：倒车 Action 反馈位移（里程计反馈）
    std::string extra_backup;
    if (current_stage_ == NavStage::BACKING_UP) {
      std::ostringstream ss;
      ss << " [倒车位移: " << std::fixed << std::setprecision(2)
         << latest_backup_feedback_dist_ << "m/0.10m (里程计反馈)]";
      extra_backup = ss.str();
    }

    // 当前停车线索简写
    std::string stop_clue_str = "运行中";
    if (is_still_ || current_stage_ == NavStage::PARK_AND_OBSERVE ||
        current_stage_ == NavStage::SUCCEEDED || current_stage_ == NavStage::FAILED ||
        current_stage_ == NavStage::CANCELED) {
      stop_clue_str = latest_stop_record_.trigger_reason;
    }

    // 阶段显示：重规划次数合并
    std::string stage_display = stage_to_string(current_stage_);
    if (current_stage_ == NavStage::PATH_FOLLOWING_REPLANNING && replan_counter_ > 0) {
      stage_display += " [#" + std::to_string(replan_counter_) + "]";
    }

    // 终端 1 Hz 单行中文摘要输出
    std::ostringstream summary_ss;
    summary_ss << "[" << get_time_only_str() << "] "
               << "[目标 " << uuid_display << "] "
               << "[阶段: " << stage_display
               << " (" << std::fixed << std::setprecision(1) << stage_duration_sec << "s)] "
               << "[剩余: " << dist_str << "] "
               << "[指令: " << cmd_str << "] "
               << "[里程计: " << odom_str << "] "
               << "[定位: " << loc_str << "] "
               << "[线索: " << stop_clue_str << "] "
               << "[会话日志: 记录中] [原始 rosbag: " << bag_str << "]"
               << extra_backup;

    const std::string line = summary_ss.str();
    std::cout << line << std::endl;

    // 同步将摘要写入会话文件 terminal_summary.log
    std::ofstream sum_file(summary_log_path_, std::ios::app);
    if (sum_file.is_open()) {
      sum_file << line << "\n";
    }

    // 发布只读 /navigation/status 诊断话题
    publish_navigation_status_locked(now);
  }

  bool has_active_node_type(const std::string & target_type) const
  {
    for (const auto & pair : active_bt_node_types_) {
      if (pair.second == target_type) return true;
    }
    return false;
  }

  std::string get_active_bt_nodes_str_locked() const
  {
    if (active_bt_nodes_.empty()) {
      return "NONE";
    }
    std::ostringstream ss;
    bool first = true;
    for (const auto & pair : active_bt_node_types_) {
      if (!first) ss << ",";
      ss << pair.first << "(" << pair.second << ")";
      first = false;
    }
    return ss.str();
  }

  std::string get_task_status_locked() const
  {
    if (current_stage_ == NavStage::SUCCEEDED) return "SUCCEEDED";
    if (current_stage_ == NavStage::CANCELED) return "CANCELED";
    if (current_stage_ == NavStage::FAILED) return "FAILED";
    if (!active_uuid_.empty()) return "ACTIVE";
    return "IDLE";
  }

  void publish_navigation_status_locked(const rclcpp::Time & now)
  {
    if (!nav_status_pub_) return;

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "navigation_status";
    status.hardware_id = "carcar_navigation";

    // 级别定义
    if (current_stage_ == NavStage::FAILED) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (current_stage_ == NavStage::BACKING_UP ||
               current_stage_ == NavStage::SPINNING ||
               current_stage_ == NavStage::PARK_AND_OBSERVE ||
               current_stage_ == NavStage::RECOVERING_LOCALIZATION ||
               current_stage_ == NavStage::RECOVERY_WAITING ||
               evidence_incomplete_ || !loc_ready_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    status.message = stage_to_string(current_stage_) + " | " + motion_feedback_str_;

    auto add_kv = [&status](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      status.values.push_back(kv);
    };

    const std::string stage_code = stage_to_code(current_stage_);
    const std::string task_status = get_task_status_locked();
    const std::string bt_node_str = get_active_bt_nodes_str_locked();
    const std::string short_uuid = active_uuid_.empty() ? "无活动目标" : uuid_short(active_uuid_);
    const std::string full_uuid = active_uuid_.empty() ? "NONE" : active_uuid_;
    const std::string action_type = (!active_uuid_.empty() && goals_by_uuid_.find(active_uuid_) != goals_by_uuid_.end()) ?
      goals_by_uuid_[active_uuid_].action_type : "None";

    // 停车原因及证据等级判断
    std::string effective_cause_code = "NONE";
    std::string effective_evidence = "NONE";
    const bool is_stopped = is_still_ || current_stage_ == NavStage::PARK_AND_OBSERVE ||
                            current_stage_ == NavStage::SUCCEEDED || current_stage_ == NavStage::FAILED ||
                            current_stage_ == NavStage::CANCELED;
    if (is_stopped) {
      effective_cause_code = latest_stop_record_.cause_code.empty() ? "NONE" : latest_stop_record_.cause_code;
      effective_evidence = latest_stop_record_.evidence_status.empty() ? "NONE" : latest_stop_record_.evidence_status;
    }

    const bool data_valid = (odom_stamp_.nanoseconds() > 0 && (now - odom_stamp_).seconds() <= motion_data_timeout_);

    // 统一字段
    add_kv("stage_code", stage_code);
    add_kv("full_goal_uuid", full_uuid);
    add_kv("session_id", session_id_);
    add_kv("session_logging", "ACTIVE");
    add_kv("bt_heartbeat_age_s",permit_received_==std::chrono::steady_clock::time_point{}?"MISSING":
      std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-permit_received_).count()));
    add_kv("motion_permit_token",std::to_string(permit_token_));
    for (auto it = recovery_fields_.begin(); it != recovery_fields_.end(); ++it) {
      add_kv("recovery_" + it.key(), it.value().get<std::string>());
    }
    if (recovery_received_ != std::chrono::steady_clock::time_point{}) {
      add_kv("recovery_status_age_s", std::to_string(std::chrono::duration<double>(
        std::chrono::steady_clock::now() - recovery_received_).count()));
    }
    add_kv("current_bt_node", bt_node_str);
    add_kv("task_status", task_status);
    add_kv("cause_code", effective_cause_code);
    add_kv("evidence_level", effective_evidence);
    add_kv("stop_event_id", std::to_string(latest_stop_record_.stop_event_id));
    add_kv("data_validity", data_valid ? "VALID" : "EXPIRED");

    // 兼容既有字段
    add_kv("target_uuid", short_uuid);
    add_kv("action_type", action_type);
    add_kv("stage", stage_to_string(current_stage_));
    add_kv("motion_feedback", motion_feedback_str_);
    add_kv("is_still", is_still_ ? "true" : "false");
    add_kv("stop_category", latest_stop_record_.stop_category);
    add_kv("stop_reason", latest_stop_record_.trigger_reason);
    add_kv("stop_evidence_status", latest_stop_record_.evidence_status);
    add_kv("related_anomalies", latest_stop_record_.related_anomalies);
    add_kv("recovery_or_terminal", latest_stop_record_.recovery_or_terminal);
    add_kv("cmd_vel_nav_linear_x", std::to_string(latest_cmd_vel_nav_.linear.x));
    add_kv("cmd_vel_nav_angular_z", std::to_string(latest_cmd_vel_nav_.angular.z));
    add_kv("cmd_vel_linear_x", std::to_string(latest_cmd_vel_.linear.x));
    add_kv("cmd_vel_angular_z", std::to_string(latest_cmd_vel_.angular.z));
    add_kv("odom_linear_x", std::to_string(latest_odom_.twist.twist.linear.x));
    add_kv("odom_angular_z", std::to_string(latest_odom_.twist.twist.angular.z));
    add_kv("chassis_watchdog_stopped", chassis_watchdog_stopped_ ? "true" : "false");
    add_kv("localization_ready", loc_ready_ ? "true" : "false");
    add_kv("bag_recording", !record_raw_bag_ ? "DISABLED" :
      (is_recording_bag_ ? "RECORDING" : (evidence_incomplete_ ? "INCOMPLETE" : "STOPPED")));

    diag_array.status.push_back(status);
    nav_status_pub_->publish(diag_array);
  }

  // 内部状态互斥锁与路径
  std::mutex state_mutex_;
  carcar_navigation::GoalStatusPolicy goal_status_policy_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr recovery_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr permit_status_sub_;
  std::chrono::steady_clock::time_point permit_received_{};
  uint64_t permit_token_{0};
  nlohmann::json recovery_fields_ = nlohmann::json::object();
  std::chrono::steady_clock::time_point recovery_received_{};
  std::mutex pose_mutex_;
  std::string sessions_base_dir_;
  std::string session_id_;
  std::string session_dir_;
  std::string global_frame_;
  std::string base_frame_;
  std::string base_params_file_;
  std::string experiment_params_file_;
  std::string default_bt_xml_;
  std::string default_nav_through_poses_bt_xml_;

  std::string jsonl_path_;
  std::string text_log_path_;
  std::string summary_log_path_;
  std::string session_info_path_;
  std::string session_state_path_;

  bool full_event_display_{false};
  double motion_linear_still_threshold_{0.01};
  double motion_angular_still_threshold_{0.02};
  double motion_still_duration_{1.0};
  double motion_data_timeout_{0.6};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  RobotPose cached_robot_pose_;

  // 行为树节点解析字典
  std::map<std::string, std::string> bt_node_name_to_type_;

  // ROS 话题订阅与发布
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr nav_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_to_pose_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr nav_to_pose_feedback_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_through_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>::SharedPtr nav_through_feedback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loc_ready_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr loc_diag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr progress_guard_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr rear_clear_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr backup_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::BackUp_FeedbackMessage>::SharedPtr backup_feedback_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  rclcpp::TimerBase::SharedPtr timer_1hz_;
  rclcpp::TimerBase::SharedPtr timer_2hz_status_;
  rclcpp::TimerBase::SharedPtr timer_check_limits_;
  rclcpp::TimerBase::SharedPtr timer_pose_cache_;

  // 会话与任务跟踪
  size_t goal_counter_{0};
  size_t event_counter_{0};
  size_t replan_counter_{0};
  uint64_t stop_event_counter_{0};
  std::string active_uuid_;
  GoalTask unassociated_goal_;
  std::map<std::string, GoalTask> goals_by_uuid_;

  // 行为树活跃节点跟踪
  std::set<std::string> active_bt_nodes_;
  std::map<std::string, std::string> active_bt_node_types_;

  // 阶段状态机
  NavStage current_stage_{NavStage::IDLE};
  std::chrono::steady_clock::time_point stage_start_time_{std::chrono::steady_clock::now()};
  bool is_replanning_{false};
  bool backup_odom_moving_{false};
  float latest_backup_feedback_dist_{0.0F};
  rclcpp::Time last_backup_feedback_stamp_{0, 0, RCL_ROS_TIME};

  ObstacleStats latest_obstacles_;
  bool loc_ready_{false};
  rclcpp::Time loc_ready_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time loc_diag_stamp_{0, 0, RCL_ROS_TIME};
  std::string latest_loc_diag_summary_{"未知"};

  geometry_msgs::msg::Twist latest_cmd_vel_nav_;
  rclcpp::Time cmd_vel_nav_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};

  // 底盘诊断与看门狗
  bool chassis_watchdog_stopped_{false};
  double chassis_last_cmd_age_s_{0.0};
  double chassis_battery_voltage_{0.0};
  rclcpp::Time chassis_diag_stamp_{0, 0, RCL_ROS_TIME};

  // 运动反馈与静止判据
  rclcpp::Time still_start_stamp_{0, 0, RCL_ROS_TIME};
  bool is_still_{false};
  std::string motion_feedback_str_{"未知"};

  // 停车记录
  // 停车记录与证据滑动窗口
  std::vector<StopRecord> stop_records_;
  StopRecord latest_stop_record_;
  double evidence_window_pre_s_{10.0};
  double evidence_window_post_s_{5.0};
  std::deque<StateSnapshot> ring_buffer_;
  ActiveEvidenceWindow active_window_;
  std::string latest_anomaly_{"无"};
  rclcpp::TimerBase::SharedPtr timer_5hz_;

  // 告警统计
  std::map<std::string, AlertAggregator> alert_aggregators_;
  std::string latest_progress_guard_msg_;
  rclcpp::Time progress_guard_stamp_{0, 0, RCL_ROS_TIME};

  // rosbag2 录包
  // 重规划与调试参数
  bool full_bt_debug_events_{false};
  double replanning_summary_interval_s_{30.0};
  int successful_replan_count_{0};
  rclcpp::Time last_replan_summary_time_{0, 0, RCL_ROS_TIME};

  // rosbag2 录包参数与配额控制
  bool record_raw_bag_{false};
  double raw_bag_max_duration_s_{120.0};
  int64_t raw_bag_max_bytes_{268435456LL};
  rclcpp::Time bag_start_time_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<rosbag2_transport::Recorder> bag_recorder_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> bag_executor_;
  std::thread bag_thread_;
  bool is_recording_bag_{false};
  bool evidence_incomplete_{false};
  std::string incomplete_reason_;
  uint64_t current_bag_bytes_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavEventLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
