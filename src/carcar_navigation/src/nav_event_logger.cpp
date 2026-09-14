// NAV-008/NAV-011: 导航事件监控、终端状态摘要与多维度会话录包节点
// 遵循约定：全 C++ 实现；只读监听各话题与 Action 状态；零驱动输出，绝不发布运动指令。
// 启动时自动建立 log/nav_sessions/<时间>_<PID>/ 会话目录，保存参数快照、配置、行为树 XML、
// 结构化 events.jsonl、中文 events.log，并后台自动分段录制 rosbag2。
// 终端每秒输出状态摘要，并在阶段变化或异常时即时输出；高频重复告警自动合并计数。

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <array>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
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
    case NavStage::SPINNING: return "转向";
    case NavStage::REAR_CHECK: return "后方检查";
    case NavStage::BACKING_UP: return "倒车";
    case NavStage::RECOVERING_LOCALIZATION: return "定位恢复中";
    case NavStage::SUCCEEDED: return "到达";
    case NavStage::CANCELED: return "取消";
    case NavStage::FAILED: return "失败";
    case NavStage::IDLE: return "待命";
    default: return "未知";
  }
}

struct GoalTask
{
  std::string goal_id;
  std::string uuid;
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

struct AlertAggregator
{
  std::string last_msg;
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

    // 初始化会话目录
    init_session();

    // 订阅目标下发
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_goal_pose, this, std::placeholders::_1));

    // 订阅行为树事件流（精细捕获叶节点）
    bt_log_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      "/behavior_tree_log", rclcpp::QoS(100).reliable(),
      std::bind(&NavEventLogger::on_bt_log, this, std::placeholders::_1));

    // 订阅 Action 状态
    action_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_action_status, this, std::placeholders::_1));

    // 订阅 Action 反馈（剩余距离）
    nav_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
      "/navigate_to_pose/_action/feedback", rclcpp::QoS(10).reliable(),
      std::bind(&NavEventLogger::on_nav_feedback, this, std::placeholders::_1));

    // 订阅激光扫描
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&NavEventLogger::on_scan, this, std::placeholders::_1));

    // 订阅定位就绪与门控状态
    loc_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/localization_monitor/ready", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&NavEventLogger::on_loc_ready, this, std::placeholders::_1));

    // 订阅两级速度
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_cmd_vel_ = *msg;
        cmd_vel_stamp_ = this->now();
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odometry", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_odom_ = *msg;
        odom_stamp_ = this->now();

        // 倒车运动状态判定与位移统计
        if (current_stage_ == NavStage::BACKING_UP) {
          if (msg->twist.twist.linear.x < -0.01) {
            if (!backup_odom_moving_) {
              backup_odom_moving_ = true;
              record_event_locked("BACKUP_ODOM_FEEDBACK_MOVING", active_uuid_, "ntpe_BackUp",
                "里程计反馈产生后退位移 (vx=" + std::to_string(msg->twist.twist.linear.x) + " m/s)");
            }
          }
        }
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

    // 1 Hz 终端摘要刷新定时器
    timer_1hz_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NavEventLogger::publish_terminal_summary, this));

    // 2 秒检查一次磁盘空间与录包容量
    timer_check_limits_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&NavEventLogger::check_storage_limits, this));

    RCLCPP_INFO(this->get_logger(),
      "NAV-011 导航事件监控与会话录包系统已就绪: 会话目录=%s", session_dir_.c_str());
  }

  ~NavEventLogger() override
  {
    close_session();
  }

private:
  void init_session()
  {
    const auto pid = getpid();
    session_id_ = get_session_timestamp_id() + "_" + std::to_string(pid);
    session_dir_ = sessions_base_dir_ + "/" + session_id_;
    std::filesystem::create_directories(session_dir_);

    jsonl_path_ = session_dir_ + "/events.jsonl";
    text_log_path_ = session_dir_ + "/events.log";
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
    s["is_recording_bag"] = is_recording_bag_;
    s["evidence_incomplete"] = evidence_incomplete_;
    s["evidence_incomplete_reason"] = incomplete_reason_;
    s["total_bag_bytes"] = current_bag_bytes_;
    s["total_events"] = event_counter_;
    s["total_goals"] = goals_by_uuid_.size();

    std::ofstream f(session_state_path_);
    if (f.is_open()) {
      f << s.dump(2) << "\n";
    }
  }

  void start_rosbag_recorder()
  {
    try {
      auto writer = std::make_shared<rosbag2_cpp::Writer>();
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = session_dir_ + "/bag";
      storage_options.storage_id = "sqlite3";
      storage_options.max_bagfile_size = 1073741824ULL;  // 1 GiB 分卷

      rosbag2_transport::RecordOptions record_options;
      record_options.rmw_serialization_format = "cdr";
      record_options.include_hidden_topics = true;
      record_options.include_unpublished_topics = true;
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
        "/behavior_tree_log",
        "/rear_clear/status",
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
      record_event_locked("ROSBAG_RECORDING_STARTED", "NONE", "rosbag2",
        "rosbag2 自动录包已启动 (分卷限额: 1 GiB, 存储: sqlite3)");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "启动 rosbag2 自动录包失败: %s", e.what());
      is_recording_bag_ = false;
      evidence_incomplete_ = true;
      incomplete_reason_ = std::string("rosbag2启动异常: ") + e.what();
    }
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

    std::error_code ec;
    auto space = std::filesystem::space(session_dir_, ec);
    if (!ec && is_recording_bag_) {
      if (space.available < 2147483648ULL) {  // 磁盘低于 2 GiB
        is_recording_bag_ = false;
        evidence_incomplete_ = true;
        incomplete_reason_ = "磁盘剩余空间低于 2 GiB (" + std::to_string(space.available / (1024 * 1024)) + " MB)";
        if (bag_recorder_) {
          bag_recorder_->stop();
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        record_event_locked("ROSBAG_STOPPED_DISK_LOW", active_uuid_, "Recorder", incomplete_reason_);
        std::cout << "\n[告警] 磁盘剩余空间低于 2 GiB，停止原始录包，继续记录状态摘要与结构化事件 (证据标记为不完整)\n" << std::endl;
      } else if (bag_size >= 10737418240ULL) {  // 单次达到 10 GiB
        is_recording_bag_ = false;
        evidence_incomplete_ = true;
        incomplete_reason_ = "单次会话录包达到 10 GiB 保护上限";
        if (bag_recorder_) {
          bag_recorder_->stop();
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        record_event_locked("ROSBAG_STOPPED_SIZE_LIMIT", active_uuid_, "Recorder", incomplete_reason_);
        std::cout << "\n[告警] 单次会话录包达到 10 GiB 保护上限，停止原始录包，继续记录状态摘要与结构化事件 (证据标记为不完整)\n" << std::endl;
      }
    }
  }

  void close_session()
  {
    if (is_recording_bag_ && bag_recorder_) {
      bag_recorder_->stop();
      is_recording_bag_ = false;
    }
    if (bag_executor_) {
      bag_executor_->cancel();
    }
    if (bag_thread_.joinable()) {
      bag_thread_.join();
    }
    write_session_state("ENDED");
  }

  RobotPose get_current_pose()
  {
    RobotPose pose;
    pose.frame_id = global_frame_;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.yaw = tf2::getYaw(transform.transform.rotation);
      pose.valid = true;
      pose.stamp = transform.header.stamp;
    } catch (const tf2::TransformException &) {
      pose.valid = false;
    }
    return pose;
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

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto robot_pose = get_current_pose();
    const std::string goal_id = "goal_" + std::to_string(++goal_counter_);

    unassociated_goal_ = GoalTask();
    unassociated_goal_.goal_id = goal_id;
    unassociated_goal_.uuid = "PENDING_ACTION_UUID";
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

  void on_nav_feedback(const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::string uuid = uuid_to_hex(msg->goal_id.uuid);
    if (goals_by_uuid_.find(uuid) != goals_by_uuid_.end()) {
      goals_by_uuid_[uuid].remaining_distance = msg->feedback.distance_remaining;
      goals_by_uuid_[uuid].last_distance_update = this->now();
    }
  }

  void on_action_status(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & s : msg->status_list) {
      const std::string uuid = uuid_to_hex(s.goal_info.goal_id.uuid);

      // 若为新 UUID，注册独立目标记录（旧目标终态不得覆盖新目标）
      if (goals_by_uuid_.find(uuid) == goals_by_uuid_.end()) {
        GoalTask task;
        task.uuid = uuid;
        task.goal_id = "action_" + uuid_short(uuid);
        task.start_time = get_iso_timestamp();
        task.is_active = true;

        if (unassociated_goal_.is_active) {
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
        active_uuid_ = uuid;
        transition_stage_locked(NavStage::PLANNING, "Action 目标激活 (UUID=" + uuid_short(uuid) + ")");
        record_event_locked("ACTION_GOAL_ACCEPTED", uuid, "bt_navigator", "新目标已被接受并激活");
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
          transition_stage_locked(NavStage::SUCCEEDED, "导航成功抵达目标点");
        }
        record_event_locked("NAVIGATION_SUCCEEDED", uuid, "bt_navigator", "导航目标成功抵达 (SUCCEEDED)");
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
        task.is_active = false;
        task.is_terminal = true;
        task.end_time = get_iso_timestamp();
        if (active_uuid_ == uuid) {
          transition_stage_locked(NavStage::FAILED, "导航任务中止失败 (ABORTED)");
        }
        record_event_locked("NAVIGATION_ABORTED", uuid, "bt_navigator", "导航任务异常中止 (ABORTED)");
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        task.is_active = false;
        task.is_terminal = true;
        task.end_time = get_iso_timestamp();
        if (active_uuid_ == uuid) {
          transition_stage_locked(NavStage::CANCELED, "导航任务被取消 (CANCELED)");
        }
        record_event_locked("NAVIGATION_CANCELED", uuid, "bt_navigator", "导航任务已被取消 (CANCELED)");
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        task.is_active = true;
        active_uuid_ = uuid;
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

      // 1. 叶节点：WaitForLocalizationStatus
      if (node == "ntpe_WaitForLocalizationStatus" || node == "ntppe_WaitForLocalizationStatus" || node == "WaitForLocalizationStatus") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::WAITING_FOR_LOCALIZATION, "正在等待定位健康就绪");
        }
      }
      // 2. 叶节点：ComputePathToPose / ComputePathThroughPoses
      else if (node == "ntpe_ComputePathToPose" || node == "ntppe_ComputePathThroughPoses" || node == "ComputePathToPose") {
        if (curr == "RUNNING") {
          is_replanning_ = true;
          if (current_stage_ == NavStage::PATH_FOLLOWING || current_stage_ == NavStage::PATH_FOLLOWING_REPLANNING) {
            transition_stage_locked(NavStage::PATH_FOLLOWING_REPLANNING, "周期重规划触发 (路径跟随中)");
          } else {
            transition_stage_locked(NavStage::PLANNING, "全局路径规划中");
          }
        } else {
          is_replanning_ = false;
          if (curr == "SUCCESS") {
            if (current_stage_ == NavStage::PATH_FOLLOWING_REPLANNING) {
              transition_stage_locked(NavStage::PATH_FOLLOWING, "全局重规划成功完成，继续跟随");
            }
          } else if (curr == "FAILURE") {
            record_event_locked("PLAN_FAILED", active_uuid_, node, "全局规划失败 (无通行路径)");
          }
        }
      }
      // 3. 叶节点：FollowPath
      else if (node == "ntpe_FollowPath" || node == "ntppe_FollowPath" || node == "FollowPath") {
        if (curr == "RUNNING") {
          if (is_replanning_) {
            transition_stage_locked(NavStage::PATH_FOLLOWING_REPLANNING, "开始执行路径跟随 (并行重规划中)");
          } else {
            transition_stage_locked(NavStage::PATH_FOLLOWING, "开始执行路径跟随");
          }
        } else if (curr == "FAILURE") {
          record_event_locked("CONTROL_FAILED", active_uuid_, node, "局部控制器失败 (受阻或偏离)");
        }
      }
      // 4. 叶节点：Wait (ntpe_WaitForReplan, ntpe_BackUpDeniedWait)
      else if (node == "ntpe_WaitForReplan" || node == "ntpe_BackUpDeniedWait" || node == "ntppe_WaitForReplan" || node == "Wait") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::RECOVERY_WAITING, "触发恢复等待 (" + node + ")");
        }
      }
      // 5. 叶节点：Spin (ntpe_SpinForObservability)
      else if (node == "ntpe_SpinForObservability" || node == "ntppe_SpinForObservability" || node == "Spin") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::SPINNING, "触发转向恢复 (" + node + ")");
        }
      }
      // 6. 叶节点：RearClear (后方净空检查)
      else if (node == "ntpe_RearClear" || node == "ntppe_RearClear" || node == "RearClear") {
        if (curr == "RUNNING" || (curr == "IDLE" && prev == "RUNNING")) {
          transition_stage_locked(NavStage::REAR_CHECK, "执行倒车后方净空检查");
        }
      }
      // 7. 叶节点：BackUp (真实倒车动作，绝不误认父节点 ntpe_BackUpOnlyWhenRearClear 或 ntpe_RearClearAndBackUp)
      else if (node == "ntpe_BackUp" || node == "ntppe_BackUp" || node == "BackUp") {
        if (curr == "RUNNING") {
          backup_odom_moving_ = false;
          transition_stage_locked(NavStage::BACKING_UP, "执行受限倒车恢复 (限额 0.15m, 0.05m/s, 最长 6s)");
          record_event_locked("BACKUP_ACTION_STARTED", active_uuid_, node, "倒车 Action 开始执行");
        } else if (curr == "SUCCESS") {
          record_event_locked("BACKUP_ACTION_SUCCESS", active_uuid_, node, "倒车 Action 完成");
        } else if (curr == "FAILURE") {
          record_event_locked("BACKUP_ACTION_FAILURE", active_uuid_, node, "倒车 Action 失败中止");
        }
      }
      // 8. 叶节点：RecoverLocalization
      else if (node == "ntpe_RecoverLocalization" || node == "ntppe_RecoverLocalization" || node == "RecoverLocalization") {
        if (curr == "RUNNING") {
          transition_stage_locked(NavStage::RECOVERING_LOCALIZATION, "定位恢复中 (静止更新/全局重定位)");
        }
      }
    }
  }

  void on_rear_clear_status(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool passed = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
    const std::string reason = msg->message;
    std::string detail;
    for (const auto & kv : msg->values) {
      if (kv.key == "detail") detail = kv.value;
    }

    if (passed) {
      record_event_locked("REAR_CLEAR_PASSED", active_uuid_, "RearClear",
        "后方检查通过: 净空正常，允许倒车");
    } else {
      record_event_locked("REAR_CLEAR_DENIED", active_uuid_, "RearClear",
        "后方检查拒绝: 原因=" + reason + " (" + detail + ")");
      // 立即输出终端告警
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

    // 过滤关于 DWB、控制器及超时的关键日志
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

    if (!key_fault.empty()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      record_event_locked("CONTROLLER_FAULT", active_uuid_, msg->name, key_fault + ": " + text);

      // 高频重复告警合并
      const auto now = this->now();
      auto & agg = alert_aggregators_[key_fault];
      if (agg.count == 0 || (now - agg.last_time).seconds() > 2.0) {
        agg.count = 1;
        agg.last_time = now;
        std::cout << "\n[" << get_time_only_str() << "] [告警] " << key_fault << std::endl;
      } else {
        agg.count++;
        agg.last_time = now;
        if (agg.count % 5 == 0) {
          std::cout << "[" << get_time_only_str() << "] [告警合并] " << key_fault
                    << " (连续触发 " << agg.count << " 次)" << std::endl;
        }
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

    // 阶段变化立即输出
    std::cout << "\n[" << get_time_only_str() << "] >>> 阶段切换: [" << prev_str
              << "] -> [" << next_str << "] (" << reason << ")" << std::endl;
  }

  void record_event_locked(
    const std::string & event_type, const std::string & uuid,
    const std::string & node_name, const std::string & description)
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

    if (!active_uuid_.empty() && goals_by_uuid_.find(active_uuid_) != goals_by_uuid_.end()) {
      const auto & task = goals_by_uuid_[active_uuid_];
      uuid_display = uuid_short(active_uuid_);
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

    // 指令速度检查
    std::string cmd_str = "缺失";
    const double cmd_age = (now - cmd_vel_stamp_).seconds();
    if (cmd_vel_stamp_.nanoseconds() > 0) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2) << "vx=" << latest_cmd_vel_.linear.x
         << " wz=" << latest_cmd_vel_.angular.z;
      if (cmd_age > 0.6) {
        ss << "(过期)";
      }
      cmd_str = ss.str();
    }

    // 里程计速度检查
    std::string odom_str = "缺失";
    const double odom_age = (now - odom_stamp_).seconds();
    if (odom_stamp_.nanoseconds() > 0) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2) << "vx=" << latest_odom_.twist.twist.linear.x
         << " wz=" << latest_odom_.twist.twist.angular.z;
      if (odom_age > 0.6) {
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
      bag_str = evidence_incomplete_ ? "已停止(证据不完整)" : "已停止";
    }

    // 倒车特殊显示：倒车 Action 反馈位移（里程计反馈）
    std::string extra_backup;
    if (current_stage_ == NavStage::BACKING_UP) {
      std::ostringstream ss;
      ss << " [倒车位移: " << std::fixed << std::setprecision(2)
         << latest_backup_feedback_dist_ << "m/0.15m (里程计反馈)]";
      extra_backup = ss.str();
    }

    // 终端 1 Hz 单行摘要输出
    std::cout << "[" << get_time_only_str() << "] "
              << "[目标 " << uuid_display << "] "
              << "[阶段: " << stage_to_string(current_stage_)
              << " (" << std::fixed << std::setprecision(1) << stage_duration_sec << "s)] "
              << "[剩余: " << dist_str << "] "
              << "[指令: " << cmd_str << "] "
              << "[里程计: " << odom_str << "] "
              << "[定位: " << loc_str << "] "
              << "[录包: " << bag_str << "]"
              << extra_backup << std::endl;
  }

  // 内部状态
  std::mutex state_mutex_;
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
  std::string session_info_path_;
  std::string session_state_path_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr nav_feedback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loc_ready_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr rear_clear_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr backup_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::BackUp_FeedbackMessage>::SharedPtr backup_feedback_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  rclcpp::TimerBase::SharedPtr timer_1hz_;
  rclcpp::TimerBase::SharedPtr timer_check_limits_;

  // 会话与任务跟踪
  size_t goal_counter_{0};
  size_t event_counter_{0};
  std::string active_uuid_;
  GoalTask unassociated_goal_;
  std::map<std::string, GoalTask> goals_by_uuid_;

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
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};

  // 告警统计
  std::map<std::string, AlertAggregator> alert_aggregators_;

  // rosbag2 录包
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
