// NAV-012：回调所有权、机器人受阻上下文及坐标一致的安全检查。
#pragma once
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <behaviortree_cpp_v3/tree_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/node_thread.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace carcar_navigation {
using Steady = std::chrono::steady_clock;
using TimePoint = Steady::time_point;
inline double seconds(TimePoint start) {
  return std::chrono::duration<double>(Steady::now() - start).count();
}
inline TimePoint after(double s) {
  return Steady::now() + std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(s));
}
inline double normalize(double a) {return std::atan2(std::sin(a), std::cos(a));}

// ROS 时钟与回调采样可能差几毫秒，略负的年龄是刚收到，不是过期。
inline constexpr double kRosTimeJitter = 0.05;
inline bool ros_age_fresh(double age, double max_age, double jitter = kRosTimeJitter)
{
  if (!std::isfinite(age) || !std::isfinite(max_age) || max_age <= 0 ||
      !std::isfinite(jitter) || jitter < 0) {
    return false;
  }
  if (age < -jitter) {return false;}
  return (age < 0.0 ? 0.0 : age) <= max_age;
}

inline bool heading_target_flipped(double previous_yaw, double new_yaw, double threshold = 0.35)
{
  return std::abs(normalize(new_yaw - previous_yaw)) >= threshold;
}

enum class HeadingAlignEvent { Hold, NewTarget, Oscillating };

// 同一轮绕障里路径参考左右翻转时，不要把两段对准合成一次 8 s 转向失败。
inline HeadingAlignEvent classify_heading_align(
  bool in_turn, double previous_target, double new_target,
  unsigned flips_so_far, double flip_threshold, unsigned max_flips)
{
  if (!std::isfinite(previous_target) || !std::isfinite(new_target) ||
      !std::isfinite(flip_threshold) || flip_threshold <= 0 || max_flips < 1) {
    return HeadingAlignEvent::Oscillating;
  }
  if (!in_turn) {return HeadingAlignEvent::NewTarget;}
  if (!heading_target_flipped(previous_target, new_target, flip_threshold)) {
    return HeadingAlignEvent::Hold;
  }
  if (flips_so_far + 1 >= max_flips) {return HeadingAlignEvent::Oscillating;}
  return HeadingAlignEvent::NewTarget;
}

struct ForwardProgressSample {
  bool displaced{false};
  bool plausible{false};
  double forward{0.0};
  double lateral{0.0};
};

// 只用轮式里程计相对上一采样基准的车体前向投影认定有效进展。
// 单步跳变超过 0.15 m 视为定位/里程计异常，不为转向预算续期。
inline ForwardProgressSample classify_forward_progress(
  double dx, double dy, double baseline_yaw,
  double displacement_threshold = 0.03, double max_step = 0.15)
{
  ForwardProgressSample result;
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(baseline_yaw) ||
      !std::isfinite(displacement_threshold) || displacement_threshold <= 0.0 ||
      !std::isfinite(max_step) || max_step <= displacement_threshold) {
    return result;
  }
  const double distance = std::hypot(dx, dy);
  result.displaced = distance >= displacement_threshold;
  result.plausible = result.displaced && distance <= max_step;
  result.forward = dx * std::cos(baseline_yaw) + dy * std::sin(baseline_yaw);
  result.lateral = -dx * std::sin(baseline_yaw) + dy * std::cos(baseline_yaw);
  return result;
}

// 倒车恢复几何限额，与 nav2_experimental.yaml / 实验行为树保持一致。
// 实验上限：单次 0.20 m、累计 0.40 m。默认倒车后冷却 12 s，冷却期内改走跟随或转向；
// 净前向 0.20 m 后重置额度。能否脱困须实测。
inline constexpr double kMaxSingleBackupDistance = 0.20;
inline constexpr double kMaxBackupBudget = 0.40;
inline constexpr double kMaxBackupSpeed = 0.05;
inline constexpr double kMaxBackupTimeAllowance = 6.0;
struct SafetyResult {
  bool ok{false};
  std::string code{"DATA_MISSING"};
  std::string detail;
};
struct SensorSnapshot {
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
  TimePoint costmap_received{}, scan_received{}, footprint_received{};
  uint64_t costmap_count{0}, scan_count{0}, footprint_count{0};
};
class SensorCache {
public:
  SensorCache(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group,
    const std::string & scan, const std::string & costmap, const std::string & footprint);
  SensorSnapshot snapshot() const;
private:
  mutable std::mutex mutex_;
  SensorSnapshot data_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};
struct HealthSample {
  bool received{false};
  bool value{false};
  TimePoint received_at{};
  bool fresh(double max_age=1.0) const {
    return received && received_at!=TimePoint{} && seconds(received_at)<=max_age;
  }
  bool healthy(double max_age=1.0) const {return value && fresh(max_age);}
};
struct ActionState {
  bool accepted{false}, terminal{false}, success{false}, rejected{false};
  bool cancel_requested{false}, cancel_ack{false}, timed_out{false};
  std::string uuid, reason;
  double elapsed{0};
  uint64_t revision{0};
  double response_remaining{0},result_remaining{0},cancel_remaining{0};
};
class ManagedSession {
public:
  virtual ~ManagedSession() = default;
  virtual void poll() = 0;
  virtual void cancel() = 0;
  virtual ActionState state() const = 0;
};
struct HeadingReference {
  bool valid{false};
  double yaw{0}, error{0};
  size_t segment{0};
};
HeadingReference heading_reference(const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal,
  double lookahead = 0.30);
bool fresh_stamp(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now,
  TimePoint received, double age);
bool scan_points_in_base(const sensor_msgs::msg::LaserScan & scan, tf2_ros::Buffer & tf,
  const std::string & base, std::vector<geometry_msgs::msg::Point> & points);
SafetyResult swept_clear(const SensorSnapshot & data, tf2_ros::Buffer & tf,
  const rclcpp::Time & now, const std::string & base, double dx, double dy,
  double max_age = 0.5);

// 一个机器人实例的运行期上下文；blackboard 持有强引用，注册表只保留弱引用。
class RecoveryRuntime {
public:
  static std::shared_ptr<RecoveryRuntime> get(const BT::NodeConfiguration & config);
  explicit RecoveryRuntime(rclcpp::Node::SharedPtr node);
  ~RecoveryRuntime();
  std::shared_ptr<SensorCache> sensors(const std::string & scan = "/scan",
    const std::string & costmap = "/local_costmap/costmap_raw",
    const std::string & footprint = "/local_costmap/published_footprint");
  bool healthy() const;
  HealthSample localization_ready() const;
  HealthSample recovery_allowed() const;
  void note_tick();
  bool odom_fresh() const;
  bool still(double duration) const;
  void restart_stillness();
  nav_msgs::msg::Odometry::ConstSharedPtr odom() const;
  std::string goal_uuid(bool through) const;
  bool pose(geometry_msgs::msg::PoseStamped & result, const std::string & frame = "");
  SafetyResult inputs_ready();
  void permit(bool allowed);
  void diagnostic(const std::string & phase, const std::string & reason);
  void fail(const std::string & reason);
  void begin_recovery();
  void forward_progress(double distance);
  bool reserve_backup(double distance);
  void release_backup(double distance);
  void note_backup(bool succeeded = true);
  bool backup_cooling() const;
  bool request_observe_replan();

  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr group;
  tf2_ros::Buffer tf;
  std::string base_frame{"base_footprint"}, global_frame{"map"};
  bool supervised{false}, fault{false};
  std::string fault_reason;
  std::string navigation_uuid,previous_navigation_uuid;
  uint64_t epoch{1}, action_sequence{1};
  std::shared_ptr<ManagedSession> motion;
  // 保留待取消请求直到结果到达或运行上下文析构，晚到响应仍能按 UUID 取消。
  std::vector<std::shared_ptr<ManagedSession>> retiring;
  double backup_used{0}, forward_distance{0};
  bool spin_used{false}, recovery_active{false};
  unsigned escape_stage{0}, observe_replans_used{0};
  TimePoint observe_started{};
  TimePoint recovery_started{};
  TimePoint last_backup_at_{};
  bool last_backup_ok_{true};
  double response_timeout{1}, cancel_timeout{1}, settle_timeout{1}, still_duration{0.4};
  double data_age{0.5}, recovery_timeout{90}, backup_cooldown{12};
  unsigned max_observe_replans{1};
private:
  struct DiagSnapshot {
    std::string phase, reason;
    bool fault{false};
    uint64_t epoch{0};
    std::string navigation_uuid, previous_navigation_uuid;
    double backup_used{0}, recovery_elapsed{0}, recovery_deadline{0};
    unsigned observe_replans_used{0}, max_observe_replans{0};
    TimePoint last_bt_tick{};
    std::shared_ptr<ManagedSession> motion;
  };
  void observe_uuid(const std::string & id, bool through);
  void publish_recovery_status();
  void emit_recovery_status(DiagSnapshot snap);
  mutable std::mutex mutex_;
  std::map<std::string,std::shared_ptr<SensorCache>> caches_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  TimePoint odom_received_{}, still_since_{};
  HealthSample ready_{}, recovery_allowed_{};
  std::string pose_uuid_, through_uuid_;
  std::vector<std::string> retired_ids_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_allowed_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr pose_feedback_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>::SharedPtr through_feedback_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr permit_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  TimePoint last_diag_{}, last_bt_tick_{};
  std::string last_diag_key_;
  DiagSnapshot diag_snapshot_;
  std::unique_ptr<nav2_util::NodeThread> thread_;
};
}  // namespace carcar_navigation
