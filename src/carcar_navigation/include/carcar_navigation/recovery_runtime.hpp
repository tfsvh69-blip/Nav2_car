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
  unsigned escape_stage{0};
  TimePoint observe_started{};
  TimePoint recovery_started{};
  double response_timeout{1}, cancel_timeout{1}, settle_timeout{2}, still_duration{1};
  double data_age{0.5}, recovery_timeout{90};
private:
  struct DiagSnapshot {
    std::string phase, reason;
    bool fault{false};
    uint64_t epoch{0};
    std::string navigation_uuid, previous_navigation_uuid;
    double backup_used{0}, recovery_elapsed{0}, recovery_deadline{0};
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
