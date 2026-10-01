#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "carcar_camera/depth_logic.hpp"

namespace {
using Steady = std::chrono::steady_clock;

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}
}  // namespace

class DepthLayerSupervisor : public rclcpp::Node
{
public:
  DepthLayerSupervisor()
  : Node("depth_layer_supervisor")
  {
    health_timeout_s_ = declare_parameter("health_timeout_s", 0.50);
    recovery_stable_s_ = declare_parameter("recovery_stable_s", 3.0);
    rebuild_wait_s_ = declare_parameter("rebuild_wait_s", 1.0);
    switch_timeout_s_ = declare_parameter("switch_timeout_s", 5.0);
    layer_parameter_ = declare_parameter(
      "layer_parameter", std::string("depth_voxel_layer.enabled"));
    status_frame_ = declare_parameter("status_frame", std::string("base_link"));
    if (health_timeout_s_ <= 0.0 || recovery_stable_s_ <= 0.0 ||
      rebuild_wait_s_ < 0.0 || switch_timeout_s_ <= 0.0) {
      throw std::invalid_argument("深度层监督超时参数无效");
    }

    allowed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/navigation/perception_motion_allowed",
      rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/navigation/fusion_status", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/camera/navigation/status_marker", rclcpp::QoS(1).reliable());
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/camera/navigation/healthy", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        raw_health_ = msg->data;
        last_health_received_ = Steady::now();
      });
    const auto costmap_qos = rclcpp::QoS(1).reliable().transient_local();
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {
        last_local_costmap_ = Steady::now();
      });
    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {
        last_global_costmap_ = Steady::now();
      });

    local_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/local_costmap/local_costmap");
    global_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/global_costmap/global_costmap");
    local_clear_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap");
    global_clear_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&DepthLayerSupervisor::tick, this));
    publish_allowed(true);
  }

private:
  enum class Phase {IDLE, SET_PARAMETERS, CLEAR_COSTMAPS, REBUILD, FAILED};

  double now_steady() const
  {
    return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  }

  bool health_fresh() const
  {
    if (!raw_health_ || last_health_received_ == Steady::time_point{}) {
      return false;
    }
    return std::chrono::duration<double>(Steady::now() - last_health_received_).count() <=
      health_timeout_s_;
  }

  bool interfaces_ready() const
  {
    return local_parameters_->service_is_ready() && global_parameters_->service_is_ready() &&
      local_clear_->service_is_ready() && global_clear_->service_is_ready();
  }

  void publish_allowed(bool allowed)
  {
    motion_allowed_ = allowed;
    std_msgs::msg::Bool msg;
    msg.data = allowed;
    allowed_pub_->publish(msg);
  }

  void begin_switch(bool enable)
  {
    if (phase_ != Phase::IDLE || !interfaces_ready()) {
      return;
    }
    target_enabled_ = enable;
    phase_ = Phase::SET_PARAMETERS;
    switch_started_ = Steady::now();
    pending_ = 2;
    operation_ok_ = true;
    failure_reason_.clear();
    publish_allowed(false);

    const std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter(layer_parameter_, enable)};
    auto callback = [this](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
      {
        try {
          const auto results = future.get();
          if (results.empty()) {
            operation_ok_ = false;
            failure_reason_ = "EMPTY_PARAMETER_RESULT";
          }
          for (const auto & result : results) {
            if (!result.successful) {
              operation_ok_ = false;
              failure_reason_ = "SET_PARAMETER_FAILED: " + result.reason;
            }
          }
        } catch (const std::exception & error) {
          operation_ok_ = false;
          failure_reason_ = std::string("SET_PARAMETER_EXCEPTION: ") + error.what();
        }
        --pending_;
      };
    local_parameters_->set_parameters(params, callback);
    global_parameters_->set_parameters(params, callback);
  }

  void begin_clear()
  {
    phase_ = Phase::CLEAR_COSTMAPS;
    pending_ = 2;
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto callback = [this](
      rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
      {
        try {
          (void)future.get();
        } catch (const std::exception & error) {
          operation_ok_ = false;
          failure_reason_ = std::string("CLEAR_COSTMAP_EXCEPTION: ") + error.what();
        }
        --pending_;
      };
    local_clear_->async_send_request(request, callback);
    global_clear_->async_send_request(request, callback);
  }

  void fail_switch(const std::string & reason)
  {
    phase_ = Phase::FAILED;
    failure_reason_ = reason;
    publish_allowed(false);
    RCLCPP_ERROR(get_logger(), "深度层切换失败，保持停车: %s", reason.c_str());
  }

  void tick_switch()
  {
    if (phase_ == Phase::IDLE || phase_ == Phase::FAILED) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(Steady::now() - switch_started_).count();
    if (elapsed > switch_timeout_s_) {
      fail_switch("SWITCH_TIMEOUT");
      return;
    }
    if (phase_ == Phase::SET_PARAMETERS && pending_ == 0) {
      if (!operation_ok_) {
        fail_switch(failure_reason_);
      } else {
        begin_clear();
      }
      return;
    }
    if (phase_ == Phase::CLEAR_COSTMAPS && pending_ == 0) {
      if (!operation_ok_) {
        fail_switch(failure_reason_);
      } else {
        phase_ = Phase::REBUILD;
        rebuild_started_ = Steady::now();
      }
      return;
    }
    if (phase_ == Phase::REBUILD &&
      std::chrono::duration<double>(Steady::now() - rebuild_started_).count() >= rebuild_wait_s_ &&
      last_local_costmap_ > rebuild_started_ && last_global_costmap_ > rebuild_started_)
    {
      active_enabled_ = target_enabled_;
      phase_ = Phase::IDLE;
      initial_sync_needed_ = false;
      state_ = active_enabled_ ? carcar_camera::FusionState::FUSED :
        carcar_camera::FusionState::LASER_ONLY;
      publish_allowed(true);
      RCLCPP_INFO(
        get_logger(), "深度层切换完成: %s",
        active_enabled_ ? "激光+深度" : "仅激光");
    }
  }

  void tick()
  {
    const bool healthy = health_fresh();
    const auto now = Steady::now();
    if (healthy) {
      if (healthy_since_ == Steady::time_point{}) {
        healthy_since_ = now;
      }
    } else {
      healthy_since_ = Steady::time_point{};
    }

    if (phase_ == Phase::FAILED) {
      publish_allowed(false);
      publish_status(healthy);
      return;
    }
    tick_switch();
    if (phase_ != Phase::IDLE) {
      publish_allowed(false);
      publish_status(healthy);
      return;
    }

    if (initial_sync_needed_) {
      state_ = healthy ? carcar_camera::FusionState::RECOVERING :
        carcar_camera::FusionState::LASER_ONLY;
      begin_switch(false);
    } else if (active_enabled_ && !healthy) {
      state_ = carcar_camera::FusionState::LASER_ONLY;
      if (interfaces_ready()) {
        begin_switch(false);
      } else {
        fail_switch("COSTMAP_INTERFACES_UNAVAILABLE_DURING_DEGRADE");
      }
    } else if (!active_enabled_ && healthy) {
      state_ = carcar_camera::FusionState::RECOVERING;
      const double stable = std::chrono::duration<double>(now - healthy_since_).count();
      if (stable >= recovery_stable_s_) {
        begin_switch(true);
      }
    } else {
      state_ = active_enabled_ ? carcar_camera::FusionState::FUSED :
        carcar_camera::FusionState::LASER_ONLY;
    }
    if (phase_ == Phase::IDLE) {
      publish_allowed(true);
    }
    publish_status(healthy);
  }

  void publish_status(bool healthy)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435FusionSupervisor";
    status.hardware_id = "nav2_costmaps";
    status.level = phase_ == Phase::FAILED ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (state_ == carcar_camera::FusionState::FUSED ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = phase_ == Phase::FAILED ? "SWITCH_FAILED" :
      carcar_camera::state_name(state_);
    add_value(status, "state", carcar_camera::state_name(state_));
    add_value(status, "camera_healthy", healthy ? "true" : "false");
    add_value(status, "depth_layer_enabled", active_enabled_ ? "true" : "false");
    add_value(status, "motion_allowed", motion_allowed_ ? "true" : "false");
    add_value(status, "switching", phase_ != Phase::IDLE ? "true" : "false");
    add_value(
      status, "local_costmap_rebuilt",
      last_local_costmap_ > rebuild_started_ ? "true" : "false");
    add_value(
      status, "global_costmap_rebuilt",
      last_global_costmap_ > rebuild_started_ ? "true" : "false");
    add_value(status, "failure_reason", failure_reason_);
    array.status.push_back(status);
    status_pub_->publish(array);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = array.header.stamp;
    marker.header.frame_id = status_frame_;
    marker.ns = "d435_fusion_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = 0.34;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.055;
    marker.color.a = 1.0;
    marker.color.r = phase_ == Phase::FAILED || state_ == carcar_camera::FusionState::LASER_ONLY ?
      1.0F : 0.1F;
    marker.color.g = state_ == carcar_camera::FusionState::FUSED ? 1.0F : 0.65F;
    marker.color.b = state_ == carcar_camera::FusionState::RECOVERING ? 1.0F : 0.1F;
    marker.text = phase_ == Phase::FAILED ? "D435: SWITCH_FAILED / STOPPED" :
      std::string("D435: ") + carcar_camera::state_name(state_);
    marker_pub_->publish(marker);
  }

  double health_timeout_s_{0.5};
  double recovery_stable_s_{3.0};
  double rebuild_wait_s_{1.0};
  double switch_timeout_s_{5.0};
  std::string layer_parameter_;
  std::string status_frame_;
  bool raw_health_{false};
  bool active_enabled_{false};
  bool target_enabled_{false};
  bool motion_allowed_{true};
  bool initial_sync_needed_{true};
  bool operation_ok_{true};
  int pending_{0};
  Phase phase_{Phase::IDLE};
  carcar_camera::FusionState state_{carcar_camera::FusionState::LASER_ONLY};
  Steady::time_point last_health_received_{};
  Steady::time_point healthy_since_{};
  Steady::time_point switch_started_{};
  Steady::time_point rebuild_started_{};
  Steady::time_point last_local_costmap_{};
  Steady::time_point last_global_costmap_{};
  std::string failure_reason_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allowed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::shared_ptr<rclcpp::AsyncParametersClient> local_parameters_;
  std::shared_ptr<rclcpp::AsyncParametersClient> global_parameters_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthLayerSupervisor>());
  rclcpp::shutdown();
  return 0;
}
