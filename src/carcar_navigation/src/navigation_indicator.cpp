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

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "carcar_interfaces/msg/indicator_command.hpp"
#include "carcar_navigation/navigation_indicator_logic.hpp"

namespace carcar_navigation
{

class NavigationIndicator : public rclcpp::Node
{
public:
  NavigationIndicator()
  : Node("navigation_indicator")
  {
    brightness_ = static_cast<uint8_t>(std::clamp(
      static_cast<int>(this->declare_parameter<int>(
        "brightness", kDefaultIndicatorBrightness)), 0, 255));
    mute_audio_ = this->declare_parameter<bool>("mute_audio", false);
    preview_mode_ = this->declare_parameter<bool>("preview_mode", false);
    status_timeout_s_ = this->declare_parameter<double>("status_timeout_s", 2.0);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);

    indicator_pub_ = this->create_publisher<carcar_interfaces::msg::IndicatorCommand>(
      "/hardware/indicator/command", rclcpp::QoS(10).reliable());

    status_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/status", rclcpp::QoS(10).reliable(),
      std::bind(&NavigationIndicator::on_navigation_status, this, std::placeholders::_1));

    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&NavigationIndicator::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
      "NAV-014 导航声光提示节点已就绪: 亮度=%u, 静音=%s, 预览模式=%s, 发布频率=%.1f Hz",
      brightness_, mute_audio_ ? "true" : "false", preview_mode_ ? "true" : "false", publish_rate_hz_);
  }

  void publish_off()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_) {
        return;
      }
      shutdown_ = true;
      // preview_mode 不向底盘发灭灯，避免离线预览改写硬件状态。
      if (indicator_pub_ && !preview_mode_) {
        carcar_interfaces::msg::IndicatorCommand cmd;
        cmd.stamp = this->now();
        cmd.sender_id = "navigation_indicator_shutdown";
        cmd.sequence = ++sequence_;
        cmd.r = 0;
        cmd.g = 0;
        cmd.b = 0;
        cmd.beep_trigger = false;
        cmd.beep_duration_ms = 0;
        indicator_pub_->publish(cmd);
      }
    }
    if (timer_) {
      timer_->cancel();
    }
  }

private:
  void on_navigation_status(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) {
      return;
    }
    last_status_time_ = this->now();

    for (const auto & status : msg->status) {
      if (status.name == "navigation_status") {
        for (const auto & kv : status.values) {
          if (kv.key == "stage_code") {
            latest_input_.stage_code = kv.value;
          } else if (kv.key == "cause_code") {
            latest_input_.cause_code = kv.value;
          } else if (kv.key == "is_still") {
            latest_input_.is_still = (kv.value == "true" || kv.value == "True");
          } else if (kv.key == "task_status") {
            latest_input_.task_status = kv.value;
          }
        }
      }
    }
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) {
      return;
    }
    const auto now = this->now();
    const double now_sec = now.seconds();

    double status_age_s = 100.0;
    if (last_status_time_.nanoseconds() > 0) {
      status_age_s = (now - last_status_time_).seconds();
    }
    latest_input_.status_age_s = status_age_s;

    IndicatorOutput output;
    state_machine_.update(
      now_sec, latest_input_, mute_audio_, status_timeout_s_, brightness_, output);

    carcar_interfaces::msg::IndicatorCommand cmd;
    cmd.stamp = now;
    cmd.sender_id = "navigation_indicator";
    cmd.sequence = ++sequence_;
    cmd.r = output.color.r;
    cmd.g = output.color.g;
    cmd.b = output.color.b;
    cmd.beep_trigger = output.beep_trigger;
    cmd.beep_duration_ms = output.beep_duration_ms;

    if (!preview_mode_) {
      indicator_pub_->publish(cmd);
    }

    const bool state_changed = (output.display_desc != last_display_desc_ || output.beep_trigger);
    const bool periodic_log = (now_sec - last_preview_log_time_ >= 2.0);

    if (preview_mode_ && (state_changed || periodic_log)) {
      std::ostringstream ss;
      ss << "[声光预览] RGB=(" << static_cast<int>(cmd.r) << ","
         << static_cast<int>(cmd.g) << "," << static_cast<int>(cmd.b) << ") "
         << output.display_desc;
      if (output.beep_trigger) {
        ss << " | [蜂鸣触发: " << output.beep_duration_ms << "ms]";
      }
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      last_preview_log_time_ = now_sec;
    }

    last_display_desc_ = output.display_desc;
  }

  std::mutex mutex_;
  uint8_t brightness_{kDefaultIndicatorBrightness};
  bool mute_audio_{false};
  bool preview_mode_{false};
  bool shutdown_{false};
  double status_timeout_s_{2.0};
  double publish_rate_hz_{20.0};
  uint64_t sequence_{0};

  rclcpp::Publisher<carcar_interfaces::msg::IndicatorCommand>::SharedPtr indicator_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  IndicatorStatusInput latest_input_;
  IndicatorStateMachine state_machine_;

  std::string last_display_desc_;
  double last_preview_log_time_{0.0};
};

}  // namespace carcar_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<carcar_navigation::NavigationIndicator>();
  std::weak_ptr<carcar_navigation::NavigationIndicator> weak = node;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak] {
    if (auto n = weak.lock()) { n->publish_off(); }
  });
  rclcpp::spin(node);
  if (rclcpp::ok()) { node->publish_off(); }
  rclcpp::shutdown();
  return 0;
}
