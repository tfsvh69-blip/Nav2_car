#pragma once
// NAV-012：实验导航最终速度联锁；默认关闭，BT 授权租约过期输出零速。
#include "carcar_navigation/motion_gate.hpp"
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

class NavigationMotionGate : public rclcpp::Node {
public:
  NavigationMotionGate() : Node("navigation_motion_gate") {
    const double lease=declare_parameter("permit_timeout",0.20);
    const double command_age=declare_parameter("command_timeout",0.30);
    if (!std::isfinite(lease) || lease<=0 || lease>0.20 ||
      !std::isfinite(command_age) || command_age<=0 || command_age>0.30) {
      throw std::invalid_argument("联锁超时必须为正且不超过 0.20/0.30 秒");
    }
    gate_=carcar_navigation::MotionGate(lease,command_age);
    output_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(1));
    permit_=create_subscription<std_msgs::msg::UInt64>("/navigation/motion_permit",rclcpp::QoS(1),
      [this](std_msgs::msg::UInt64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);gate_.permit(msg->data,now_steady());});
    input_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_smoothed_raw",rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);gate_.command(*msg,now_steady());});
    diag_=create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/navigation/motion_gate/status",10);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this] {
      std::lock_guard<std::mutex> lock(mutex_);
      double now=now_steady(); output_->publish(gate_.output(now));
      bool enabled=gate_.enabled(now);
      if (enabled!=last_enabled_ || now-last_diag_>=1) {
        diagnostic_msgs::msg::DiagnosticStatus s;
        s.name="NavigationMotionGate"; s.hardware_id="software_interlock";
        s.message=enabled?"OPEN":"CLOSED"; s.level=enabled?0:1;
        diag_->publish(s); last_enabled_=enabled; last_diag_=now;
      }
    });
    output_->publish(geometry_msgs::msg::Twist{});
  }
  void stop() {std::lock_guard<std::mutex> lock(mutex_);gate_.permit(0,now_steady()); output_->publish(geometry_msgs::msg::Twist{});}
private:
  static double now_steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  carcar_navigation::MotionGate gate_;
  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr permit_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool last_enabled_{false}; double last_diag_{0};
};
