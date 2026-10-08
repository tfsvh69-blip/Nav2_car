#pragma once
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>

namespace carcar_navigation {
// 单调时间由调用方传入，生产节点与单元测试使用同一逻辑。
class MotionGate {
public:
  MotionGate(double lease=0.20,double command_age=0.30,
    bool require_perception=false,double perception_timeout=0.30)
  : lease_(lease),command_age_(command_age),perception_timeout_(perception_timeout),
    require_perception_(require_perception),perception_allowed_(!require_perception) {}
  void permit(uint64_t token,double now) {
    expire(now);
    if (require_perception_ && !perception_allowed_) {token_=0; return;}
    if (token==0 || token!=token_) {have_command_=false;}
    token_=token; permit_time_=now;
  }
  void command(const geometry_msgs::msg::Twist & command,double now) {
    expire(now);
    if (require_perception_ && !perception_allowed_) {return;}
    if (!token_) {return;}
    const double values[]={command.linear.x,command.linear.y,command.linear.z,
      command.angular.x,command.angular.y,command.angular.z};
    for (double value : values) {if (!std::isfinite(value)) {have_command_=false; return;}}
    command_=command; command_time_=now; have_command_=true;
  }
  void perception_allowed(bool allowed) {
    perception_allowed_=allowed;
    if (!allowed) {have_command_=false;}
  }
  void perception_heartbeat(bool allowed,double now) {
    expire(now);
    if (require_perception_ && allowed!=perception_allowed_) {
      token_=0; have_command_=false;
    }
    perception_time_=now;
    perception_allowed(allowed);
    if (require_perception_ && !allowed) {token_=0;}
  }
  geometry_msgs::msg::Twist output(double now) {
    expire(now);
    if (perception_allowed_ && token_ && have_command_ && now-command_time_<=command_age_) {
      return command_;
    }
    return geometry_msgs::msg::Twist{};
  }
  bool enabled(double now) {expire(now); return perception_allowed_ && token_!=0;}
  bool perception_allowed() const {return perception_allowed_;}
private:
  void expire(double now) {
    if (require_perception_ && (now-perception_time_>perception_timeout_ || now<perception_time_)) {
      perception_allowed_=false; token_=0; have_command_=false;
    }
    if (now-permit_time_>lease_ || now<permit_time_) {token_=0; have_command_=false;}
  }
  uint64_t token_{0};
  double permit_time_{0},command_time_{0},lease_,command_age_,perception_timeout_;
  double perception_time_{-INFINITY};
  bool require_perception_{false};
  bool have_command_{false};
  bool perception_allowed_{true};
  geometry_msgs::msg::Twist command_;
};
}  // namespace carcar_navigation
