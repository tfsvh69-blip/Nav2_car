#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

namespace carcar_mapping_console {
// 所有时限以调用方提供的车端单调秒计，绝不使用客户端时钟。
class Safety {
public:
  // 从最后一次收到推杆起保持运动。到点只归零，不撤使能。挑战不参与续租。
  // 断联后慢档最多再走约 5cm，快档约 18cm。会话/雷达/里程计过期仍是 0.30/0.50s，那些仍撤使能。
  static constexpr double control_lease_seconds = 1.0;
  struct Velocity { double x = 0, yaw = 0; };
  struct Timing {double gateway_age, scan_age, odom_age, lease_remaining, presence_remaining, challenge_age, request_gap;
    uint64_t sequence, accepted_sequence, rejected_requests; bool pressed; std::string request_result;};
  std::string session, reason = "尚未使能", gear = "slow";
  bool armed = false, require_video = false;
  double gateway_at = -1e9, scan_at = -1e9, odom_at = -1e9, video_at = -1e9;
  bool session_ready = false, topology_ok = true;
  std::deque<std::pair<std::string, double>> frames;
  void stop(const std::string & why) {
    armed = false; pressed_ = false; challenges_.clear(); reason = why;
  }
  void release_motion(const std::string & why) {
    pressed_ = false; input_x_ = input_yaw_ = 0; reason = why;
  }
  void connect(const std::string & id) {
    if (id.empty() || id == session) return;
    session = id; sequence_ = accepted_sequence_ = rejected_requests_ = 0;
    last_request_at_=request_gap_=challenge_age_=-1;request_result_="none";
    gear = "slow"; stop("已连接，请使能");
  }
  void disconnect(const std::string & id) {
    if (session == id) {stop("操作者已断开"); session.clear();}
  }
  // 松手只归零；序号屏障拒绝松手之前已排队的运动请求。
  bool release(const std::string & id, uint64_t sequence) {
    if (id != session || sequence <= sequence_) return false;
    sequence_ = sequence; pressed_ = false; reason = armed ? "已使能，松手停车" : "尚未使能";
    return true;
  }
  void challenge(std::string token, double now) {
    prune(now);
    challenges_.emplace_back(std::move(token), now);
    while (challenges_.size() > 16) challenges_.pop_front();
  }
  void frame(const std::string & id, double now) {
    if (!frames.empty() && frames.back().first == id) return;
    frames.emplace_back(id, now); video_at = now;
    while (!frames.empty() && now - frames.front().second > 2.0) frames.pop_front();
  }
  bool frame_fresh(const std::string & id, double now) const {
    for (const auto & f : frames) if (f.first == id) return now - f.second < 1.0;
    return false;
  }
  std::string health(double now) const {
    if (!topology_ok) return "速度出口拓扑异常";
    if (!session_ready || now - gateway_at > 0.30) return "建图会话未就绪";
    if (now - scan_at > 0.50) return "雷达数据过期";
    if (now - odom_at > 0.50) return "轮式里程计过期";
    if (require_video && now - video_at >= 1.0) return "相机源帧过期";
    return "";
  }
  bool enable(const std::string & id, const std::string & displayed, double now) {
    if (id != session || id.empty()) return false;
    const auto error = health(now);
    if (!error.empty() || (require_video && !frame_fresh(displayed, now))) {
      stop(error.empty() ? "浏览器画面过期" : error); return false;
    }
    armed = true; pressed_ = false; presence_deadline_ = now + control_lease_seconds;
    reason = "已使能，按住摇杆行驶"; return true;
  }
  bool heartbeat(const std::string & id, uint64_t sequence, const std::string & token, double now) {
    if (!armed || id != session || sequence <= sequence_) return false;
    note_request(now);
    sequence_ = sequence;
    token_deadline(token,now);
    presence_deadline_ = now;accepted_sequence_=sequence;request_result_="heartbeat_accepted";return true;
  }
  bool intent(const std::string & id, uint64_t sequence, const std::string & token,
              const std::string & direction, const std::string & displayed, double now) {
    if (direction != "forward" && direction != "backward" && direction != "left" && direction != "right") {
      if (id == session) stop("方向无效");
      return false;
    }
    return drive(id, sequence, token, direction == "forward" ? 1.0 : direction == "backward" ? -1.0 : 0.0,
      direction == "left" ? 1.0 : direction == "right" ? -1.0 : 0.0, displayed, now);
  }
  bool drive(const std::string & id, uint64_t sequence, const std::string & token,
             double x, double yaw, const std::string & displayed, double now) {
    if (!armed || id != session || sequence <= sequence_) return false;
    note_request(now);
    sequence_ = sequence;
    token_deadline(token,now);
    if (require_video && !frame_fresh(displayed, now)) {stop("浏览器画面过期"); return false;}
    const auto error = health(now);
    if (!error.empty()) {stop(error); return false;}
    if (!std::isfinite(x) || !std::isfinite(yaw) || std::abs(x) > 1 || std::abs(yaw) > 1) {
      stop("摇杆输入无效"); return false;
    }
    const double norm = std::max(1.0, std::hypot(x, yaw));
    input_x_ = x / norm; input_yaw_ = yaw / norm;
    presence_deadline_ = now + control_lease_seconds;accepted_sequence_=sequence;request_result_="drive_accepted";
    displayed_ = displayed; deadline_ = now + control_lease_seconds; pressed_ = true;
    reason = "按住行驶"; return true;
  }
  void change_gear(const std::string & value) {
    if (value != "slow" && value != "normal" && value != "fast") return;
    gear = value; stop("已换档，请重新使能");
  }
  Velocity tick(double now) {
    if (armed) {
      const auto error = health(now);
      if (!error.empty()) stop(error);
      else if (pressed_ && require_video && !frame_fresh(displayed_, now)) stop("浏览器画面过期");
      else if (pressed_ && now >= deadline_) release_motion("操纵授权超时");
      // 松手后没有可继续执行的运动；静止不靠浏览器心跳维持使能。
      // 兼容旧网页的heartbeat只记录回声期限，不能续租deadline_或触发运动。
    }
    Velocity v;
    if (!armed || !pressed_) return v;
    const double x = gear == "slow" ? 0.05 : gear == "normal" ? 0.10 : 0.18;
    const double yaw = gear == "slow" ? 0.10 : gear == "normal" ? 0.20 : 0.36;
    v.x = input_x_ * x; v.yaw = input_yaw_ * yaw;
    return v;
  }
  Timing timing(double now) const {return {now-gateway_at,now-scan_at,now-odom_at,deadline_-now,presence_deadline_-now,
    challenge_age_,request_gap_,sequence_,accepted_sequence_,rejected_requests_,pressed_,request_result_};}
private:
  // 挑战只留作诊断。续租看最后一次收到推杆的时刻，不看挑战何时签发。
  double token_deadline(const std::string &token, double now) {
    double sign_time=-1;for(const auto &c:challenges_)if(c.first==token){sign_time=c.second;break;}
    challenge_age_=sign_time<0?-1:now-sign_time;prune(now);
    return sign_time;
  }
  void note_request(double now) {request_gap_ = last_request_at_ < 0 ? -1 : now-last_request_at_; last_request_at_=now;}
  void prune(double now) {while (!challenges_.empty() && now - challenges_.front().second > 2.0) challenges_.pop_front();}
  uint64_t sequence_ = 0, accepted_sequence_=0, rejected_requests_=0;
  std::string request_result_="none";
  double deadline_ = -1, presence_deadline_ = -1;
  bool pressed_ = false;
  double input_x_ = 0, input_yaw_ = 0;
  double last_request_at_ = -1, request_gap_ = -1, challenge_age_ = -1;
  std::string displayed_;
  std::deque<std::pair<std::string, double>> challenges_;
};
}  // namespace carcar_mapping_console
