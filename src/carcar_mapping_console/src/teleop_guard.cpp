#include "carcar_mapping_console/safety.hpp"
#include "carcar_mapping_console/common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
using namespace carcar_mapping_console;
class TeleopGuard : public rclcpp::Node {
public:
  TeleopGuard() : Node("mapping_teleop_guard") {
    safety_.require_video = declare_parameter("require_video", false);
    output_ = create_publisher<geometry_msgs::msg::Twist>("/mapping/cmd_vel", rclcpp::QoS(1));
    state_ = create_publisher<std_msgs::msg::String>("/mapping/control/state", rclcpp::QoS(1));
    request_ = create_subscription<std_msgs::msg::String>("/mapping/control/request", rclcpp::QoS(10),
      [this](const std_msgs::msg::String & message) {
        try {
          if (message.data.size() > 2048) return;
          auto value = json::parse(message.data);
          const auto id = value.value("session", ""); const auto action = value.value("action", "");
          if (action == "connect") {safety_.connect(id); log_transition("connect"); return;}
          if (id != safety_.session || id.empty()) return;
          if (action == "disconnect") safety_.disconnect(id);
          else if (action == "stop") safety_.stop("网页请求停车: " + value.value("cause", std::string("manual")).substr(0,64));
          else if (action == "gear") safety_.change_gear(value.value("gear", ""));
          else if (action == "enable") safety_.enable(id, value.value("frame", ""), steady());
          else if (action == "release" || action == "drive" || action == "heartbeat") {
            if (!value.at("sequence").is_number_unsigned()) {safety_.stop("指令序号无效"); return;}
            const auto seq = value.at("sequence").get<uint64_t>();
            if (action == "release") safety_.release(id, seq);
            else if (action == "heartbeat") safety_.heartbeat(id, seq, value.value("challenge", ""), steady());
            else safety_.drive(id, seq, value.value("challenge", ""), value.at("x").get<double>(),
              value.at("yaw").get<double>(), value.value("frame", ""), steady());
          }
          else if (action == "intent") {
            if (!value.at("sequence").is_number_unsigned()) {safety_.stop("指令序号无效"); return;}
            safety_.intent(id, value.at("sequence").get<uint64_t>(),
              value.value("challenge", ""), value.value("direction", ""), value.value("frame", ""), steady());
          }
          log_transition(action);
        } catch (const std::exception &) {safety_.stop("操纵消息无效"); log_transition("invalid_request");}
      });
    readiness_ = create_subscription<std_msgs::msg::String>("/mapping/session/state", rclcpp::QoS(1),
      [this](const std_msgs::msg::String & message) {
        try {auto v = json::parse(message.data); safety_.session_ready = v.at("ready").get<bool>(); safety_.gateway_at = steady();}
        catch (...) {safety_.stop("建图会话消息异常"); safety_.session_ready = false;}
      });
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {if (!msg->ranges.empty() && fresh(msg->header.stamp)) safety_.scan_at = steady();});
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/wheel/odometry", rclcpp::QoS(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {if (fresh(msg->header.stamp)) safety_.odom_at = steady();});
    image_ = create_subscription<sensor_msgs::msg::CompressedImage>("/camera/debug/rgb_reference/compressed", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (!msg->data.empty() && msg->format.find("jpeg") != std::string::npos && fresh(msg->header.stamp) && stamp > last_frame_stamp_) {
          last_frame_stamp_ = stamp;
          safety_.frame(frame_id(msg->header.stamp.sec, msg->header.stamp.nanosec), steady());
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      if(!rclcpp::ok())return;
      try {
      const auto now_s = steady();
      timer_gap_ms_ = tick_at_ == 0 ? 0 : (now_s-tick_at_)*1000; tick_at_=now_s;
      if (now_s - topology_at_ > 0.5) {
        topology_at_ = now_s;
        const auto pubs = get_publishers_info_by_topic(output_->get_topic_name());
        safety_.topology_ok = pubs.size() == 1 && pubs.front().node_name() == get_name() &&
          get_publishers_info_by_topic("/scan").size() == 1 && get_publishers_info_by_topic("/wheel/odometry").size() == 1;
      }
      const auto velocity = safety_.tick(now_s);
      log_transition("tick");
      geometry_msgs::msg::Twist command; command.linear.x = velocity.x; command.angular.z = velocity.yaw;
      output_->publish(command);
      if (now_s - state_at_ >= 0.05) {
        state_at_ = now_s; const auto token = nonce(); safety_.challenge(token, now_s);
        json value{{"challenge", token}, {"armed", safety_.armed}, {"reason", safety_.reason},
          {"gear", safety_.gear}, {"x", velocity.x}, {"yaw", velocity.yaw}, {"health", safety_.health(now_s)},
          {"controller_session", safety_.session}, {"require_video", safety_.require_video},
          {"supported_gears", {"slow","normal","fast"}}, {"ack_sequence", safety_.timing(now_s).sequence},
          {"control_protocol",2},{"control_lease_ms",Safety::control_lease_seconds*1000},{"idle_heartbeat_required",false},
          {"control_revision",++control_revision_},{"control_epoch",control_epoch_},{"issued_steady_ms",now_s*1000},
          {"accepted_sequence",safety_.timing(now_s).accepted_sequence},
          {"timing", timing_json(now_s)}, {"last_stop", last_stop_}};
        std_msgs::msg::String state; state.data = value.dump(); state_->publish(state);
      }
      }catch(const rclcpp::exceptions::RCLError&e){
        if(rclcpp::ok())RCLCPP_ERROR(get_logger(),"手动门控ROS接口失效：%s",e.what());
      }
    });
  }
  ~TeleopGuard() override {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {try {output_->publish(zero);} catch (...) {}}
  }
private:
  json timing_json(double now_s) const {
    const auto t=safety_.timing(now_s);
    return {{"gateway_age_ms",t.gateway_age*1000},{"scan_age_ms",t.scan_age*1000},{"odom_age_ms",t.odom_age*1000},
      {"lease_remaining_ms",t.lease_remaining*1000},{"presence_remaining_ms",t.presence_remaining*1000},
      {"challenge_age_ms",t.challenge_age<0?-1:t.challenge_age*1000},{"request_gap_ms",t.request_gap<0?-1:t.request_gap*1000},
      {"timer_gap_ms",timer_gap_ms_},{"pressed",t.pressed},{"request_result",t.request_result},
      {"accepted_sequence",t.accepted_sequence},{"rejected_requests",t.rejected_requests}};
  }
  void log_transition(const std::string & action) {
    const auto now_s=steady(); const auto health=safety_.health(now_s);
    const auto key=std::to_string(safety_.armed)+"|"+safety_.reason+"|"+safety_.gear+"|"+health;
    if (key==transition_key_) return;
    json event{{"source","guard"},{"action",action},{"server_steady",now_s},{"armed",safety_.armed},
      {"reason",safety_.reason},{"health",health},{"gear",safety_.gear},{"timing",timing_json(now_s)}};
    if (was_armed_&&!safety_.armed) {event["event"]="permission_revoked";last_stop_=event;RCLCPP_WARN(get_logger(),"MAPPING_EVENT %s",event.dump().c_str());}
    else RCLCPP_INFO(get_logger(),"MAPPING_EVENT %s",event.dump().c_str());
    was_armed_=safety_.armed;transition_key_=key;
  }
  bool fresh(const builtin_interfaces::msg::Time & stamp) {
    const double age = (now() - rclcpp::Time(stamp)).seconds(); return std::isfinite(age) && age >= -0.1 && age <= 0.5;
  }
  Safety safety_; double state_at_ = 0, topology_at_ = 0; int64_t last_frame_stamp_ = 0;
  uint64_t control_revision_ = 0;
  const std::string control_epoch_ = nonce();
  double tick_at_=0,timer_gap_ms_=0; bool was_armed_=false;
  std::string transition_key_; json last_stop_=json::object();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_, readiness_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); auto node = std::make_shared<TeleopGuard>(); rclcpp::spin(node); node.reset(); rclcpp::shutdown(); return 0;
}
