#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/http_server.hpp"
#include "carcar_mapping_console/data_flow.hpp"
#include "carcar_mapping_console/task_process.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <slam_toolbox/srv/deserialize_pose_graph.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <nav2_map_server/map_io.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/imgcodecs.hpp>
#include <boost/beast.hpp>
#include <boost/beast/http.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <ctime>
#include <iostream>
#include <deque>
using namespace carcar_mapping_console;
namespace fs = std::filesystem;
namespace {
bool inspection_blocks_mapping() {
  try {
    boost::asio::io_context io;
    boost::beast::tcp_stream stream(io);
    stream.expires_after(std::chrono::milliseconds(300));
    stream.connect({boost::asio::ip::make_address("127.0.0.1"), 8081});
    boost::beast::http::request<boost::beast::http::empty_body> request{boost::beast::http::verb::get, "/api/state", 11};
    request.set(boost::beast::http::field::host, "127.0.0.1:8081");
    boost::beast::http::write(stream, request);
    boost::beast::flat_buffer buffer;
    boost::beast::http::response<boost::beast::http::string_body> response;
    boost::beast::http::read(stream, buffer, response);
    if (response.result() != boost::beast::http::status::ok) return false;
    const auto value = json::parse(response.body(), nullptr, false);
    if (!value.is_object() || !value.contains("run") || !value["run"].is_object()) return false;
    const auto status = value["run"].value("status", "");
    return status == "RUNNING" || status == "PAUSED" || status == "PAUSING" || status == "CANCELING" || status == "STOPPING";
  } catch (const std::exception &) {return false;}
}
}
class Gateway : public rclcpp::Node {
public:
  Gateway() : Node("mapping_web_gateway") {
    for (const auto & name : {"slam_toolbox", "nav2_map_server", "realsense2_camera"}) {
      const auto xml = read_file(fs::path(ament_index_cpp::get_package_share_directory(name)) / "package.xml");
      const auto begin = xml.find("<version>"), end = xml.find("</version>", begin);
      if (begin == std::string::npos || end == std::string::npos) throw std::runtime_error("依赖版本无法读取");
      versions_[name] = xml.substr(begin + 9, end - begin - 9);
    }
    if (versions_.at("slam_toolbox") != "2.6.10") throw std::runtime_error("工程格式仅验收SLAM Toolbox 2.6.10，拒绝未验证版本");
    const auto share = ament_index_cpp::get_package_share_directory("carcar_mapping_console");
    web_root_ = declare_parameter("web_root", share + "/web");
    root_ = fs::absolute(declare_parameter("map_root", (fs::current_path() / "src/carcar_navigation/maps/mapping_sessions").string()));
    mode_ = declare_parameter("mode", "new"); bundle_id_ = declare_parameter("bundle_id", "");
    managed_ = mode_ == "standby";
    backend_hardware_ = declare_parameter("backend_hardware", true);
    slam_params_file_ = declare_parameter("slam_params_file", std::string(""));
    if (!slam_params_file_.empty()) {
      if (!fs::is_regular_file(slam_params_file_)) throw std::runtime_error("候选SLAM参数文件不存在");
      slam_params_file_ = fs::canonical(slam_params_file_).string();
      if (mode_ == "resume") throw std::runtime_error("候选SLAM入口仅供新建地图对照");
    }
    prepare_exe_ = ament_index_cpp::get_package_prefix("carcar_mapping_console") + "/lib/carcar_mapping_console/mapping_prepare";
    log_root_ = fs::absolute(declare_parameter("log_root", (fs::current_path() / "log/mapping_tablet").string()));
    const auto address = declare_parameter("bind_address", "0.0.0.0");
    const auto port = declare_parameter("port", 8080);
    if ((mode_ != "new" && mode_ != "resume" && !managed_) || port < 1 || port > 65535) throw std::runtime_error("模式或端口无效");
    if (mode_ == "resume") {
      bundle_ = validate_bundle(root_, bundle_id_);
      reference_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      if (nav2_map_server::loadMapFromYaml((bundle_ / "map.yaml").string(), *reference_) != nav2_map_server::LOAD_MAP_SUCCESS)
        throw std::runtime_error("历史导航地图无法读取");
      reference_png_ = encode(*reference_); phase_ = "REFERENCE"; forward_ = false;
    } else {phase_ = managed_ ? "STANDBY" : "WAITING"; forward_ = !managed_;}
    accepting_ = !managed_;
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    requests_ = create_publisher<std_msgs::msg::String>("/mapping/control/request", rclcpp::QoS(10));
    session_state_ = create_publisher<std_msgs::msg::String>("/mapping/session/state", rclcpp::QoS(1));
    relay_ = create_publisher<sensor_msgs::msg::LaserScan>("/mapping/slam_scan", rclcpp::SensorDataQoS().keep_last(1));
    deserialize_ = create_client<slam_toolbox::srv::DeserializePoseGraph>("/slam_toolbox/deserialize_map");
    serialize_ = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        if (!std::isfinite(age) || age < -0.1 || age > 0.5 || message->ranges.empty()) return;
        std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return; scan_ = message; scan_at_ = steady();
        scan_source_at_=scan_at_-std::max(0.0,age);
        if (one_scan_) {
          expected_pose_stamp_ = rclcpp::Time(message->header.stamp).nanoseconds();
          one_scan_ = false; relay_->publish(*message);
        } else if (forward_) relay_->publish(*message);
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        if (!valid_map(*message)) return;
        std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return; map_ = message; map_at_ = steady(); map_pending_ = message;
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!accepting_) return;
        if (message->header.frame_id != "map") return;
        const auto & p = message->pose.pose;
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !valid_rotation(p.orientation)) {
          error_ = "SLAM位姿无效，请停车、查看诊断并重启本次任务"; phase_ = "FAILED";
          forward_ = false; one_scan_ = false; expected_pose_stamp_ = 0; return;
        }
        if (phase_ == "TRIAL" && expected_pose_stamp_ != 0 && rclcpp::Time(message->header.stamp).nanoseconds() == expected_pose_stamp_) {
          candidate_ = {message->pose.pose.position.x, message->pose.pose.position.y, yaw(message->pose.pose.orientation)};
          phase_ = "CANDIDATE"; expected_pose_stamp_ = 0;
        }
        if (phase_ == "WAITING") have_pose_ = true;
        if (phase_ == "CURRENT" && forward_) dirty_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/wheel/odometry", rclcpp::QoS(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        if (!std::isfinite(age) || age < -0.1 || age > 0.5) return;
        std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return; odom_ = message; odom_at_ = steady();
      });
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>("/camera/debug/rgb_reference/compressed", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        if (age < -0.1 || age > 0.5 || !std::isfinite(age) || message->data.empty() || message->format.find("jpeg") == std::string::npos) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (image_ && rclcpp::Time(message->header.stamp) <= rclcpp::Time(image_->header.stamp)) return;
        image_ = message; image_at_ = steady();
      });
    control_sub_ = create_subscription<std_msgs::msg::String>("/mapping/control/state", rclcpp::QoS(1),
      [this](const std_msgs::msg::String & msg) {
        try {
          auto control = json::parse(msg.data);
          const auto session = control.value("controller_session", ""); control.erase("controller_session");
          {std::lock_guard<std::mutex> lock(mutex_);
            const auto key=std::to_string(control.value("armed",false))+"|"+control.value("reason","")+"|"+control.value("gear","")+"|"+control.value("health","");
            if(key!=control_event_key_){auto event=control;event.erase("challenge");record_event_locked("guard_state",event);control_event_key_=key;}
            guard_session_ = session; control_ = control; control_at_ = steady();
            if(control.contains("issued_steady_ms")){
              control_traces_.push_back({control.value("challenge",""),control.value("control_revision",uint64_t{0}),
                control.value("issued_steady_ms",control_at_*1000)});
              while(control_traces_.size()>16)control_traces_.pop_front();}}
          if (server_) server_->broadcast(control.dump());
        } catch (...) {}
      });
    log_sub_ = create_subscription<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(100).reliable().transient_local(),
      [this](const rcl_interfaces::msg::Log & log) {
        if (log.name.find("slam_toolbox") == std::string::npos || log.level < 40) return;
        std::lock_guard<std::mutex> lock(mutex_);
        // 排除订阅时回放的历史错误，只接受本轮恢复开始之后的日志。
        if ((phase_ == "LOADING" || phase_ == "TRIAL" || phase_ == "CANDIDATE") && rclcpp::Time(log.stamp).nanoseconds() >= restore_ros_stamp_) {
          error_ = "SLAM 恢复错误: " + log.msg; phase_ = "FAILED"; forward_ = false; one_scan_ = false;
        }
      });
    // HTTP监听前生成完整待命状态，避免首个请求读到{}或诊断erase(null)。
    update_state();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {update_state();});
    server_ = std::make_unique<HttpServer>(address, static_cast<unsigned short>(port),
      [this](const HttpRequest & request) {return http(request);},
      [this](const std::string & id, const std::string & body) {control(id, body);});
    encoder_ = std::thread([this] {encode_loop();});
    RCLCPP_INFO(get_logger(), "离线建图页面 http://%s:%ld；无自动避障，初始零运动", address.c_str(), static_cast<long>(port));
  }
  ~Gateway() override {
    running_ = false;
    if(server_)server_->stop();
    server_.reset();
    if (encoder_.joinable()) encoder_.join();
    if (job_.joinable()) job_.join();
    camera_.stop(); backend_.stop();
  }
private:
  static double yaw(const geometry_msgs::msg::Quaternion & q) {return std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));}
  static bool valid_rotation(const geometry_msgs::msg::Quaternion & q) {
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(norm) && norm > 0.5 && norm < 1.5;
  }
  static bool valid_map(const nav_msgs::msg::OccupancyGrid & m) {
    return m.header.frame_id == "map" && m.info.width && m.info.height && m.info.resolution > 0 &&
      std::isfinite(m.info.resolution) && std::isfinite(m.info.origin.position.x) && std::isfinite(m.info.origin.position.y) &&
      valid_rotation(m.info.origin.orientation) && static_cast<uint64_t>(m.info.width) * m.info.height == m.data.size() && m.data.size() <= 100000000;
  }
  static std::vector<unsigned char> encode(const nav_msgs::msg::OccupancyGrid & map) {
    cv::Mat image(map.info.height, map.info.width, CV_8UC1);
    for (uint32_t y = 0; y < map.info.height; ++y) for (uint32_t x = 0; x < map.info.width; ++x) {
      const int value = map.data[y * map.info.width + x];
      image.at<unsigned char>(map.info.height - 1 - y, x) = value < 0 ? 160 : static_cast<unsigned char>(255 - std::clamp(value, 0, 100) * 255 / 100);
    }
    std::vector<unsigned char> png;
    if (!cv::imencode(".png", image, png)) throw std::runtime_error("地图 PNG 编码失败");
    return png;
  }
  static json metadata(const nav_msgs::msg::OccupancyGrid & map) {
    return {{"width", map.info.width}, {"height", map.info.height}, {"resolution", map.info.resolution},
      {"origin", {map.info.origin.position.x, map.info.origin.position.y, yaw(map.info.origin.orientation)}}};
  }
  bool reference_view() const {return mode_ == "resume" && reference_ && phase_ != "CURRENT" && phase_ != "SAVING";}
  void encode_loop() {
    while (running_) {
      nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending;
      {std::lock_guard<std::mutex> lock(mutex_); pending.swap(map_pending_);}
      if (pending) {
        try {
          auto png = encode(*pending);
          std::lock_guard<std::mutex> lock(mutex_);
          if (pending == map_) {map_png_ = std::move(png); encoded_map_ = pending; ++map_version_;}
        } catch (const std::exception & e) {std::lock_guard<std::mutex> lock(mutex_); error_ = e.what();}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  void update_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (managed_ && (task_state_ == "STARTING" || task_state_ == "RUNNING")) {
      if ((backend_started_ && !backend_.alive()) || (task_state_ == "STARTING" && steady() - start_at_ > 45) ||
        (task_state_ == "RUNNING" && (steady() - control_at_ > 2 || steady() - scan_at_ > 2 || steady() - odom_at_ > 2 ||
          (phase_ == "CURRENT" && steady() - map_at_ > 15)))) {
        task_state_ = "FAULT"; phase_ = "FAILED"; accepting_ = forward_ = one_scan_ = false;
        busy_ = false; completed_operation_ = operation_;
        error_ = "建图任务退出或启动超时。查看诊断后可在平板重启；未保存进度可能丢失。";
        publish_request({{"action", "stop"}, {"session", controller_}});
      } else if (task_state_ == "STARTING" && steady() - control_at_ < .3 &&
        steady() - scan_at_ < .5 && steady() - odom_at_ < .5 && serialize_->service_is_ready() &&
        (phase_ == "CURRENT" || phase_ == "REFERENCE")) {
        task_state_ = "RUNNING"; busy_ = false; completed_operation_ = operation_;
        result_ = mode_ == "resume" ? "工程已载入，请指定实物位置与朝向并核对匹配。" : "输入已就绪，请确认现场后使能。";
      }
      if (steady() - control_at_ < .3 && !controller_.empty() && guard_session_ != controller_)
        publish_request({{"action", "connect"}, {"session", controller_}});
    }
    if (phase_ == "WAITING" && have_pose_ && map_ && !map_png_.empty()) phase_ = "CURRENT";
    if (phase_ == "TRIAL" && steady() - trial_at_ > 6) {phase_ = "FAILED"; one_scan_ = false; error_ = "首帧匹配超时，请重启本次建图任务后再试";}
    std_msgs::msg::String state;
    const bool ready = phase_ == "CURRENT" && !busy_ && !save_failed_ && !map_png_.empty() && steady() - map_at_ < 10.0 &&
      (!managed_ || task_state_ == "RUNNING");
    state.data = json{{"ready", ready}}.dump(); session_state_->publish(state);
    json telemetry{{"phase", phase_}, {"busy", busy_}, {"error", error_}, {"result", result_}, {"control", control_},
      {"scan_age", steady() - scan_at_}, {"odom_age", steady() - odom_at_}, {"map_age", steady() - map_at_},
      {"video_age", steady() - image_at_}, {"control_age", steady() - control_at_}, {"save_reminder", steady() - last_save_at_ >= 300},
      {"bundle", bundle_id_}, {"operation", operation_}, {"completed_operation", completed_operation_},
      {"managed", managed_}, {"task_state", task_state_}, {"dirty", dirty_}, {"save_failed", save_failed_},
      {"last_save_age", steady() - last_save_at_}, {"last_saved_bundle", last_saved_bundle_},
      {"video_requested", video_requested_}, {"video_running", video_requested_ && camera_.alive()},
      {"task_age", task_state_ == "STANDBY" ? 0.0 : steady() - start_at_},
      {"map_version", reference_view() ? "reference-" + bundle_id_ : "live-" + std::to_string(map_version_)}};
    auto displayed_map = reference_view() ? reference_ : encoded_map_;
    if (displayed_map) telemetry["map"] = metadata(*displayed_map);
    if (phase_ == "CANDIDATE") telemetry["pose"] = candidate_;
    else if (phase_ == "CURRENT" || phase_ == "SAVING" || phase_ == "WAITING") {
      try {auto transform = tf_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        telemetry["pose"] = {transform.transform.translation.x, transform.transform.translation.y, yaw(transform.transform.rotation)};
      } catch (const tf2::TransformException &) {}
    }
    if (steady() - scan_render_at_ >= 0.199) {
      scan_render_at_ = steady();json points=json::array();bool valid=false;int64_t source_stamp=-1;
      if (scan_ && steady() - scan_source_at_ < 0.5) {
        try {
          geometry_msgs::msg::TransformStamped transform;
          double px = 0, py = 0, angle = 0;
          if (phase_ == "CANDIDATE") {
            transform = tf_->lookupTransform("base_footprint", scan_->header.frame_id, tf2::TimePointZero);
            const double theta = candidate_[2];
            px = candidate_[0] + std::cos(theta)*transform.transform.translation.x - std::sin(theta)*transform.transform.translation.y;
            py = candidate_[1] + std::sin(theta)*transform.transform.translation.x + std::cos(theta)*transform.transform.translation.y;
            angle = theta + yaw(transform.transform.rotation);
          } else if (phase_ == "CURRENT" || phase_ == "WAITING") {
            transform = tf_->lookupTransform("map", scan_->header.frame_id, rclcpp::Time(scan_->header.stamp), rclcpp::Duration::from_seconds(0));
            px = transform.transform.translation.x; py = transform.transform.translation.y; angle = yaw(transform.transform.rotation);
          } else throw std::runtime_error("等待初始位姿");
          valid=true;source_stamp=rclcpp::Time(scan_->header.stamp).nanoseconds();
          for (size_t i = 0; i < scan_->ranges.size(); ++i) {
            const double range = scan_->ranges[i], theta = angle + scan_->angle_min + i * scan_->angle_increment;
            if (std::isfinite(range) && range >= std::max(0.10f, scan_->range_min) && range <= std::min(3.0f, scan_->range_max))
              points.push_back({px + range * std::cos(theta), py + range * std::sin(theta)});
          }
        } catch (const std::exception &) {}
      }
      if(valid!=scan_preview_valid_||source_stamp!=scan_preview_stamp_||points!=scan_points_){
        ++scan_version_;scan_points_=std::move(points);scan_preview_valid_=valid;
        scan_preview_stamp_=source_stamp;scan_preview_at_=valid?scan_source_at_:-1e9;
      }
    }
    telemetry["dataflow_protocol"]=1;telemetry["scan_version"]=scan_version();
    status_=telemetry;status_["control"]=compact_control(control_);status_["control"].erase("challenge");
    telemetry["scan"] = scan_points_; telemetry_ = std::move(telemetry);
  }
  std::string scan_version()const{return scan_stream_id_+"-"+std::to_string(scan_version_);}
  void publish_request(json value) {
    std_msgs::msg::String msg; msg.data = value.dump();
    try {requests_->publish(msg);} catch (const rclcpp::exceptions::RCLError &) {}
  }
  void record_event_locked(const std::string & type, const json & detail) {
    json event{{"event",type},{"server_steady",steady()},
      {"server_unix_ms",std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()},
      {"detail",detail}};
    events_.push_back(event);while(events_.size()>128)events_.pop_front();
    RCLCPP_INFO(get_logger(),"MAPPING_EVENT %s",event.dump().c_str());
  }
  void control(const std::string & id, const std::string & body) {
    if (body.empty()) {
      const auto detail=server_?server_->diagnostics().value("last_disconnect",json::object()):json::object();
      {std::lock_guard<std::mutex> lock(mutex_); if (controller_ == id) {controller_.clear();record_event_locked("controller_disconnected",detail);}}
      publish_request({{"action", "disconnect"}, {"session", id}}); return;
    }
    try {
      auto value = json::parse(body); const auto action = value.value("action", "");
      if (action == "connect") {std::lock_guard<std::mutex> lock(mutex_); controller_ = id;record_event_locked("controller_connected",json::object());}
      if(action=="client_event") {
        std::lock_guard<std::mutex> lock(mutex_);
        if(controller_!=id||steady()-client_event_at_<.1)return;
        client_event_at_=steady();
        const auto name=value.value("event",std::string());
        if(name.empty()||name.size()>64||name.find_first_not_of("abcdefghijklmnopqrstuvwxyz_0123456789")!=std::string::npos)return;
        json metrics=json::object();const auto raw=value.value("metrics",json::object());
        for(const auto key:{"control_gap_ms","http_gap_ms","round_trip_ms","loop_gap_ms","ws_close_code","ws_close_was_clean"})
          if(raw.contains(key)&&raw[key].is_number()&&std::isfinite(raw[key].get<double>()))metrics[key]=raw[key];
        record_event_locked("client_"+name,metrics);return;
      }
      if (action != "connect" && action != "enable" && action != "intent" && action != "drive" && action != "heartbeat" && action != "release" && action != "stop" && action != "gear") return;
      if ((action == "enable" || action == "drive" || action == "intent") && inspection_blocks_mapping()) return;
      if(action=="drive"||action=="intent"||action=="heartbeat") {
        std::lock_guard<std::mutex> lock(mutex_);const auto token=value.value("challenge","");
        reply_timing_={{"matched",false}};
        for(const auto & trace:control_traces_)if(trace.token==token){
          reply_timing_={{"matched",true},{"control_revision",trace.revision},{"reply_age_ms",steady()*1000-trace.issued_ms}};
          if(value.contains("response_delay_ms")&&value["response_delay_ms"].is_number()){
            const auto delay=value["response_delay_ms"].get<double>();
            if(std::isfinite(delay)&&delay>=0&&delay<=10000)reply_timing_["client_response_delay_ms"]=delay;
          }
          break;
        }
      }
      // 客户端绝不能自行选择/覆盖会话 ID。
      value["session"] = id; publish_request(value);
    } catch (...) {publish_request({{"session", id}, {"action", "stop"}});}
  }
  static HttpResponse response(const json & value, unsigned status = 200) {
    const auto body = value.dump(); return {status, "application/json; charset=utf-8", {body.begin(), body.end()}, {}};
  }
  json bundles() const {
    json items = json::array();
    if (fs::exists(root_)) for (const auto & item : fs::directory_iterator(root_)) {
      if (!item.is_directory() || item.is_symlink()) continue;
      const auto id = item.path().filename().string();
      try {
        validate_bundle(root_, id);
        const auto manifest = json::parse(read_file(item.path() / "manifest.json"));
        items.push_back({{"id", id}, {"valid", true}, {"source", manifest.value("source_bundle", "")}});
      } catch (const std::exception & e) {items.push_back({{"id", id}, {"valid", false}, {"error", e.what()}});}
    }
    std::sort(items.begin(), items.end(), [](const json & a, const json & b) {return a.at("id").get<std::string>() > b.at("id").get<std::string>();});
    return {{"bundles", items}};
  }
  void clear_inputs_locked() {
    accepting_ = forward_ = one_scan_ = have_pose_ = false; expected_pose_stamp_ = 0;
    map_.reset(); map_pending_.reset(); encoded_map_.reset(); reference_.reset(); scan_.reset(); odom_.reset(); image_.reset();
    map_png_.clear(); reference_png_.clear(); scan_points_ = json::array(); control_ = json::object(); guard_session_.clear();
    scan_preview_valid_=false;scan_preview_stamp_=-1;scan_preview_at_=-1e9;++scan_version_;control_traces_.clear();
    scan_source_at_=-1e9;
    reply_timing_=json::object();
    scan_at_ = odom_at_ = map_at_ = control_at_ = image_at_ = -1e9;
    ++map_version_; tf_->clear();
  }
  void start_task(const std::string & mode, const std::string & id) {
    nav_msgs::msg::OccupancyGrid::SharedPtr reference;
    fs::path directory;
    std::vector<unsigned char> png;
    if (mode == "resume") {
      directory = validate_bundle(root_, id); reference = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      if (nav2_map_server::loadMapFromYaml((directory / "map.yaml").string(), *reference) != nav2_map_server::LOAD_MAP_SUCCESS)
        throw std::runtime_error("历史导航地图无法读取");
      png = encode(*reference);
    }
    camera_.stop(); backend_.stop();
    std::vector<std::string> args{prepare_exe_, "--backend-only", "--mode", mode, "--map-root", root_.string()};
    if (!slam_params_file_.empty()) {args.push_back("--slam-params"); args.push_back(slam_params_file_);}
    if (backend_hardware_) args.push_back("--hardware");
    if (!id.empty()) {args.push_back("--bundle"); args.push_back(id);}
    const auto log = log_root_ / ("task_" + nonce() + ".log");
    {
      std::lock_guard<std::mutex> lock(mutex_); clear_inputs_locked();
      // 重建订阅以重新收到仍在运行的 /tf_static 发布者的保留消息。
      listener_.reset(); listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
      mode_ = mode; bundle_id_ = id; bundle_ = directory; reference_ = reference; reference_png_ = std::move(png);
      phase_ = mode == "resume" ? "REFERENCE" : "WAITING"; task_state_ = "STARTING";
      backend_started_ = false; start_at_ = steady(); last_save_at_ = start_at_;
      dirty_ = save_failed_ = video_requested_ = false; task_log_ = log; camera_log_.clear();
      last_saved_bundle_.clear(); error_.clear(); result_ = "正在预检并启动，请保持停车。";
    }
    backend_.start(args, log);
    {std::lock_guard<std::mutex> lock(mutex_); backend_started_ = true; accepting_ = true; forward_ = mode == "new";}
  }
  void close_task() {
    {std::lock_guard<std::mutex> lock(mutex_); accepting_ = forward_ = one_scan_ = false; task_state_ = phase_ = "STOPPING";}
    camera_.stop(); backend_.stop();
    std::lock_guard<std::mutex> lock(mutex_); clear_inputs_locked();
    phase_ = task_state_ = "STANDBY"; mode_ = "standby"; bundle_id_.clear();
    busy_ = dirty_ = save_failed_ = video_requested_ = backend_started_ = false;
    completed_operation_ = operation_; error_.clear(); result_ = "本次任务已结束，硬件任务已退出；网页待命。";
    if (!last_saved_bundle_.empty()) result_ += "最近已保存版本: " + last_saved_bundle_;
  }
  HttpResponse lifecycle(const std::string & target, const json & input) {
    std::string mode, id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!managed_) return response({{"error", "旧终端入口不提供任务切换，请使用常驻平板入口"}}, 409);
      if (controller_.empty() || input.value("session", "") != controller_) return response({{"error", "需要当前操作者会话"}}, 403);
      if (busy_ && !(target == "/api/end" && task_state_ == "STARTING")) return response({{"error", "操作执行中"}}, 409);
      if (target == "/api/start") {
        if (inspection_blocks_mapping()) return response({{"error", "巡检还在进行，请先在巡检页取消或等这轮结束"}}, 409);
        if (task_state_ != "STANDBY") return response({{"error", "请先结束或重启当前任务"}}, 409);
        mode = input.value("mode", "new"); id = input.value("bundle", "");
        if (mode != "new" && mode != "resume") return response({{"error", "建图模式无效"}}, 400);
        if (mode == "resume" && !slam_params_file_.empty())
          return response({{"error", "候选参数仅供新建地图对照；旧工程请使用原入口续建"}}, 409);
        if (mode == "resume") validate_bundle(root_, id); else id.clear();
      } else if (target == "/api/restart") {
        if (task_state_ == "STANDBY") return response({{"error", "尚无任务，请选择新建或续建"}}, 409);
        if (dirty_ && !input.value("acknowledge_loss", false)) return response({{"error", "重启可能丢失未保存进度，需要明确确认"}}, 409);
        mode = last_saved_bundle_.empty() ? mode_ : "resume";
        id = last_saved_bundle_.empty() ? bundle_id_ : last_saved_bundle_;
        if (mode == "resume" && !slam_params_file_.empty())
          return response({{"error", "候选入口不从已保存图恢复；请保存结束后新建下一轮对照"}}, 409);
      } else if (target == "/api/end") {
        if (dirty_ && !input.value("discard", false)) return response({{"error", "有未保存进度，请保存结束或明确放弃"}}, 409);
      } else if (target == "/api/video") {
        if (task_state_ != "RUNNING" || !backend_hardware_) return response({{"error", "相机仅在真实建图任务内按需开启"}}, 409);
      } else return response({{"error", "接口不存在"}}, 404);
      busy_ = true; ++operation_; error_.clear(); result_.clear();
      if (target == "/api/start") {task_state_ = "STARTING"; start_at_ = steady();}
      if (target == "/api/restart" || target == "/api/end") {task_state_ = "STOPPING"; accepting_ = forward_ = one_scan_ = false;}
    }
    publish_request({{"action", "stop"}, {"session", input.at("session")}});
    if (job_.joinable()) job_.join();
    job_ = std::thread([this, target, input, mode, id] {
      try {
        if (target == "/api/start" || target == "/api/restart") {
          start_task(mode, id); // 就绪和失败由独立状态轮询确认，而不是以 spawn 成功冒充启动成功。
        } else if (target == "/api/end") close_task();
        else {
          const bool enabled = input.at("enabled").get<bool>();
          camera_.stop();
          fs::path log;
          if (enabled) {
            log = log_root_ / ("camera_" + nonce() + ".log");
            camera_.start({"ros2", "launch", "carcar_camera", "camera_mapping_rgb.launch.xml"}, log);
          }
          std::lock_guard<std::mutex> lock(mutex_); video_requested_ = enabled; camera_log_ = log;
          if (!enabled) {image_.reset(); image_at_ = -1e9;}
          busy_ = false; completed_operation_ = operation_;
          result_ = enabled ? "已请求开启视频；以持续更新的画面为准，不影响目视遥控。" : "视频已关闭，目视随行遥控可继续。";
        }
      } catch (const std::exception & e) {
        camera_.stop(); if (target != "/api/video") backend_.stop();
        std::lock_guard<std::mutex> lock(mutex_); busy_ = false; completed_operation_ = operation_; error_ = e.what();
        if (target != "/api/video") {task_state_ = "FAULT"; phase_ = "FAILED"; accepting_ = forward_ = false;}
        else video_requested_ = false;
      }
    });
    std::lock_guard<std::mutex> lock(mutex_); return response({{"accepted", true}, {"operation", operation_}}, 202);
  }
  HttpResponse http(const HttpRequest & request) {
    const auto target = request.target.substr(0, request.target.find('?'));
    if (request.method == "GET" && (target == "/" || target == "/app.js" || target == "/rtc_control.js" || target == "/style.css")) {
      const auto name = target == "/" ? "index.html" : target.substr(1);
      auto data = read_file(web_root_ / name);
      return {200, target == "/" ? "text/html; charset=utf-8" : target == "/style.css" ? "text/css; charset=utf-8" : "application/javascript; charset=utf-8", {data.begin(), data.end()}, {}};
    }
    if (request.method == "GET" && (target == "/api/state"||target=="/api/status")) {
      json value;{std::lock_guard<std::mutex> lock(mutex_);value=target=="/api/status"?status_:telemetry_;}
      value["inspection_running"]=inspection_blocks_mapping();
      return response(value);
    }
    if(request.method=="GET"&&target=="/api/scan") {
      json value;{std::lock_guard<std::mutex> lock(mutex_);const auto version=scan_version();
        if(request.target=="/api/scan?version="+version)return {204,"application/json",{}, {}};
        const bool valid=scan_preview_valid_&&steady()-scan_preview_at_<.5;
        value={{"version",version},{"frame_id","map"},{"valid",valid},
          {"source_age_ms",valid?(steady()-scan_preview_at_)*1000:-1},
          {"points",valid?scan_points_:json::array()}};}
      return response(value);
    }
    if (request.method == "GET" && target == "/api/bundles") {
      {std::lock_guard<std::mutex> lock(mutex_); if (managed_ && task_state_ != "STANDBY") return response({{"error", "请在待命时选择工程"}}, 409);}
      return response(bundles());
    }
    if (request.method == "GET" && target == "/api/diagnostics") {
      fs::path task, camera; json value;
      {std::lock_guard<std::mutex> lock(mutex_); task = task_log_; camera = camera_log_; value = telemetry_;value["events"]=events_;value["diagnostics_version"]=2;value["control"].erase("challenge");
        value["reply_timing"]=reply_timing_;value["dataflow"]={{"status_period_ms",250},{"scan_period_ms",200},
          {"scan_version",scan_version()},{"scan_points",scan_points_.size()}};}
      if(server_)value["transport"]=server_->diagnostics();
      value["task_log"] = log_tail(task); value["camera_log"] = log_tail(camera); return response(value);
    }
    if (request.method == "GET" && target == "/api/map.png") {
      std::lock_guard<std::mutex> lock(mutex_); const auto & bytes = reference_view() ? reference_png_ : map_png_;
      auto displayed_map = reference_view() ? reference_ : encoded_map_;
      return {bytes.empty() ? 503U : 200U, "image/png", bytes,
        displayed_map ? std::map<std::string,std::string>{{"X-Map-Meta", metadata(*displayed_map).dump()}} : std::map<std::string,std::string>{}};
    }
    if (request.method == "GET" && target == "/api/video.jpg") {
      sensor_msgs::msg::CompressedImage::ConstSharedPtr image;
      {std::lock_guard<std::mutex> lock(mutex_); if (steady() - image_at_ >= 1) return response({{"error", "视频已过期"}}, 503); image = image_;}
      if (!image) return response({{"error", "等待视频"}}, 503);
      return {200, "image/jpeg", image->data, {{"X-Frame-Id", frame_id(image->header.stamp.sec, image->header.stamp.nanosec)}}};
    }
    if (request.method != "POST") return response({{"error", "接口不存在"}}, 404);
    const auto input = json::parse(request.body);
    if (target == "/api/start" || target == "/api/end" || target == "/api/restart" || target == "/api/video") return lifecycle(target, input);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (controller_.empty() || input.value("session", "") != controller_) return response({{"error", "需要当前操作者会话"}}, 403);
      if (busy_) return response({{"error", "操作执行中"}}, 409);
      if (target == "/api/confirm") {
        if (phase_ != "CANDIDATE") return response({{"error", "没有可确认的匹配候选"}}, 409);
        phase_ = "CURRENT"; forward_ = true; error_.clear(); return response({{"ok", true}});
      }
      if (target == "/api/save" || target == "/api/finish") {
        if (target == "/api/finish" && !managed_) return response({{"error", "此入口不提供任务结束"}}, 409);
        if (phase_ != "CURRENT" || !map_) return response({{"error", "当前工程尚不可保存"}}, 409);
        phase_ = "SAVING"; forward_ = false; busy_ = true; error_.clear();
      } else if (target == "/api/resume") {
        if (mode_ != "resume" || (phase_ != "REFERENCE" && phase_ != "CANDIDATE"))
          return response({{"error", "恢复状态无效；故障后须重启本次建图任务"}}, 409);
        std::vector<double> hint = input.at("pose").get<std::vector<double>>();
        if (hint.size() != 3 || !std::all_of(hint.begin(), hint.end(), [](double v) {return std::isfinite(v);}))
          return response({{"error", "位置或朝向无效"}}, 400);
        const auto & m = *reference_;
        const double rotation = yaw(m.info.origin.orientation), dx = hint[0] - m.info.origin.position.x, dy = hint[1] - m.info.origin.position.y;
        const double gx = (std::cos(rotation)*dx + std::sin(rotation)*dy) / m.info.resolution;
        const double gy = (-std::sin(rotation)*dx + std::cos(rotation)*dy) / m.info.resolution;
        if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0 || gy < 0 || gx >= m.info.width || gy >= m.info.height)
          return response({{"error", "初始位置超出旧图范围"}}, 400);
        const int x = static_cast<int>(std::floor(gx)), y = static_cast<int>(std::floor(gy));
        if (m.data[y*m.info.width+x] < 0 || m.data[y*m.info.width+x] >= 50)
          return response({{"error", "请在旧图已覆盖的空闲区域选择车体位置"}}, 400);
        hint[2] = std::remainder(hint[2], 2*M_PI); hint_ = hint;
        phase_ = "LOADING"; forward_ = false; one_scan_ = false; expected_pose_stamp_ = 0;
        busy_ = true; error_.clear(); restore_ros_stamp_ = now().nanoseconds();
      } else return response({{"error", "接口不存在"}}, 404);
      ++operation_; result_.clear();
    }
    publish_request({{"action", "stop"}, {"session", input.at("session")}});
    if (job_.joinable()) job_.join();
    job_ = std::thread([this, target] {
      try {wait_stopped(); if (target == "/api/save" || target == "/api/finish") {save(target == "/api/finish"); if (target == "/api/finish") close_task();} else resume();}
      catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex_); error_ = error.what();
        if ((target == "/api/save" || target == "/api/finish") && phase_ == "SAVING") {phase_ = "CURRENT"; forward_ = false; save_failed_ = true;}
        else {phase_ = "FAILED"; forward_ = false; one_scan_ = false;}
        busy_ = false; completed_operation_ = operation_;
      }
    });
    std::lock_guard<std::mutex> lock(mutex_);
    return response({{"accepted", true}, {"operation", operation_}}, 202);
  }
  void wait_stopped() {
    double stationary_since = -1;
    const double until = steady() + 5;
    while (running_ && steady() < until) {
      bool stationary;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stationary = odom_ && steady() - odom_at_ < 0.5 &&
          std::abs(odom_->twist.twist.linear.x) < 0.015 && std::abs(odom_->twist.twist.angular.z) < 0.03 &&
          steady() - control_at_ < 0.3 && !control_.value("armed", true) &&
          std::abs(control_.value("x", 1.0)) < 1e-6 && std::abs(control_.value("yaw", 1.0)) < 1e-6;
      }
      if (stationary) {if (stationary_since < 0) stationary_since = steady(); if (steady() - stationary_since > 0.3) return;}
      else stationary_since = -1;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("未确认停车，操作取消");
  }
  template<class Client, class Request> auto call(Client & client, Request request) {
    if (!client->wait_for_service(std::chrono::seconds(2))) throw std::runtime_error("SLAM 服务不可用");
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      client->remove_pending_request(future); throw std::runtime_error("SLAM 服务超时，请重启本次建图任务");
    }
    return future.get();
  }
  void resume() {
    validate_bundle(root_, bundle_id_);
    auto req = std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    req->filename = (bundle_ / "graph").string(); req->match_type = req->START_AT_GIVEN_POSE;
    {std::lock_guard<std::mutex> lock(mutex_); req->initial_pose.x = hint_[0]; req->initial_pose.y = hint_[1]; req->initial_pose.theta = hint_[2];}
    call(deserialize_, req);
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != "LOADING") throw std::runtime_error("SLAM 报告恢复错误，请重启后重试");
    phase_ = "TRIAL"; busy_ = false; completed_operation_ = operation_; one_scan_ = true; trial_at_ = steady();
    result_ = "恢复调用结束，等待首帧候选；尚未确认对齐";
  }
  void save(bool finish = false) {
    {std::lock_guard<std::mutex> lock(mutex_); if (phase_ != "SAVING") throw std::runtime_error("建图会话已失效，拒绝保存");}
    const double cutoff = steady();
    bool updated = false;
    for (int i = 0; i < 400 && running_; ++i) {
      {std::lock_guard<std::mutex> lock(mutex_); updated = map_ && map_at_ > cutoff + 3.05;}
      if (updated) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!updated) throw std::runtime_error("等待冻结后的地图更新超时，未保存工程");
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    {std::lock_guard<std::mutex> lock(mutex_); map = map_;}
    fs::create_directories(root_);
    const auto time = std::time(nullptr); std::tm tm{}; localtime_r(&time, &tm);
    std::ostringstream name; name << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << nonce().substr(0, 8);
    const auto directory = root_ / name.str();
    if (!fs::create_directory(directory)) throw std::runtime_error("工程目录重名，拒绝覆盖");
    json manifest{{"complete", false}, {"slam_version", versions_.at("slam_toolbox")}, {"base_frame", "base_footprint"},
      {"resolution", map->info.resolution}, {"created", name.str()}, {"source_bundle", bundle_id_},
      {"limits", {{"slow", {0.05, 0.10}}, {"normal", {0.10, 0.20}}, {"fast", {0.18, 0.36}}}}, {"range_m", 3.0},
      {"versions", versions_}, {"offline_validation_is_not_hardware_acceptance", true}};
    auto write_manifest = [&] {std::ofstream file(directory / "manifest.json", std::ios::trunc); file << manifest.dump(2) << '\n'; file.flush(); if (!file) throw std::runtime_error("工程清单写入失败");};
    write_manifest();
    try {
      nav2_map_server::SaveParameters parameters;
      parameters.map_file_name = (directory / "map").string(); parameters.image_format = "pgm";
      parameters.free_thresh = 0.25; parameters.occupied_thresh = 0.65;
      if (!nav2_map_server::saveMapToFile(*map, parameters)) throw std::runtime_error("导航地图保存失败");
      manifest["grid_saved"] = true; write_manifest();
      auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>(); request->filename = (directory / "graph").string();
      if (call(serialize_, request)->result != 0) throw std::runtime_error("SLAM 工程保存失败");
      manifest["graph_saved"] = true;
      for (const auto & file : {"map.yaml", "map.pgm", "graph.posegraph", "graph.data"}) {
        if (!fs::is_regular_file(directory / file) || fs::file_size(directory / file) == 0) throw std::runtime_error("保存产物缺失");
        manifest["sha256"][file] = sha256(directory / file);
      }
      // 保存当次 SLAM 实际参数，而不是把候选配置当成读回。
      auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "slam_toolbox");
      auto future = parameters_client->get_parameters({"resolution", "max_laser_range", "min_laser_range", "map_update_interval", "scan_topic", "map_frame", "base_frame", "odom_frame"});
      if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) throw std::runtime_error("SLAM 参数读回超时，工程暂不标记完整");
      for (const auto & parameter : future.get()) manifest["runtime_parameters"][parameter.get_name()] = parameter.value_to_string();
      manifest["complete"] = true; write_manifest();
      validate_bundle(root_, name.str());
      std::lock_guard<std::mutex> lock(mutex_);
      if (phase_ != "SAVING") throw std::runtime_error("保存期间建图会话失效，工程不标记完整");
      phase_ = finish ? "STOPPING" : "CURRENT"; forward_ = !finish; busy_ = finish;
      if (finish) task_state_ = "STOPPING";
      save_failed_ = dirty_ = false; last_saved_bundle_ = name.str();
      last_save_at_ = steady(); completed_operation_ = operation_; result_ = "地图与工程保存完整: " + name.str();
    } catch (const std::exception & error) {
      manifest["complete"] = false; manifest["error"] = error.what(); write_manifest(); throw;
    }
  }
  fs::path web_root_, root_, bundle_, log_root_, task_log_, camera_log_;
  std::string mode_, bundle_id_, phase_, controller_, guard_session_, error_, result_, prepare_exe_, last_saved_bundle_;
  std::string task_state_ = "STANDBY";
  std::string slam_params_file_;
  TaskProcess backend_, camera_;
  bool managed_ = false, backend_hardware_ = true, backend_started_ = false, accepting_ = true;
  bool dirty_ = false, save_failed_ = false, video_requested_ = false;
  double start_at_ = steady();
  std::mutex mutex_; std::atomic<bool> running_{true}; std::thread encoder_, job_;
  bool busy_ = false, forward_ = false, one_scan_ = false, have_pose_ = false;
  int64_t expected_pose_stamp_ = 0, restore_ros_stamp_ = 0;
  double map_at_ = -1e9, odom_at_ = -1e9, scan_at_ = -1e9, image_at_ = -1e9, control_at_ = -1e9;
  double trial_at_ = 0, scan_render_at_ = 0, last_save_at_ = steady(); uint64_t map_version_ = 0, operation_ = 0, completed_operation_ = 0;
  std::vector<double> hint_{0,0,0}, candidate_{0,0,0};
  json control_ = json::object(), telemetry_ = json::object(), status_=json::object(),scan_points_ = json::array(), versions_ = json::object();
  struct ControlTrace {std::string token;uint64_t revision;double issued_ms;};
  std::deque<ControlTrace> control_traces_;json reply_timing_=json::object();
  const std::string scan_stream_id_=nonce();uint64_t scan_version_=0;
  bool scan_preview_valid_=false;int64_t scan_preview_stamp_=-1;double scan_preview_at_=-1e9;
  double scan_source_at_=-1e9;
  std::deque<json> events_;std::string control_event_key_;double client_event_at_=-1e9;
  nav_msgs::msg::OccupancyGrid::SharedPtr reference_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_, map_pending_, encoded_map_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  sensor_msgs::msg::CompressedImage::ConstSharedPtr image_;
  std::vector<unsigned char> reference_png_, map_png_;
  std::unique_ptr<tf2_ros::Buffer> tf_; std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<HttpServer> server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr requests_, session_state_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr relay_;
  rclcpp::Client<slam_toolbox::srv::DeserializePoseGraph>::SharedPtr deserialize_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {auto node = std::make_shared<Gateway>(); rclcpp::spin(node); node.reset();}
  catch (const std::exception & error) {std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
