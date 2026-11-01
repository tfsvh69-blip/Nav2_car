#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/http_server.hpp"
#include "carcar_mapping_console/data_flow.hpp"
#include "carcar_mapping_console/task_process.hpp"
#include "carcar_mapping_console/operator_sessions.hpp"
#include "carcar_mapping_console/task_requests.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "carcar_mapping_console/navigation_view.hpp"
#include "carcar_mapping_console/publication.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
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
bool inspection_blocks_mapping(unsigned short port=8081) {
  try {
    boost::asio::io_context io;
    boost::beast::tcp_stream stream(io);
    stream.expires_after(std::chrono::milliseconds(300));
    stream.connect({boost::asio::ip::make_address("127.0.0.1"), port});
    boost::beast::http::request<boost::beast::http::empty_body> request{boost::beast::http::verb::get, "/api/state", 11};
    request.set(boost::beast::http::field::host, "127.0.0.1:"+std::to_string(port));
    boost::beast::http::write(stream, request);
    boost::beast::flat_buffer buffer;
    boost::beast::http::response<boost::beast::http::string_body> response;
    boost::beast::http::read(stream, buffer, response);
    if (response.result() != boost::beast::http::status::ok) return false;
    const auto value = json::parse(response.body(), nullptr, false);
    if (!value.is_object()) return false;
    if (value.contains("navigation_trial") && value["navigation_trial"].is_object() && value["navigation_trial"].value("active", false)) return true;
    if (!value.contains("run") || !value["run"].is_object()) return false;
    const auto status = value["run"].value("status", "");
    return status == "RUNNING" || status == "PAUSED" || status == "PAUSING" || status == "CANCELING" || status == "STOPPING";
  } catch (const std::exception &) {return false;}
}
}
#include "carcar_mapping_console/storage.hpp"
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
    autonomy_enabled_=declare_parameter("autonomy_enabled",false);
    const auto inspection_port=declare_parameter("inspection_port",8081);
    if(inspection_port<1||inspection_port>65535)throw std::runtime_error("巡检网页端口无效");
    inspection_port_=static_cast<unsigned short>(inspection_port);
    task_id_=declare_parameter("task_id",std::string(""));
    task_requests_.reset(task_id_);
    control_mode_=declare_parameter("control_mode",std::string("manual"));
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
      bundle_ = autonomy_enabled_?validate_autonomy_bundle(root_,bundle_id_):validate_bundle(root_, bundle_id_);
      reference_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      if (nav2_map_server::loadMapFromYaml((bundle_ / "map.yaml").string(), *reference_) != nav2_map_server::LOAD_MAP_SUCCESS)
        throw std::runtime_error("历史导航地图无法读取");
      reference_png_ = encode(*reference_); phase_ = "REFERENCE"; forward_ = false;
    } else {phase_ = managed_ ? "STANDBY" : "WAITING"; forward_ = !managed_;}
    if(autonomy_enabled_&&!managed_)task_state_="RUNNING";
    accepting_ = !managed_;
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    requests_ = create_publisher<std_msgs::msg::String>("/mapping/control/request", rclcpp::QoS(10));
    session_state_ = create_publisher<std_msgs::msg::String>("/mapping/session/state", rclcpp::QoS(1));
    relay_ = create_publisher<sensor_msgs::msg::LaserScan>("/mapping/slam_scan", rclcpp::SensorDataQoS().keep_last(1));
    autonomy_requests_=create_publisher<std_msgs::msg::String>("/mapping/autonomy/request",rclcpp::QoS(10));
    autonomy_sub_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{auto state=json::parse(m.data);std::lock_guard<std::mutex> lock(mutex_);
        if(task_id_.empty()&&!managed_){task_id_=state.value("task_id","");task_requests_.reset(task_id_);}
        if(state.value("task_id","")!=task_id_)return;
        autonomy_state_=state;autonomy_at_=steady();
        const auto source_ns=state.value("publication",json::object()).value("source_stamp_ns",int64_t(0));
        if(source_ns>0)autonomy_at_-=std::max(0.,(now().nanoseconds()-source_ns)*1e-9);
        for(const auto&result:state.value("request_results",json::array()))
          task_requests_.acknowledge(result.value("request_id",std::string()),result.value("request_result",std::string()),result.value("reason",std::string()));
        task_requests_.acknowledge(state.value("request_id",std::string()),state.value("request_result",std::string()),
          state.value("error",std::string()).empty()?state.value("reason",std::string()):state.value("error",std::string()));
        const auto speed_result=state.value("speed",json::object());
        if(!speed_result.value("pending",true)&&(speed_result.value("phase","")=="READY"||speed_result.value("phase","")=="FAILED"))
          task_requests_.acknowledge(speed_result.value("request_id",std::string()),speed_result.value("phase","")=="READY"?"accepted":"rejected",speed_result.value("error",std::string()));
        if(!pending_halt_id_.empty()){
          bool confirmed=state.value("request_id",std::string())==pending_halt_id_&&state.value("request_result",std::string())=="accepted";
          for(const auto&result:state.value("request_results",json::array()))
            if(result.value("request_id",std::string())==pending_halt_id_&&result.value("request_result",std::string())=="accepted")confirmed=true;
          const bool locked=state.value("phase",std::string())=="FAULT"&&state.value("navigation_terminal",false)&&state.value("still",false);
          if(confirmed||locked)pending_halt_id_.clear();
          else if(steady()-halt_sent_at_>=.2){
            std_msgs::msg::String stop;stop.data=json{{"task_id",task_id_},{"request_id",pending_halt_id_},{"action","halt"}}.dump();
            autonomy_requests_->publish(stop);halt_sent_at_=steady();
          }
        }
        control_mode_=state.value("control_mode",std::string("manual"));
        const auto check=state.value("operator_check",json::object());
        if(!pending_check_id_.empty()&&
           ((state.value("request_id",std::string())==pending_check_id_&&state.value("request_result",std::string())=="rejected")||
            (check.value("request_id",std::string())==pending_check_id_&&
             (check.value("phase",std::string())=="FAILED"||check.value("phase",std::string())=="CANCELLED"))))pending_check_id_.clear();
        if(!pending_check_id_.empty()&&check.value("request_id",std::string())==pending_check_id_&&
           check.value("phase",std::string())=="READY"&&state.value("phase",std::string())=="PAUSED"&&
           state.value("generation",uint64_t{0})==check.value("generation",uint64_t{0})&&
           state.value("still",false)&&state.value("navigation_terminal",false)&&!busy_&&!save_failed_&&
           (!managed_||task_state_=="RUNNING")){
          // 只认本次检查事务的车端终态；恢复入图不等于恢复运动。
          phase_="CURRENT";accepting_=forward_=true;one_scan_=false;error_.clear();
          automatic_saved_=false;dirty_=true;pending_check_id_.clear();
          result_="检查通过，车辆保持暂停；请选择继续建图、返回起点或手动接管";
        }
        const bool recovering=state.value("phase","")=="RECOVERING"||state.value("recovery_pending",false);
        if((!state.value("quality_fault",std::string()).empty()&&!recovering)||state.value("phase","")=="FAULT"){
          forward_=one_scan_=false;
          // 故障锁存持续生效，但不能用重复心跳打断正在保存的诊断证据，或覆盖保存失败原因。
          if(phase_!="SAVING")phase_="FAILED";
          if(!save_failed_)error_=state.value("reason","SLAM质量锁停");
        }
      }catch(...){}});
    recovery_sub_=create_subscription<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{const auto state=json::parse(m.data);std::lock_guard<std::mutex> lock(mutex_);
        if(state.value("task_id",std::string())!=task_id_)return;
        recovery_gate_=state.value("gate_write",false);
      }catch(...){}});
    filtered_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){if(!autonomy_enabled_)return;
        const double age=(now()-rclcpp::Time(m->header.stamp)).seconds();if(!std::isfinite(age)||age<-.1||age>.5||m->ranges.empty())return;
        std::lock_guard<std::mutex> lock(mutex_);if(!accepting_||recovery_gate_)return;
        if(one_scan_){expected_pose_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();one_scan_=false;relay_->publish(*m);}
        else if(forward_)relay_->publish(*m);
      });
    deserialize_ = create_client<slam_toolbox::srv::DeserializePoseGraph>("/slam_toolbox/deserialize_map");
    serialize_ = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
        if (!std::isfinite(age) || age < -0.1 || age > 0.5 || message->ranges.empty()) return;
        std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return; scan_ = message; scan_at_ = steady();
        scan_source_at_=scan_at_-std::max(0.0,age);
        if (!autonomy_enabled_ && one_scan_) {
          expected_pose_stamp_ = rclcpp::Time(message->header.stamp).nanoseconds();
          one_scan_ = false; relay_->publish(*message);
        } else if (!autonomy_enabled_ && forward_) relay_->publish(*message);
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        if (!valid_map(*message)) return;
        std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return; map_ = message; map_at_ = steady(); map_pending_ = message;
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", autonomy_enabled_?rclcpp::QoS(1).reliable().transient_local():rclcpp::QoS(1),
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
    const auto cost_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto path_qos = rclcpp::QoS(1).reliable();
    global_cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", cost_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {navigation_.grid(0, *message, steady());});
    local_cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", cost_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {navigation_.grid(1, *message, steady());});
    global_plan_sub_ = create_subscription<nav_msgs::msg::Path>("/plan", path_qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {navigation_.path(0, *message, steady());});
    local_plan_sub_ = create_subscription<nav_msgs::msg::Path>("/received_global_plan", path_qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {navigation_.path(1, *message, steady());});
    prediction_sub_ = create_subscription<nav_msgs::msg::Path>("/lookahead_collision_arc", path_qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {navigation_.path(2, *message, steady());});
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
    autonomy_timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{automatic_save();});
    server_ = std::make_unique<HttpServer>(address, static_cast<unsigned short>(port),
      [this](const HttpRequest & request) {return http(request);},
      [this](const std::string & id, const std::string & body) {control(id, body);},ControlPolicy::MultiClient);
    server_->set_inspect_upstream("127.0.0.1",inspection_port_);
    encoder_ = std::thread([this] {encode_loop();});
    RCLCPP_INFO(get_logger(), "建图页面 http://%s:%ld；%s，初始零运动", address.c_str(), static_cast<long>(port),
      autonomy_enabled_?"二维自主组合实验：原始激光近场保护，场地边界须人工核对":"旧手动入口：无自动避障");
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
    update_manual_handover_locked();
    for(const auto&pending:task_requests_.ready(steady())){
      const auto id=pending.at("request_id").get<std::string>();
      if(!autonomy_enabled_||(managed_&&task_state_!="RUNNING")||steady()-autonomy_at_>.5||!manual_handover_.empty()||!pending_halt_id_.empty()||!pending_check_id_.empty())
        task_requests_.acknowledge(id,"rejected","继续未下发：车端状态已改变或正在停车／重新检查，请等待状态同步");
      else {publish_request({{"action","stop"},{"session",operators_.owner()}});send_autonomy(pending.at("action"),id,pending.at("extra"));}
    }
    if(steady()-storage_at_>=1){storage_=storage_status(root_);storage_at_=steady();}
    if (managed_ && (task_state_ == "STARTING" || task_state_ == "RUNNING")) {
      if ((backend_started_ && !backend_.alive()) || (task_state_ == "STARTING" && steady() - start_at_ > 45) ||
        (task_state_ == "RUNNING" && (steady() - control_at_ > 2 || steady() - scan_at_ > 2 || steady() - odom_at_ > 2 ||
          (phase_ == "CURRENT" && steady() - map_at_ > 15)))) {
        const bool exited=backend_started_&&!backend_.alive();
        const bool startup_timeout=task_state_=="STARTING";
        task_state_ = exited||startup_timeout ? "FAULT" : "RUNNING";
        phase_ = "FAILED"; accepting_ = forward_ = one_scan_ = false;
        busy_ = false; completed_operation_ = operation_;
        error_ = exited||startup_timeout ? "建图任务退出或启动超时。查看诊断后可在平板重启；未保存进度可能丢失。" :
          "车端数据超过2秒未更新，已停止运动及入图；数据恢复后请重新检查";
        publish_request({{"action", "stop"}, {"session", operators_.owner()}});
      } else if (task_state_ == "STARTING" && steady() - control_at_ < .3 &&
        steady() - scan_at_ < .5 && steady() - odom_at_ < .5 && serialize_->service_is_ready() &&
        (phase_ == "CURRENT" || phase_ == "REFERENCE")) {
        task_state_ = "RUNNING"; busy_ = false; completed_operation_ = operation_;
        result_ = mode_ == "resume" ? "工程已载入，请指定实物位置与朝向并核对匹配。" : "输入已就绪，请确认现场后使能。";
      }
      if (steady() - control_at_ < .3 && !operators_.owner().empty() && guard_session_ != operators_.owner())
        publish_request({{"action", "connect"}, {"session", operators_.owner()}});
    }
    if (phase_ == "WAITING" && have_pose_ && map_ && !map_png_.empty()) phase_ = "CURRENT";
    if (phase_ == "TRIAL" && steady() - trial_at_ > 6) {phase_ = "FAILED"; one_scan_ = false; error_ = "首帧匹配超时，请重启本次建图任务后再试";}
    std_msgs::msg::String state;
    const bool ready = phase_ == "CURRENT" && !busy_ && !save_failed_ && storage_.value("start_allowed",false) && !map_png_.empty() && steady() - map_at_ < 10.0 &&
      (!managed_ || task_state_ == "RUNNING");
    state.data = json{{"ready", ready},{"storage",storage_}}.dump(); session_state_->publish(state);
    json telemetry{{"phase", phase_}, {"busy", busy_}, {"error", error_}, {"result", result_}, {"control", control_},
      {"manual_control",manual_status_locked()},{"task_operations",task_requests_.status()},
      {"autonomy_enabled",autonomy_enabled_},{"control_mode",control_mode_},{"autonomy",autonomy_state_},{"halt_pending",!pending_halt_id_.empty()},
      {"autonomy_age_ms",autonomy_state_.empty()?-1:(steady()-autonomy_at_)*1000},
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
    telemetry["navigation"]=navigation_.snapshot(*tf_, steady(), now().seconds());
    telemetry["dataflow_protocol"]=1;telemetry["scan_version"]=scan_version();
    telemetry["storage"]=storage_;
    status_=telemetry;status_["control"]=compact_control(control_);status_["control"].erase("challenge");
    telemetry["scan"] = scan_points_; telemetry_ = std::move(telemetry);
  }
  std::string scan_version()const{return scan_stream_id_+"-"+std::to_string(scan_version_);}
  json manual_status_locked(const std::string&id={})const{
    auto value=operators_.status(id);
    const bool ready=manual_handover_.empty()&&!operators_.owner().empty()&&guard_session_==operators_.owner()&&steady()-control_at_<.3;
    value["ready"]=ready;value["phase"]=manual_handover_.empty()?"READY":"STOPPING";
    value["reason"]=manual_notice_;
    return value;
  }
  void update_manual_handover_locked(){
    if(manual_handover_.empty())return;
    if(!operators_.connected(manual_handover_)){manual_handover_.clear();manual_notice_="接管设备已断开，车辆保持停车";return;}
    if(steady()-manual_handover_at_>5){manual_handover_.clear();manual_notice_="接管未完成：未取得持续新鲜停稳反馈，原遥控权保留";return;}
    const bool stopped=odom_&&steady()-odom_at_<.5&&std::abs(odom_->twist.twist.linear.x)<.015&&
      std::abs(odom_->twist.twist.linear.y)<.015&&std::abs(odom_->twist.twist.angular.z)<.03&&
      steady()-control_at_<.3&&!control_.value("armed",true)&&std::abs(control_.value("x",1.))<1e-6&&std::abs(control_.value("yaw",1.))<1e-6&&
      (!autonomy_enabled_||(steady()-autonomy_at_<.5&&!autonomy_state_.value("auto_start_pending",false)&&autonomy_state_.value("phase","")=="MANUAL"&&autonomy_state_.value("still",false)&&autonomy_state_.value("navigation_terminal",false)));
    if(!stopped){manual_still_at_=-1;return;}
    if(manual_still_at_<0)manual_still_at_=steady();
    if(steady()-manual_still_at_<.3)return;
    const auto next=manual_handover_;manual_handover_.clear();operators_.claim(next);
    publish_request({{"action","connect"},{"session",next}});manual_notice_="遥控权已交接；请在本页使能后操作摇杆";
  }
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
    std::lock_guard<std::mutex> lock(mutex_);
    if(body.empty()){
      const bool was_owner=operators_.disconnect(id);
      if(manual_handover_==id){manual_handover_.clear();manual_notice_="接管设备已断开，车辆保持停车";}
      record_event_locked("operator_disconnected",{{"manual_owner",was_owner}});
      if(was_owner)publish_request({{"action","disconnect"},{"session",id}});
      return;
    }
    try{
      auto value=json::parse(body);const auto action=value.value("action","");
      if(action=="connect"){
        const bool first_owner=operators_.connect(id,nonce());
        record_event_locked("operator_connected",{{"clients",operators_.size()},{"manual_owner",first_owner}});
        if(first_owner)publish_request({{"action","connect"},{"session",id}});
        return;
      }
      if(!operators_.connected(id))return;
      if(action=="client_event"){
        if(steady()-client_event_at_<.1)return;
        client_event_at_=steady();const auto name=value.value("event",std::string());
        if(name.empty()||name.size()>64||name.find_first_not_of("abcdefghijklmnopqrstuvwxyz_0123456789")!=std::string::npos)return;
        json metrics=json::object();const auto raw=value.value("metrics",json::object());
        for(const auto key:{"control_gap_ms","http_gap_ms","round_trip_ms","loop_gap_ms","ws_close_code","ws_close_was_clean"})
          if(raw.contains(key)&&raw[key].is_number()&&std::isfinite(raw[key].get<double>()))metrics[key]=raw[key];
        record_event_locked("client_"+name,metrics);return;
      }
      if(action=="stop"){
        const auto cause=value.value("cause",std::string());
        // 非持有端的失焦或连接清理不干扰正在操作的摇杆；明确停车对所有设备开放。
        if(!operators_.owns(id)&&!cause.empty()&&cause!="manual")return;
        manual_handover_.clear();
        const bool continuation_sent=task_requests_.continuation_submitted();
        task_requests_.cancel_continuations("任意设备明确停车已撤销旧继续请求，车辆保持停车");
        value["session"]=operators_.owner();publish_request(value);
        if(autonomy_enabled_&&(continuation_sent||autonomy_state_.value("auto_start_pending",false)||autonomy_state_.value("control_mode",control_mode_)=="automatic")){
          const auto request_id=nonce();task_requests_.submit(request_id,"halt",json::object(),steady());send_autonomy("halt",request_id);
        }
        return;
      }
      if(!operators_.owns(id)||!manual_handover_.empty())return;
      if(action!="enable"&&action!="intent"&&action!="drive"&&action!="heartbeat"&&action!="release"&&action!="gear")return;
      if((action=="enable"||action=="drive"||action=="intent")&&inspection_blocks_mapping(inspection_port_))return;
      if(autonomy_enabled_&&autonomy_state_.value("control_mode",std::string("manual"))=="automatic"&&
        (action=="enable"||action=="drive"||action=="intent"))return;
      if(action=="drive"||action=="intent"||action=="heartbeat"){
        const auto token=value.value("challenge","");reply_timing_={{"matched",false}};
        for(const auto&trace:control_traces_)if(trace.token==token){
          reply_timing_={{"matched",true},{"control_revision",trace.revision},{"reply_age_ms",steady()*1000-trace.issued_ms}};
          if(value.contains("response_delay_ms")&&value["response_delay_ms"].is_number()){
            const auto delay=value["response_delay_ms"].get<double>();
            if(std::isfinite(delay)&&delay>=0&&delay<=10000)reply_timing_["client_response_delay_ms"]=delay;
          }
          break;
        }
      }
      value["session"]=id;publish_request(value);
    }catch(...){if(operators_.owns(id))publish_request({{"session",id},{"action","stop"}});}
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
        bool compatible=true;std::string block_reason;
        try{validate_autonomy_bundle(root_,id);}catch(const std::exception&e){compatible=false;block_reason=e.what();}
        items.push_back({{"id", id}, {"valid", true}, {"source", manifest.value("source_bundle", "")},
          {"autonomy_compatible",compatible},{"autonomy_block_reason",block_reason},
          {"publication",bundle_publication(item.path())},{"diagnostic_only",manifest.value("diagnostic_only",false)}});
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
  void start_task(const std::string & mode, const std::string & id,const std::string & control_mode="manual") {
    nav_msgs::msg::OccupancyGrid::SharedPtr reference;
    fs::path directory;
    std::vector<unsigned char> png;
    if (mode == "resume") {
      directory = autonomy_enabled_?validate_autonomy_bundle(root_,id):validate_bundle(root_, id); reference = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      if (nav2_map_server::loadMapFromYaml((directory / "map.yaml").string(), *reference) != nav2_map_server::LOAD_MAP_SUCCESS)
        throw std::runtime_error("历史导航地图无法读取");
      png = encode(*reference);
    }
    camera_.stop(); backend_.stop();
    std::vector<std::string> args{prepare_exe_, "--backend-only", "--mode", mode, "--map-root", root_.string()};
    const auto task_id=nonce();
    if(autonomy_enabled_){args.push_back("--autonomy");args.push_back("--control-mode");args.push_back(control_mode);args.push_back("--task-id");args.push_back(task_id);}
    if (!slam_params_file_.empty()) {args.push_back("--slam-params"); args.push_back(slam_params_file_);}
    if (backend_hardware_) args.push_back("--hardware");
    if (!id.empty()) {args.push_back("--bundle"); args.push_back(id);}
    const auto log = log_root_ / ("task_" + nonce() + ".log");
    {
      std::lock_guard<std::mutex> lock(mutex_); clear_inputs_locked();
      // 重建订阅以重新收到仍在运行的 /tf_static 发布者的保留消息。
      listener_.reset(); listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
      mode_ = mode; bundle_id_ = id; bundle_ = directory; reference_ = reference; reference_png_ = std::move(png);
      task_id_=task_id;task_requests_.reset(task_id_);control_mode_=control_mode;autonomy_state_=json::object();autonomy_at_=-1e9;automatic_saved_=false;recovery_gate_=false;pending_halt_id_.clear();
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
    task_id_.clear();task_requests_.reset("");autonomy_state_=json::object();autonomy_at_=-1e9;pending_halt_id_.clear();
    if(!status_.empty()){status_["autonomy"]=json::object();status_["halt_pending"]=false;status_["autonomy_age_ms"]=-1;}
    busy_ = dirty_ = save_failed_ = video_requested_ = backend_started_ = false;
    completed_operation_ = operation_; error_.clear(); result_ = "本次任务已结束，硬件任务已退出；网页待命。";
    if (!last_saved_bundle_.empty()) result_ += "最近已保存版本: " + last_saved_bundle_;
  }
  HttpResponse lifecycle(const std::string & target, const json & input) {
    std::string mode, id, manual_owner;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!managed_) return response({{"error", "旧终端入口不提供任务切换，请使用常驻平板入口"}}, 409);
      if (!operators_.connected(input.value("session", ""))) return response({{"error", "需要已连接的设备会话"}}, 403);
      if (busy_ && !(target == "/api/end" && task_state_ == "STARTING")) return response({{"error", "操作执行中"}}, 409);
      if (target == "/api/start") {
        if (inspection_blocks_mapping(inspection_port_)) return response({{"error", "巡检还在进行，请先在巡检页取消或等这轮结束"}}, 409);
        if (task_state_ != "STANDBY") return response({{"error", "请先结束或重启当前任务"}}, 409);
        mode = input.value("mode", "new"); id = input.value("bundle", "");
        const auto requested_control=input.value("control_mode",std::string("manual"));
        if(requested_control!="manual"&&requested_control!="automatic")return response({{"error","网页控制方式无效，没有启动任务或下发目标"}},400);
        if(requested_control=="automatic"&&!autonomy_enabled_)
          return response({{"error","自动建图未启动：当前入口未启用自主组合；没有下发目标"}},409);
        if (mode != "new" && mode != "resume") return response({{"error", "建图模式无效"}}, 400);
        if (mode == "resume" && !slam_params_file_.empty())
          return response({{"error", "候选参数仅供新建地图对照；旧工程请使用原入口续建"}}, 409);
        if (mode == "resume") {if(autonomy_enabled_)validate_autonomy_bundle(root_,id);else validate_bundle(root_, id);} else id.clear();
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
      manual_owner=operators_.owner();
      busy_ = true; ++operation_; error_.clear(); result_.clear();
      if (target == "/api/start") {task_state_ = "STARTING"; start_at_ = steady();}
      if (target == "/api/restart" || target == "/api/end") {task_state_ = "STOPPING"; forward_ = one_scan_ = false;if(!autonomy_enabled_)accepting_=false;}
    }
    publish_request({{"action", "stop"}, {"session", manual_owner}});
    if(autonomy_enabled_&&target!="/api/start"&&target!="/api/video")send_autonomy("halt",nonce());
    if (job_.joinable()) job_.join();
    job_ = std::thread([this, target, input, mode, id] {
      try {
        if(autonomy_enabled_&&(target=="/api/restart"||target=="/api/end"))wait_stopped();
        if (target == "/api/start" || target == "/api/restart") {
          const auto control_mode=input.value("control_mode",std::string("manual"));
          if(control_mode!="manual"&&control_mode!="automatic")throw std::runtime_error("控制方式无效，没有下发目标");
          if(control_mode=="automatic"&&!autonomy_enabled_)
            throw std::runtime_error("自动启动未执行：当前入口未启用自主组合，没有下发目标");
          start_task(mode, id,control_mode); // 就绪和失败由独立状态轮询确认，而不是以 spawn 成功冒充启动成功。
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
      json value;{std::lock_guard<std::mutex> lock(mutex_);value=target=="/api/status"?status_:telemetry_;
        if(target=="/api/status")value["halt_pending"]=!pending_halt_id_.empty();}
      value["inspection_running"]=inspection_blocks_mapping(inspection_port_);
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
    if (request.method == "GET" && (target == "/api/navigation/global-costmap.png" || target == "/api/navigation/local-costmap.png")) {
      const auto layer = navigation_.image(target.find("global") != std::string::npos ? 0 : 1, *tf_, steady(), now().seconds());
      return {layer.bytes.empty() ? 503U : 200U, "image/png", layer.bytes, {{"X-Nav-Meta", layer.meta.dump()}}};
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
    if(target=="/api/operator"){
      std::lock_guard<std::mutex> lock(mutex_);const auto id=input.value("session",std::string());
      if(!operators_.connected(id))return response({{"error","设备会话尚未连接，请等待连接恢复"}},403);
      operators_.label(id,input.value("device_label",std::string()));
      const auto action=input.value("action",std::string("status"));
      if(action=="claim"){
        if(autonomy_enabled_&&(steady()-autonomy_at_>=.5||autonomy_state_.value("phase","")!="MANUAL"||autonomy_state_.value("auto_start_pending",false)||autonomy_state_.value("control_mode",control_mode_)!="manual"))
          return response({{"error","接管未开始：请先选择手动接管，等待车端停稳"}},409);
        if(!operators_.owns(id)||!manual_handover_.empty()){
          if(!manual_handover_.empty()&&manual_handover_!=id)return response({{"error","另一端正在接管，请等待本次停车交接完成"}},409);
          if(manual_handover_.empty()){
            task_requests_.cancel_continuations("遥控权交接已撤销待执行的继续请求，车辆保持停车");
            manual_handover_=id;manual_handover_at_=steady();manual_still_at_=-1;
            manual_notice_="正在停车交接遥控权；等待新鲜停稳反馈";
            publish_request({{"action","stop"},{"session",operators_.owner()}});
            record_event_locked("manual_takeover_requested",{{"client_id",operators_.status(id)["client_id"]}});
          }
        }
      }else if(action!="status")return response({{"error","设备操作无效"}},400);
      const auto own=operators_.status(id);
      return response({{"client_id",own["client_id"]},{"is_owner",own["is_owner"]},{"manual_control",manual_status_locked(id)}},action=="claim"?202:200);
    }
    if(target=="/api/autonomy") {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!operators_.connected(input.value("session","")))return response({{"error","需要已连接的设备会话"}},403);
      const auto action=input.at("action").get<std::string>();
      if(!manual_handover_.empty()&&!TaskRequests::stopping(action))
        return response({{"error","遥控权正在停车交接，本次操作未下发；等待交接结果，仍可暂停／停车"}},409);
      if(input.contains("task_id")&&input["task_id"]!=task_id_)return response({{"error","请求属于已结束的任务，没有下发；等待页面同步"}},409);
      if(operators_.size()>1&&!TaskRequests::stopping(action)&&(!input.contains("operation_epoch")||!input.contains("task_id")))
        return response({{"error","多端操作协议需要刷新页面，当前请求没有下发"}},409);
      if(!autonomy_enabled_||(!TaskRequests::stopping(action)&&(busy_||steady()-autonomy_at_>.5)))return response({{"error","自主建图未就绪，没有下发新目标；检查车端任务状态"}},409);
      if(!TaskRequests::stopping(action)&&!pending_halt_id_.empty())return response({{"error","停车请求尚待车端确认，没有提交新操作；等待导航终态与停稳"}},409);
      if(!TaskRequests::stopping(action)&&action!="recheck"&&!pending_check_id_.empty())return response({{"error","重新检查尚未结束，没有提交新操作；请等待检查结果，也可停车中止"}},409);
      if(action!="start"&&action!="resume"&&action!="pause"&&action!="manual"&&action!="finish"&&action!="return"&&action!="recheck"&&action!="halt"&&action!="set_speed")return response({{"error","操作无效"}},400);
      if(action=="recheck"&&save_failed_)return response({{"error","故障候选尚未保存完成，未开始重新检查；请查看保存错误并恢复磁盘或SLAM保存服务，车端会重试保存。也可结束本次任务"}},409);
      if(action=="recheck"||action=="start"||action=="resume"||action=="return"||action=="manual"){
        if(managed_&&task_state_!="RUNNING")return response({{"error","车端进程未运行，请先结束故障任务并重新启动；未下发目标"}},409);
        const auto disk=storage_status(root_);if(!disk.value("start_allowed",false))return response({{"error",disk["reason"]}},409);
      }
      json extra=json::object();
      if(action=="set_speed"){
        const auto phase=autonomy_state_.value("phase",std::string());
        if(phase!="EXPLORING"&&phase!="RETURNING"&&phase!="MANUAL"&&phase!="PAUSED")return response({{"error","调速未提交：车端正在停车、恢复或保存，等待该阶段完成"}},409);
        if(!input.contains("speed_mps")||!input["speed_mps"].is_number())return response({{"error","调速未提交：speed_mps需要以米/秒表示的数值"}},400);
        const double speed=input["speed_mps"].get<double>();
        if(!std::isfinite(speed)||speed<.05||speed>.18)return response({{"error","调速未提交：允许范围0.05～0.18米/秒"}},400);
        extra["speed_mps"]=speed;
      }
      const auto id=input.value("request_id",nonce());
      const auto submission=task_requests_.submit(id,action,extra,steady(),input.value("operation_epoch",json(nullptr)),action=="recheck"?pending_check_id_:std::string());
      if(!submission.error.empty())return response({{"error",submission.error},{"request_id",id}},409);
      if(action=="recheck")pending_check_id_=submission.canonical;
      if(TaskRequests::stopping(action))pending_check_id_.clear();
      if(submission.forward){
        if(action!="set_speed")publish_request({{"action","stop"},{"session",operators_.owner()}});
        // 网页暂停统一走可中止恢复的停车动作，避免旧页面的暂停在恢复阶段被拒绝。
        send_autonomy(action=="pause"?"halt":action,submission.canonical,extra);
      }
      record_event_locked("shared_task_request",{{"request_id",id},{"canonical_request_id",submission.canonical},{"action",action},{"queued",submission.queued},{"forwarded",submission.forward}});
      return response({{"accepted",true},{"request_id",id},{"canonical_request_id",submission.canonical},{"queued",submission.queued},{"coalesced",submission.canonical!=id},{"note","请求已登记；以各自请求结果、车端终态和停稳为准"}},202);
    }
    if(target=="/api/maps/review") {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!operators_.connected(input.value("session","")))return response({{"error","需要已连接的设备会话"}},403);
      if(task_state_!="STANDBY")return response({{"error","请结束建图并确认停车后审核发布"}},409);
      const auto request_id=input.value("request_id",nonce());
      auto reviewed=review_bundle(root_,input.at("bundle"),input.at("approve"),input.at("note"));
      reviewed["request_id"]=request_id;record_event_locked("map_review",reviewed);return response(reviewed);
    }
    if (target == "/api/start" || target == "/api/end" || target == "/api/restart" || target == "/api/video") return lifecycle(target, input);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!operators_.connected(input.value("session", ""))) return response({{"error", "需要已连接的设备会话"}}, 403);
      if (busy_) return response({{"error", "操作执行中"}}, 409);
      if (target == "/api/confirm") {
        if (phase_ != "CANDIDATE") return response({{"error", "没有可确认的匹配候选"}}, 409);
        phase_ = "CURRENT"; forward_ = true; error_.clear(); return response({{"ok", true}});
      }
      if (target == "/api/save" || target == "/api/finish") {
        if(autonomy_enabled_&&(steady()-autonomy_at_>.5||!autonomy_state_.value("navigation_terminal",false)||!autonomy_state_.value("still",false)||
          autonomy_state_.value("phase","")=="EXPLORING"||autonomy_state_.value("phase","")=="RETURNING"||
          autonomy_state_.value("phase","")=="STOPPING"||autonomy_state_.value("phase","")=="RECOVERING"||
          autonomy_state_.value("phase","")=="CHECKPOINTING"||autonomy_state_.value("phase","")=="CHECKING"))
          return response({{"error","先暂停自动任务，等待导航终态和停稳后保存"}},409);
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
        if(autonomy_enabled_)stationary=stationary&&steady()-autonomy_at_<.5&&
          autonomy_state_.value("navigation_terminal",false)&&autonomy_state_.value("still",false)&&
          autonomy_state_.value("phase","")!="EXPLORING"&&autonomy_state_.value("phase","")!="RETURNING"&&
          autonomy_state_.value("phase","")!="STOPPING"&&autonomy_state_.value("phase","")!="RECOVERING"&&
          autonomy_state_.value("phase","")!="CHECKPOINTING";
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
  void send_autonomy(const std::string & action,const std::string & id,json extra=json::object()) {
    if(action=="halt")pending_check_id_.clear();
    // 停车可以先于监督器启动；首个真实心跳到达后重发同一ID，直到车端确认。
    if(action=="halt"&&!task_id_.empty()){pending_halt_id_=id;halt_sent_at_=steady();}
    extra["task_id"]=task_id_;extra["request_id"]=id;extra["action"]=action;
    std_msgs::msg::String m;m.data=extra.dump();
    autonomy_requests_->publish(m);
  }
  void automatic_save() {
    bool diagnostic=false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!autonomy_enabled_||automatic_saved_||busy_||steady()<save_retry_at_||steady()-autonomy_at_>.5||!map_||
        !autonomy_state_.value("save_required",false)||!autonomy_state_.value("navigation_terminal",false)||!autonomy_state_.value("still",false))return;
      diagnostic=autonomy_state_.value("phase","")=="FAULT";
      automatic_saved_=true;phase_="SAVING";forward_=one_scan_=false;busy_=true;++operation_;
      publish_request({{"action","stop"},{"session",operators_.owner()}});
    }
    if(job_.joinable())job_.join();
    job_=std::thread([this,diagnostic]{
      try{wait_stopped();save(false,diagnostic,false);std::lock_guard<std::mutex> lock(mutex_);forward_=false;
        if(diagnostic)phase_="FAILED";
        send_autonomy("save_done",nonce());
      }catch(const std::exception&e){std::lock_guard<std::mutex> lock(mutex_);automatic_saved_=false;save_retry_at_=steady()+5;busy_=false;save_failed_=true;forward_=one_scan_=false;phase_="FAILED";error_=std::string("停车后保存未完成：")+e.what();completed_operation_=operation_;}
    });
  }
  void save(bool finish = false,bool diagnostic=false,bool keep_writing=true) {
    require_storage(root_,true);
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
    if(autonomy_enabled_){
      std::lock_guard<std::mutex> lock(mutex_);manifest["autonomy"]=autonomy_state_;manifest["diagnostic_only"]=diagnostic;
      manifest["review_required"]=true;
      manifest["control_mode"]=control_mode_;manifest["explore_commit"]="326cf8a0b487c34246bb8f3326afbcd69576dc60";
      manifest["filter"]="laser_filters 2.0.10 Speckle Distance";
    }
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
      auto future = parameters_client->get_parameters({"resolution", "max_laser_range", "min_laser_range", "map_update_interval", "scan_topic", "map_frame", "base_frame", "odom_frame", "ceres_loss_function"});
      if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) throw std::runtime_error("SLAM 参数读回超时，工程暂不标记完整");
      for (const auto & parameter : future.get()) manifest["runtime_parameters"][parameter.get_name()] = parameter.value_to_string();
      manifest["complete"] = true; write_manifest();
      validate_bundle(root_, name.str());
      if(autonomy_enabled_)atomic_json(directory/"publication.json",{{"status",diagnostic?"diagnostic":"candidate"},{"asset_sha256",manifest["sha256"]}});
      std::lock_guard<std::mutex> lock(mutex_);
      if (phase_ != "SAVING") throw std::runtime_error("保存期间建图会话失效，工程不标记完整");
      phase_ = diagnostic ? "FAILED" : (finish ? "STOPPING" : "CURRENT");
      forward_ = resume_scan_forwarding(finish,diagnostic,keep_writing); busy_ = finish && !diagnostic;
      if (finish) task_state_ = "STOPPING";
      save_failed_ = dirty_ = false; last_saved_bundle_ = name.str();
      last_save_at_ = steady(); completed_operation_ = operation_; result_ = "地图与工程保存完整: " + name.str();
    } catch (const std::exception & error) {
      manifest["complete"] = false; manifest["error"] = error.what(); write_manifest(); throw;
    }
  }
  fs::path web_root_, root_, bundle_, log_root_, task_log_, camera_log_;
  std::string mode_, bundle_id_, phase_, guard_session_, error_, result_, prepare_exe_, last_saved_bundle_;
  OperatorSessions operators_;TaskRequests task_requests_;
  std::string manual_handover_,manual_notice_;double manual_handover_at_=-1,manual_still_at_=-1;
  std::string task_state_ = "STANDBY";
  std::string slam_params_file_;
  TaskProcess backend_, camera_;
  bool managed_ = false, backend_hardware_ = true, backend_started_ = false, accepting_ = true;
  bool autonomy_enabled_=false,automatic_saved_=false;double save_retry_at_=-1e9;
  std::string control_mode_="manual",task_id_;
  unsigned short inspection_port_=8081;
  json autonomy_state_=json::object(),storage_=json::object();double autonomy_at_=-1e9,storage_at_=-1e9;
  std::string pending_check_id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autonomy_requests_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_sub_;
  bool recovery_gate_=false;
  std::string pending_halt_id_;double halt_sent_at_=-1e9;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr filtered_sub_;
  rclcpp::TimerBase::SharedPtr autonomy_timer_;
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
  NavigationView navigation_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, global_cost_sub_, local_cost_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_, local_plan_sub_, prediction_sub_;
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
