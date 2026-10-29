// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <cmath>
#include <thread>
#include <array>
#include <deque>
#include <fstream>
#include <future>
#include <mutex>
#include <optional>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_map_server/map_io.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <carcar_interfaces/srv/set_motion_hold.hpp>
#include <carcar_interfaces/srv/set_camera_servo.hpp>
#include <carcar_interfaces/msg/indicator_command.hpp>
#include <carcar_mapping_console/http_server.hpp>
#include <carcar_mapping_console/common.hpp>
#include <carcar_mapping_console/publication.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <carcar_mapping_console/safety.hpp>
#include "carcar_inspection/store.hpp"
#include "carcar_inspection/teaching.hpp"
#include "carcar_inspection/executor.hpp"
#include "carcar_inspection/upload.hpp"
#include "carcar_inspection/navigation_debug.hpp"
#include "carcar_inspection/mapping_status_monitor.hpp"
#include "carcar_inspection/navigation_speed.hpp"

namespace carcar_inspection {
namespace fs = std::filesystem;
using carcar_mapping_console::steady;
using carcar_mapping_console::nonce;
using HttpRequest = carcar_mapping_console::HttpRequest;
using HttpResponse = carcar_mapping_console::HttpResponse;
using Hold = carcar_interfaces::srv::SetMotionHold;
using Servo = carcar_interfaces::srv::SetCameraServo;
using Navigate = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
using Clear = nav2_msgs::srv::ClearEntireCostmap;
double yaw(const geometry_msgs::msg::Quaternion & q) {
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
std::string map_version(const nav_msgs::msg::OccupancyGrid & map) {
  json geometry{{"width",map.info.width},{"height",map.info.height},{"resolution",map.info.resolution},
    {"x",map.info.origin.position.x},{"y",map.info.origin.position.y},{"yaw",yaw(map.info.origin.orientation)}};
  const auto text=geometry.dump(); unsigned char digest[EVP_MAX_MD_SIZE];unsigned int count=0;
  auto * ctx=EVP_MD_CTX_new();require(ctx!=nullptr,"地图摘要初始化失败");
  EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr);EVP_DigestUpdate(ctx,text.data(),text.size());
  EVP_DigestUpdate(ctx,map.data.data(),map.data.size());EVP_DigestFinal_ex(ctx,digest,&count);EVP_MD_CTX_free(ctx);
  std::ostringstream out;for(unsigned i=0;i<count;++i)out<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(digest[i]);return out.str();
}
HttpResponse reply(const json & value, unsigned status=200) {
  const auto text=value.dump();return {status,"application/json; charset=utf-8",{text.begin(),text.end()},{}};
}
struct Frame {uint64_t sequence;cv::Mat rgb;double received;int64_t stamp;};

class Console : public rclcpp::Node {
public:
  Console() : Node("inspection_console"),tf_(get_clock()),listener_(tf_),
    store_(declare_parameter("data_root",std::string("/home/jetson/.local/share/carcar_inspection"))),
    executor_([this](const std::string & kind,const json & context,bool start){return step(kind,context,start);},[this]{halt();}) {
    hardware_=declare_parameter("enable_hardware",false);
    navigation_=declare_parameter("enable_navigation",false);
    goal_navigation_=declare_parameter("enable_goal_navigation",navigation_);
    require_depth_=declare_parameter("require_depth",true);
    load_map_=create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    speed_=std::make_unique<NavigationSpeed>(*this,hardware_&&(navigation_||goal_navigation_));
    settle_=declare_parameter("settle_seconds",0.75);
    stop_seconds_=declare_parameter("stop_seconds",0.5);
    frame_age_=declare_parameter("frame_age_seconds",1.0);
    operation_timeout_=declare_parameter("operation_timeout_seconds",30.0);
    navigation_timeout_=declare_parameter("navigation_timeout_seconds",180.0);
    upload_url_=declare_parameter("upload_url",std::string(""));
    mapping_status_=std::make_unique<MappingStatusMonitor>(
      declare_parameter("mapping_status_url",std::string("http://127.0.0.1:8080/api/status")));
    require(settle_>=0.2&&stop_seconds_>=0.2&&frame_age_>0&&frame_age_<=2&&operation_timeout_>=5&&navigation_timeout_>=10,"候选时间参数不满足保护范围");
    map_root_=declare_parameter("map_root",std::string("/home/jetson/luhao/my_nav_carcar/src/carcar_navigation/maps"));
    web_root_=declare_parameter("web_root",ament_index_cpp::get_package_share_directory("carcar_inspection")+"/web");
    const auto address=declare_parameter("bind_address",std::string("127.0.0.1"));
    const auto port=declare_parameter("port",8081);require(port>0&&port<65536,"网页端口无效");
    hold_client_=create_client<Hold>("/hardware/inspection/motion_hold");
    servo_client_=create_client<Servo>("/hardware/inspection/camera_servo");
    nav_=rclcpp_action::create_client<Navigate>(this,"navigate_to_pose");
    local_clear_=create_client<Clear>("/local_costmap/clear_entirely_local_costmap");
    global_clear_=create_client<Clear>("/global_costmap/clear_entirely_global_costmap");
    const auto sensor=rclcpp::SensorDataQoS().keep_last(2);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/wheel/odometry",sensor,[this](nav_msgs::msg::Odometry::ConstSharedPtr value){
      const auto age=(now()-rclcpp::Time(value->header.stamp)).seconds();
      const auto & v=value->twist.twist;
      wheel_vx_=v.linear.x;
      const bool quiet=std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.angular.z)&&
        std::hypot(v.linear.x,v.linear.y)<0.005&&std::abs(v.angular.z)<0.01;
      if(age<-.05||age>.3||!quiet)still_since_=0;
      else if(!still_since_||steady()-odom_at_>.3)still_since_=steady();
      if(age>=-.05&&age<=.3)odom_at_=steady();
    });
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",sensor,[this](sensor_msgs::msg::LaserScan::ConstSharedPtr value){
      if(!require_depth_&&std::count_if(value->ranges.begin(),value->ranges.end(),[](float r){return std::isfinite(r)&&r>=.1f&&r<=3.f;})<12)return;
      const auto age=(now()-rclcpp::Time(value->header.stamp)).seconds();if(age>=-.05&&age<=.5&&!value->ranges.empty())scan_at_=steady();
    });
    image_sub_=create_subscription<sensor_msgs::msg::Image>(declare_parameter("rgb_topic",std::string("/camera/camera/color/image_raw")),sensor,
      [this](sensor_msgs::msg::Image::ConstSharedPtr value){image(*value);});
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value){if(value->info.width&&value->info.height&&value->data.size()==static_cast<size_t>(value->info.width)*value->info.height)live_map_version_=map_version(*value);});
    ready_sub_=create_subscription<std_msgs::msg::Bool>("/localization_monitor/ready",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr value){localized_=value->data;localization_ready_at_=steady();});
    fault_sub_=create_subscription<std_msgs::msg::Bool>("/localization_monitor/fault",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr value){localization_fault_=value->data;localization_fault_at_=steady();});
    activation_pose_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",rclcpp::QoS(1),[this](const geometry_msgs::msg::PoseWithCovarianceStamped&m){
      const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();const auto age=(now()-rclcpp::Time(m.header.stamp)).seconds();
      if(activation_localization_pending_&&activation_initial_stamp_>0&&stamp>activation_initial_stamp_&&age>=-.05&&age<=.5&&m.header.frame_id=="map"&&
        std::isfinite(m.pose.pose.position.x)&&std::isfinite(m.pose.pose.position.y)&&std::isfinite(m.pose.covariance[0])&&std::isfinite(m.pose.covariance[7]))activation_localization_pending_=false;
    });
    perception_sub_=create_subscription<std_msgs::msg::Bool>("/navigation/perception_motion_allowed",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr value){perception_=value->data;perception_at_=steady();});
    obstacle_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/camera/navigation/obstacles",sensor,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr value){obstacle_stamp_=rclcpp::Time(value->header.stamp).nanoseconds();});
    clearing_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/camera/navigation/clearing",sensor,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr value){clearing_stamp_=rclcpp::Time(value->header.stamp).nanoseconds();});
    filter_diag_sub_=create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/camera/navigation/diagnostics",rclcpp::QoS(1),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr value){
        for(const auto & status:value->status)if(status.name=="D435DepthFilter"){
          filter_ok_=false;filter_diag_at_=steady();
          for(const auto & item:status.values)if(item.key=="valid_measurements")try{filter_ok_=status.level==0&&std::stoull(item.value)>0;}catch(const std::exception &){filter_ok_=false;}
        }
      });
    const std::array<std::string,2> layer_topics{"/local_costmap/depth_stvl/diagnostics","/global_costmap/depth_stvl/diagnostics"};
    for(size_t i=0;i<2;++i)layer_subs_[i]=create_subscription<diagnostic_msgs::msg::DiagnosticArray>(layer_topics[i],rclcpp::QoS(1),
      [this,i](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr value){
        layer_current_[i]=false;
        for(const auto & status:value->status)if(status.name=="depth_stvl"){
          layer_stamp_[i]=rclcpp::Time(value->header.stamp).nanoseconds();
          for(const auto & item:status.values){
            if(item.key=="source_current")layer_current_[i]=status.level==0&&item.value=="true";
            if(item.key=="update_seq")try{layer_sequence_[i]=std::stoull(item.value);}catch(const std::exception &){layer_current_[i]=false;}
          }
        }
      });
    const auto costmap_qos=rclcpp::QoS(1).reliable().transient_local();
    local_map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap",costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value){local_map_stamp_=rclcpp::Time(value->header.stamp).nanoseconds();navigation_debug_.grid(1,*value,steady());});
    global_map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",costmap_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value){global_map_stamp_=rclcpp::Time(value->header.stamp).nanoseconds();navigation_debug_.grid(0,*value,steady());});
    global_path_sub_=create_subscription<nav_msgs::msg::Path>("/plan",rclcpp::QoS(1),
      [this](nav_msgs::msg::Path::ConstSharedPtr value){navigation_debug_.path(0,*value,steady());});
    local_path_sub_=create_subscription<nav_msgs::msg::Path>("/local_plan",rclcpp::QoS(1),
      [this](nav_msgs::msg::Path::ConstSharedPtr value){navigation_debug_.path(1,*value,steady());});
    if(hardware_) {
      velocity_=create_publisher<geometry_msgs::msg::Twist>(declare_parameter("cmd_vel_topic",std::string("/cmd_vel")),rclcpp::QoS(1));
      initial_pose_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",rclcpp::QoS(1));
      indicator_=create_publisher<carcar_interfaces::msg::IndicatorCommand>("/hardware/indicator/command",rclcpp::QoS(1));
      nav_velocity_sub_=create_subscription<geometry_msgs::msg::Twist>("/inspection/navigation/cmd_vel",rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr value){nav_velocity_=*value;nav_velocity_at_=steady();});
      depth_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/inspection/navigation/points",sensor);
      depth_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/points",sensor,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr value){
          const auto age=(now()-rclcpp::Time(value->header.stamp)).seconds();
          if(depth_enabled_&&board_==94&&age>=-.05&&age<=.3&&rclcpp::Time(value->header.stamp).nanoseconds()>depth_open_stamp_){depth_pub_->publish(*value);depth_forwarded_at_=steady();}
        });
    }
    discover_maps();update_snapshot();
    server_=std::make_unique<carcar_mapping_console::HttpServer>(address,static_cast<unsigned short>(port),
      [this](const HttpRequest & request){return http(request);},
      [this](const std::string & session,const std::string & body){enqueue_control(session,body);});
    // /inspect prefixes must remain on this console, including isolated test ports.
    server_->set_inspect_upstream("127.0.0.1",static_cast<unsigned short>(port));
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    upload_thread_=std::thread([this]{
      while(!shutting_down_) {
        try {
          if(!upload_url_.empty()) {
            const auto item=store_.next_upload();
            if(!item.is_null()) {
              const auto receipt=post_photo(upload_url_,item.at("id"),item.at("sha256"),item.at("path"));
              store_.finish_upload(item.at("id"),receipt.status=="CONFIRMED"?"CONFIRMED":"PENDING");
            }
          }
        } catch (const std::exception &) {}
        for(int i=0;i<10&&!shutting_down_;++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
    RCLCPP_INFO(get_logger(),"APP-001 网页 http://%s:%d，硬件控制=%s，自动导航=%s；不自动启动硬件",address.c_str(),static_cast<int>(port),hardware_?"启用":"关闭",navigation_?"启用":"关闭");
    mapping_status_->start();
  }
  ~Console() override {shutdown();}
  void shutdown() {
    if(shutting_down_.exchange(true))return;
    mapping_status_->stop();
    if(upload_thread_.joinable())upload_thread_.join();
    if(server_)server_->stop();
    executor_.cancel();if(velocity_)velocity_->publish(geometry_msgs::msg::Twist{});
    if(trial_active())halt();
  }
private:
  struct Job {double deadline;std::string name;std::function<void()> run;std::function<void(const std::string &)> expire;};
  static std::string command_not_run(const std::string & name,const std::string & why) {
    return "上位机没有执行「"+name+"」："+why+"。这条操作没有生效，目标没有下发，车不会动。这是巡检控制台自己的排队超时，不是定位失败，也不是导航器拒绝。请再操作一次；若连续出现，查看车上日志里的 APP-001 请求未执行，处理方向是巡检控制台的控制循环，不是改地图或重设目标点。";
  }
  void enqueue_control(const std::string & session,const std::string & body) {
    std::lock_guard<std::mutex> lock(jobs_mutex_);
    if(jobs_.size()>=64)return;
    jobs_.push_back({steady()+1,"/ws/control",[this,session,body]{
      if(body.empty()) {if(session_==session){session_.clear();manual_.disconnect(session);teaching_.disconnect(session);if(trial_active())stop_trial("CANCELED","操作网页已断开，取消本次到点试验");}}
      else {
        const auto value=json::parse(body);
        if(value.value("action","")=="connect"){session_=session;manual_.connect(session);}
        else if(session==session_)control(value);
      }
    },[this](const std::string & why){RCLCPP_ERROR(get_logger(),"APP-001 请求未执行 target=/ws/control reason=%s",why.c_str());}});
  }
  HttpResponse http(const HttpRequest & request) {
    const auto target=request.target.substr(0,request.target.find('?'));
    if(request.method=="GET") {
      if(target=="/api/state"){std::lock_guard<std::mutex> lock(snapshot_mutex_);return reply(snapshot_);}
      if(target=="/api/navigation/global-costmap.png"||target=="/api/navigation/local-costmap.png"){
        const auto layer=navigation_debug_.image(target=="/api/navigation/global-costmap.png"?0:1,tf_,steady(),now().seconds());
        return {layer.first.empty()?404U:200U,"image/png",layer.first,{{"X-Navigation-Layer",layer.second.dump(-1,' ',true)}}};
      }
      if(target=="/api/map.png"){std::lock_guard<std::mutex> lock(snapshot_mutex_);return {map_png_.empty()?404U:200U,"image/png",map_png_,{}};}
      if(target=="/api/video.jpg"){std::lock_guard<std::mutex> lock(snapshot_mutex_);return {jpeg_.empty()?503U:200U,"image/jpeg",jpeg_,{{"X-Frame-Sequence",std::to_string(jpeg_sequence_)},{"X-Source-Stamp",std::to_string(jpeg_stamp_)}}};}
      if(target.rfind("/api/media/",0)==0) {
        const auto id=target.substr(11);const auto path=store_.media_path(id);
        if(!fs::is_regular_file(path))return reply({{"error","照片不存在"}},404);
        const auto bytes=carcar_mapping_console::read_file(path);return {200,"image/jpeg",{bytes.begin(),bytes.end()},{}};
      }
      require(target.find("..") == std::string::npos && target.find('%') == std::string::npos,"资源路径无效");
      const auto path=web_root_/(target=="/"?"index.html":target.substr(1));
      if(!fs::is_regular_file(path))return reply({{"error","页面资源不存在，请先构建网页"}},404);
      const auto suffix=path.extension().string();const std::string type=suffix==".html"?"text/html; charset=utf-8":suffix==".js"?"text/javascript; charset=utf-8":suffix==".css"?"text/css; charset=utf-8":"application/octet-stream";
      const auto bytes=carcar_mapping_console::read_file(path);return {200,type,{bytes.begin(),bytes.end()},{}};
    }
    if(request.method!="POST")return reply({{"error","不支持的方法"}},405);
    auto promise=std::make_shared<std::promise<HttpResponse>>();auto future=promise->get_future();
    const auto body=json::parse(request.body);
    const auto enqueued=steady();
    {std::lock_guard<std::mutex> lock(jobs_mutex_);if(jobs_.size()>=64){
        const auto full=command_not_run(target,"等待处理的操作已有64条");
        RCLCPP_ERROR(get_logger(),"APP-001 请求未执行 target=%s reason=queue_full",target.c_str());
        return reply({{"error",full}},429);
      }
      jobs_.push_back({steady()+5,target,[this,promise,body,target]{try {
        require(!session_.empty()&&body.value("session","")==session_,"操作者连接已变化，请重新连接");
        const auto result=command(target,body);update_snapshot();promise->set_value(reply(result));
      }catch(const std::exception & e){error_=e.what();RCLCPP_ERROR(get_logger(),"APP-001 请求被拒绝 target=%s reason=%s",target.c_str(),e.what());promise->set_value(reply({{"error",e.what()}},409));}},
      [this,promise,target,enqueued](const std::string & why){
        const auto message=command_not_run(target,why+"，已等待"+std::to_string(steady()-enqueued).substr(0,4)+"秒");
        error_=message;RCLCPP_ERROR(get_logger(),"APP-001 请求未执行 target=%s waited_s=%.2f reason=%s",target.c_str(),steady()-enqueued,why.c_str());
        try{promise->set_value(reply({{"error",message}},503));}catch(const std::future_error &){}}});}
    if(future.wait_for(std::chrono::seconds(5))!=std::future_status::ready){
      const auto message=command_not_run(target,"控制循环5秒内没有轮到这条请求");
      RCLCPP_ERROR(get_logger(),"APP-001 请求未执行 target=%s waited_s=%.2f reason=http_wait",target.c_str(),steady()-enqueued);
      return reply({{"error",message}},503);
    }
    return future.get();
  }
  json command(const std::string & target,const json & body);
  void control(const json & body);
  void discover_maps() {
    maps_=json::array();map_paths_.clear();
    auto add=[this](const fs::path & file,const std::string & id) {
      if(!fs::is_regular_file(file)||fs::is_symlink(file))return;
      if(!carcar_mapping_console::navigation_map_visible(file))return;
      valid_id(id);map_paths_[id]=file;maps_.push_back({{"id",id},{"name",id}});
    };
    if(!fs::is_directory(map_root_))return;
    for(const auto & entry:fs::directory_iterator(map_root_)) {
      if(entry.is_regular_file()&&entry.path().extension()==".yaml")add(entry.path(),entry.path().stem());
    }
    const auto sessions=map_root_/"mapping_sessions";
    if(fs::is_directory(sessions))for(const auto & entry:fs::directory_iterator(sessions))
      if(entry.is_directory()&&!entry.is_symlink())add(entry.path()/"map.yaml",entry.path().filename());
  }
  void load_planning_floor() {
    require(!trial_blocking(),"请先取消到点试验并确认停车");
    trial_={{"phase","IDLE"}};
    require(executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&!pending_,"请先结束当前执行或调整");
    require(teaching_.phase=="IDLE"||teaching_.phase=="CONFIRMED"||teaching_.phase=="FAULT","请先完成当前示教");
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id="map";
    map.info.resolution=0.05f;map.info.width=400;map.info.height=200;
    map.info.origin.orientation.w=1;
    map.data.assign(static_cast<size_t>(map.info.width)*map.info.height,0);
    loaded_map_=map;map_id_="planning_200m2";map_version_=map_version(map);
    map_info_={{"id",map_id_},{"version",map_version_},{"width",map.info.width},{"height",map.info.height},
      {"resolution",map.info.resolution},{"planning",true},
      {"origin",{{"x",0.0},{"y",0.0},{"yaw",0.0}}}};
    cv::Mat pixels(map.info.height,map.info.width,CV_8UC1,cv::Scalar(255));
    std::vector<uint8_t> encoded;cv::imencode(".png",pixels,encoded);
    {std::lock_guard<std::mutex> lock(snapshot_mutex_);map_png_=std::move(encoded);}
  }
  void remove_point(const std::string & id) {
    require(executor_.phase!="RUNNING"&&executor_.phase!="PAUSED","请先结束当前巡检");
    valid_id(id);
    const auto point=store_.get("points",id);
    require(!point.is_null()&&point.at("map_id")==map_id_,"这个点不在当前场地");
    for(auto task:store_.list("tasks")) {
      if(task.at("map_id")!=map_id_||!task.at("point_ids").is_array()) continue;
      json ids=json::array(); bool changed=false;
      for(const auto & item:task.at("point_ids")) {
        if(item.get<std::string>()==id) changed=true; else ids.push_back(item);
      }
      if(!changed) continue;
      if(ids.empty()) store_.erase("tasks",task.at("id"));
      else {task["point_ids"]=ids; store_.put("tasks",task,task.at("revision").get<int>());}
    }
    store_.erase("points",id);
  }
  void rename_point(const std::string & id,const std::string & name) {
    valid_id(id);
    require(!name.empty()&&name.size()<=180,"名称须为1～180字节");
    auto point=store_.get("points",id);
    require(!point.is_null()&&point.at("map_id")==map_id_,"这个点不在当前场地");
    point["name"]=name;
    store_.put("points",point,point.at("revision").get<int>());
  }
  void select_map(const std::string & id) {
    require(!trial_blocking(),"请先取消到点试验并确认停车");
    require(executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&!pending_,"请先结束当前执行或调整");
    require(teaching_.phase=="IDLE"||teaching_.phase=="CONFIRMED"||teaching_.phase=="FAULT","请先完成当前示教");
    const auto found=map_paths_.find(id);require(found!=map_paths_.end(),"地图不存在");
    trial_={{"phase","IDLE"}};
    nav_msgs::msg::OccupancyGrid map;
    require(nav2_map_server::loadMapFromYaml(found->second.string(),map)==nav2_map_server::LOAD_MAP_SUCCESS,"地图加载失败");
    require(map.info.width&&map.info.height&&map.data.size()==static_cast<size_t>(map.info.width)*map.info.height,"地图数据不完整");
    loaded_map_=map;map_id_=id;map_version_=map_version(map);map_info_={{"id",id},{"version",map_version_},{"width",map.info.width},{"height",map.info.height},
      {"resolution",map.info.resolution},{"origin",{{"x",map.info.origin.position.x},{"y",map.info.origin.position.y},{"yaw",yaw(map.info.origin.orientation)}}}};
    cv::Mat pixels(map.info.height,map.info.width,CV_8UC1);
    for(unsigned y=0;y<map.info.height;++y)for(unsigned x=0;x<map.info.width;++x){const int v=map.data[y*map.info.width+x];pixels.at<uint8_t>(map.info.height-1-y,x)=v<0?160:static_cast<uint8_t>(255-v*255/100);}
    std::vector<uint8_t> encoded;cv::imencode(".png",pixels,encoded);std::lock_guard<std::mutex> lock(snapshot_mutex_);map_png_=std::move(encoded);
  }
  bool still() const {return steady()-odom_at_<.3&&still_since_>0&&steady()-still_since_>=stop_seconds_;}
  json pose() {
    try {const auto t=tf_.lookupTransform("map","base_footprint",tf2::TimePointZero);
      const auto age=(now()-rclcpp::Time(t.header.stamp)).seconds();if(age<-.05||age>.5)return nullptr;
      return {{"x",t.transform.translation.x},{"y",t.transform.translation.y},{"yaw",yaw(t.transform.rotation)}};
    }catch(const tf2::TransformException &){return nullptr;}
  }
  bool base_health() {return !activation_localization_pending_&&steady()-odom_at_<.3&&steady()-scan_at_<.5&&!pose().is_null()&&localized_&&!localization_fault_&&
    steady()-localization_ready_at_<.5&&steady()-localization_fault_at_<.5&&map_version_==live_map_version_&&!map_version_.empty();}
  bool perception_health() const {return !require_depth_||(perception_&&steady()-perception_at_<.3);}
  bool topology() const {
    if(!hardware_)return false;
    const auto publishers=get_publishers_info_by_topic(velocity_->get_topic_name());
    return publishers.size()==1&&publishers.front().node_name()==get_name();
  }
  void image(const sensor_msgs::msg::Image & value) {
    const auto age=(now()-rclcpp::Time(value.header.stamp)).seconds();
    if(age<-.05||age>frame_age_||!value.width||!value.height||value.width>4096||value.height>2160||
      value.step<value.width*3||value.data.size()<static_cast<size_t>(value.step)*value.height||
      (value.encoding!="rgb8"&&value.encoding!="bgr8"))return;
    cv::Mat rgb(value.height,value.width,CV_8UC3,const_cast<uint8_t*>(value.data.data()),value.step);rgb=rgb.clone();
    if(value.encoding=="bgr8")cv::cvtColor(rgb,rgb,cv::COLOR_BGR2RGB);
    const auto stamp=rclcpp::Time(value.header.stamp).nanoseconds();if(stamp<=last_image_stamp_)return;
    last_image_stamp_=stamp;frames_.push_back({++frame_sequence_,rgb,steady(),stamp});while(frames_.size()>12)frames_.pop_front();
    cv::Mat bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);std::vector<uint8_t> encoded;cv::imencode(".jpg",bgr,encoded,{cv::IMWRITE_JPEG_QUALITY,80});
    std::lock_guard<std::mutex> lock(snapshot_mutex_);jpeg_=std::move(encoded);jpeg_sequence_=frame_sequence_;jpeg_stamp_=stamp;
  }
  const Frame * frame(uint64_t sequence) const {for(const auto & f:frames_)if(f.sequence==sequence)return &f;return nullptr;}
  bool frame_fresh(const Frame * value) const {return value&&steady()-value->received<frame_age_&&(now().nanoseconds()-value->stamp)*1e-9<frame_age_;}
  json save_photo(const Frame & value,const std::string & id,bool queue_upload) {
    require(frame_fresh(&value),"图像已经过期");require(fs::space(store_.root()).available>64ULL*1024*1024,"可用存储空间不足64MiB候选保护限额");
    const auto path=store_.media_path(id);require(!fs::exists(path),"照片编号已存在，拒绝覆盖");
    const auto temporary=path.string()+".partial.jpg";cv::Mat bgr;cv::cvtColor(value.rgb,bgr,cv::COLOR_RGB2BGR);
    require(cv::imwrite(temporary,bgr,{cv::IMWRITE_JPEG_QUALITY,95}),"照片写入失败");
    const int fd=::open(temporary.c_str(),O_RDONLY|O_CLOEXEC);require(fd>=0,"照片无法校验落盘");const int rc=::fsync(fd);::close(fd);require(rc==0,"照片同步落盘失败");
    fs::rename(temporary,path);const int dir=::open(path.parent_path().c_str(),O_DIRECTORY|O_RDONLY|O_CLOEXEC);require(dir>=0,"照片目录无法同步");const int drc=::fsync(dir);::close(dir);require(drc==0,"照片目录同步失败");
    const auto checksum=carcar_mapping_console::sha256(path);
    if(queue_upload)store_.enqueue_upload(id,checksum,path,!upload_url_.empty());
    return {{"id",id},{"source_stamp_ns",value.stamp},{"sha256",checksum},
      {"position_feedback",false},{"upload_status",!queue_upload?"TEACHING_ONLY":upload_url_.empty()?"PENDING_CONFIGURATION":"PENDING"},{"width",value.rgb.cols},{"height",value.rgb.rows}};
  }
  template<class Service,class Callback> void call(typename rclcpp::Client<Service>::SharedPtr client,
      std::shared_ptr<typename Service::Request> request,Callback callback) {
    require(!pending_,"请等待上一条硬件请求结束");require(client->service_is_ready(),"所需车端接口尚未就绪");
    auto result=client->async_send_request(request);const auto request_id=result.request_id;
    auto future=result.future.share();pending_deadline_=steady()+1.5;
    pending_=[this,client,future,request_id,callback]() mutable {
      if(future.wait_for(std::chrono::seconds(0))==std::future_status::ready){pending_=nullptr;callback(future.get());return;}
      if(steady()>pending_deadline_){client->remove_pending_request(request_id);pending_=nullptr;throw std::runtime_error("硬件接口响应超时，保持停车并重新核对状态");}
    };
  }
  void hold(bool enabled,std::function<void()> done={}) {
    auto request=std::make_shared<Hold::Request>();request->owner_token=hardware_owner_;request->driver_epoch=driver_epoch_;request->hold=enabled;
    call<Hold>(hold_client_,request,[this,enabled,done](Hold::Response::SharedPtr result){
      require(result->accepted&&result->held==enabled,result->reason);driver_epoch_=result->driver_epoch;board_=result->board_command;held_=result->held;
      if(done)done();
    });
  }
  void servo(int command,std::function<void()> done) {
    auto request=std::make_shared<Servo::Request>();request->owner_token=hardware_owner_;request->driver_epoch=driver_epoch_;
    request->command_id=nonce();request->expected_board_command=board_;request->board_command=command;
    request->deadline_steady_ns=static_cast<int64_t>((steady()+.4)*1e9);
    call<Servo>(servo_client_,request,[this,done](Servo::Response::SharedPtr result){
      require(result->driver_epoch==driver_epoch_&&result->sent,result->reason);board_=result->board_command;last_servo_at_=steady();last_servo_stamp_=now().nanoseconds();if(done)done();
    });
  }
  void tick();
  void update_snapshot();
  void halt();
  void persist_run(const std::string & status);
  BT::NodeStatus step(const std::string & kind,const json & context,bool start,bool trial=false);
  void start_navigation(const json & target);
  bool trial_active() const {
    const auto phase=trial_.value("phase","");
    return phase=="PREPARING"||phase=="NAVIGATING"||phase=="STOPPING";
  }
  bool trial_blocking() const {return trial_active()||trial_.value("phase","")=="LOCKED";}
  std::string trial_block_reason() {
    if(!hardware_)return "请先连接实车";
    if(!goal_navigation_)return "当前是静止测试，尚未启用单点导航";
    if(!speed_->ready())return "导航速度档位尚未确认，请查看速度设置并重试切档";
    if(map_id_.empty()||(map_info_.is_object()&&map_info_.value("planning",false)))return "请载入实测地图，规划草图不能导航";
    if(!base_health())return "请先完成小车初始定位，确认地图、雷达和底盘反馈有效";
    if(!topology())return "速度出口检查未通过，保持停车";
    if(!nav_->action_server_is_ready())return "导航服务尚未就绪";
    return "";
  }
  void stop_trial(const std::string & terminal,const std::string & reason) {
    if(!trial_active()||trial_.at("phase")=="STOPPING")return;
    trial_["phase"]="STOPPING";trial_["result"]=terminal;trial_["error"]=reason;
    RCLCPP_INFO(get_logger(),"APP-001 单点终态 target=/api/teach/navigate request_id=%s result=%s nav2_result=%s reason=%s；请求停车，调角及保存等待停稳确认",
      trial_.value("request_id","").c_str(),terminal.c_str(),trial_.value("nav2_result","NOT_SENT").c_str(),reason.c_str());
    trial_stop_at_=steady();halt();
  }
  void tick_trial() {
    if(!trial_active())return;
    const auto phase=trial_.at("phase").get<std::string>();
    const json context{{"parking",true},{"pose",trial_.at("target")}};
    if(phase=="PREPARING") {
      if(step("restore",context,!trial_step_started_,true)==BT::NodeStatus::SUCCESS) {
        trial_["phase"]="NAVIGATING";trial_step_started_=false;
      }else trial_step_started_=true;
    }else if(phase=="NAVIGATING") {
      const auto result=step("navigate",context,!trial_step_started_,true);trial_step_started_=true;
      if(result==BT::NodeStatus::SUCCESS)stop_trial("ARRIVED","");
      else if(result==BT::NodeStatus::FAILURE)stop_trial("FAILED",navigation_failure_reason_);
    }else {
      if(!pending_&&!held_)hold(true);
      if(held_&&still()&&nav_terminal_&&!nav_waiting_&&!pending_) {
        trial_["phase"]=trial_.at("result");
        trial_["reached_pose"]=pose();
        if(!trial_["reached_pose"].is_null()) {
          const auto & current=trial_["reached_pose"];const auto & desired=trial_["target"];
          trial_["arrival_error"]={{"distance_m",std::hypot(number(current,"x")-number(desired,"x"),number(current,"y")-number(desired,"y"))},
            {"yaw_rad",std::abs(std::remainder(number(current,"yaw")-number(desired,"yaw"),2*M_PI))}};
        }
        store_.event("teaching_navigation_finished",trial_);
      }else if(steady()-trial_stop_at_>10) {
        trial_["phase"]="LOCKED";trial_["error"]="导航取消或停稳尚未确认，保持锁停，请检查实车";
      }
    }
  }
  void valid_map_pose(const json & value) const {
    validate_pose(value);require(!map_id_.empty(),"请先选择地图");
    const double angle=yaw(loaded_map_.info.origin.orientation),dx=number(value,"x")-loaded_map_.info.origin.position.x,dy=number(value,"y")-loaded_map_.info.origin.position.y;
    const int x=static_cast<int>(std::floor((std::cos(angle)*dx+std::sin(angle)*dy)/loaded_map_.info.resolution));
    const int y=static_cast<int>(std::floor((-std::sin(angle)*dx+std::cos(angle)*dy)/loaded_map_.info.resolution));
    require(x>=0&&y>=0&&x<static_cast<int>(loaded_map_.info.width)&&y<static_cast<int>(loaded_map_.info.height)&&loaded_map_.data[y*loaded_map_.info.width+x]==0,"停车目标必须位于已知空闲栅格，实际净空还须导航和现场验证");
  }

  bool hardware_=false,navigation_=false,goal_navigation_=false,held_=false,localized_=false,localization_fault_=true,perception_=false,depth_enabled_=false,require_depth_=true;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_;
  json map_activation_={{"phase","IDLE"}};
  bool activation_localization_pending_=false;int64_t activation_initial_stamp_=0;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr activation_pose_sub_;
  double activation_deadline_=0;uint64_t activation_generation_=0;
  json trial_={{"phase","IDLE"}};bool trial_step_started_=false;double trial_stop_at_=0;
  std::string upload_url_;
  std::unique_ptr<MappingStatusMonitor> mapping_status_;
  std::unique_ptr<NavigationSpeed> speed_;
  double wheel_vx_=0;
  geometry_msgs::msg::Twist last_output_;
  // 控制循环只读后台缓存；不能在此同步请求8080，以免与巡检转发互等。
  bool mapping_blocks_inspection() const {
    return mapping_status_->blocks_inspection();
  }
  std::thread upload_thread_;
  double settle_=0,stop_seconds_=0,frame_age_=0,operation_timeout_=0,navigation_timeout_=0;
  double odom_at_=-1e9,scan_at_=-1e9,still_since_=0,perception_at_=-1e9,last_servo_at_=-1e9,nav_velocity_at_=-1e9,last_snapshot_=0;
  double depth_open_at_=0,depth_forwarded_at_=-1e9;
  double localization_ready_at_=-1e9,localization_fault_at_=-1e9,filter_diag_at_=-1e9;
  bool filter_ok_=false;
  int64_t obstacle_stamp_=0,clearing_stamp_=0;
  std::array<int64_t,2> layer_stamp_{};
  std::array<uint64_t,2> layer_sequence_{},clear_barrier_{};
  std::array<bool,2> layer_current_{};
  int64_t last_image_stamp_=0,last_servo_stamp_=0,depth_open_stamp_=0,local_map_stamp_=0,global_map_stamp_=0,clear_stamp_=0;
  uint64_t frame_sequence_=0,jpeg_sequence_=0;int64_t jpeg_stamp_=0;int board_=-1;
  std::string session_,hardware_owner_=nonce(),driver_epoch_,map_id_,map_version_,live_map_version_,error_;
  fs::path map_root_,web_root_;std::map<std::string,fs::path> map_paths_;json maps_=json::array(),map_info_=nullptr,snapshot_;
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;Store store_;Teaching teaching_;Executor executor_;
  carcar_mapping_console::Safety manual_;
  std::deque<Frame> frames_;std::vector<uint8_t> jpeg_,map_png_;std::mutex snapshot_mutex_,jobs_mutex_;std::deque<Job> jobs_;
  std::unique_ptr<carcar_mapping_console::HttpServer> server_;std::atomic<bool> shutting_down_{false};
  std::function<void()> pending_;double pending_deadline_=0;
  std::string operation_,operation_stage_,pending_terminal_;double operation_at_=0;uint64_t operation_generation_=0;
  json run_=nullptr;int run_revision_=0;
  nav_msgs::msg::OccupancyGrid loaded_map_;
  std::shared_future<GoalHandle::SharedPtr> nav_goal_future_;GoalHandle::SharedPtr nav_goal_;
  uint64_t nav_generation_=0;bool nav_terminal_=true,nav_waiting_=false,nav_success_=false,cancel_requested_=false;
  std::string nav_result_="NOT_SENT",navigation_failure_reason_;
  std::shared_future<Clear::Response::SharedPtr> local_clear_future_,global_clear_future_;
  double alarm_until_=0;bool alarm_trigger_=false,failure_alarm_started_=false;int alarm_color_=0;
  geometry_msgs::msg::Twist nav_velocity_;
  rclcpp::Client<Hold>::SharedPtr hold_client_;rclcpp::Client<Servo>::SharedPtr servo_client_;
  rclcpp::Client<Clear>::SharedPtr local_clear_,global_clear_;rclcpp_action::Client<Navigate>::SharedPtr nav_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_;
  rclcpp::Publisher<carcar_interfaces::msg::IndicatorCommand>::SharedPtr indicator_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_,local_map_sub_,global_map_sub_;
  NavigationDebug navigation_debug_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_,local_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_,fault_sub_,perception_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_,clearing_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr filter_diag_sub_;
  std::array<rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr,2> layer_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

json Console::command(const std::string & target,const json & body) {
  if(target=="/api/maps/activate") {
    require(!mapping_blocks_inspection(),"建图尚未结束，请先结束建图任务");
    require(map_activation_.value("phase","")!="HOLDING"&&map_activation_.value("phase","")!="LOADING","地图切换执行中，不重复提交");
    require(!trial_blocking()&&executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&!pending_&&still(),"地图启用未执行：请先结束运动任务并确认停稳");
    discover_maps();const auto id=body.at("map_id").get<std::string>();
    const auto found=map_paths_.find(id);require(found!=map_paths_.end(),"地图未发布或不存在");
    require(load_map_->service_is_ready(),"定位地图服务未启动，请先启动纯激光定位组合；没有切换地图或下发目标");
    const auto request_id=body.value("request_id",nonce());valid_id(request_id);
    activation_localization_pending_=true;activation_initial_stamp_=0;
    select_map(id);map_activation_={{"phase","HOLDING"},{"map_id",id},{"request_id",request_id}};activation_deadline_=steady()+8;
    const auto generation=++activation_generation_;
    hold(true,[this,id,request_id,generation,path=found->second.string()]{
      if(generation!=activation_generation_)return;
      map_activation_["phase"]="LOADING";
      auto request=std::make_shared<nav2_msgs::srv::LoadMap::Request>();request->map_url=path;
      load_map_->async_send_request(request,[this,id,request_id,generation](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture f){
        if(generation!=activation_generation_)return;
        if(f.get()->result==nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS){
          map_activation_={{"phase","WAITING_LOCALIZATION"},{"map_id",id},{"request_id",request_id},{"reason","地图已加载，保持停车；请按实车位置重新定位，旧点位不迁移"}};
        }else {map_activation_={{"phase","FAILED"},{"map_id",id},{"request_id",request_id}};error_="地图加载失败，保持停车；检查map_server日志";}
        RCLCPP_INFO(get_logger(),"地图启用 request_id=%s map=%s result=%s；保持停车，没有下发目标",request_id.c_str(),id.c_str(),map_activation_.at("phase").get<std::string>().c_str());
      });
    });
    return {{"accepted",true},{"request_id",request_id},{"note","地图启用已受理，保持停车；以加载与重新定位结果为准"}};
  }
  if(target=="/api/navigation/speed") {
    require(!trial_blocking()&&executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&pending_terminal_.empty()&&nav_terminal_&&!nav_waiting_,
      "车端健康检查拒绝切档：当前导航或任务尚未结束；没有修改档位或下发新目标，请先取消并停稳");
    require(still(),"车端健康检查拒绝切档：轮式反馈尚未确认停稳；没有修改档位或下发目标，请先停车");
    speed_->select(number(body,"max_linear_mps"),steady());
    manual_.stop("切换导航速度档位，保持停车");
    store_.event("navigation_speed_requested",{{"max_linear_mps",number(body,"max_linear_mps")}});
    RCLCPP_INFO(get_logger(),"APP-001 切档已受理 target=/api/navigation/speed speed_mps=%.2f；等待控制器参数读回，没有下发目标",number(body,"max_linear_mps"));
    return {{"accepted",true},{"speed",speed_->state()}};
  }
  if(target=="/api/map/select") {select_map(body.at("map_id"));return {{"accepted",true}};}
  if(target=="/api/plan/floor") {load_planning_floor();return {{"accepted",true}};}
  if(target=="/api/points/remove") {remove_point(body.at("id"));return {{"accepted",true}};}
  if(target=="/api/points/rename") {rename_point(body.at("id"),body.at("name"));return store_.get("points",body.at("id"));}
  if(target=="/api/localize") {
    require(map_activation_.value("phase","")!="HOLDING"&&map_activation_.value("phase","")!="LOADING"&&map_activation_.value("phase","")!="FAILED","请先完成地图启用并核对加载结果，再提交实车初值");
    require(!trial_blocking(),"到点试验期间不能修改初始定位，请先取消并停稳");
    require(hardware_&&initial_pose_&&executor_.phase!="RUNNING"&&nav_terminal_,"请先保持停车并结束当前导航");
    require(still(),"车体尚未停稳");valid_map_pose(body.at("pose"));
    geometry_msgs::msg::PoseWithCovarianceStamped value;value.header.frame_id="map";value.header.stamp=now();
    value.pose.pose.position.x=number(body.at("pose"),"x");value.pose.pose.position.y=number(body.at("pose"),"y");
    const auto heading=number(body.at("pose"),"yaw");value.pose.pose.orientation.z=std::sin(heading/2);value.pose.pose.orientation.w=std::cos(heading/2);
    value.pose.covariance[0]=value.pose.covariance[7]=.25;value.pose.covariance[35]=.0685;
    if(map_activation_.value("phase","")!="IDLE"){
      activation_localization_pending_=true;activation_initial_stamp_=rclcpp::Time(value.header.stamp).nanoseconds();map_activation_["phase"]="WAITING_HEALTH";
      localized_=false;localization_ready_at_=-1e9;localization_fault_at_=-1e9;
    }
    manual_.stop("已提交初始定位，等待定位健康");initial_pose_->publish(value);return {{"accepted",true},{"message","已提交定位初值，仍需定位检查"}};
  }
  if(target=="/api/teach/begin") {
    require(!trial_blocking(),"请先等待导航到达并停稳，或取消本次导航");
    require(hardware_,"当前入口未启用硬件控制，可先编辑草稿和任务");
    require(executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&pending_terminal_.empty()&&nav_terminal_&&!pending_,"请先结束任务或上一条请求");
    require(!map_id_.empty(),"请先选择地图");manual_.stop("进入相机示教");depth_enabled_=false;error_.clear();
    driver_epoch_.clear();teaching_.begin(session_);const auto revision=teaching_.revision;
    hold(true,[this,revision]{if(teaching_.revision==revision&&teaching_.phase=="ACQUIRING_HOLD")teaching_.acquired(driver_epoch_,board_);});
    return {{"accepted",true},{"revision",teaching_.revision}};
  }
  if(target=="/api/teach/step") {
    require(hardware_&&teaching_.owner==session_&&!pending_,"请先开始本次调整，并等待上一条请求");
    const auto command=teaching_.step(number(body,"target_degrees"),body.at("revision").get<uint64_t>(),still(),frame_sequence_,steady());
    const auto revision=teaching_.revision;const auto expected=command;
    servo(command,[this,revision,expected]{if(teaching_.revision==revision)teaching_.sent(true,board_,board_==expected?"":"发送档位不一致",steady());});
    return {{"accepted",true},{"revision",teaching_.revision}};
  }
  if(target=="/api/teach/viewed") {
    require(teaching_.owner==session_,"此调整不属于当前会话");
    require(body.at("revision").get<uint64_t>()==teaching_.revision,"角度状态已经变化，请刷新画面");
    const auto sequence=body.at("frame_sequence").get<uint64_t>();const auto * seen=frame(sequence);
    require(frame_fresh(seen)&&seen->stamp>last_servo_stamp_,"画面已过期或来自本次调角之前");
    teaching_.viewed(sequence);return {{"accepted",true}};
  }
  if(target=="/api/teach/confirm") {
    require(!trial_blocking(),"请先等待导航和停车完成");
    if(body.contains("trial_id"))require(trial_.value("phase","")=="ARRIVED"&&trial_.value("request_id","")==body.at("trial_id")&&trial_.value("map_version","")==map_version_,"本次候选点尚未导航到达，请重新验证");
    require(teaching_.owner==session_&&!pending_,"请等待当前调整结果");
    const auto sequence=body.at("frame_sequence").get<uint64_t>();const auto * seen=frame(sequence);
    teaching_.check_confirm(body.at("revision").get<uint64_t>(),sequence,still(),frame_fresh(seen)&&seen->stamp>last_servo_stamp_,steady(),settle_);
    require(map_version_==live_map_version_&&!map_version_.empty(),"选择的地图与当前定位地图不一致");
    const auto current=pose();require(!current.is_null()&&localized_&&!localization_fault_,"定位未就绪，不能确认点位");
    const auto purpose=body.value("purpose",std::string("point"));
    require(purpose=="point"||purpose=="approach"||purpose=="navigation","确认类型无效");
    if(purpose!="navigation"&&map_info_.is_object()&&map_info_.value("planning",false))
      throw std::runtime_error("规划草图只用来摆位置。等小车在真地图上定位好了，再记下这里。");
    if(purpose=="navigation")require(board_==94,"导航参考必须恢复到板端94档，再查看新画面并确认");
    auto photo=save_photo(*seen,nonce(),false);photo["role"]="TEACHING_REFERENCE";photo["pose_estimate"]=current;
    if(body.contains("trial_id")){
      photo["navigation_request_id"]=trial_.at("request_id");photo["navigation_target"]=trial_.at("target");
      photo["navigation_result"]=trial_.value("nav2_result","UNKNOWN");photo["arrival_error"]=trial_.value("arrival_error",json(nullptr));
      photo["arrival_policy"]=trial_.value("arrival_policy","");
    }
    json result;
    if(purpose=="navigation") {
      auto old=store_.get("settings","navigation_view");
      result=store_.put("settings",{{"id","navigation_view"},{"driver_epoch",driver_epoch_},{"board_command",94},
        {"target_degrees",teaching_.target},{"reference",photo},{"human_confirmed",true}},old.is_null()?0:old.at("revision").get<int>());
      depth_enabled_=true;depth_open_stamp_=now().nanoseconds();
    } else if(purpose=="approach") {
      const auto id=body.at("point_id").get<std::string>();valid_id(id);valid_map_pose(current);
      auto point=store_.get("points",id);require(!point.is_null()&&point.at("map_version")==map_version_,"要追加的拍摄目标不存在或地图版本不同");
      auto approaches=approach_list(point);require(approaches.size()<3,"每个拍摄目标最多3个停车位");
      approaches.push_back({{"pose",current},{"target_degrees",teaching_.target},{"board_command",board_},{"verified",true},{"reference",photo}});
      point["approaches"]=approaches;
      result=store_.put("points",point,point.at("revision").get<int>());
    } else {
      const auto id=body.value("id",nonce());valid_id(id);valid_map_pose(current);
      json approach{{"pose",current},{"target_degrees",teaching_.target},{"board_command",board_},{"verified",true},{"reference",photo}};
      result=store_.put("points",{{"id",id},{"name",body.at("name")},{"map_id",map_id_},{"map_version",map_version_},
        {"approaches",json::array({approach})},
        {"angle_mapping_version","s1_270_command_v1"}},body.value("expected_revision",0));
    }
    teaching_.confirmed();store_.event("teaching_confirmed",{{"purpose",purpose},{"record",result}});return result;
  }
  if(target=="/api/points/save") {
    auto point=body.at("point");point["id"]=point.value("id",nonce());point["map_id"]=map_id_;point["map_version"]=map_version_;
    point["board_command"]=servo_command(number(point,"target_degrees"));point["verified"]=false;
    if(point.contains("field_fixed")) require(point.at("field_fixed").is_boolean(),"现场标记无效");
    if(map_info_.is_object()&&map_info_.value("planning",false)) point["field_fixed"]=false;
    if(point.contains("approaches")) for(auto & approach:point["approaches"]) approach["verified"]=false;
    point.erase("reference");point["angle_mapping_version"]="s1_270_command_v1";valid_map_pose(point.at("pose"));
    return store_.put("points",point,body.value("expected_revision",0));
  }
  if(target=="/api/teach/navigate") {
    const auto id=body.at("request_id").get<std::string>();valid_id(id);
    const auto desired=body.at("pose");valid_map_pose(desired);
    if(trial_.value("request_id","")==id) {
      require(trial_.at("target")==desired&&trial_.at("map_version")==map_version_,"同一次请求不能更换目标");return trial_;
    }
    require(!trial_blocking()&&!pending_&&pending_terminal_.empty()&&nav_terminal_&&!nav_waiting_&&executor_.phase!="RUNNING"&&executor_.phase!="PAUSED","请先结束当前导航、巡检或硬件请求");
    require(!mapping_blocks_inspection(),"请先结束建图，再下发巡检目标");
    const auto reason=trial_block_reason();require(reason.empty(),reason);
    require(still(),"请等待车体停稳后再下发新的目标");
    manual_.stop("开始单点导航试验");error_.clear();
    if(teaching_.phase!="IDLE")teaching_.fault("本次重新导航，到达后再开始调角");
    trial_={{"phase","PREPARING"},{"request_id",id},{"target",desired},{"map_id",map_id_},{"map_version",map_version_},{"reached_pose",nullptr},{"error",""},
      {"nav2_result","NOT_SENT"},{"arrival_policy","NAV2_SUCCESS_THEN_CONFIRM_ACTUAL_PARKING"}};
    trial_step_started_=false;return trial_;
  }
  if(target=="/api/teach/cancel-navigation") {
    require(trial_active(),"当前没有正在进行的到点试验");
    stop_trial("CANCELED","已请求取消，等待导航结束并确认停车");return trial_;
  }
  if(target=="/api/tasks/save") {
    auto task=body.at("task");task["id"]=task.value("id",nonce());task["map_id"]=map_id_;task["map_version"]=map_version_;
    valid_map_pose(task.at("final_parking_pose"));validate_task(task);
    for(const auto & id:task.at("point_ids")){const auto point=store_.get("points",id);require(!point.is_null()&&point.at("map_version")==map_version_,"任务包含其他地图版本或不存在的点位");}
    return store_.put("tasks",task,body.value("expected_revision",0));
  }
  if(target=="/api/manual/enable") {
    require(!trial_blocking(),"请先结束到点试验，再使用遥控");
    if(mapping_blocks_inspection()) throw std::runtime_error("建图还在进行。请先在建图页结束任务，确认车停着，再回来。");
    require(hardware_&&!pending_&&pending_terminal_.empty()&&nav_terminal_&&executor_.phase!="RUNNING"&&executor_.phase!="PAUSED","请先结束巡检或当前硬件请求");
    require(board_<0||board_==94,"行驶前相机须保持板端94档");
    require(base_health()&&topology()&&still(),"定位、轮式、雷达或速度出口未就绪");
    if(held_)hold(false,[this]{manual_.session_ready=base_health();manual_.topology_ok=topology();manual_.gateway_at=steady();manual_.scan_at=scan_at_;manual_.odom_at=odom_at_;require(manual_.enable(session_,"",steady()),manual_.reason);});
    else require(manual_.enable(session_,"",steady()),manual_.reason);
    return {{"accepted",true}};
  }
  if(target=="/api/run/start") {
    require(!trial_blocking(),"请先结束单点导航试验");
    const auto request_id=body.at("request_id").get<std::string>();valid_id(request_id);
    const auto previous=store_.get("runs",request_id);if(!previous.is_null()) {
      require(previous.at("task_id")==body.at("task_id"),"同一启动请求不能用于其他任务");return previous;
    }
    require(!(map_info_.is_object()&&map_info_.value("planning",false)),"规划草图不能用来自动巡检。请先在真地图上重新记下这些位置。");
    if(mapping_blocks_inspection()) throw std::runtime_error("建图还在进行。请先在建图页结束任务，确认车停着，再回来。");
    require(hardware_&&navigation_,"自动导航尚未显式启用；请先完成静止及导航验收");
    require(executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&pending_terminal_.empty()&&!pending_&&nav_terminal_,"已有执行或请求尚未结束");
    require(teaching_.phase=="CONFIRMED"||teaching_.phase=="IDLE","请先完成角度确认");
    require(base_health()&&topology()&&nav_->action_server_is_ready(),"地图、定位、雷达、轮式或Nav2未就绪");
    require(!frames_.empty()&&frame_fresh(&frames_.back()),"RGB相机尚未提供新鲜图像");
    require(fs::space(store_.root()).available>64ULL*1024*1024,"巡检存储空间不足");
    const auto task=store_.get("tasks",body.at("task_id"));require(!task.is_null()&&task.at("map_version")==map_version_,"任务地图版本不一致");
    json snapshot=task;snapshot["points"]=json::array();
    for(const auto & id:task.at("point_ids")) {const auto point=store_.get("points",id);require(!point.is_null()&&point.value("verified",false)&&point.at("map_version")==map_version_,"任务包含尚未试拍确认或版本不一致的点位");snapshot["points"].push_back(point);}
    run_={{"id",request_id},{"task_id",task.at("id")},{"snapshot",snapshot},{"captures",json::object()},
      {"status","RUNNING"},{"parking_status","PENDING"},{"created_stamp_ns",now().nanoseconds()},{"upload_status","PENDING_CONFIGURATION"}};
    run_revision_=0;error_.clear();manual_.stop("开始自动巡检");persist_run("RUNNING");executor_.start(snapshot);return run_;
  }
  if(target=="/api/run/pause"||target=="/api/run/cancel") {
    require(!run_.is_null()&&(executor_.phase=="RUNNING"||executor_.phase=="PAUSED"),"没有可操作的巡检任务");
    if(target=="/api/run/pause") {executor_.pause();pending_terminal_="PAUSED";}
    else {executor_.cancel();halt();pending_terminal_="CANCELED";}
    operation_at_=steady();persist_run(pending_terminal_=="PAUSED"?"PAUSING":"CANCELING");return {{"accepted",true}};
  }
  if(target=="/api/run/resume") {
    require(!run_.is_null()&&run_.at("status")=="PAUSED"&&pending_terminal_.empty()&&nav_terminal_&&!pending_&&held_&&still(),"任务尚未安全暂停");
    require(base_health()&&topology(),"继续前的定位或运动保护检查未通过");executor_.resume();persist_run("RUNNING");return {{"accepted",true}};
  }
  throw std::runtime_error("未知操作");
}

void Console::control(const json & body) {
  const auto action=body.value("action",std::string());
  if(action=="release")manual_.release(session_,body.at("sequence"));
  else if(action=="stop"){manual_.stop("网页请求停车");if(trial_active())stop_trial("CANCELED","网页请求取消到点试验并停车");}
  else if(action=="drive"&&!trial_blocking()&&executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&pending_terminal_.empty()&&!held_)
    manual_.drive(session_,body.at("sequence"),body.at("challenge"),number(body,"x"),number(body,"yaw"),"",steady());
}
void Console::persist_run(const std::string & status) {
  if(run_.is_null())return;
  run_["status"]=status;run_["point_index"]=executor_.point_index;run_["approach_index"]=executor_.approach_index;
  run_["parking"]=executor_.parking;run_["missed"]=executor_.missed;
  run_["step"]=operation_;run_["error"]=error_;run_["updated_stamp_ns"]=now().nanoseconds();
  run_=store_.put("runs",run_,run_revision_);run_revision_=run_.at("revision");
}
void Console::halt() {
  manual_.stop("执行已停止");depth_enabled_=false;++operation_generation_;
  if(nav_goal_&&!nav_terminal_&&!cancel_requested_)nav_->async_cancel_goal(nav_goal_);
  cancel_requested_=true;
  if(velocity_)velocity_->publish(geometry_msgs::msg::Twist{});
}
void Console::start_navigation(const json & target) {
  require(speed_->ready(),"导航速度档位未确认，没有下发目标；请查看速度设置并重试切档");
  require(nav_terminal_&&!nav_waiting_,"上次导航尚未结束");require(base_health()&&topology()&&perception_health(),"导航启动前健康检查失败");
  Navigate::Goal request;request.pose.header.frame_id="map";request.pose.header.stamp=now();
  request.pose.pose.position.x=number(target,"x");request.pose.pose.position.y=number(target,"y");
  const auto heading=number(target,"yaw");request.pose.pose.orientation.z=std::sin(heading/2);request.pose.pose.orientation.w=std::cos(heading/2);
  const auto generation=++nav_generation_;nav_terminal_=false;nav_waiting_=true;nav_success_=false;cancel_requested_=false;nav_velocity_at_=-1e9;nav_goal_.reset();
  nav_result_="PENDING";navigation_failure_reason_.clear();
  if(trial_active())trial_["nav2_result"]=nav_result_;
  rclcpp_action::Client<Navigate>::SendGoalOptions options;
  options.goal_response_callback=[this,generation](GoalHandle::SharedPtr handle) {
    if(generation!=nav_generation_){if(handle)nav_->async_cancel_goal(handle);return;}
    nav_waiting_=false;nav_goal_=handle;if(!handle){nav_terminal_=true;nav_success_=false;nav_result_="REJECTED";if(trial_active())trial_["nav2_result"]=nav_result_;}
    else if(cancel_requested_)nav_->async_cancel_goal(handle);
  };
  options.result_callback=[this,generation](const GoalHandle::WrappedResult & result) {
    if(generation!=nav_generation_)return;
    nav_terminal_=true;nav_success_=result.code==rclcpp_action::ResultCode::SUCCEEDED;
    nav_result_=nav_success_?"SUCCEEDED":result.code==rclcpp_action::ResultCode::ABORTED?"ABORTED":result.code==rclcpp_action::ResultCode::CANCELED?"CANCELED":"UNKNOWN";
    if(trial_active())trial_["nav2_result"]=nav_result_;
    RCLCPP_INFO(get_logger(),"APP-001 导航返回 target=/navigate_to_pose result=%s",nav_result_.c_str());
  };
  nav_goal_future_=nav_->async_send_goal(request,options);
}
BT::NodeStatus Console::step(const std::string & kind,const json & context,bool start,bool trial) {
  const bool captured=!trial&&!context.value("parking",false)&&run_.at("captures").contains(context.at("id").get<std::string>());
  if(captured&&kind!="restore")return BT::NodeStatus::SUCCESS;
  if(start) {operation_=kind;operation_stage_.clear();operation_at_=steady();++operation_generation_;if(!trial)persist_run("RUNNING");}
  if(kind!="navigate") require(steady()-operation_at_<operation_timeout_,"步骤超时："+kind);
  require(base_health()&&topology(),"巡检期间定位、传感器或速度出口失效");
  if(kind=="stop") {
    if(pending_)return BT::NodeStatus::RUNNING;
    if(!held_){hold(true);return BT::NodeStatus::RUNNING;}
    return still()&&nav_terminal_&&!nav_waiting_?BT::NodeStatus::SUCCESS:BT::NodeStatus::RUNNING;
  }
  if(kind=="restore"||kind=="adjust") {
    if(kind=="restore"&&operation_stage_=="DONE")return BT::NodeStatus::SUCCESS;
    if(pending_)return BT::NodeStatus::RUNNING;
    if(!held_){hold(true);return BT::NodeStatus::RUNNING;}
    if(!still())return BT::NodeStatus::RUNNING;
    const int target=kind=="restore"?94:context.at("board_command").get<int>();
    if(board_!=target) {
      depth_enabled_=false;
      if(steady()-last_servo_at_>=.3)servo(board_+(target>board_?1:-1),[]{});
      return BT::NodeStatus::RUNNING;
    }
    if(steady()-last_servo_at_<settle_)return BT::NodeStatus::RUNNING;
    if(kind=="restore"&&!require_depth_) {
      if(operation_stage_=="RELEASING")return BT::NodeStatus::RUNNING;
      operation_stage_="RELEASING";const auto generation=operation_generation_;
      hold(false,[this,generation]{if(generation==operation_generation_)operation_stage_="DONE";});
      return BT::NodeStatus::RUNNING;
    }
    if(kind=="adjust") {depth_enabled_=false;return !frames_.empty()&&frame_fresh(&frames_.back())&&frames_.back().stamp>last_servo_stamp_&&frames_.back().received>last_servo_at_+settle_?BT::NodeStatus::SUCCESS:BT::NodeStatus::RUNNING;}
    if(operation_stage_.empty()) {depth_enabled_=true;depth_open_stamp_=now().nanoseconds();depth_open_at_=steady();operation_stage_="WAIT_DEPTH";return BT::NodeStatus::RUNNING;}
    if(operation_stage_=="WAIT_DEPTH") {
      if(!perception_health()||perception_at_<depth_open_at_||depth_forwarded_at_<depth_open_at_||!filter_ok_||
        filter_diag_at_<depth_open_at_||obstacle_stamp_<=depth_open_stamp_||clearing_stamp_<=depth_open_stamp_)return BT::NodeStatus::RUNNING;
      require(local_clear_->service_is_ready()&&global_clear_->service_is_ready(),"导航障碍地图恢复接口尚未就绪");
      local_clear_future_=local_clear_->async_send_request(std::make_shared<Clear::Request>()).future.share();
      global_clear_future_=global_clear_->async_send_request(std::make_shared<Clear::Request>()).future.share();
      clear_stamp_=now().nanoseconds();clear_barrier_=layer_sequence_;operation_stage_="REBUILD";return BT::NodeStatus::RUNNING;
    }
    if(operation_stage_=="REBUILD") {
      if(local_clear_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready||global_clear_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready||
        local_map_stamp_<=clear_stamp_||global_map_stamp_<=clear_stamp_||!perception_health()||!filter_ok_||
        obstacle_stamp_<=clear_stamp_||clearing_stamp_<=clear_stamp_||!layer_current_[0]||!layer_current_[1]||
        layer_stamp_[0]<=clear_stamp_||layer_stamp_[1]<=clear_stamp_||layer_sequence_[0]<=clear_barrier_[0]||layer_sequence_[1]<=clear_barrier_[1])return BT::NodeStatus::RUNNING;
      local_clear_future_.get();global_clear_future_.get();operation_stage_="RELEASING";
      const auto generation=operation_generation_;hold(false,[this,generation]{if(generation==operation_generation_)operation_stage_="DONE";});
      return BT::NodeStatus::RUNNING;
    }
    return operation_stage_=="DONE"?BT::NodeStatus::SUCCESS:BT::NodeStatus::RUNNING;
  }
  if(kind=="navigate") {
    if(start) {require(!held_&&(depth_enabled_||!require_depth_),"导航姿态或停车许可尚未恢复放行");start_navigation(context.at("pose"));}
    require(perception_health(),"导航深度感知健康失效");
    if(!nav_terminal_||nav_waiting_) {
      if(steady()-operation_at_>=navigation_timeout_) {
        navigation_failure_reason_="上位机单点导航等待超过"+std::to_string(static_cast<int>(navigation_timeout_))+"秒，尚未收到成功终态；已请求取消并停车，没有进入调角或保存。请查看本次Nav2执行与轮式反馈，不是网页提交超时";
        if(trial)trial_["failure_code"]="NAVIGATION_TIMEOUT";
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    if(!nav_success_) {
      navigation_failure_reason_="导航执行层返回"+nav_result_+"，本次未确认到达；已请求停车，没有进入调角或保存。请查看本次Nav2规划／执行日志与现场障碍";
      if(trial)trial_["failure_code"]="NAV2_"+nav_result_;
      return BT::NodeStatus::FAILURE;
    }
    const auto current=pose();if(current.is_null()) {
      navigation_failure_reason_="Nav2已返回成功，但当前定位估计不可用；已请求停车，没有进入调角或保存，请检查定位与TF数据新鲜度";
      if(trial)trial_["failure_code"]="ARRIVAL_POSE_UNAVAILABLE";
      return BT::NodeStatus::FAILURE;
    }
    const auto desired=context.at("pose");
    // 单点用于示教：Nav2确认到达后，由人核对实际停车点并保存实测位姿。
    // 整轮巡检保留既有严格到点复核，不用候选目标覆盖实际位置。
    if(!trial&&(std::hypot(number(current,"x")-number(desired,"x"),number(current,"y")-number(desired,"y"))>.10||
      std::abs(std::remainder(number(current,"yaw")-number(desired,"yaw"),2*M_PI))>.15)) {
      navigation_failure_reason_="Nav2已成功，但整轮巡检到点复核超出0.10m／0.15rad；保持停车，不执行拍摄";
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
  if(kind=="capture") {
    require(held_&&still()&&!frames_.empty(),"拍照前停车或图像检查未通过");
    const auto & value=frames_.back();require(frame_fresh(&value)&&value.stamp>last_servo_stamp_&&value.received>last_servo_at_+settle_,"拍照图像不是本次姿态的新帧");
    auto photo=save_photo(value,nonce(),true);photo["point_id"]=context.at("id");photo["map_version"]=map_version_;photo["pose_estimate"]=pose();
    photo["approach_index"]=context.value("approach_index",0);
    photo["target_degrees"]=context.at("target_degrees");photo["board_command"]=board_;photo["angle_mapping_version"]=context.at("angle_mapping_version");
    run_["captures"][context.at("id").get<std::string>()]=photo;persist_run("RUNNING");return BT::NodeStatus::SUCCESS;
  }
  throw std::runtime_error("未知巡检步骤");
}

void Console::tick() {
  if(shutting_down_)return;
  std::deque<Job> jobs;{std::lock_guard<std::mutex> lock(jobs_mutex_);jobs.swap(jobs_);}
  try {
    speed_->tick(steady());
    for(auto & job:jobs){
      if(steady()<job.deadline)job.run();
      else if(job.expire)job.expire("控制循环在截止时间前没有执行");
      else RCLCPP_ERROR(get_logger(),"APP-001 请求未执行 target=%s reason=deadline",job.name.c_str());
    }
    if(pending_){auto callback=pending_;callback();}
    teaching_.stopped(still());
    if(!frames_.empty())teaching_.image(frames_.back().sequence,frame_fresh(&frames_.back())&&frames_.back().stamp>last_servo_stamp_,steady(),settle_);
    tick_trial();
    if(executor_.phase=="RUNNING") {
      const auto result=executor_.tick();
      if(result==BT::NodeStatus::SUCCESS){run_["parking_status"]="SUCCEEDED";persist_run("SUCCEEDED");}
      else if(result==BT::NodeStatus::FAILURE)throw std::runtime_error("巡检步骤失败");
    }
  }catch(const std::exception & e) {
    error_=e.what();manual_.stop(error_);depth_enabled_=false;
    if(trial_active())stop_trial("FAILED",error_);
    if(teaching_.phase!="IDLE"&&teaching_.phase!="CONFIRMED")teaching_.fault(error_);
    if(!run_.is_null()&&(executor_.phase=="RUNNING"||executor_.phase=="FAILED")) {
      alarm_color_=(operation_=="capture"||operation_=="adjust")?2:1;failure_alarm_started_=false;executor_.cancel();halt();pending_terminal_="FAILED";operation_at_=steady();
      if(executor_.parking)run_["parking_status"]="FAILED";
      try {persist_run("STOPPING");}catch(const std::exception &){/* 无法写盘也必须继续停车。 */}
    }
    RCLCPP_ERROR(get_logger(),"APP-001 %s",error_.c_str());
  }
  if(!pending_terminal_.empty()) {
    try {
      if(!pending_&&!held_)hold(true);
      if(held_&&still()&&nav_terminal_&&!nav_waiting_&&!pending_) {
        if(pending_terminal_=="FAILED"&&!failure_alarm_started_){alarm_until_=steady()+3;alarm_trigger_=true;failure_alarm_started_=true;}
        if(pending_terminal_!="FAILED"||steady()>=alarm_until_){const auto terminal=pending_terminal_;pending_terminal_.clear();persist_run(terminal);}
      }else if(steady()-operation_at_>10) {error_="取消或停稳未能确认，保持锁停";pending_terminal_.clear();persist_run("LOCKED");}
    }catch(const std::exception & e){error_=e.what();if(steady()-operation_at_>10)pending_terminal_.clear();}
  }
  manual_.session_ready=base_health()&&board_==94;manual_.topology_ok=topology();manual_.gateway_at=steady();manual_.scan_at=scan_at_;manual_.odom_at=odom_at_;
  if(hardware_&&!topology()&&!held_&&!pending_&&hold_client_->service_is_ready()) {
    manual_.stop("速度出口存在其他发布者，保持停车");
    try{hold(true);}catch(const std::exception & e){error_=e.what();}
  }
  const auto v=manual_.tick(steady());geometry_msgs::msg::Twist output;
  if(!held_&&pending_terminal_.empty()&&topology()) {
    if(((executor_.phase=="RUNNING"&&operation_=="navigate")||trial_.value("phase","")=="NAVIGATING")&&!nav_terminal_&&!nav_waiting_&&base_health()&&perception_health()&&(depth_enabled_||!require_depth_)&&steady()-nav_velocity_at_<.3&&speed_->ready()) {
      output=nav_velocity_;
      output.linear.x=std::clamp(output.linear.x,-.05,speed_->limit());
    }
    else if(!trial_blocking()&&executor_.phase!="RUNNING"&&executor_.phase!="PAUSED"&&board_==94){output.linear.x=v.x;output.angular.z=v.yaw;}
  }
  last_output_=output;
  if(velocity_)velocity_->publish(output);
  if(indicator_&&alarm_until_>0) {
    carcar_interfaces::msg::IndicatorCommand light;const bool active=steady()<alarm_until_;
    if(active&&static_cast<int>(steady()*4)%2==0){light.r=30;if(alarm_color_==2)light.g=30;}
    light.beep_trigger=active&&alarm_trigger_;light.beep_duration_ms=light.beep_trigger?3000:0;indicator_->publish(light);alarm_trigger_=false;
    if(!active)alarm_until_=0;
  }
  if(steady()-last_snapshot_>=.2){update_snapshot();last_snapshot_=steady();}
  if(server_&&!session_.empty()) {
    const auto challenge=nonce();manual_.challenge(challenge,steady());
    server_->broadcast(json{{"challenge",challenge},{"issued_steady_ms",steady()*1000},{"control_lease_ms",300},
      {"armed",manual_.armed},{"reason",manual_.reason},{"session",session_}}.dump());
  }
}
void Console::update_snapshot() {
  if((map_activation_.value("phase","")=="HOLDING"||map_activation_.value("phase","")=="LOADING")&&steady()>activation_deadline_){
    ++activation_generation_;map_activation_["phase"]="FAILED";error_="地图切换确认超时，保持停车；检查map_server实际地图后重新定位";
  }
  if(map_activation_.value("phase","")=="WAITING_HEALTH"&&base_health()&&still())map_activation_["phase"]="READY";
  const auto current=pose();const auto view=store_.get("settings","navigation_view");
  auto trial=trial_;trial["active"]=trial_active();
  json value{{"ui_version","inspection-20261021-5"},{"hardware_enabled",hardware_},{"navigation_enabled",navigation_},
    {"navigation_speed",speed_->state()},{"require_depth",require_depth_},{"map_activation",map_activation_},
    {"motion",{{"command_linear_mps",last_output_.linear.x},{"wheel_linear_mps",steady()-odom_at_<.3?json(wheel_vx_):json(nullptr)}}},
    {"navigation_debug",navigation_debug_.snapshot(tf_,steady(),now().seconds())},
    {"goal_navigation_enabled",goal_navigation_},{"navigation_trial",trial},{"goal_navigation_block_reason",trial_block_reason()},
    {"maps",maps_},{"map",map_info_},{"pose",current},{"points",store_.list("points")},{"tasks",store_.list("tasks")},{"runs",store_.list("runs")},
    {"uploads",store_.uploads()},
    {"run",run_},{"teaching",teaching_.state()},{"navigation_view",view},{"motion_held",held_},{"error",error_},
    {"health",{{"still",still()},{"odom",steady()-odom_at_<.3},{"scan",steady()-scan_at_<.5},{"localization",!activation_localization_pending_&&localized_&&!localization_fault_&&!current.is_null()&&steady()-localization_ready_at_<.5&&steady()-localization_fault_at_<.5},
      {"map_matches",!map_version_.empty()&&map_version_==live_map_version_},{"rgb",!frames_.empty()&&frame_fresh(&frames_.back())},
      {"perception",perception_health()},{"hardware_interface",hold_client_->service_is_ready()&&servo_client_->service_is_ready()},
      {"velocity_topology",topology()}}},{"frame_sequence",frame_sequence_}};
  std::lock_guard<std::mutex> lock(snapshot_mutex_);snapshot_=std::move(value);
}
}  // namespace carcar_inspection

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<carcar_inspection::Console>();
    std::weak_ptr<carcar_inspection::Console> weak=node;
    node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak]{if(auto locked=weak.lock())locked->shutdown();});
    rclcpp::spin(node);node->shutdown();node.reset();
  }catch(const std::exception & error){std::cerr<<"巡检网页启动失败："<<error.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
  if(rclcpp::ok())rclcpp::shutdown();
  return 0;
}
