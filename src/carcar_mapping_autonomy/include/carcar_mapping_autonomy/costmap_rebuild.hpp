// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace carcar_mapping_autonomy {
// 仅由已验证SLAM恢复流程显式启动；不负责返程失败时清障，也不放行运动。
// 调用方必须保持冻结/零输出，并在ready后核对当前map/mask几何、足迹及实际规划。
// 所有回调及tick应使用Node默认的互斥callback group；now为steady_clock秒数。
class CostmapRebuild {
public:
  using Clear = nav2_msgs::srv::ClearEntireCostmap;
  using Costmap = nav2_msgs::msg::Costmap;
  explicit CostmapRebuild(rclcpp::Node & node):node_(node) {
    global_clear_=node_.create_client<Clear>("/global_costmap/clear_entirely_global_costmap");
    local_clear_=node_.create_client<Clear>("/local_costmap/clear_entirely_local_costmap");
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    global_sub_=node_.create_subscription<Costmap>("/global_costmap/costmap_raw",qos,
      [this](Costmap::ConstSharedPtr m){observe(std::move(m),global_,"map");});
    local_sub_=node_.create_subscription<Costmap>("/local_costmap/costmap_raw",qos,
      [this](Costmap::ConstSharedPtr m){observe(std::move(m),local_,"odom");});
    std::string topic;node_.get_parameter_or<std::string>("filtered_scan_topic",topic,"/scan/filtered");
    scan_sub_=node_.create_subscription<sensor_msgs::msg::LaserScan>(topic,rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){observe_scan(*m);});
  }
  ~CostmapRebuild(){remove_pending();}
  bool begin(double now) {
    if(pending())return false;
    remove_pending();
    phase_="WAITING_SERVICES";error_.clear();started_at_=now;deadline_=now+5;last_tick_=now;
    cleared_at_=0;clear_ros_ns_=0;minimum_required_stamp_ns_=0;latest_scan_ns_=0;scan_received_at_=0;scan_count_=0;
    global_={};local_={};clear_done_={false,false};++generation_;
    if(!std::isfinite(now)){fail("重建时钟无效");return false;}
    RCLCPP_INFO(node_.get_logger(),"NAV-018 恢复缓存同步开始 generation=%lu；保持冻结与零输出，等待双代价地图重建",generation_);
    return true;
  }
  void tick(double now) {
    last_tick_=now;
    if(!pending())return;
    if(!std::isfinite(now)||now<started_at_){fail("重建单调时钟无效或倒退");return;}
    // 先判期限，已经迟到的future不能把超时重建改写为成功。
    if(now>deadline_){fail("双代价地图清理及重建超过5秒："+waiting_reason());return;}
    try {
      if(phase_=="WAITING_SERVICES"){
        if(!global_clear_->service_is_ready()||!local_clear_->service_is_ready())return;
        auto global=global_clear_->async_send_request(std::make_shared<Clear::Request>());
        clear_ids_[0]=global.request_id;global_future_=global.future.share();
        auto local=local_clear_->async_send_request(std::make_shared<Clear::Request>());
        clear_ids_[1]=local.request_id;local_future_=local.future.share();phase_="CLEARING";
      }
      if(phase_=="CLEARING"){
        if(!clear_done_[0]&&global_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
          if(!global_future_.get()){fail("全局代价地图清理返回空应答");return;}clear_done_[0]=true;clear_ids_[0]=-1;
        }
        if(!clear_done_[1]&&local_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
          if(!local_future_.get()){fail("局部代价地图清理返回空应答");return;}clear_done_[1]=true;clear_ids_[1]=-1;
        }
        if(clear_done_[0]&&clear_done_[1]){
          cleared_at_=now;clear_ros_ns_=node_.now().nanoseconds();phase_="REBUILDING";
          if(clear_ros_ns_<=0){fail("ROS时间未就绪，无法区分恢复前后观测");return;}
          RCLCPP_INFO(node_.get_logger(),"NAV-018 双代价地图清理应答已到 generation=%lu；等待至少1秒、3帧新激光和新full costmap",generation_);
        }
      }
      if(phase_=="REBUILDING"&&now-cleared_at_>=1&&scan_count_>=3&&scan_fresh(now)&&
         qualified(global_,now)&&qualified(local_,now)){
        phase_="READY";
        RCLCPP_INFO(node_.get_logger(),"NAV-018 恢复缓存同步完成 generation=%lu scans=%zu elapsed=%.3f；仍需外层地图配准及路径预检后才允许运动",
          generation_,scan_count_,now-started_at_);
      }
    }catch(const std::exception & e){fail(std::string("代价地图重建通信异常：")+e.what());}
  }
  // 完成重建不代表观测永久有效；外层提交前仍需持续检查新鲜度。
  bool ready()const{return phase_=="READY"&&std::isfinite(last_tick_)&&scan_fresh(last_tick_)&&
    qualified(global_,last_tick_)&&qualified(local_,last_tick_);}
  bool failed()const{return phase_=="FAILED";}
  bool pending()const{return phase_=="WAITING_SERVICES"||phase_=="CLEARING"||phase_=="REBUILDING";}
  Costmap::ConstSharedPtr global_costmap()const{return global_.message;}
  Costmap::ConstSharedPtr local_costmap()const{return local_.message;}
  nlohmann::json state()const {
    return {{"phase",phase_},{"ready",ready()},{"failed",failed()},{"error",error_},
      {"generation",generation_},{"started_at",started_at_},{"cleared_at",cleared_at_},
      {"minimum_required_stamp_ns",minimum_required_stamp_ns_},{"latest_scan_stamp_ns",latest_scan_ns_},
      {"scan_count",scan_count_},{"scan_age_s",latest_scan_ns_?nlohmann::json(last_tick_-scan_received_at_):nlohmann::json(nullptr)},
      {"global_clear_done",clear_done_[0]},{"local_clear_done",clear_done_[1]},
      {"global",describe(global_)},{"local",describe(local_)}};
  }
private:
  struct Evidence {Costmap::ConstSharedPtr message;double received_at=0;};
  static double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp){
    if(stamp.sec<0||stamp.nanosec>=1000000000)return -1;
    return int64_t(stamp.sec)*1000000000+stamp.nanosec;
  }
  bool stamp_fresh(int64_t stamp,double maximum_age)const {
    if(stamp<=0)return false;
    const double age=double(node_.now().nanoseconds()-stamp)*1e-9;
    return std::isfinite(age)&&age>=-.1&&age<=maximum_age;
  }
  static bool valid(const Costmap & m,const char*frame) {
    const auto & a=m.metadata;const auto & p=a.origin.position;const auto & q=a.origin.orientation;
    const uint64_t cells=uint64_t(a.size_x)*a.size_y;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return m.header.frame_id==frame&&a.layer=="master"&&cells>0&&cells<=12000000&&m.data.size()==cells&&
      std::isfinite(a.resolution)&&a.resolution>0&&std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
      std::isfinite(norm)&&std::abs(norm-1)<1e-3&&std::abs(q.x)<1e-6&&std::abs(q.y)<1e-6;
  }
  void observe(Costmap::ConstSharedPtr m,Evidence & evidence,const char*frame) {
    if(phase_!="REBUILDING"&&phase_!="READY")return;
    const auto stamp=stamp_ns(m->header.stamp);
    if(stamp<=clear_ros_ns_||!stamp_fresh(stamp,1)||!valid(*m,frame))return;
    // 只保存递增的完整master栅格，旧DDS缓存及迟到帧不能覆盖新证据。
    if(evidence.message&&stamp<=stamp_ns(evidence.message->header.stamp))return;
    evidence.message=std::move(m);evidence.received_at=seconds();
  }
  void observe_scan(const sensor_msgs::msg::LaserScan & m) {
    if(phase_!="REBUILDING"&&phase_!="READY")return;
    const auto stamp=stamp_ns(m.header.stamp);
    if(stamp<=clear_ros_ns_||stamp<=latest_scan_ns_||!stamp_fresh(stamp,.5)||m.header.frame_id.empty()||
       m.ranges.size()<12||m.ranges.size()>100000||!std::isfinite(m.range_min)||!std::isfinite(m.range_max)||
       m.range_min<0||m.range_max<=m.range_min||!std::isfinite(m.angle_min)||!std::isfinite(m.angle_increment)||
       std::abs(m.angle_increment)<1e-9)return;
    size_t valid_returns=0;
    for(float r:m.ranges)if(std::isfinite(r)&&r>=m.range_min&&r<=m.range_max)++valid_returns;
    if(valid_returns<12)return;
    latest_scan_ns_=stamp;scan_received_at_=seconds();++scan_count_;
    // 固定第三帧的阈值；不能要求3Hz costmap追上每一帧20Hz激光。
    if(scan_count_==3)minimum_required_stamp_ns_=stamp;
  }
  bool scan_fresh(double now)const {
    return latest_scan_ns_>0&&now-scan_received_at_>=-.1&&now-scan_received_at_<=.5&&stamp_fresh(latest_scan_ns_,.5);
  }
  bool qualified(const Evidence & e,double now)const {
    if(!e.message||minimum_required_stamp_ns_<=0||e.received_at<cleared_at_||now-e.received_at<-.1||now-e.received_at>1)return false;
    const auto stamp=stamp_ns(e.message->header.stamp);
    if(stamp<minimum_required_stamp_ns_||!stamp_fresh(stamp,1))return false;
    const auto update=stamp_ns(e.message->metadata.update_time);
    // 1.1.20的raw publisher未填update_time；0明确代表不可用，而非新的更新证据。
    return update==0||(update>=minimum_required_stamp_ns_&&stamp_fresh(update,1));
  }
  nlohmann::json describe(const Evidence & e)const {
    nlohmann::json j={{"qualified",qualified(e,last_tick_)},{"received_at",e.received_at}};
    if(!e.message){j["age_s"]=nullptr;return j;}
    const auto & m=*e.message;const auto & a=m.metadata;const auto & q=a.origin.orientation;
    const auto update=stamp_ns(a.update_time);
    j.update({{"frame_id",m.header.frame_id},{"width",a.size_x},{"height",a.size_y},{"resolution",a.resolution},
      {"origin",{a.origin.position.x,a.origin.position.y,2*std::atan2(q.z,q.w)}},{"origin_z",a.origin.position.z},
      {"stamp_ns",stamp_ns(m.header.stamp)},{"update_stamp_ns",update},{"update_time_available",update!=0},
      {"age_s",last_tick_-e.received_at}});
    return j;
  }
  std::string waiting_reason()const {
    if(phase_=="WAITING_SERVICES")return "全局／局部clear_entirely服务未齐备";
    if(phase_=="CLEARING")return "全局／局部clear_entirely应答未齐备";
    if(scan_count_<3)return "清理后不足3帧不同时间戳的有效激光";
    if(!scan_fresh(last_tick_))return "清理后的激光已陈旧";
    if(!qualified(global_,last_tick_))return "缺少清理后全局完整代价地图或其时间不合格";
    if(!qualified(local_,last_tick_))return "缺少清理后局部完整代价地图或其时间不合格";
    return "重建观察窗口未完成";
  }
  void remove_pending() {
    if(clear_ids_[0]>=0){global_clear_->remove_pending_request(clear_ids_[0]);clear_ids_[0]=-1;}
    if(clear_ids_[1]>=0){local_clear_->remove_pending_request(clear_ids_[1]);clear_ids_[1]=-1;}
  }
  void fail(const std::string & reason) {
    remove_pending();phase_="FAILED";error_="车端恢复缓存同步失败："+reason+"；保持停车和工程冻结，请检查Nav2服务、激光及代价地图。";
    RCLCPP_WARN(node_.get_logger(),"NAV-018 恢复缓存同步失败 generation=%lu reason=%s",generation_,error_.c_str());
  }
  rclcpp::Node & node_;
  std::string phase_="IDLE",error_;
  uint64_t generation_=0;size_t scan_count_=0;
  double started_at_=0,deadline_=0,last_tick_=0,cleared_at_=0,scan_received_at_=0;
  int64_t clear_ros_ns_=0,minimum_required_stamp_ns_=0,latest_scan_ns_=0;
  std::array<bool,2>clear_done_{false,false};std::array<int64_t,2>clear_ids_{-1,-1};
  Evidence global_,local_;
  rclcpp::Client<Clear>::SharedPtr global_clear_,local_clear_;
  std::shared_future<Clear::Response::SharedPtr>global_future_,local_future_;
  rclcpp::Subscription<Costmap>::SharedPtr global_sub_,local_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
}  // namespace carcar_mapping_autonomy
