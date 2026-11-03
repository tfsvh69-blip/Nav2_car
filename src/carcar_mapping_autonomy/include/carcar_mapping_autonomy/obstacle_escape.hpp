#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <nav2_msgs/msg/costmap.hpp>
#include "carcar_mapping_autonomy/backup_motion.hpp"
#include "carcar_mapping_autonomy/retreat_planner.hpp"

namespace carcar_mapping_autonomy {
// 自动避障只协调成熟动作和证据；速度出口仍由监督器唯一持有。
class ObstacleEscape {
public:
  struct Context {
    double now=0,scan_age=0,odom_age=0;
    Pose pose,wheel,laser,origin;
    double radius=20;
    bool healthy=false,still=false,navigation_terminal=false,tf_ready=false;
    bool automatic=false,operator_hold=false,write_hold=false,loop_wait=false;
    uint64_t loop_generation=0;
    std::string task_id,request_id;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    std::vector<RetreatScan> scans;
  };
  struct Update {bool resumed=false,failed=false,fault=false;std::string reason;};
  explicit ObstacleEscape(rclcpp::Node&node):node_(node) {
    retreat_enabled_=node.declare_parameter("obstacle_escape_enabled",false);
    enabled_=node.declare_parameter("obstacle_wait_enabled",false)||retreat_enabled_;
    wait_=node.declare_parameter("obstacle_escape_wait_seconds",.8);
    limit_=node.declare_parameter("obstacle_escape_max_seconds",45.);
    max_attempts_=node.declare_parameter("obstacle_escape_max_attempts",6);
    options_.max_distance=node.declare_parameter("obstacle_escape_max_distance_m",.25);
    options_.max_speed=node.declare_parameter("obstacle_escape_speed_mps",.03);
    total_limit_=node.declare_parameter("obstacle_escape_max_total_distance_m",.75);
    if(!std::isfinite(wait_)||wait_<.1||wait_>5||!std::isfinite(limit_)||limit_<.5||limit_>45||
       max_attempts_<1||max_attempts_>6||!std::isfinite(total_limit_)||total_limit_<=0||total_limit_>.75||
       !std::isfinite(options_.max_distance)||options_.max_distance<=0||options_.max_distance>.25||
       !std::isfinite(options_.max_speed)||options_.max_speed<=0||options_.max_speed>.03)
      throw std::runtime_error("自动避障参数越界，未开放运动");
    if(!enabled_)return;
    backup_=std::make_unique<BackupMotion>(node);
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    global_=node.create_subscription<nav2_msgs::msg::Costmap>("/global_costmap/costmap_raw",qos,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr m){observe(*m,"map",global_at_,global_stamp_);});
    local_=node.create_subscription<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",qos,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr m){observe(*m,"odom",local_at_,local_stamp_);});
  }
  bool enabled()const{return enabled_;}
  bool active()const{return active_;}
  // 对外提供监督器交接屏障：真实终态后还须严格停稳；内部取消流程仍使用协议terminal。
  bool terminal()const{return !backup_||backup_->settled();}
  bool moving()const{return active_&&phase_=="BACKING"&&backup_&&backup_->active()&&!backup_->terminal()&&
    backup_->status().value("phase",std::string())=="MOVING";}
  double speed()const{return selected_.speed;}
  // 只有正常导航已实际推进，才重置同一受阻地点的脱困预算。
  void normal_progress(const Pose&wheel){
    if(episode_&&!active_&&std::hypot(wheel.x-normal_anchor_.x,wheel.y-normal_anchor_.y)>=.25){
      episode_=false;attempts_=0;total_distance_=0;rechecked_=false;rechecks_=0;
    }
  }
  bool trigger(const Context&c,const std::string&reason) {
    if(!enabled_||!c.automatic||c.operator_hold)return false;
    if(active_)return true;
    if(!episode_){episode_=true;started_=c.now;attempts_=0;total_distance_=0;rechecked_=false;rechecks_=0;attempt_unaccounted_=false;}
    active_=true;phase_="STOPPING";reason_="当前车位受阻，正在自动停稳复核；尚未执行后退";
    trigger_reason_=reason;waiting_since_=c.now;last_scan_ns_=c.scan?rclcpp::Time(c.scan->header.stamp).nanoseconds():0;
    task_id_=c.task_id;request_id_=c.request_id;
    start_loop_=c.loop_generation;
    return true;
  }
  void cancel(const std::string&reason) {
    if(!enabled_)return;
    if(backup_&&!backup_->terminal())backup_->cancel(reason);
    if(active_){phase_="CANCELLED";reason_=reason;}
    active_=false;episode_=false;
  }
  Update tick(const Context&c) {
    Update out;
    if(!enabled_)return out;
    last_wheel_=c.wheel;
    backup_->tick(c.now,c.wheel,c.odom_age<=.5,c.still);
    if(backup_->status().value("phase",std::string())=="FAULT"&&!backup_->terminal()) {
      out.fault=true;out.reason="自动后退取消／终态确认超时，保持锁停；"+backup_->status().value("reason",std::string());return out;
    }
    if(!active_)return out;
    if(!c.automatic||c.operator_hold){cancel("操作者抢占自动避障，等待动作取消及停稳");return out;}
    if(c.now-started_>limit_){return fail("自动避障已达到总时间上限，持续没有通过全部检查的安全路线");}
    if(!c.healthy||c.write_hold||c.loop_wait||!c.tf_ready||c.loop_generation!=start_loop_) {
      if(!backup_->terminal())backup_->cancel("健康／坐标条件变化，取消后退并保持零输出");
      phase_="STOPPING";reason_="等待定位、观测及地图稳定，未放行后退";
      start_loop_=c.loop_generation;waiting_since_=c.now;return out;
    }
    if(!costmaps_fresh(c.now)){
      if(!backup_->terminal())backup_->cancel("局部／全局完整代价图过期，保持停车");
      phase_="STOPPING";reason_="局部／全局完整代价图未就绪或过期，等待新观测";return out;
    }
    if(phase_=="STOPPING") {
      if(!backup_->terminal()||!c.navigation_terminal||!c.still)return out;
      if(attempt_unaccounted_){total_distance_+=std::max(0.,backup_->distance_progress());attempt_unaccounted_=false;}
      phase_="WAITING";waiting_since_=c.now;
      last_scan_ns_=c.scan?rclcpp::Time(c.scan->header.stamp).nanoseconds():0;
      reason_="已取消原导航并停稳，等待新扫描后自动复核";return out;
    }
    if(phase_=="BACKING"||phase_=="VERIFYING") {
      if(backup_->failed()){
        phase_="STOPPING";reason_="后退动作未完成，取消并停稳后自动复核其他安全动作；"+
          backup_->status().value("reason",std::string());return out;
      }
      if(backup_->succeeded()) {
        total_distance_+=backup_->distance_progress();
        attempt_unaccounted_=false;
        return resume("短距离后退及轮式停稳核验通过，自动重新规划；原起点与预算保持");
      }
      if(backup_->status().value("phase",std::string())=="VERIFYING")phase_="VERIFYING";
      if(backup_->terminal()){phase_="VERIFYING";reason_="后退动作已结束，正在核对轮式进度与连续停稳";return out;}
      const double remaining=std::max(0.,selected_.distance-backup_->distance_progress());
      auto options=options_;options.response_seconds+=std::max(0.,c.scan_age);
      std::string why;
      if(!c.map||c.scans.empty()||!retreat_safe(*c.map,c.scans,c.pose,remaining,selected_.speed,options,&why)){
        backup_->cancel("后退扫掠或制动空间发生变化："+why);
        phase_="VERIFYING";reason_="后退安全条件变化，已取消并保持零输出；"+why;return out;
      }
      if(phase_=="VERIFYING")return out;
      reason_="安全后退中，持续核对激光、轮式进度和制动空间";return out;
    }
    if(phase_!="WAITING")return out;
    if(c.now-waiting_since_<wait_||!c.still||!c.navigation_terminal||!c.map||!c.scan||
       rclcpp::Time(c.scan->header.stamp).nanoseconds()<=last_scan_ns_)return out;
    // 首次只等新观测并重新规划；瞬时经过的人员无需触发机械脱困。
    if(!rechecked_){rechecked_=true;++rechecks_;return resume("新扫描已到，先自动重新规划确认障碍是否已离开");}
    if(!retreat_enabled_){
      if(rechecks_>=6)return fail("新扫描复核和重新规划已达到6次上限；自动短退尚未现场验收，保持暂停");
      ++rechecks_;return resume("新扫描已到，自动重新规划；短退未启用，不发送倒车动作");
    }
    if(attempts_>=max_attempts_||total_distance_>=total_limit_)
      return fail("低速脱困次数或累计后退距离已用尽，持续没有可执行的安全路线");
    auto options=options_;options.max_distance=std::min(options.max_distance,total_limit_-total_distance_);
    options.response_seconds+=std::max(0.,c.scan_age);
    selected_=plan_retreat(*c.map,c.scans,c.pose,options);
    if(!selected_.allowed) {
      reason_="自动复核中："+selected_.reason;
      std::string why;
      if(!selected_.needs_retreat&&
         retreat_safe(*c.map,c.scans,c.pose,0,0,options,&why)){
        // 不通过清除栅格制造通路；等待Nav2正常射线更新后仍须实际规划通过。
        return resume("当前原始激光足迹安全，自动重新预检导航；未清障或执行后退");
      }
      waiting_since_=c.now;last_scan_ns_=rclcpp::Time(c.scan->header.stamp).nanoseconds();return out;
    }
    const Pose end{c.pose.x-selected_.distance*std::cos(c.pose.yaw),
      c.pose.y-selected_.distance*std::sin(c.pose.yaw),c.pose.yaw};
    if(std::hypot(end.x-c.origin.x,end.y-c.origin.y)>c.radius){
      reason_="后退会越过本轮探索半径，等待其他安全路线";waiting_since_=c.now;return out;
    }
    if(!backup_->available()){reason_="等待Nav2 /backup动作服务就绪；保持零输出";return out;}
    const double allowance=std::min(30.,std::max(3.,selected_.distance/selected_.speed+3.));
    if(backup_->start(selected_.distance,selected_.speed,allowance,c.wheel,c.now,
        c.task_id+"-obstacle-escape-"+std::to_string(++sequence_))){
      ++attempts_;phase_="BACKING";reason_="后方完整扫掠和制动检查通过，开始有界低速后退";
      attempt_unaccounted_=true;
      RCLCPP_INFO(node_.get_logger(),"自动避障 task_id=%s request_id=%s attempt=%d distance=%.4fm speed=%.4fm/s；%s",
        task_id_.c_str(),request_id_.c_str(),attempts_,selected_.distance,selected_.speed,selected_.reason.c_str());
    }
    return out;
  }
  Json status()const {
    return {{"enabled",enabled_},{"retreat_enabled",retreat_enabled_},{"rechecks",rechecks_},{"active",active_},{"phase",phase_},{"reason",reason_},{"task_id",task_id_},{"request_id",request_id_},
      {"attempts",attempts_},{"max_attempts",max_attempts_},{"distance_m",selected_.distance},
      {"speed_mps",selected_.speed},{"progress_m",backup_?backup_->distance_progress():0.},
      {"total_distance_m",total_distance_},{"max_total_distance_m",total_limit_},
      {"max_seconds",limit_},{"remaining_seconds",episode_?std::max(0.,limit_-(steady()-started_)):limit_},{"trigger_reason",trigger_reason_},{"motion",backup_?backup_->status():Json::object()}};
  }
private:
  Update fail(const std::string&reason) {
    if(backup_&&!backup_->terminal())backup_->cancel(reason);
    active_=false;phase_="FAILED";reason_=reason;return {false,true,false,reason};
  }
  Update resume(const std::string&reason) {
    active_=false;phase_="REPLANNING";reason_=reason;normal_anchor_=last_wheel_;return {true,false,false,reason};
  }
  static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  bool stamp_fresh(int64_t stamp)const {
    const double age=double(node_.now().nanoseconds()-stamp)*1e-9;
    return stamp>0&&std::isfinite(age)&&age>=-.1&&age<=1.;
  }
  void observe(const nav2_msgs::msg::Costmap&m,const std::string&frame,double&at,int64_t&stamp) {
    const uint64_t cells=uint64_t(m.metadata.size_x)*m.metadata.size_y;
    const auto ns=rclcpp::Time(m.header.stamp).nanoseconds();
    if(m.header.frame_id!=frame||m.metadata.layer!="master"||cells==0||cells>12000000||m.data.size()!=cells||
       !std::isfinite(m.metadata.resolution)||m.metadata.resolution<=0||!stamp_fresh(ns)||ns<=stamp)return;
    stamp=ns;at=steady();
  }
  bool costmaps_fresh(double now)const{return now-global_at_<=1&&now-local_at_<=1&&stamp_fresh(global_stamp_)&&stamp_fresh(local_stamp_);}
  rclcpp::Node&node_;
  bool enabled_=false,retreat_enabled_=false,active_=false,episode_=false,rechecked_=false,attempt_unaccounted_=false;
  std::string phase_="IDLE",reason_,trigger_reason_,task_id_,request_id_;
  int max_attempts_=6,attempts_=0,rechecks_=0;
  double wait_=.8,limit_=45,total_limit_=.75,started_=0,waiting_since_=0,total_distance_=0;
  double global_at_=-1e9,local_at_=-1e9;int64_t global_stamp_=0,local_stamp_=0,last_scan_ns_=0;
  uint64_t sequence_=0,start_loop_=0;
  RetreatOptions options_;RetreatPlan selected_;
  Pose last_wheel_,normal_anchor_;
  std::unique_ptr<BackupMotion>backup_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_,local_;
};
}  // namespace carcar_mapping_autonomy
