#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>

namespace carcar_mapping_autonomy {
using Json = nlohmann::json;
struct Pose {double x=0, y=0, yaw=0;};
struct Limits {
  double radius=20.0, explore_seconds=2400.0, return_seconds=600.0;
  bool valid() const {return std::isfinite(radius)&&radius>0&&radius<=20&&
    std::isfinite(explore_seconds)&&explore_seconds>0&&explore_seconds<=2400&&
    std::isfinite(return_seconds)&&return_seconds>0&&return_seconds<=600;}
};
// 纯状态模块：所有异步取消/停稳完成由适配器明确回报，禁止以请求成功冒充停车。
class Task {
public:
  Limits limits;
  Pose origin;
  std::string phase="MANUAL", mode="manual", reason, stop_target, request_id;
  std::string end_reason,outcome,return_result="not_started",resume_phase="EXPLORING";
  bool anchored=false, save_required=false;
  uint64_t generation=0;
  double elapsed=0, return_started=0, stop_started=0;

  bool moving() const {return phase=="EXPLORING"||phase=="RETURNING"||phase=="MANUAL";}
  bool automatic() const {return phase=="EXPLORING"||phase=="RETURNING";}
  bool inside(const Pose & p) const {return !anchored || std::hypot(p.x-origin.x,p.y-origin.y)<=limits.radius;}
  void start(double now, const Pose & pose, bool ready, bool still) {
    if (!limits.valid()||!ready||!still||save_required||(phase!="MANUAL"&&phase!="PAUSED"))
      throw std::runtime_error("自动启动未执行：请先就绪、停稳并处理保存／故障；没有下发目标");
    if (!anchored) {origin=pose;anchored=true;}
    if(resume_phase=="RETURNING"){
      if(return_started==0)return_started=now;
      mode="automatic";++generation;last_tick_=now;
      if(now-return_started>=limits.return_seconds){phase="SAVE_PENDING";save_required=true;return_result="timeout";reason="返程预算已耗尽，保持停车保存";}
      else{phase="RETURNING";return_result="running";reason="恢复返程，沿用原返程预算";}
      return;
    }
    if (!inside(pose)||elapsed>=limits.explore_seconds) throw std::runtime_error("本次自动探索预算已用尽，不能通过接管或重连重置");
    mode="automatic";phase="EXPLORING";reason="自动探索运行中";last_tick_=now;++generation;
  }
  void stop(const std::string & target,const std::string & why,double now,const std::string & result="") {
    if(target=="FINISH"||target=="FAULT"){end_reason=why;outcome=result.empty()?(target=="FAULT"?"fault":"operator_finished"):result;}
    if (phase=="STOPPING") {
      if(target=="FAULT") {stop_target=target;reason=why;}
      else if(target=="PAUSED"&&stop_target!="FAULT") {
        if(stop_target=="FINISH")resume_phase="RETURNING";
        stop_target=target;reason=why;
      }
      return;
    }
    if(automatic()&&(target=="PAUSED"||target=="MANUAL"||target=="RECOVER"||target=="CHECKPOINT"))resume_phase=phase;
    stop_target=target;reason=why;phase="STOPPING";stop_started=now;++generation;
  }
  void stopped(double now,bool terminal,bool still,bool healthy) {
    if(phase!="STOPPING")return;
    if(!terminal||!still) {
      if(now-stop_started>5) {phase="FAULT";reason="取消或停稳确认超时，保持锁停；查看导航终态与轮式反馈";end_reason=reason;outcome="cancel_timeout";save_required=false;}
      return;
    }
    if(stop_target=="FINISH") {
      if(healthy&&anchored) {phase="RETURNING";if(return_started==0)return_started=now;return_result="running";reason+="；正在返回起点";}
      else {phase="SAVE_PENDING";save_required=true;}
    } else if(stop_target=="FAULT") {phase="FAULT";save_required=true;}
    else if(stop_target=="SAVE") {phase="SAVE_PENDING";save_required=true;}
    else if(stop_target=="RECOVER") {phase="RECOVERING";reason="已取消并停稳，正在验证可信检查点与当前位置";}
    else if(stop_target=="CHECKPOINT") {phase="CHECKPOINTING";reason="已停稳，生成最近可信检查点后继续原任务";}
    else if(stop_target=="MANUAL") {phase="MANUAL";mode="manual";}
    else {phase="PAUSED";}
  }
  void returned(bool success,const std::string & why,bool nearby=false) {
    phase="SAVE_PENDING";return_result=success?(nearby?"nearby":"succeeded"):"failed";
    reason=(end_reason.empty()?"":end_reason+"；")+(success?(nearby?"已在起点附近停稳，等待保存候选":"已返回起点并停稳，等待保存候选"):why);save_required=true;++generation;
  }
  void tick(double now,const Pose & pose,bool healthy,const std::string & health_reason="") {
    const double dt=std::max(0.0,now-last_tick_);last_tick_=now;
    if(phase=="EXPLORING") {
      elapsed+=dt;
      if(!healthy) stop("FAULT",health_reason.empty()?"感知／SLAM／TF失效，取消目标并冻结后续写图":health_reason,now);
      else if(!inside(pose)) stop("FINISH","达到探索半径边界",now,"radius_budget");
      else if(elapsed>=limits.explore_seconds)stop("FINISH","探索时间预算耗尽",now,"time_budget");
    } else if(phase=="RETURNING") {
      if(!healthy) stop("FAULT",health_reason.empty()?"返程期间质量失效，停车并保留候选":health_reason,now);
      else if(now-return_started>=limits.return_seconds){return_result="timeout";stop("SAVE","返程超时，原地停稳保存",now);}
    }
  }
  void save_done() {save_required=false;if(phase=="SAVE_PENDING")phase="REVIEW_REQUIRED";}
  void recovered() {
    if(phase!="RECOVERING")throw std::runtime_error("恢复结果已过期，未恢复任务");
    phase=resume_phase;reason="检查点与多帧定位验证通过，按原任务预算继续";++generation;
  }
  void checkpointed() {
    if(phase!="CHECKPOINTING")throw std::runtime_error("检查点结果已过期，未恢复任务");
    phase=resume_phase;reason="稳定检查点已保存，按原任务预算继续";++generation;
  }
  void save_stopped() {if(phase=="STOPPING"&&stop_target=="SAVE") {phase="SAVE_PENDING";save_required=true;}}
  Json status() const {return {{"phase",phase},{"control_mode",mode},{"reason",reason},
    {"end_reason",end_reason},{"outcome",outcome},{"return_result",return_result},
    {"generation",generation},{"stop_target",stop_target},{"anchored",anchored},{"origin",{origin.x,origin.y,origin.yaw}},
    {"radius_m",limits.radius},{"explore_seconds",limits.explore_seconds},{"return_seconds",limits.return_seconds},
    {"elapsed_seconds",elapsed},{"remaining_seconds",std::max(0.0,limits.explore_seconds-elapsed)},
    {"save_required",save_required},{"request_id",request_id}};}
private:
  double last_tick_=0;
};
// 正常回环期间扫描接纳会暂停。这段位移不能当成“拒绝写图”；调用方在回环事件时重置轮式基准。
// 等待时间指跨过平移接纳门槛之后的时间，不包括此前静止或原地调头。
inline bool acceptance_stalled(bool have_accept,bool loop_active,double since_loop,double since_eligible_motion,double travel){
  if(!have_accept||loop_active||!(since_loop>2.0))return false;
  return since_eligible_motion>3.0&&travel>0.15;
}
}  // namespace carcar_mapping_autonomy
