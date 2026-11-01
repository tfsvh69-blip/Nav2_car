#pragma once
#include "carcar_mapping_autonomy/recovery_core.hpp"

namespace carcar_mapping_autonomy {
// 六帧AMCL及三帧SLAM之后，仅管理提交前新扫描的资格；不改变任何几何门槛。
class RecoveryCommitGuard {
public:
  static constexpr double wait_limit_seconds=5.0;
  bool ready=false;
  int64_t scan_stamp_ns=0;
  ReferenceScore score;
  MultiFrameVerification observations;
  void reset(double now){*this={};waiting_since_=now;}
  void invalidate(double now){
    if(ready||waiting_since_<0)waiting_since_=now;
    ready=false;observations.reset();
  }
  bool observe(const Pose&pose,int64_t stamp,double now,const ReferenceScore&next){
    if(stamp<=scan_stamp_ns)return ready;
    scan_stamp_ns=stamp;score=next;
    const bool was_ready=ready;
    ready=observations.observe(pose,stamp,now,next.acceptable(),3,.2);
    if(ready)waiting_since_=-1;
    else if(was_ready||waiting_since_<0)waiting_since_=now;
    return ready;
  }
  bool expired(double now)const{return !ready&&waiting_since_>=0&&now-waiting_since_>wait_limit_seconds;}
  double waiting_seconds(double now)const{return waiting_since_<0?0:std::max(0.,now-waiting_since_);}
  static bool agrees(const Pose&candidate,const Pose&verified){
    return std::hypot(candidate.x-verified.x,candidate.y-verified.y)<.12&&
      std::abs(std::remainder(candidate.yaw-verified.yaw,2*M_PI))<.10;
  }
private:
  double waiting_since_=-1;
};
}  // namespace carcar_mapping_autonomy
