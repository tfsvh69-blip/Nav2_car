#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace carcar_camera {
// 时间来自 steady_clock；云时间戳仅用于检查采样时间持续推进。
class RequiredStvlHealth {
public:
  RequiredStvlHealth(double input_timeout=0.5, double layer_timeout=0.75,
    double stable_time=3.0)
  : input_timeout_(input_timeout), layer_timeout_(layer_timeout), stable_time_(stable_time) {
    for (auto v : {input_timeout, layer_timeout, stable_time}) {
      if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("STVL 健康超时无效");}
    }
  }
  void camera(bool healthy, double now) {
    camera_ok_=healthy; camera_time_=now;
    if (!healthy) {close();}
  }
  void cloud(int64_t stamp, double stamp_age, double now) {
    if (stamp<=0 || !std::isfinite(stamp_age) || stamp_age<0 || stamp_age>input_timeout_) {
      cloud_ok_=false; close(); return;
    }
    if (stamp<=cloud_stamp_) {return;}  // 重复旧消息不能延长租约。
    cloud_stamp_=stamp; cloud_time_=now; cloud_ok_=true;
  }
  void layer(size_t index, bool current, uint64_t sequence, double now) {
    auto & l=layers_.at(index);
    if (l.seen && sequence==l.sequence) {
      if (!current) {l.current=false; close();}
      return;
    }
    if (l.seen && sequence<l.sequence) {close(); l.updates=0;}
    l.seen=true; l.sequence=sequence; l.time=now; l.current=current; ++l.updates;
    if (!current) {close();}
  }
  bool allowed(double now) {
    reason_=failure(now);
    if (!reason_.empty()) {close(); return false;}
    if (!std::isfinite(healthy_since_)) {
      healthy_since_=now;
      for (size_t i=0;i<2;++i) {barrier_[i]=layers_[i].updates;}
    }
    if (now<healthy_since_ || now-healthy_since_<stable_time_ ||
      layers_[0].updates-barrier_[0]<2 || layers_[1].updates-barrier_[1]<2) {
      reason_="RECOVERING"; return false;
    }
    reason_="HEALTHY"; return true;
  }
  const std::string & reason() const {return reason_;}
private:
  struct Layer {bool seen=false, current=false; uint64_t sequence=0, updates=0;
    double time=-INFINITY;};
  static bool fresh(double t, double now, double timeout) {
    return std::isfinite(t) && now>=t && now-t<=timeout;
  }
  std::string failure(double now) const {
    if (!camera_ok_ || !fresh(camera_time_,now,input_timeout_)) {return "CAMERA_UNHEALTHY";}
    if (!cloud_ok_ || !fresh(cloud_time_,now,input_timeout_)) {return "CLOUD_STAMP_NOT_ADVANCING";}
    for (size_t i=0;i<2;++i) {
      if (!layers_[i].current || !fresh(layers_[i].time,now,layer_timeout_)) {
        return i==0 ? "LOCAL_STVL_UNHEALTHY" : "GLOBAL_STVL_UNHEALTHY";
      }
    }
    return {};
  }
  void close() {healthy_since_=INFINITY;}
  double input_timeout_, layer_timeout_, stable_time_;
  bool camera_ok_=false, cloud_ok_=false;
  double camera_time_=-INFINITY, cloud_time_=-INFINITY, healthy_since_=INFINITY;
  int64_t cloud_stamp_=0;
  std::array<Layer,2> layers_{};
  std::array<uint64_t,2> barrier_{};
  std::string reason_="STARTUP_STOP";
};
}  // namespace carcar_camera
