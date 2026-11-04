#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include "carcar_mapping_autonomy/recovery_core.hpp"

namespace carcar_mapping_autonomy {
// 激光至车体的变换由调用者按扫描时刻取得；纯几何入口不查询最新TF或修改地图。
inline std::vector<ReferenceMatcher::Ray> reference_scan_rays(
    const sensor_msgs::msg::LaserScan&scan,const tf2::Transform&laser_in_base,double mapping_range) {
  const auto origin=laser_in_base*tf2::Vector3(0,0,0);
  std::vector<ReferenceMatcher::Ray> rays;
  std::vector<size_t> valid;valid.reserve(scan.ranges.size());
  // 与已修复的回环复核一致：先筛有效原始束，再限量，短量程空地不再被整圈抽样削减证据。
  for(size_t i=0;i<scan.ranges.size();++i){
    const double range=scan.ranges[i];
    if(!std::isfinite(range)||range<std::max(.15,double(scan.range_min))||
       range>std::min(mapping_range,double(scan.range_max))-.05)continue;
    valid.push_back(i);
  }
  const size_t count=std::min<size_t>(180,valid.size());rays.reserve(count);
  for(size_t sample=0;sample<count;++sample){
    // 保留原角序与首尾，不重复取样；有效点不足50时仍由原门槛拒绝。
    const size_t selected=count==1?0:sample*(valid.size()-1)/(count-1);
    const size_t i=valid[selected];const double range=scan.ranges[i];
    const double angle=scan.angle_min+i*double(scan.angle_increment);
    const auto point=laser_in_base*tf2::Vector3(range*std::cos(angle),range*std::sin(angle),0);
    rays.push_back({point.x(),point.y(),origin.x(),origin.y()});
  }
  return rays;
}
} // namespace carcar_mapping_autonomy
