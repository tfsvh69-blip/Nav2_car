#pragma once
#include <cmath>
#include <limits>
#include <nav_msgs/msg/path.hpp>
#include "carcar_mapping_autonomy/task.hpp"

namespace carcar_mapping_autonomy {
// 以有序路径弧长判断中间点；邻近的折返段、隔墙目标不能仅凭直距提前通过。
inline double handoff_path_distance(const nav_msgs::msg::Path &path,const Pose &current,const Pose &target) {
  const auto invalid=std::numeric_limits<double>::infinity();
  if(path.header.frame_id!="map"||path.poses.size()<2||path.poses.size()>20000||
      !std::isfinite(current.x)||!std::isfinite(current.y)||!std::isfinite(target.x)||!std::isfinite(target.y))return invalid;
  double arc=0,current_arc=0,current_error=invalid,target_arc=0,target_error=invalid;
  for(size_t i=1;i<path.poses.size();++i){
    const auto&a=path.poses[i-1].pose.position;const auto&b=path.poses[i].pose.position;
    if(!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(b.x)||!std::isfinite(b.y))return invalid;
    const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
    if(length<1e-10)continue;
    auto project=[&](const Pose&p){return std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(length*length),0.,1.);};
    const auto u=project(current),v=project(target);
    const double ce=std::hypot(current.x-a.x-u*dx,current.y-a.y-u*dy);
    const double te=std::hypot(target.x-a.x-v*dx,target.y-a.y-v*dy);
    // Nav2按周期重规划；只在路径开头的有限弧长内找车位，避免跨折返段跳跃。
    if(arc<=.50&&ce<current_error){current_error=ce;current_arc=arc+u*length;}
    if(te+1e-9<target_error){target_error=te;target_arc=arc+v*length;}
    arc+=length;
  }
  if(current_error>.25||target_error>.11)return invalid;
  if(target_arc<current_arc){
    if(current_arc-target_arc>.20||std::hypot(current.x-target.x,current.y-target.y)>.20)return invalid;
    return 0;
  }
  return target_arc-current_arc;
}
}
