#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <limits>
#include <utility>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "carcar_mapping_autonomy/task.hpp"
namespace carcar_mapping_autonomy {
inline bool same_grid(const nav_msgs::msg::OccupancyGrid & a,const nav_msgs::msg::OccupancyGrid & b) {
  return a.header.frame_id==b.header.frame_id&&a.info.width==b.info.width&&a.info.height==b.info.height&&
    a.info.resolution==b.info.resolution&&a.info.origin==b.info.origin&&a.data==b.data;
}
inline bool grid_valid(const nav_msgs::msg::OccupancyGrid & m) {
  const auto & q=m.info.origin.orientation;
  return m.header.frame_id=="map"&&m.info.width&&m.info.height&&m.info.resolution>0&&
    std::isfinite(m.info.resolution)&&std::isfinite(m.info.origin.position.x)&&std::isfinite(m.info.origin.position.y)&&
    std::isfinite(q.w)&&std::isfinite(q.z)&&std::abs(q.x)<1e-6&&std::abs(q.y)<1e-6&&
    std::abs(q.w*q.w+q.z*q.z-1)<1e-3&&
    static_cast<uint64_t>(m.info.width)*m.info.height==m.data.size()&&m.data.size()<=100000000;
}
inline double grid_yaw(const nav_msgs::msg::OccupancyGrid & m) {return 2*std::atan2(m.info.origin.orientation.z,m.info.origin.orientation.w);}
inline Pose cell_world(const nav_msgs::msg::OccupancyGrid & m,int x,int y) {
  const auto a=grid_yaw(m),gx=(x+.5)*m.info.resolution,gy=(y+.5)*m.info.resolution;
  return {m.info.origin.position.x+std::cos(a)*gx-std::sin(a)*gy,
    m.info.origin.position.y+std::sin(a)*gx+std::cos(a)*gy,0};
}
// 圆包络覆盖当前配置矩形所有航向；未知/图外一律不是安全停靠点。
struct FootprintEvidence {bool invalid=false,outside=false;size_t unknown=0,occupied=0;};
inline bool free_footprint(const nav_msgs::msg::OccupancyGrid & m,const Pose & p,double radius=.22,FootprintEvidence*evidence=nullptr) {
  if(evidence)*evidence={};
  if(!grid_valid(m)||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(radius)||radius<0){if(evidence)evidence->invalid=true;return false;}
  const auto outside=[&]{if(evidence)evidence->outside=true;return false;};
  const auto a=grid_yaw(m),dx=p.x-m.info.origin.position.x,dy=p.y-m.info.origin.position.y;
  const auto gx=(std::cos(a)*dx+std::sin(a)*dy)/m.info.resolution;
  const auto gy=(-std::sin(a)*dx+std::cos(a)*dy)/m.info.resolution;
  if(!std::isfinite(gx)||!std::isfinite(gy)||gx<0||gy<0||gx>=m.info.width||gy>=m.info.height)return outside();
  const double cells=radius/m.info.resolution;
  if(!std::isfinite(cells)||cells>std::min(m.info.width,m.info.height))return outside();
  if(gx-cells<=0||gy-cells<=0||gx+cells>=m.info.width||gy+cells>=m.info.height)return outside();
  // 闭区间相交也要检查负方向相切的前一格；floor(min)会在整数边界漏掉该格。
  const int x0=std::max(0,static_cast<int>(std::ceil(gx-cells))-1),x1=static_cast<int>(std::floor(gx+cells));
  const int y0=std::max(0,static_cast<int>(std::ceil(gy-cells))-1),y1=static_cast<int>(std::floor(gy+cells));
  // 在栅格局部坐标计算圆到每个方格的最近点；保留真实亚格位置，不用格心距离近似。
  // 即使地图旋转，该坐标变换仍保留圆形包络；未知、障碍及图外一律拒绝相交。
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
    const auto v=m.data[static_cast<size_t>(y)*m.info.width+x];if(v>=0&&v<50)continue;
    const double nx=std::clamp(gx,static_cast<double>(x),static_cast<double>(x+1));
    const double ny=std::clamp(gy,static_cast<double>(y),static_cast<double>(y+1));
    const double ex=gx-nx,ey=gy-ny;
    if(ex*ex+ey*ey<=cells*cells+1e-12){
      if(!evidence)return false;
      if(v<0)++evidence->unknown;else ++evidence->occupied;
    }
  }
  return !evidence||(evidence->unknown==0&&evidence->occupied==0);
}
// 保留已验收的整格对角余量，给跟踪转弯留出空间；基础审查仍为0.22米。
inline bool planning_footprint(const nav_msgs::msg::OccupancyGrid&m,const Pose&p){
  return free_footprint(m,p,.22+std::sqrt(2.0)*m.info.resolution);
}
// 二维栅格射线逐格遍历（DDA）：观测点必须真的能看到前沿，不能隔墙投影到另一侧。
// 仅终点所在未知格（含SLAM裁剪边界外一格）可未知；中途必须是已知自由区。
inline bool observation_visible(const nav_msgs::msg::OccupancyGrid&m,const Pose&from,const Pose&to){
  if(!grid_valid(m)||!std::isfinite(from.x)||!std::isfinite(from.y)||!std::isfinite(to.x)||!std::isfinite(to.y))return false;
  const double a=grid_yaw(m),c=std::cos(a),s=std::sin(a),r=m.info.resolution;
  auto local=[&](const Pose&p){const double dx=p.x-m.info.origin.position.x,dy=p.y-m.info.origin.position.y;return std::pair<double,double>{(c*dx+s*dy)/r,(-s*dx+c*dy)/r};};
  const auto p=local(from),q=local(to);const int w=m.info.width,h=m.info.height;
  if(!std::isfinite(p.first)||!std::isfinite(p.second)||!std::isfinite(q.first)||!std::isfinite(q.second)||
     p.first<0||p.second<0||p.first>=w||p.second>=h||q.first< -1||q.second< -1||q.first>=w+1.||q.second>=h+1.)return false;
  int x=std::floor(p.first),y=std::floor(p.second);const int tx=std::floor(q.first),ty=std::floor(q.second);
  auto clear=[&](int cx,int cy){const bool inside=cx>=0&&cy>=0&&cx<w&&cy<h;
    if(cx==tx&&cy==ty)return !inside||m.data[cy*w+cx]<50;
    return inside&&m.data[cy*w+cx]>=0&&m.data[cy*w+cx]<50;};
  const double dx=q.first-p.first,dy=q.second-p.second,inf=std::numeric_limits<double>::infinity();
  const int sx=(dx>0)-(dx<0),sy=(dy>0)-(dy<0);
  double next_x=sx?((sx>0?x+1.:x)-p.first)/dx:inf,next_y=sy?((sy>0?y+1.:y)-p.second)/dy:inf;
  const double step_x=sx?1./std::abs(dx):inf,step_y=sy?1./std::abs(dy):inf;
  for(int steps=0;steps<20000;++steps){
    if(!clear(x,y))return false;
    if(x==tx&&y==ty)return true;
    if(next_x+1e-12<next_y){x+=sx;next_x+=step_x;}
    else if(next_y+1e-12<next_x){y+=sy;next_y+=step_y;}
    else{if(!clear(x+sx,y)||!clear(x,y+sy))return false;x+=sx;y+=sy;next_x+=step_x;next_y+=step_y;}
  }
  return false;
}
struct ObservationCounts {size_t clearance=0,connected=0,visible=0,progress=0;};
inline std::optional<Pose> safe_observation(const nav_msgs::msg::OccupancyGrid & m,const Pose & target,const Task & t,
                                          const std::vector<uint8_t>*reachable=nullptr,
                                          const Pose*current=nullptr,ObservationCounts*counts=nullptr) {
  if(!grid_valid(m)||!std::isfinite(target.x)||!std::isfinite(target.y)||
     (reachable&&reachable->size()!=m.data.size()))return {};
  const auto a=grid_yaw(m),dx=target.x-m.info.origin.position.x,dy=target.y-m.info.origin.position.y;
  const auto gx=(std::cos(a)*dx+std::sin(a)*dy)/m.info.resolution;
  const auto gy=(-std::sin(a)*dx+std::cos(a)*dy)/m.info.resolution;
  if(!std::isfinite(gx)||!std::isfinite(gy))return {};
  // 候选只允许落在前沿0.8米内。按这个半径开窗，避免每次目标都扫描整张增长中的地图并堵住速度循环。
  const int reach=static_cast<int>(std::min(std::ceil(.8/m.info.resolution),static_cast<double>(std::max(m.info.width,m.info.height))));
  if(gx< -reach||gy< -reach||gx>=m.info.width+static_cast<double>(reach)||gy>=m.info.height+static_cast<double>(reach))return {};
  const int cx=static_cast<int>(std::floor(gx)),cy=static_cast<int>(std::floor(gy));
  const int x0=std::max(0,cx-reach),y0=std::max(0,cy-reach);
  const int x1=std::min(static_cast<int>(m.info.width)-1,cx+reach),y1=std::min(static_cast<int>(m.info.height)-1,cy+reach);
  std::optional<Pose> best;double distance=.8;
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
    if(m.data[static_cast<size_t>(y)*m.info.width+x]!=0)continue;
    auto p=cell_world(m,x,y);const double d=std::hypot(p.x-target.x,p.y-target.y);
    if(d>=.8||!t.inside(p)||!planning_footprint(m,p))continue;
    if(counts)++counts->clearance;
    if(reachable&&!(*reachable)[static_cast<size_t>(y)*m.info.width+x])continue;
    if(counts)++counts->connected;
    if(!observation_visible(m,p,target))continue;
    if(counts)++counts->visible;
    // 将到点距离要求纳入候选枚举，不能选中已到过的最近点后直接放弃其余候选。
    if(current&&std::hypot(p.x-current->x,p.y-current->y)<=.25)continue;
    if(counts)++counts->progress;
    if(d<distance){distance=d;best=p;}
  }
  return best;
}
inline std::optional<nav_msgs::msg::OccupancyGrid> planning_keepout_mask(const nav_msgs::msg::OccupancyGrid&m){
  // Humble KeepoutFilter不支持旋转网格；拒绝而非错误套用坐标。
  if(!grid_valid(m)||std::abs(grid_yaw(m))>1e-6)return {};
  auto mask=m;mask.data.assign(m.data.size(),100);
  for(unsigned y=0;y<m.info.height;++y)for(unsigned x=0;x<m.info.width;++x){
    if(m.data[y*m.info.width+x]<0||m.data[y*m.info.width+x]>=50)continue;
    if(planning_footprint(m,cell_world(m,x,y)))mask.data[y*m.info.width+x]=0;
  }
  return mask;
}
}  // namespace carcar_mapping_autonomy
