#pragma once
#include "carcar_mapping_autonomy/planning_departure.hpp"

namespace carcar_mapping_autonomy {
// 精确检查圆形车体沿直线扫过的区域；不以离散采样漏过格角，也不修改地图。
inline bool free_connector(const nav_msgs::msg::OccupancyGrid &m, const Pose &a,
                           const Pose &b, double radius=.22) {
  if (!free_footprint(m,a,radius)||!free_footprint(m,b,radius)) return false;
  const double angle=grid_yaw(m),c=std::cos(angle),s=std::sin(angle),r=m.info.resolution;
  auto local=[&](const Pose&p) {
    const double x=p.x-m.info.origin.position.x,y=p.y-m.info.origin.position.y;
    return Pose{(c*x+s*y)/r,(-s*x+c*y)/r,0};
  };
  const auto from=local(a),to=local(b);
  const double dx=to.x-from.x,dy=to.y-from.y,length2=dx*dx+dy*dy,rr=radius/r;
  const int x0=std::max(0,int(std::floor(std::min(from.x,to.x)-rr))-1);
  const int y0=std::max(0,int(std::floor(std::min(from.y,to.y)-rr))-1);
  const int x1=std::min(int(m.info.width)-1,int(std::floor(std::max(from.x,to.x)+rr)));
  const int y1=std::min(int(m.info.height)-1,int(std::floor(std::max(from.y,to.y)+rr)));
  auto point_segment=[&](double x,double y) {
    const double t=length2>0?std::clamp(((x-from.x)*dx+(y-from.y)*dy)/length2,0.,1.):0;
    return std::pow(x-from.x-t*dx,2)+std::pow(y-from.y-t*dy,2);
  };
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
    const auto value=m.data[size_t(y)*m.info.width+x];
    if(value>=0&&value<50)continue;
    // 先用线段与方格的裁剪区间判断相交，再计算端点/格角的最近距离。
    double lo=0,hi=1;
    auto clip=[&](double p,double d,double minimum,double maximum) {
      if(std::abs(d)<1e-15)return p>=minimum&&p<=maximum;
      double u=(minimum-p)/d,v=(maximum-p)/d;if(u>v)std::swap(u,v);
      lo=std::max(lo,u);hi=std::min(hi,v);return lo<=hi;
    };
    if(clip(from.x,dx,x,x+1.)&&clip(from.y,dy,y,y+1.))return false;
    auto endpoint=[&](const Pose&p){return std::pow(p.x-std::clamp(p.x,double(x),x+1.),2)+
      std::pow(p.y-std::clamp(p.y,double(y),y+1.),2);};
    const double distance2=std::min({endpoint(from),endpoint(to),point_segment(x,y),
      point_segment(x+1.,y),point_segment(x,y+1.),point_segment(x+1.,y+1.)});
    if(distance2<=rr*rr+1e-12)return false;
  }
  return true;
}

// 真实车位安全但所在格心不可用时，从最多三格范围选择可连续到达的入口。
// 入口仍须在原规划掩膜内；禁止打开不安全格心、穿过未知区或绕过0.22米包络。
inline std::vector<Pose> departure_entries(const nav_msgs::msg::OccupancyGrid&m,
                                         const Pose&current,
                                         const nav_msgs::msg::OccupancyGrid&mask) {
  std::vector<Pose> result;
  if(!grid_valid(m)||mask.info!=m.info||mask.data.size()!=m.data.size()||!free_footprint(m,current))return result;
  const double angle=grid_yaw(m),dx=current.x-m.info.origin.position.x,dy=current.y-m.info.origin.position.y;
  const int cx=std::floor((std::cos(angle)*dx+std::sin(angle)*dy)/m.info.resolution);
  const int cy=std::floor((-std::sin(angle)*dx+std::cos(angle)*dy)/m.info.resolution);
  for(int y=std::max(0,cy-3);y<=std::min(int(m.info.height)-1,cy+3);++y)
    for(int x=std::max(0,cx-3);x<=std::min(int(m.info.width)-1,cx+3);++x) {
      if(mask.data[size_t(y)*m.info.width+x]!=0)continue;
      const auto p=cell_world(m,x,y);
      if(std::hypot(p.x-current.x,p.y-current.y)>std::min(.15,3.*m.info.resolution))continue;
      if(free_connector(m,current,p))result.push_back(p);
    }
  std::stable_sort(result.begin(),result.end(),[&](const Pose&a,const Pose&b){
    return std::hypot(a.x-current.x,a.y-current.y)<std::hypot(b.x-current.x,b.y-current.y);
  });
  return result;
}
}  // namespace carcar_mapping_autonomy
