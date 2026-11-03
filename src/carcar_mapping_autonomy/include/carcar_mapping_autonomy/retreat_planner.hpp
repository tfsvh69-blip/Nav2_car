#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "carcar_mapping_autonomy/subcell_departure.hpp"

namespace carcar_mapping_autonomy {
struct RetreatOptions {
  double max_distance=.25,max_speed=.03,braking_deceleration=.20,response_seconds=.15;
  double margin=.02,front=.16,rear=.14,half_width=.13;
  double dynamic_radius=std::hypot(.14,.13)+.02;
};
struct RetreatPlan {
  bool allowed=false;
  double distance=0,speed=0,clearance=0;
  std::string reason;
  bool clears_dynamic_envelope=false;
  bool needs_retreat=false;
};
struct RetreatScan {
  // 调用者限制观测龄期，并以固定odom坐标把各帧雷达原点变换到本次当前车体系。
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  Pose laser_in_base;
};

namespace retreat_detail {
constexpr double pi=3.14159265358979323846;
struct Point {double x,y;};
struct Rectangle {double left,right,bottom,top;};
inline bool finite_pose(const Pose&p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);}
inline bool valid_options(const RetreatOptions&o){
  const double values[]={o.max_distance,o.max_speed,o.braking_deceleration,o.response_seconds,
    o.margin,o.front,o.rear,o.half_width,o.dynamic_radius};
  for(double v:values)if(!std::isfinite(v))return false;
  return o.max_distance>0&&o.max_distance<=.25&&o.max_speed>0&&o.max_speed<=.03&&
    o.braking_deceleration>0&&o.response_seconds>=0&&o.margin>0&&o.margin<=.10&&
    o.front>0&&o.front<=1&&o.rear>0&&o.rear<=1&&o.half_width>0&&o.half_width<=1&&
    o.dynamic_radius>0&&o.dynamic_radius<=1;
}
inline double wrapped(double a){return std::remainder(a,2*pi);}
inline double point_rectangle_distance(const Point&p,const Rectangle&r){
  return std::hypot(p.x-std::clamp(p.x,r.left,r.right),p.y-std::clamp(p.y,r.bottom,r.top));
}
inline bool inside(const Point&p,const Rectangle&r){
  return p.x>=r.left-1e-12&&p.x<=r.right+1e-12&&p.y>=r.bottom-1e-12&&p.y<=r.top+1e-12;
}
inline double stop_distance(double speed,const RetreatOptions&o){
  return speed*o.response_seconds+speed*speed/(2*o.braking_deceleration);
}
// 速度由当前可用于制动的距离反推；分母形式避免极短空间时两个近数相减。
inline double speed_for_stop_distance(double distance,const RetreatOptions&o){
  if(distance<=0)return 0;
  const double root=std::sqrt(o.response_seconds*o.response_seconds+2*distance/o.braking_deceleration);
  return std::min(o.max_speed,2*distance/(root+o.response_seconds));
}
struct Observation {
  const sensor_msgs::msg::LaserScan*scan=nullptr;
  Pose laser;
  std::vector<Point> endpoints;
};
inline bool observe(const sensor_msgs::msg::LaserScan&s,const Pose&laser,
                    const RetreatOptions&o,Observation&result,std::string&reason){
  if(!finite_pose(laser)||s.ranges.size()<3||s.ranges.size()>100000||
    !std::isfinite(s.angle_min)||!std::isfinite(s.angle_max)||
    !std::isfinite(s.angle_increment)||std::abs(s.angle_increment)<1e-8||
    std::abs(s.angle_increment)>.10||!std::isfinite(s.range_min)||
    !std::isfinite(s.range_max)||s.range_min<0||s.range_max<=s.range_min||
    std::abs((s.angle_min+(s.ranges.size()-1)*double(s.angle_increment))-s.angle_max)>
      1.01*std::abs(s.angle_increment)){
    reason="原始激光或安装变换无效，不能核验后退扫掠区域";return false;
  }
  // 本车雷达位于车身上方；若安装原点在模型之外，应先修正外廓/安装事实。
  if(laser.x< -o.rear||laser.x>o.front||std::abs(laser.y)>o.half_width){
    reason="雷达原点不在已声明车身范围内，后向视线模型不适用";return false;
  }
  result.scan=&s;result.laser=laser;result.endpoints.reserve(s.ranges.size());
  for(size_t i=0;i<s.ranges.size();++i){
    const double range=s.ranges[i];
    if(!std::isfinite(range)||range<s.range_min||range>=s.range_max)continue;
    const double a=laser.yaw+s.angle_min+i*double(s.angle_increment);
    result.endpoints.push_back({laser.x+range*std::cos(a),laser.y+range*std::sin(a)});
  }
  return true;
}
// 求射线穿过矩形的最远距离；新占用区位于原雷达原点后方，不要求扫描穿透车身。
inline double ray_exit(const Point&origin,double angle,const Rectangle&r){
  const double dx=std::cos(angle),dy=std::sin(angle);
  double lo=0,hi=std::numeric_limits<double>::infinity();
  auto clip=[&](double p,double d,double minimum,double maximum){
    if(std::abs(d)<1e-14)return p>=minimum-1e-12&&p<=maximum+1e-12;
    double a=(minimum-p)/d,b=(maximum-p)/d;if(a>b)std::swap(a,b);
    lo=std::max(lo,a);hi=std::min(hi,b);return lo<=hi+1e-12;
  };
  if(!clip(origin.x,dx,r.left,r.right)||!clip(origin.y,dy,r.bottom,r.top)||hi<0)return -1;
  return hi;
}
using Polygon=std::vector<Point>;
inline double cross(const Point&a,const Point&b,const Point&p){return (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);}
inline double area(const Polygon&p){double a=0;if(p.size()<3)return 0;for(size_t i=0;i<p.size();++i){const auto&q=p[(i+1)%p.size()];a+=p[i].x*q.y-p[i].y*q.x;}return std::abs(a)/2;}
inline Rectangle bounds(const Polygon&p){
  Rectangle r{p.front().x,p.front().x,p.front().y,p.front().y};
  for(const auto&v:p){r.left=std::min(r.left,v.x);r.right=std::max(r.right,v.x);r.bottom=std::min(r.bottom,v.y);r.top=std::max(r.top,v.y);}
  return r;
}
inline bool overlaps(const Rectangle&a,const Rectangle&b){return a.left<=b.right&&a.right>=b.left&&a.bottom<=b.top&&a.top>=b.bottom;}
inline Polygon halfplane(const Polygon&p,const Point&a,const Point&b,bool keep_left){
  Polygon result;if(p.empty())return result;
  for(size_t i=0;i<p.size();++i){const auto&from=p[i];const auto&to=p[(i+1)%p.size()];
    const double u=cross(a,b,from)*(keep_left?1:-1),v=cross(a,b,to)*(keep_left?1:-1);
    const bool inside_u=u>=0,inside_v=v>=0;
    if(inside_u)result.push_back(from);
    if(inside_u!=inside_v){const double t=u/(u-v);result.push_back({from.x+t*(to.x-from.x),from.y+t*(to.y-from.y)});}
  }
  return result;
}
inline bool covered(const std::vector<Observation>&observations,double distance,const RetreatOptions&o,std::string&reason){
  if(distance<=1e-12)return true;
  const Rectangle rear{-o.rear-o.margin-distance,-o.rear-o.margin,-o.half_width-o.margin,o.half_width+o.margin};
  const Polygon rectangle{{rear.left,rear.bottom},{rear.right,rear.bottom},{rear.right,rear.top},{rear.left,rear.top}};
  std::vector<Polygon> uncovered{rectangle};
  for(const auto&observation:observations){
    const Point origin{observation.laser.x,observation.laser.y};const auto&s=*observation.scan;
    // float角度首尾闭合和共边误差只允许1e-7弧度容差，本车范围内小于亚微米。
    const double half=std::abs(s.angle_increment)/2+1e-7;double farthest=0;
    for(const auto&p:rectangle)farthest=std::max(farthest,std::hypot(p.x-origin.x,p.y-origin.y));
    // 每束的半角空隙必须被几何余量覆盖；缺测束根本不贡献观测区域。
    if(farthest*std::sin(half)>o.margin/2)continue;
    for(size_t i=0;i<s.ranges.size();++i){
      const double range=s.ranges[i];if(!std::isfinite(range)||range<s.range_min||range>=s.range_max)continue;
      const double a=observation.laser.yaw+s.angle_min+i*double(s.angle_increment);
      // 有限回波以内才是已观测空间；扇区用内接三角形，不外推至回波之外。
      const double radius=std::min(range-1e-6,farthest+o.margin);
      if(radius<=0)continue;
      const Point triangle[]={origin,{origin.x+radius*std::cos(a-half),origin.y+radius*std::sin(a-half)},
        {origin.x+radius*std::cos(a+half),origin.y+radius*std::sin(a+half)}};
      const Rectangle triangle_bounds{std::min({triangle[0].x,triangle[1].x,triangle[2].x}),
        std::max({triangle[0].x,triangle[1].x,triangle[2].x}),
        std::min({triangle[0].y,triangle[1].y,triangle[2].y}),
        std::max({triangle[0].y,triangle[1].y,triangle[2].y})};
      if(!overlaps(triangle_bounds,rear))continue;
      // 完整矩形减去每个有效束的凸三角形，剩余多边形就是仍未获证实的区域。
      std::vector<Polygon> next;
      for(const auto&piece:uncovered){
        if(!overlaps(triangle_bounds,bounds(piece))){next.push_back(piece);continue;}
        Polygon remaining=piece;
        for(size_t edge=0;edge<3&&!remaining.empty();++edge){
          auto outside=halfplane(remaining,triangle[edge],triangle[(edge+1)%3],false);
          if(area(outside)>1e-12)next.push_back(std::move(outside));
          remaining=halfplane(remaining,triangle[edge],triangle[(edge+1)%3],true);
        }
      }
      uncovered=std::move(next);
      if(uncovered.empty())return true;
      // 对极端噪声/相互交叉扇区限制工作量；拒绝不能完成的审查而非阻塞控制循环。
      if(uncovered.size()>4096){reason="后向多帧视线过于碎片化，无法完成有界覆盖审查";return false;}
    }
  }
  reason="后向原始激光联合观测仍有NaN、缺束或视场空隙，不能确认完整后退扫掠区";return false;
}
inline Pose end_pose(const Pose&p,double distance){return {p.x-distance*std::cos(p.yaw),p.y-distance*std::sin(p.yaw),p.yaw};}
inline bool check(const nav_msgs::msg::OccupancyGrid&m,const std::vector<Observation>&observations,const Pose&p,
                  double distance,double speed,const RetreatOptions&o,std::string&reason,double*clearance=nullptr){
  const double total=distance+stop_distance(speed,o);
  if(!std::isfinite(distance)||!std::isfinite(speed)||distance<0||speed<0||speed>o.max_speed+1e-12||
    !std::isfinite(total)||total>o.max_distance+1e-10){reason="后退距离、速度或制动预算超出本轮上限";return false;}
  if(!free_connector(m,p,end_pose(p,total),.22)){reason="后退整段不满足原图0.22米净空，未知区域不得绕过";return false;}
  const Rectangle swept{-o.rear-o.margin-total,o.front+o.margin,-o.half_width-o.margin,o.half_width+o.margin};
  const Rectangle actual{-o.rear-total,o.front,-o.half_width,o.half_width};
  double minimum=std::numeric_limits<double>::infinity();
  for(const auto&observation:observations)for(const auto&point:observation.endpoints){
    if(inside(point,swept)){reason="原始激光回波进入实际矩形及余量的连续后退扫掠区";return false;}
    minimum=std::min(minimum,point_rectangle_distance(point,actual));
  }
  if(!covered(observations,total,o,reason))return false;
  if(clearance)*clearance=std::isfinite(minimum)?minimum:0;
  reason.clear();return true;
}
inline nav_msgs::msg::OccupancyGrid endpoint_grid(const nav_msgs::msg::OccupancyGrid&m,
                                                 const std::vector<Observation>&observations,const Pose&p){
  auto result=m;result.data.assign(m.data.size(),0);
  const double c=std::cos(p.yaw),s=std::sin(p.yaw),a=grid_yaw(m),gc=std::cos(a),gs=std::sin(a);
  for(const auto&observation:observations)for(const auto&point:observation.endpoints){
    // 与正式动态层的3米标障范围一致；更远端点不会用于解释当前近车交叠。
    if(std::hypot(point.x-observation.laser.x,point.y-observation.laser.y)>3.)continue;
    const double x=p.x+c*point.x-s*point.y-m.info.origin.position.x;
    const double y=p.y+s*point.x+c*point.y-m.info.origin.position.y;
    const double gx=(gc*x+gs*y)/m.info.resolution,gy=(-gs*x+gc*y)/m.info.resolution;
    if(!std::isfinite(gx)||!std::isfinite(gy)||gx<0||gy<0||gx>=m.info.width||gy>=m.info.height)continue;
    const int ix=std::floor(gx),iy=std::floor(gy);
    result.data[size_t(iy)*m.info.width+ix]=100;
  }
  return result;
}
inline bool observe_many(const std::vector<RetreatScan>&scans,const RetreatOptions&o,
                         std::vector<Observation>&result,std::string&reason){
  if(scans.empty()||scans.size()>8){reason="后退原始激光观测为空或超出8帧上限";return false;}
  result.reserve(scans.size());
  for(const auto&frame:scans){
    Observation observation;if(!frame.scan){reason="后退原始激光帧缺失";return false;}
    if(!observe(*frame.scan,frame.laser_in_base,o,observation,reason))return false;
    result.push_back(std::move(observation));
  }
  size_t endpoints=0;for(const auto&frame:result)endpoints+=frame.endpoints.size();
  if(endpoints==0){reason="原始激光没有有效有限回波，不能判断当前车身或后退空间";return false;}
  return true;
}
// 单帧包装只在本次同步几何调用期间借用原消息；不缓存引用、不启动异步任务。
inline std::vector<RetreatScan> single_frame(const sensor_msgs::msg::LaserScan&s,const Pose&laser){
  return {{sensor_msgs::msg::LaserScan::ConstSharedPtr(&s,[](const sensor_msgs::msg::LaserScan*){}),laser}};
}
}  // namespace retreat_detail

// 仅检查后退；距离和速度均为正的幅值，不在本模块发布运动命令。
inline bool retreat_safe(const nav_msgs::msg::OccupancyGrid&static_map,
  const std::vector<RetreatScan>&scans,const Pose&map_pose,
  double distance,double speed,const RetreatOptions&options,std::string*reason=nullptr){
  std::string detail;std::vector<retreat_detail::Observation> observations;
  bool safe=retreat_detail::valid_options(options)&&grid_valid(static_map)&&retreat_detail::finite_pose(map_pose);
  if(!safe)detail="静态地图、车位或后退参数无效";
  if(safe)safe=retreat_detail::observe_many(scans,options,observations,detail);
  if(safe)safe=retreat_detail::check(static_map,observations,map_pose,distance,speed,options,detail);
  if(reason)*reason=detail;
  return safe;
}

inline RetreatPlan plan_retreat(const nav_msgs::msg::OccupancyGrid&static_map,
  const std::vector<RetreatScan>&scans,const Pose&map_pose,
  const RetreatOptions&options={}){
  using namespace retreat_detail;RetreatPlan result;std::vector<Observation> observations;
  if(!valid_options(options)||!grid_valid(static_map)||!finite_pose(map_pose)){
    result.reason="静态地图、车位或后退参数无效";return result;
  }
  if(!observe_many(scans,options,observations,result.reason))return result;
  if(!check(static_map,observations,map_pose,0,0,options,result.reason))return result;
  const auto marked=endpoint_grid(static_map,observations,map_pose);
  if(free_footprint(marked,map_pose,options.dynamic_radius)){
    result.clears_dynamic_envelope=true;
    result.reason="当前原始回波没有动态圆交叠，不执行无依据后退；保持停车检查历史障碍图";return result;
  }
  result.needs_retreat=true;
  // 连一微米的新增后向区域也无法获证时，直接拒绝；不是几厘米的运动下限。
  if(!check(static_map,observations,map_pose,std::min(1e-6,options.max_distance),0,options,result.reason))return result;
  double low=0,high=options.max_distance;std::string blocked;
  if(check(static_map,observations,map_pose,high,0,options,blocked))low=high;
  else for(int i=0;i<30;++i){const double mid=(low+high)/2;
    if(check(static_map,observations,map_pose,mid,0,options,blocked))low=mid;else high=mid;}
  const double available=low;
  if(available<=1e-7){result.reason=blocked.empty()?"后方没有可可靠执行的观测与制动空间":blocked;return result;}
  // 先找最短可脱离当前动态格的位置；路径安全始终由连续扫掠审查负责。
  double target=0,previous=0;
  for(int i=1;i<=512;++i){const double d=available*i/512;
    if(free_footprint(marked,end_pose(map_pose,d),options.dynamic_radius)){
      double lo=previous,hi=d;for(int j=0;j<24;++j){const double mid=(lo+hi)/2;
        if(free_footprint(marked,end_pose(map_pose,mid),options.dynamic_radius))hi=mid;else lo=mid;}
      target=hi+std::min(1e-4,(available-hi)/4);break;
    }
    previous=d;
  }
  if(target>0&&target<available){
    result.distance=target;result.speed=speed_for_stop_distance((available-target)*.9,options);
  }else{
    // 后方空间不足以完全脱离时，保留安全短退能力，不设置几厘米的固定最小距离。
    result.speed=speed_for_stop_distance(available*.25,options);
    result.distance=available-stop_distance(result.speed,options)-std::min(1e-5,available*.01);
  }
  if(result.distance<=1e-7||result.speed<=1e-6){
    result.distance=result.speed=0;result.reason="剩余空间不足以形成可靠的后退与制动动作";return result;
  }
  if(!check(static_map,observations,map_pose,result.distance,result.speed,options,result.reason,&result.clearance))return result;
  result.allowed=true;
  result.clears_dynamic_envelope=free_footprint(marked,end_pose(map_pose,result.distance),options.dynamic_radius);
  result.reason=result.clears_dynamic_envelope?"原始激光和原图连续扫掠通过，可低速短退至当前动态格之外":
    "仅有较短安全后退空间；短退后须重新停车审查，不能保证已脱离动态格";
  return result;
}
inline bool retreat_safe(const nav_msgs::msg::OccupancyGrid&static_map,
  const sensor_msgs::msg::LaserScan&scan,const Pose&map_pose,const Pose&laser_in_base,
  double distance,double speed,const RetreatOptions&options,std::string*reason=nullptr){
  return retreat_safe(static_map,retreat_detail::single_frame(scan,laser_in_base),map_pose,distance,speed,options,reason);
}
inline RetreatPlan plan_retreat(const nav_msgs::msg::OccupancyGrid&static_map,
  const sensor_msgs::msg::LaserScan&scan,const Pose&map_pose,const Pose&laser_in_base,
  const RetreatOptions&options={}){
  return plan_retreat(static_map,retreat_detail::single_frame(scan,laser_in_base),map_pose,options);
}
}  // namespace carcar_mapping_autonomy
