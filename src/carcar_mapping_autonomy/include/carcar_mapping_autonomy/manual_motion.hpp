#pragma once
#include <chrono>
#include "carcar_mapping_autonomy/retreat_planner.hpp"
#include "carcar_mapping_autonomy/manual_coverage.hpp"

namespace carcar_mapping_autonomy {
struct ManualMotionCheck {bool allowed=false;std::string reason;};
// 受限遥控只使用经过轮式对齐的近期原始激光，不读取或解冻全局地图。
// 雷达未观测区域不当作自由；复用短退候选的射线扇区和凸多边形覆盖方法。
inline ManualMotionCheck manual_motion_check(const std::vector<RetreatScan>&scans,double vx,double wz) {
  namespace d=retreat_detail;
  const auto started=std::chrono::steady_clock::now();
  if(!std::isfinite(vx)||!std::isfinite(wz)||std::abs(vx)>.05+1e-9||std::abs(wz)>.10+1e-9)
    return {false,"受限接管速度超过0.05m/s或0.10rad/s"};
  if(std::abs(vx)+std::abs(wz)<1e-9)return {true,{}};
  if(scans.size()>5)return {false,"方向观测数量超出审查预算"};
  if(scans.empty())return {false,"没有可用于方向扫掠检查的原始激光"};
  RetreatOptions options;std::vector<d::Observation> observations;
  for(const auto&sample:scans){
    if(!sample.scan||sample.scan->ranges.size()>4096)continue;
    d::Observation observation;std::string reason;
    if(d::observe(*sample.scan,sample.laser_in_base,options,observation,reason))observations.push_back(std::move(observation));
  }
  if(observations.empty())return {false,"原始激光或轮式对齐变换无效"};
  // 覆盖0.45秒扫描龄期、0.30秒命令失效、0.10秒余量和制动；不缩减既有车身外廓。
  const double duration=.85+std::max(std::abs(vx)/.20,std::abs(wz)/.80);
  d::Rectangle region{1e9,-1e9,1e9,-1e9};
  for(int step=0;step<=20;++step){
    const double t=duration*step/20,angle=wz*t;
    const double x=std::abs(wz)<1e-9?vx*t:vx*std::sin(angle)/wz;
    const double y=std::abs(wz)<1e-9?0:vx*(1-std::cos(angle))/wz;
    for(double px:{-options.rear-options.margin,options.front+options.margin})
      for(double py:{-options.half_width-options.margin,options.half_width+options.margin}){
        const double sx=x+std::cos(angle)*px-std::sin(angle)*py;
        const double sy=y+std::sin(angle)*px+std::cos(angle)*py;
        region.left=std::min(region.left,sx);region.right=std::max(region.right,sx);
        region.bottom=std::min(region.bottom,sy);region.top=std::max(region.top,sy);
      }
  }
  // 再包住相邻积分步之间的运动，不用离散采样点代替连续扫掠。
  const double discretization=std::abs(wz)<1e-9?0:(std::abs(vx)+.25*std::abs(wz))*duration/20;
  region.left-=discretization;region.right+=discretization;region.bottom-=discretization;region.top+=discretization;
  for(const auto&observation:observations)for(const auto&point:observation.endpoints)
    if(d::inside(point,region))return {false,"原始激光进入所选方向的车身扫掠与制动范围"};
  const d::Polygon rectangle{{region.left,region.bottom},{region.right,region.bottom},{region.right,region.top},{region.left,region.top}};
  // 当前车身已占据的范围不要求雷达穿透；所有回波仍须通过上面的整段碰撞检查。
  // 视线证明只针对新增扫掠区，缺少后方束不会封死向前离开的安全方向。
  const d::Point body[]={{-options.rear-options.margin,-options.half_width-options.margin},
    {options.front+options.margin,-options.half_width-options.margin},
    {options.front+options.margin,options.half_width+options.margin},
    {-options.rear-options.margin,options.half_width+options.margin}};
  std::vector<d::Polygon> uncovered;auto remaining=rectangle;
  for(size_t edge=0;edge<4&&!remaining.empty();++edge){
    auto outside=d::halfplane(remaining,body[edge],body[(edge+1)%4],false);
    if(d::area(outside)>1e-12)uncovered.push_back(std::move(outside));
    remaining=d::halfplane(remaining,body[edge],body[(edge+1)%4],true);
  }
  if(uncovered.empty())return {true,{}};
  // 先以真实盲区反例结束不可能完成的覆盖证明；此捷径只拒绝，不产生新放行路径。
  const d::Rectangle current_body{-options.rear-options.margin,options.front+options.margin,
    -options.half_width-options.margin,options.half_width+options.margin};
  if(manual_detail::has_unobserved_witness(observations,region,current_body,uncovered))
    return {false,"所选方向新增扫掠区存在原始激光缺束或盲区，保持停车"};
  // 同一原点的连续有效束可先用保守内接宽三角形覆盖，减少逐束裁剪产生的碎片。
  // 宽三角形半径取所有原三角形最小值，角跨度小于π/3；它严格位于原观测并集内。
  for(const auto&observation:observations){
    const auto&s=*observation.scan;const d::Point origin{observation.laser.x,observation.laser.y};
    // 非常规大角度交回逐束审查，避免浮点精度破坏合并所需的连续角序。
    if(std::abs(s.angle_min)>16*M_PI||std::abs(observation.laser.yaw)>16*M_PI)continue;
    const double half=std::abs(s.angle_increment)/2+1e-7;double farthest=0;
    for(const auto&p:rectangle)farthest=std::max(farthest,std::hypot(p.x-origin.x,p.y-origin.y));
    if(farthest*std::sin(half)>options.margin/2)continue;
    for(size_t i=0;i<s.ranges.size();){
      if(std::chrono::steady_clock::now()-started>std::chrono::milliseconds(8))
        return {false,"方向扫掠检查超过8毫秒预算，保持停车"};
      size_t end=i;double radius=std::numeric_limits<double>::infinity();
      for(;end<s.ranges.size()&&end-i<32&&(end-i)*std::abs(s.angle_increment)+2*half<M_PI/3;++end){
        const double range=s.ranges[end];if(!std::isfinite(range)||range<s.range_min||range>=s.range_max)break;
        radius=std::min(radius,std::min(range-1e-6,farthest+options.margin));
      }
      if(end-i>=2&&radius>1e-8){
        const double first=observation.laser.yaw+s.angle_min+i*double(s.angle_increment);
        const double last=observation.laser.yaw+s.angle_min+(end-1)*double(s.angle_increment);
        const double left=std::min(first,last)-half,right=std::max(first,last)+half;
        radius-=1e-8;const d::Point triangle[]={origin,
          {origin.x+radius*std::cos(left),origin.y+radius*std::sin(left)},
          {origin.x+radius*std::cos(right),origin.y+radius*std::sin(right)}};
        manual_detail::subtract_observed_triangle(uncovered,triangle);
        if(uncovered.empty())return {true,{}};
        if(uncovered.size()>1024)return {false,"方向观测过于碎片化，保持停车等待更完整激光"};
      }
      i=end>i?end:i+1;
    }
  }
  for(const auto&observation:observations){
    const auto&s=*observation.scan;const d::Point origin{observation.laser.x,observation.laser.y};
    const double half=std::abs(s.angle_increment)/2+1e-7;double farthest=0;
    for(const auto&p:rectangle)farthest=std::max(farthest,std::hypot(p.x-origin.x,p.y-origin.y));
    if(farthest*std::sin(half)>options.margin/2)continue;
    bool complete=s.ranges.size()*std::abs(s.angle_increment)>=2*M_PI-2e-7&&d::inside(origin,region);
    for(size_t i=0;complete&&i<s.ranges.size();++i){
      const double range=s.ranges[i],a=observation.laser.yaw+s.angle_min+i*double(s.angle_increment);
      double required=std::max(d::ray_exit(origin,a-half,region),d::ray_exit(origin,a+half,region));
      for(const auto&p:rectangle)if(std::abs(std::remainder(std::atan2(p.y-origin.y,p.x-origin.x)-a,2*M_PI))<=half)
        required=std::max(required,std::hypot(p.x-origin.x,p.y-origin.y));
      complete=std::isfinite(range)&&range>=s.range_min&&range<s.range_max&&(range-1e-6)*std::cos(half)>=required;
    }
    if(complete)return {true,{}};
    for(size_t i=0;i<s.ranges.size();++i){
      if(i%32==0&&std::chrono::steady_clock::now()-started>std::chrono::milliseconds(8))
        return {false,"方向扫掠检查超过8毫秒预算，保持停车"};
      const double range=s.ranges[i];if(!std::isfinite(range)||range<s.range_min||range>=s.range_max)continue;
      const double a=observation.laser.yaw+s.angle_min+i*double(s.angle_increment);
      const double radius=std::min(range-1e-6,farthest+options.margin);if(radius<=0)continue;
      const d::Point triangle[]={origin,{origin.x+radius*std::cos(a-half),origin.y+radius*std::sin(a-half)},
        {origin.x+radius*std::cos(a+half),origin.y+radius*std::sin(a+half)}};
      const d::Rectangle triangle_bounds{std::min({triangle[0].x,triangle[1].x,triangle[2].x}),
        std::max({triangle[0].x,triangle[1].x,triangle[2].x}),
        std::min({triangle[0].y,triangle[1].y,triangle[2].y}),
        std::max({triangle[0].y,triangle[1].y,triangle[2].y})};
      if(!d::overlaps(triangle_bounds,region))continue;
      manual_detail::subtract_observed_triangle(uncovered,triangle);if(uncovered.empty())return {true,{}};
      if(uncovered.size()>1024)return {false,"方向观测过于碎片化，保持停车等待更完整激光"};
    }
  }
  return {false,"所选方向的扫掠区仍有激光缺束或盲区，保持停车"};
}
}  // namespace carcar_mapping_autonomy
