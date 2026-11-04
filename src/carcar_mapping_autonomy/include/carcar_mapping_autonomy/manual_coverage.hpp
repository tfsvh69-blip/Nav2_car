#pragma once
#include "carcar_mapping_autonomy/retreat_planner.hpp"

namespace carcar_mapping_autonomy::manual_detail {
inline void subtract_observed_triangle(std::vector<retreat_detail::Polygon>&uncovered,
                                      const retreat_detail::Point(&triangle)[3]){
  namespace d=retreat_detail;const d::Rectangle bounds{std::min({triangle[0].x,triangle[1].x,triangle[2].x}),
    std::max({triangle[0].x,triangle[1].x,triangle[2].x}),std::min({triangle[0].y,triangle[1].y,triangle[2].y}),
    std::max({triangle[0].y,triangle[1].y,triangle[2].y})};
  std::vector<d::Polygon> next;
  for(const auto&piece:uncovered){
    if(!d::overlaps(bounds,d::bounds(piece))){next.push_back(piece);continue;}
    auto remaining=piece;
    for(size_t edge=0;edge<3&&!remaining.empty();++edge){
      auto outside=d::halfplane(remaining,triangle[edge],triangle[(edge+1)%3],false);
      if(d::area(outside)>1e-12)next.push_back(std::move(outside));
      remaining=d::halfplane(remaining,triangle[edge],triangle[(edge+1)%3],true);
    }
  }
  uncovered=std::move(next);
}

// 这里只寻找不能被任何真实观测覆盖的反例点；有限点采样绝不用于放行。
inline bool point_supported(const retreat_detail::Point&p,
                            const std::vector<retreat_detail::Observation>&observations){
  for(const auto&observation:observations){
    const auto&s=*observation.scan;const double increment=s.angle_increment;
    // 非普通单圈输入交回原有精确审查，避免漏掉多圈中的等角度有效束。
    if(std::abs(increment)<=2e-7||std::abs(s.angle_min)>16*M_PI||std::abs(observation.laser.yaw)>16*M_PI||
       s.ranges.size()*std::abs(increment)>2*M_PI+2*std::abs(increment)+1e-6)return true;
    const double dx=p.x-observation.laser.x,dy=p.y-observation.laser.y,distance=std::hypot(dx,dy);
    if(distance<1e-10)return true;
    const double bearing=std::atan2(dy,dx)-observation.laser.yaw;
    const double delta=std::remainder(bearing-s.angle_min,2*M_PI),half=std::abs(increment)/2+1e-7;
    for(int turn=-1;turn<=1;++turn){
      const double index=(delta+turn*2*M_PI)/increment;
      if(index< -2||index>double(s.ranges.size())+1)continue;
      const auto nearest=static_cast<long>(std::floor(index));
      for(long i=nearest-1;i<=nearest+1;++i){
        if(i<0||i>=static_cast<long>(s.ranges.size()))continue;
        const double range=s.ranges[static_cast<size_t>(i)];
        if(!std::isfinite(range)||range<s.range_min||range>=s.range_max)continue;
        const double error=std::abs(std::remainder(bearing-(s.angle_min+i*increment),2*M_PI));
        // 使用比原内接三角形稍宽的数值容差；只会少拒绝，不会以此证明安全。
        if(error<=half+1e-10&&distance*std::cos(error)<=(range-1e-6)*std::cos(half)+1e-8)return true;
      }
    }
  }
  return false;
}

inline bool has_unobserved_witness(const std::vector<retreat_detail::Observation>&observations,
                                   const retreat_detail::Rectangle&region,
                                   const retreat_detail::Rectangle&body,
                                   const std::vector<retreat_detail::Polygon>&uncovered){
  namespace d=retreat_detail;
  for(const auto&piece:uncovered){
    if(piece.empty()||d::area(piece)<=1e-12)continue;
    d::Point center{0,0};for(const auto&p:piece){center.x+=p.x;center.y+=p.y;}
    center.x/=piece.size();center.y/=piece.size();
    if(center.x>region.left+1e-10&&center.x<region.right-1e-10&&center.y>region.bottom+1e-10&&
       center.y<region.top-1e-10&&!d::inside(center,body)&&!point_supported(center,observations))return true;
  }
  // 优先沿最新扫描的无效束找新增扫掠区内点，最多64个反例候选，计算量有界。
  const auto&latest=observations.back();const auto&s=*latest.scan;
  const d::Point origin{latest.laser.x,latest.laser.y};
  if(!d::inside(origin,body))return false;
  size_t candidates=0;
  for(size_t i=0;i<s.ranges.size()&&candidates<64;++i){
    const double range=s.ranges[i];if(std::isfinite(range)&&range>=s.range_min&&range<s.range_max)continue;
    const double angle=latest.laser.yaw+s.angle_min+i*double(s.angle_increment);
    const double inner=d::ray_exit(origin,angle,body),outer=d::ray_exit(origin,angle,region);
    if(inner<0||outer<=inner+1e-8)continue;
    ++candidates;const double distance=(inner+outer)/2;
    const d::Point point{origin.x+distance*std::cos(angle),origin.y+distance*std::sin(angle)};
    if(!d::inside(point,body)&&d::inside(point,region)&&!point_supported(point,observations))return true;
  }
  return false;
}
}  // namespace carcar_mapping_autonomy::manual_detail
