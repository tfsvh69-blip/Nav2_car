#pragma once
#include <unordered_set>
#include "carcar_mapping_autonomy/map_geometry.hpp"
namespace carcar_mapping_autonomy {
// 复用既有探索候选的逐格射线与10厘米世界网格去重；仅用于收益估计，不提供运动许可。
inline std::unordered_set<uint64_t> visible_unknown_cells(const nav_msgs::msg::OccupancyGrid & map,const Pose&point,double unknown_depth=.75){
  std::unordered_set<uint64_t> unknown;
  if(!grid_valid(map)||std::abs(grid_yaw(map))>1e-6||!std::isfinite(unknown_depth)||unknown_depth<=0||!std::isfinite(point.x)||!std::isfinite(point.y))return unknown;
  constexpr double inf=std::numeric_limits<double>::infinity();
  // 按栅格边界逐格行进，不能用0.1米跳采样跨过仅0.05米厚的已知墙。
  const double resolution=map.info.resolution,ox=map.info.origin.position.x,oy=map.info.origin.position.y;
  auto occupied=[&](int x,int y){return x>=0&&y>=0&&x<int(map.info.width)&&y<int(map.info.height)&&map.data[size_t(y)*map.info.width+x]>=50;};
  // 3米处一度约5厘米，避免稀疏射线平移后产生大量虚假的“新增”采样格。
  for(int i=0;i<360;++i){const double angle=2*M_PI*i/360,dx=std::cos(angle),dy=std::sin(angle);
    int x=int(std::floor((point.x-ox)/resolution)),y=int(std::floor((point.y-oy)/resolution));
    const int sx=dx>=0?1:-1,sy=dy>=0?1:-1;
    const double step_x=std::abs(dx)>1e-12?resolution/std::abs(dx):inf,step_y=std::abs(dy)>1e-12?resolution/std::abs(dy):inf;
    double next_x=std::abs(dx)>1e-12?(ox+(x+(sx>0))*resolution-point.x)/dx:inf;
    double first_unknown=inf;
    double next_y=std::abs(dy)>1e-12?(oy+(y+(sy>0))*resolution-point.y)/dy:inf,range=0;
    for(size_t hops=0;hops<2000&&range<=3.;++hops){
      if(occupied(x,y)||range>first_unknown+unknown_depth)break;
      if(x<0||y<0||x>=int(map.info.width)||y>=int(map.info.height)||map.data[size_t(y)*map.info.width+x]<0){
        // 未知区内尚不知道下一道墙；只奖励近端探测深度，避免把整条未知射线当作必得收益。
        if(!std::isfinite(first_unknown))first_unknown=range;
        const auto gx=uint32_t(int32_t(std::floor((ox+(x+.5)*resolution)/.1)));
        const auto gy=uint32_t(int32_t(std::floor((oy+(y+.5)*resolution)/.1)));
        unknown.insert((uint64_t(gx)<<32)|gy);
      }
      range=std::min(next_x,next_y);
      if(std::abs(next_x-next_y)<1e-10){
        if(occupied(x+sx,y)||occupied(x,y+sy))break;
        x+=sx;y+=sy;next_x+=step_x;next_y+=step_y;
      }else if(next_x<next_y){x+=sx;next_x+=step_x;}else{y+=sy;next_y+=step_y;}
    }
  }
  return unknown;
}
// 硬冷却区域由执行客户端的 GoalMemory 导出；visited 采样只用于软重复代价。
inline bool observation_blocked_by_regions(const Json&regions,const Pose&point,double now){
  if(!regions.is_array()||regions.size()>64||!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(now))return true;
  for(const auto&region:regions){
    if(!region.is_object()||!region.contains("center")||!region["center"].is_array()||
       region["center"].size()!=2||!region["center"][0].is_number()||!region["center"][1].is_number()||
       !region.contains("radius")||!region["radius"].is_number()||!region.contains("until")||!region["until"].is_number())return true;
    const double x=region["center"][0],y=region["center"][1],radius=region["radius"],until=region["until"];
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)||radius<=0||!std::isfinite(until))return true;
    if(now<until&&std::hypot(point.x-x,point.y-y)<=radius)return true;
  }
  return false;
}
// 仅估计排名中的车体转向代价；不平滑或替换用于硬净空检验的原始路径。
// 与受测 Shim 的0.30米前视一致，避免把NavFn格心吸附和5厘米阶梯当作原地转身。
inline double path_turn_estimate(const std::vector<Pose>&path,const Pose&start,size_t end_index,double window=.30){
  const double inf=std::numeric_limits<double>::infinity();
  if(path.empty()||end_index>=path.size()||!std::isfinite(start.x)||!std::isfinite(start.y)||
     !std::isfinite(start.yaw)||!std::isfinite(window)||window<=0)return inf;
  std::vector<Pose> points{start};std::vector<double> arcs{0.};
  for(size_t i=0;i<=end_index;++i){
    if(!std::isfinite(path[i].x)||!std::isfinite(path[i].y))return inf;
    const double distance=std::hypot(path[i].x-points.back().x,path[i].y-points.back().y);
    if(!std::isfinite(distance)||!std::isfinite(arcs.back()+distance))return inf;
    if(distance>1e-9){points.push_back(path[i]);arcs.push_back(arcs.back()+distance);}
  }
  const double length=arcs.back();if(length<=1e-6)return 0.;
  const double width=std::min(window,length);
  auto at=[&](double distance){
    const size_t index=std::clamp<size_t>(std::lower_bound(arcs.begin(),arcs.end(),distance)-arcs.begin(),1,arcs.size()-1);
    const double fraction=(distance-arcs[index-1])/(arcs[index]-arcs[index-1]);
    return Pose{points[index-1].x+(points[index].x-points[index-1].x)*fraction,
      points[index-1].y+(points[index].y-points[index-1].y)*fraction,0};
  };
  double turn=0,direction=start.yaw;
  const double count=std::ceil(length/(window*.5));
  if(!std::isfinite(count)||count>200000)return inf;
  const size_t samples=std::max<size_t>(1,count);
  for(size_t i=0;i<=samples;++i){
    const double progress=length*double(i)/samples;
    const double low=std::clamp(progress-width*.5,0.,length-width);
    const auto a=at(low),b=at(low+width);
    if(std::hypot(b.x-a.x,b.y-a.y)<=1e-6)continue;
    const double angle=std::atan2(b.y-a.y,b.x-a.x);
    turn+=std::abs(std::remainder(angle-direction,2*M_PI));direction=angle;
  }
  return turn;
}
inline double frontier_score(double path_m,double new_area_m2,double turn_rad,double revisit,double turn_cost_m_per_rad=.20){

  if(!std::isfinite(path_m)||path_m<0||!std::isfinite(new_area_m2)||new_area_m2<0||
     !std::isfinite(turn_rad)||!std::isfinite(revisit)||!std::isfinite(turn_cost_m_per_rad)||turn_cost_m_per_rad<0)return std::numeric_limits<double>::infinity();
  return (path_m+turn_cost_m_per_rad*std::abs(turn_rad)+.35*std::max(0.,revisit)+.15)/std::max(.10,new_area_m2);
}
}  // namespace carcar_mapping_autonomy
