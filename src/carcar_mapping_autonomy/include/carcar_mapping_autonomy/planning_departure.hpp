#pragma once
#include "carcar_mapping_autonomy/map_geometry.hpp"
namespace carcar_mapping_autonomy {
// 允许离开额外规划余量区，原0.22米净空仍逐格检查。不会放开未知、障碍或图外。
inline bool allow_planning_departure(const nav_msgs::msg::OccupancyGrid&m,const Pose&current,
                                    nav_msgs::msg::OccupancyGrid&mask){
 if(!grid_valid(m)||mask.info!=m.info||mask.data.size()!=m.data.size()||!free_footprint(m,current))return false;
 const double a=grid_yaw(m),dx=current.x-m.info.origin.position.x,dy=current.y-m.info.origin.position.y;
 const int cx=std::floor((std::cos(a)*dx+std::sin(a)*dy)/m.info.resolution);
 const int cy=std::floor((-std::sin(a)*dx+std::cos(a)*dy)/m.info.resolution);
 if(cx<0||cy<0||cx>=static_cast<int>(m.info.width)||cy>=static_cast<int>(m.info.height)||mask.data[cy*m.info.width+cx]==0)return false;
 // Smac规划从格心出发；亚格实际位置安全但该格心不安全时，不能声称已经打通起点。
 if(!free_footprint(m,cell_world(m,cx,cy)))return false;
 constexpr int reach=3;
 for(int y=std::max(0,cy-reach);y<=std::min(static_cast<int>(m.info.height)-1,cy+reach);++y)
  for(int x=std::max(0,cx-reach);x<=std::min(static_cast<int>(m.info.width)-1,cx+reach);++x){
   if(std::hypot(x-cx,y-cy)>reach)continue;
   if(free_footprint(m,cell_world(m,x,y)))mask.data[y*m.info.width+x]=0;
  }
 return true;
}
}  // namespace carcar_mapping_autonomy
