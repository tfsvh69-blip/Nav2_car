#pragma once
#include <cmath>
#include <queue>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace carcar_mapping_autonomy {
// 只为舒适性排序计算已知实体障碍的近似距离场；未知边界不作为斥力源。
// 八邻域距离只参与软评分，原图圆包络和连续路径检查仍负责硬安全判定。
inline std::vector<float> known_obstacle_clearance(const nav_msgs::msg::OccupancyGrid&map){
  const size_t count=size_t(map.info.width)*map.info.height;
  if(!count||count!=map.data.size()||count>2000000||!std::isfinite(map.info.resolution)||map.info.resolution<=0)return {};
  constexpr float cutoff=.65f;std::vector<float> field(count,cutoff);
  using Item=std::pair<float,size_t>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
  for(size_t i=0;i<count;++i)if(map.data[i]>=50){field[i]=0;queue.push({0,i});}
  const int width=map.info.width,height=map.info.height;
  while(!queue.empty()){
    const auto[d,index]=queue.top();queue.pop();if(d>field[index])continue;
    const int x=index%width,y=index/width;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
      const int nx=x+dx,ny=y+dy;if((!dx&&!dy)||nx<0||ny<0||nx>=width||ny>=height)continue;
      const size_t next=size_t(ny)*width+nx;const float distance=d+map.info.resolution*(dx&&dy?std::sqrt(2.):1.);
      if(distance<field[next]){field[next]=distance;queue.push({distance,next});}
    }
  }
  // 去除半个栅格对角线，软奖励按障碍格边界附近的余量计算，超过舒适上限即停止奖励。
  for(auto&value:field)value=std::max(0.f,value-float(map.info.resolution*std::sqrt(.5)));
  return field;
}
}
