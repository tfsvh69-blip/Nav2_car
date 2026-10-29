#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
namespace explore {
// 仅抑制局部区域，不把一个代表点的失败扩大为整条连通前沿失败。
class GoalMemory {
  struct Region {geometry_msgs::msg::Point point;unsigned failures=0;double until=0,updated=0;bool visited=false;};
  std::vector<Region> regions_;
public:
  double radius=.40,cooldown=60.;
  unsigned failure_limit=2;
  bool blocked(const geometry_msgs::msg::Point&p,double now)const{
    for(const auto&r:regions_)if(std::hypot(r.point.x-p.x,r.point.y-p.y)<=radius&&now<r.until&&
      (r.visited||r.failures>=failure_limit))return true;
    return false;
  }
  void record(const geometry_msgs::msg::Point&p,bool success,double now){
    auto it=std::find_if(regions_.begin(),regions_.end(),[&](const Region&r){return std::hypot(r.point.x-p.x,r.point.y-p.y)<=radius;});
    if(it==regions_.end()){
      if(regions_.size()==64)regions_.erase(std::min_element(regions_.begin(),regions_.end(),[](const Region&a,const Region&b){return a.updated<b.updated;}));
      regions_.push_back({p,0,0,now,false});it=regions_.end()-1;
    }
    if(now>=it->until){it->failures=0;it->visited=false;}
    it->updated=now;it->until=now+cooldown;it->visited=success;
    if(success)it->failures=0;else ++it->failures;
  }
};
}  // namespace explore
