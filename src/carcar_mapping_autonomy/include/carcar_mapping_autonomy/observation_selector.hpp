#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include "carcar_mapping_autonomy/subcell_departure.hpp"

namespace carcar_mapping_autonomy {
// 只作静态可达性筛选；动态代价、真实路径及连续净空仍由原规划／审查链路决定。
// 缓存属于计算线程；ROS线程只提交不可变地图快照并取回带版本的结果。
class ObservationSelector {
public:
  struct Job {
    uint64_t operation=0,revision=0;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    Pose current,frontier;Task task;
  };
  struct Result {Job job;std::optional<Pose> target;std::string reason;};
  class Search {
  public:
    static std::optional<size_t> cell(const nav_msgs::msg::OccupancyGrid&m,const Pose&p){
      if(!grid_valid(m)||!std::isfinite(p.x)||!std::isfinite(p.y))return {};
      const double a=grid_yaw(m),dx=p.x-m.info.origin.position.x,dy=p.y-m.info.origin.position.y;
      const double x=(std::cos(a)*dx+std::sin(a)*dy)/m.info.resolution;
      const double y=(-std::sin(a)*dx+std::cos(a)*dy)/m.info.resolution;
      if(!std::isfinite(x)||!std::isfinite(y)||x<0||y<0||x>=m.info.width||y>=m.info.height)return {};
      return static_cast<size_t>(std::floor(y))*m.info.width+static_cast<size_t>(std::floor(x));
    }
    std::optional<Pose> select(const Job&j){
      reason_="车端选点未通过：地图无效或当前车位不满足原图净空；未提交路径规划";
      if(!j.map||!grid_valid(*j.map)||!free_footprint(*j.map,j.current))return {};
      const auto start=cell(*j.map,j.current);if(!start)return {};
      if(!map_||revision_!=j.revision){
        mask_=planning_keepout_mask(*j.map);map_=j.map;revision_=j.revision;start_.reset();
      }
      if(!mask_)return {};
      if(!start_||*start_!=*start||std::hypot(j.current.x-position_.x,j.current.y-position_.y)>1e-6){
        auto mask=*mask_;allow_planning_departure(*j.map,j.current,mask);
        reachable_.assign(mask.data.size(),0);start_=start;position_=j.current;
        std::queue<size_t> pending;
        if(mask.data[*start]==0){pending.push(*start);reachable_[*start]=1;}
        else {
          // 与规划适配器使用相同的连续衔接检查；可达格不等于放开车位所在格。
          for(const auto&p:departure_entries(*j.map,j.current,mask)) {
            const auto entry=cell(*j.map,p);
            if(entry&&!reachable_[*entry]){pending.push(*entry);reachable_[*entry]=1;}
          }
        }
        const int w=mask.info.width,h=mask.info.height;
        while(!pending.empty()){
          const auto index=pending.front();pending.pop();const int x=index%w,y=index/w;
          for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
            const int nx=x+dx,ny=y+dy;if((!dx&&!dy)||nx<0||ny<0||nx>=w||ny>=h)continue;
            const auto next=static_cast<size_t>(ny)*w+nx;
            if(mask.data[next]!=0||reachable_[next])continue;
            // 沿用Smac二维八邻接；掩膜保留一个对角格余量，最终连续路线仍需审查。
            reachable_[next]=1;pending.push(next);
          }
        }
      }
      ObservationCounts counts;
      auto result=safe_observation(*j.map,j.frontier,j.task,&reachable_,&j.current,&counts);
      const std::string layer=counts.clearance==0?"前沿附近没有足够净空":
        counts.connected==0?"前沿附近有空地，但与当前车位不连通":
        counts.visible==0?"候选与前沿之间被墙体或未知区遮挡":
        "可见候选均距车不超过0.25米，没有新的有效观测位置";
      reason_=result?"已选择满足净空、连通、视线和距离要求的观测点":
        "车端选点未通过："+layer+"；筛选数量：净空="+std::to_string(counts.clearance)+
        "，连通="+std::to_string(counts.connected)+"，可见="+std::to_string(counts.visible)+
        "，有效距离="+std::to_string(counts.progress)+"；未提交路径规划，未下发运动目标";
      return result;
    }
    const std::string& reason()const{return reason_;}
  private:
    std::string reason_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    std::optional<nav_msgs::msg::OccupancyGrid> mask_;
    uint64_t revision_=0;std::optional<size_t>start_;std::vector<uint8_t>reachable_;Pose position_;
  };
  ObservationSelector():thread_([this]{run();}){}
  ~ObservationSelector(){ {std::lock_guard<std::mutex>lock(mutex_);stopping_=true;}cv_.notify_one();thread_.join();}
  void submit(Job job){ {std::lock_guard<std::mutex>lock(mutex_);pending_=std::move(job);}cv_.notify_one();}
  std::optional<Result> take(){std::lock_guard<std::mutex>lock(mutex_);auto result=std::move(result_);result_.reset();return result;}
private:
  void run(){Search search;for(;;){Job job;
    {std::unique_lock<std::mutex>lock(mutex_);cv_.wait(lock,[this]{return stopping_||pending_.has_value();});
      if(stopping_)return;
      job=std::move(*pending_);pending_.reset();}
    auto target=search.select(job);
    {std::lock_guard<std::mutex>lock(mutex_);result_=Result{std::move(job),target,search.reason()};}
  }}
  std::mutex mutex_;std::condition_variable cv_;bool stopping_=false;
  std::optional<Job>pending_;std::optional<Result>result_;std::thread thread_;
};
}  // namespace carcar_mapping_autonomy
