#pragma once
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include "carcar_mapping_autonomy/subcell_departure.hpp"

namespace carcar_mapping_autonomy {
// 只筛选已知地图上的静态连通目标；实际路径、动态避障和最终停稳仍由原导航链路确认。
class ReturnSelector {
public:
  struct Job {
    uint64_t operation=0,generation=0,revision=0;
    std::string request_id;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    Pose current;Task task;
    double radius=.8,arrival_tolerance=.15;
    size_t max_candidates=12;
  };
  struct Result {Job job;std::vector<Pose> candidates;std::string reason;};

  class Search {
  public:
    static Result select(const Job & job) {
      Result result{job,{},""};
      const auto finite_pose=[](const Pose & pose) {
        return std::isfinite(pose.x)&&std::isfinite(pose.y)&&std::isfinite(pose.yaw);
      };
      if(!std::isfinite(job.radius)||!std::isfinite(job.arrival_tolerance)||
         job.arrival_tolerance<=0||job.radius<=job.arrival_tolerance||
         job.max_candidates==0||job.max_candidates>64||!finite_pose(job.current)||
         !finite_pose(job.task.origin)||!job.task.limits.valid()) {
        result.reason="返程候选参数无效，没有选择或下发目标";return result;
      }
      if(!job.map||!grid_valid(*job.map)||!free_footprint(*job.map,job.current)) {
        result.reason="当前位置图外、地图无效或当前足迹不安全，没有选择返程目标";return result;
      }
      const auto & map=*job.map;
      const auto start=cell(map,job.current),origin=cell(map,job.task.origin);
      if(!start||!origin) {
        result.reason="当前位置或记录起点在地图外，没有选择返程目标";return result;
      }
      auto mask=planning_keepout_mask(map);
      if(!mask) {
        result.reason="地图不支持既有规划安全掩码，没有选择返程目标";return result;
      }
      // 只复用已验收的起步余量规则；基础足迹、障碍及未知栅格保持原检查。
      allow_planning_departure(map,job.current,*mask);
      std::vector<uint8_t> reachable(mask->data.size(),0);
      std::queue<size_t> pending;
      if(mask->data[*start]==0) {reachable[*start]=1;pending.push(*start);}
      else for(const auto&p:departure_entries(map,job.current,*mask)) {
        const auto entry=cell(map,p);
        if(entry&&!reachable[*entry]){reachable[*entry]=1;pending.push(*entry);}
      }
      const int width=map.info.width,height=map.info.height;
      while(!pending.empty()) {
        const auto index=pending.front();pending.pop();const int x=index%width,y=index/width;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
          const int nx=x+dx,ny=y+dy;
          if((!dx&&!dy)||nx<0||ny<0||nx>=width||ny>=height)continue;
          const auto next=static_cast<size_t>(ny)*width+nx;
          // 沿用Smac二维八邻接和一个对角格余量，不代替连续路线审查。
          if(mask->data[next]!=0||reachable[next])continue;
          reachable[next]=1;pending.push(next);
        }
      }
      const bool origin_free=free_footprint(map,job.task.origin);
      const bool exact=origin_free&&job.task.inside(job.task.origin)&&reachable[*origin];
      if(exact)result.candidates.push_back(job.task.origin);

      const double center_radius=job.radius-job.arrival_tolerance;
      const int reach=static_cast<int>(std::min(std::ceil(center_radius/map.info.resolution),
                                               static_cast<double>(std::max(width,height))));
      const int cx=*origin%width,cy=*origin/width;
      const int x0=std::max(0,cx-reach),x1=std::min(width-1,cx+reach);
      const int y0=std::max(0,cy-reach),y1=std::min(height-1,cy+reach);
      struct Candidate {Pose pose;double origin_distance,current_distance;};
      std::vector<Candidate> nearby;
      for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
        const auto index=static_cast<size_t>(y)*width+x;
        if(!reachable[index]||map.data[index]<0||map.data[index]>=50)continue;
        auto pose=cell_world(map,x,y);pose.yaw=job.task.origin.yaw;
        const double d=std::hypot(pose.x-job.task.origin.x,pose.y-job.task.origin.y);
        // 预留导航终态误差，不能把目标点在半径内当作真实终态已在半径内。
        if(d>center_radius||!job.task.inside(pose)||!planning_footprint(map,pose))continue;
        nearby.push_back({pose,d,std::hypot(pose.x-job.current.x,pose.y-job.current.y)});
      }
      std::sort(nearby.begin(),nearby.end(),[](const Candidate & a,const Candidate & b) {
        if(a.origin_distance!=b.origin_distance)return a.origin_distance<b.origin_distance;
        if(a.current_distance!=b.current_distance)return a.current_distance<b.current_distance;
        if(a.pose.x!=b.pose.x)return a.pose.x<b.pose.x;
        return a.pose.y<b.pose.y;
      });
      constexpr double separation=.12;
      for(const auto & candidate:nearby) {
        if(result.candidates.size()>=job.max_candidates)break;
        const bool separate=std::all_of(result.candidates.begin(),result.candidates.end(),[&](const Pose & selected) {
          return std::hypot(candidate.pose.x-selected.x,candidate.pose.y-selected.y)>=separation;
        });
        if(separate)result.candidates.push_back(candidate.pose);
      }
      result.reason=exact?"起点可用，优先尝试精确起点":"起点不可用";
      if(!exact)result.reason+=origin_free?"：起点不在当前位置静态可达区域":"：起点足迹不是已知自由区";
      if(result.candidates.empty())result.reason+="；限定范围内没有安全可达的附近候选";
      else result.reason+="；已准备"+std::to_string(result.candidates.size())+"个静态可达返程目标";
      return result;
    }

  private:
    static std::optional<size_t> cell(const nav_msgs::msg::OccupancyGrid & map,const Pose & pose) {
      const double a=grid_yaw(map),dx=pose.x-map.info.origin.position.x,dy=pose.y-map.info.origin.position.y;
      const double x=(std::cos(a)*dx+std::sin(a)*dy)/map.info.resolution;
      const double y=(-std::sin(a)*dx+std::cos(a)*dy)/map.info.resolution;
      if(!std::isfinite(x)||!std::isfinite(y)||x<0||y<0||x>=map.info.width||y>=map.info.height)return {};
      return static_cast<size_t>(std::floor(y))*map.info.width+static_cast<size_t>(std::floor(x));
    }
  };

  ReturnSelector():thread_([this]{run();}){}
  ~ReturnSelector() {
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
    cv_.notify_one();thread_.join();
  }
  void submit(Job job) {
    {std::lock_guard<std::mutex> lock(mutex_);pending_=Queued{++latest_,std::move(job)};result_.reset();}
    cv_.notify_one();
  }
  std::optional<Result> take() {
    std::lock_guard<std::mutex> lock(mutex_);auto result=std::move(result_);result_.reset();return result;
  }

private:
  struct Queued {uint64_t sequence;Job job;};
  void run() {
    for(;;) {
      Queued queued;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[this]{return stopping_||pending_.has_value();});
        if(stopping_)return;
        queued=std::move(*pending_);pending_.reset();
      }
      auto result=Search::select(queued.job);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // 新请求已到达时丢弃旧计算，避免异步旧结果覆盖当前操作。
        if(!stopping_&&queued.sequence==latest_)result_=std::move(result);
      }
    }
  }
  std::mutex mutex_;std::condition_variable cv_;bool stopping_=false;uint64_t latest_=0;
  std::optional<Queued> pending_;std::optional<Result> result_;std::thread thread_;
};
}  // namespace carcar_mapping_autonomy
