#pragma once
#include <condition_variable>
#include <mutex>
#include <limits>
#include <thread>
#include <nav_msgs/msg/path.hpp>
#include "carcar_mapping_autonomy/map_geometry.hpp"

namespace carcar_mapping_autonomy {
// 独立纯计算线程：只读取不可变快照；队列最多保留一个最新请求，不访问ROS或执行器。
class RouteValidator {
public:
  struct Job {
    uint64_t operation=0,attempt=0,revision=0,path_revision=0;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    std::shared_ptr<const nav_msgs::msg::Path> path;
    Task task;Pose current,target;bool remaining=false;
  };
  struct Result {Job job;bool safe=false;std::string reason;};
  RouteValidator():thread_([this]{run();}){}
  ~RouteValidator(){ {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}cv_.notify_one();thread_.join();}
  void submit(Job job){ {std::lock_guard<std::mutex> lock(mutex_);pending_=std::move(job);}cv_.notify_one();}
  std::optional<Result> take(){std::lock_guard<std::mutex> lock(mutex_);auto value=std::move(result_);result_.reset();return value;}
  static bool check(const Job & j,std::string *reason=nullptr) {
    auto reject=[&](const std::string&r){if(reason)*reason=r;return false;};
    if(!j.map||!j.path||j.path->header.frame_id!="map"||j.path->poses.empty()||j.path->poses.size()>20000)return reject("路线格式或长度无效");
    auto safe=[&](const Pose & p){return j.task.inside(p)&&free_footprint(*j.map,p);};
    if(!safe(j.target))return reject("目标净空失效");
    if(!safe(j.current))return reject("当前车位净空失效");
    size_t start=0;
    if(j.remaining){double best=std::numeric_limits<double>::infinity();
      for(size_t i=0;i<j.path->poses.size();++i){const auto &p=j.path->poses[i].pose.position;
        const auto d=std::hypot(p.x-j.current.x,p.y-j.current.y);if(d<best){best=d;start=i;}}
    }
    Pose previous=j.current;size_t samples=0;
    for(size_t i=start;i<j.path->poses.size();++i){const auto &p=j.path->poses[i].pose.position;
      const double distance=std::hypot(p.x-previous.x,p.y-previous.y);
      if(!std::isfinite(distance)||distance>200)return false;
      const size_t steps=std::max<size_t>(1,std::ceil(distance/(j.map->info.resolution*.5)));
      if(steps>20000-samples)return false;
      samples+=steps;
      for(size_t k=1;k<=steps;++k){const double ratio=static_cast<double>(k)/steps;
        const Pose sample{previous.x+(p.x-previous.x)*ratio,previous.y+(p.y-previous.y)*ratio,0};
        if(!safe(sample))return reject("路线净空失效 x="+std::to_string(sample.x)+" y="+std::to_string(sample.y));}
      previous={p.x,p.y,0};
    }
    if(std::hypot(previous.x-j.target.x,previous.y-j.target.y)>.15)return reject("规划末点与固定观测目标偏差超过0.15米");
    return true;
  }
private:
  void run(){for(;;){Job job;{std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stopping_||pending_.has_value();});
      if(stopping_)return;
      job=std::move(*pending_);pending_.reset();}
      std::string reason;const bool safe=check(job,&reason);{std::lock_guard<std::mutex> lock(mutex_);result_=Result{std::move(job),safe,std::move(reason)};}}}
  std::mutex mutex_;std::condition_variable cv_;bool stopping_=false;
  std::optional<Job> pending_;std::optional<Result> result_;std::thread thread_;
};
}  // namespace carcar_mapping_autonomy
