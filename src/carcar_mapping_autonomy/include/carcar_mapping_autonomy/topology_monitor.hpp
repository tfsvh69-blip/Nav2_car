#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace carcar_mapping_autonomy {
// 图查询可能等待中间件内部锁；独立执行，控制线程只读取短锁保护的结果。
class TopologyMonitor {
public:
  struct Result {bool ready=false,healthy=false;double completed=0,duration_ms=0;std::string error;};
  explicit TopologyMonitor(std::function<bool()> check):check_(std::move(check)),thread_([this]{run();}){}
  ~TopologyMonitor(){ {std::lock_guard<std::mutex>lock(mutex_);stopping_=true;}cv_.notify_one();thread_.join();}
  Result result(){std::lock_guard<std::mutex>lock(mutex_);return result_;}
private:
  static double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void run(){for(;;){
    const double began=seconds();Result next;
    try{next.healthy=check_();}catch(const std::exception&e){next.error=e.what();}
    next.ready=true;next.completed=seconds();next.duration_ms=(next.completed-began)*1000;
    std::unique_lock<std::mutex>lock(mutex_);result_=std::move(next);
    if(cv_.wait_for(lock,std::chrono::milliseconds(500),[this]{return stopping_;}))return;
  }}
  std::function<bool()>check_;std::mutex mutex_;std::condition_variable cv_;bool stopping_=false;
  Result result_;std::thread thread_;
};
}  // namespace carcar_mapping_autonomy
