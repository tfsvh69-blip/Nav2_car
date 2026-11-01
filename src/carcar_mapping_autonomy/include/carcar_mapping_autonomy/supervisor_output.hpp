#pragma once
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <nlohmann/json.hpp>

namespace carcar_mapping_autonomy {
// 控制线程只提交已生成的一致快照；慢DDS及同步日志由同一输出线程承担。
// 高频状态只保留最新一份，事件独立排队；任何锁都不覆盖发布、日志或序列化。
class SupervisorOutput {
public:
  using Json=nlohmann::json;
  using Publish=std::function<void(const std::string&)>;
  using Log=std::function<void(int,const std::string&)>;
  static constexpr size_t event_capacity=64;
  SupervisorOutput(Publish publish,Log log):publish_(std::move(publish)),log_(std::move(log)),worker_([this]{run();}){}
  ~SupervisorOutput(){close();}
  SupervisorOutput(const SupervisorOutput&)=delete;
  SupervisorOutput&operator=(const SupervisorOutput&)=delete;

  void submit(Json data,bool log_change){
    auto frame=std::make_shared<Frame>();frame->data=std::move(data);frame->created=seconds();
    frame->stamp_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::shared_ptr<const Frame> replaced;
    {std::lock_guard<std::mutex>lock(mutex_);if(stopping_)return;
      frame->sequence=++submitted_;if(pending_){replaced=std::move(pending_);++replaced_;}
      pending_=frame;if(log_change)enqueue(Event{20,{},frame});}
    cv_.notify_one();
  }
  void event(int severity,std::string message){
    {std::lock_guard<std::mutex>lock(mutex_);if(stopping_)return;enqueue(Event{severity,std::move(message),{}});}
    cv_.notify_one();
  }
  Json diagnostics()const{
    std::lock_guard<std::mutex>lock(mutex_);const double now=seconds();
    return {{"mode","latest_state_bounded_events"},{"state_submitted",submitted_},{"state_published",published_},
      {"state_replaced",replaced_},{"state_pending",bool(pending_)},{"state_pending_age_seconds",pending_?now-pending_->created:0.},
      {"event_capacity",event_capacity},{"event_pending",events_.size()},{"event_high_water",event_high_water_},
      {"events_enqueued",enqueued_},{"events_logged",logged_},{"events_dropped",dropped_},
      {"publish_failures",publish_failures_},{"log_failures",log_failures_},{"last_error",last_error_},
      {"last_publish_ms",last_publish_ms_},{"max_publish_ms",max_publish_ms_},
      {"last_log_ms",last_log_ms_},{"max_log_ms",max_log_ms_},
      {"last_state_age_ms",last_state_age_ms_},{"max_state_age_ms",max_state_age_ms_},
      {"worker_stage",stage_},{"worker_stage_age_seconds",stage_=="idle"?0.:now-stage_at_}};
  }
  // 先由调用者发送停车，再等待已提交事件排空；退出不遗失最后的拒绝或停车原因。
  void close(){
    {std::lock_guard<std::mutex>lock(mutex_);stopping_=true;}
    cv_.notify_one();if(worker_.joinable())worker_.join();
  }
private:
  struct Frame {Json data;uint64_t sequence=0;double created=0;int64_t stamp_ns=0;};
  struct Event {int severity=20;std::string message;std::shared_ptr<const Frame>frame;};
  static double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void enqueue(Event event){
    if(events_.size()==event_capacity){
      // 过载时优先舍弃普通状态日志，独立的请求拒绝、取消与停车事件优先保留。
      auto ordinary=std::find_if(events_.begin(),events_.end(),[](const Event&e){return bool(e.frame);});
      events_.erase(ordinary==events_.end()?events_.begin():ordinary);++dropped_;
    }
    events_.push_back(std::move(event));++enqueued_;event_high_water_=std::max(event_high_water_,events_.size());
  }
  std::string encode(const Frame&frame)const{
    auto data=frame.data;auto status=diagnostics();status["source_sequence"]=frame.sequence;
    status["source_stamp_ns"]=frame.stamp_ns;status["source_steady_seconds"]=frame.created;
    status["source_age_seconds"]=seconds()-frame.created;data["publication"]=std::move(status);return data.dump();
  }
  void stage(const char*name){std::lock_guard<std::mutex>lock(mutex_);stage_=name;stage_at_=seconds();}
  void run(){for(;;){
    std::shared_ptr<const Frame>frame;std::optional<Event>event;
    {std::unique_lock<std::mutex>lock(mutex_);cv_.wait(lock,[this]{return stopping_||pending_||!events_.empty();});
      if(stopping_&&!pending_&&events_.empty())return;
      frame=std::move(pending_);if(!events_.empty()){event=std::move(events_.front());events_.pop_front();}}
    if(frame){stage("publish");const double began=seconds();std::string error;
      try{publish_(encode(*frame));}catch(const std::exception&e){error=e.what();}catch(...){error="状态发布出现非标准异常";}
      const double duration=(seconds()-began)*1000,age=(seconds()-frame->created)*1000;
      {std::lock_guard<std::mutex>lock(mutex_);last_publish_ms_=duration;max_publish_ms_=std::max(max_publish_ms_,duration);
        last_state_age_ms_=age;max_state_age_ms_=std::max(max_state_age_ms_,age);
        if(error.empty())++published_;else{++publish_failures_;last_error_=error.substr(0,512);}}}
    if(event){stage("log");const double began=seconds();std::string error;
      try{log_(event->severity,event->frame?"自动建图状态 "+encode(*event->frame):event->message);}
      catch(const std::exception&e){error=e.what();}catch(...){error="事件日志出现非标准异常";}
      const double duration=(seconds()-began)*1000;
      {std::lock_guard<std::mutex>lock(mutex_);last_log_ms_=duration;max_log_ms_=std::max(max_log_ms_,duration);
        if(error.empty())++logged_;else{++log_failures_;last_error_=error.substr(0,512);}}}
    stage("idle");
  }}
  Publish publish_;Log log_;mutable std::mutex mutex_;std::condition_variable cv_;bool stopping_=false;
  std::shared_ptr<const Frame>pending_;std::deque<Event>events_;
  uint64_t submitted_=0,published_=0,replaced_=0,enqueued_=0,logged_=0,dropped_=0,publish_failures_=0,log_failures_=0;
  size_t event_high_water_=0;double last_publish_ms_=0,max_publish_ms_=0,last_log_ms_=0,max_log_ms_=0,last_state_age_ms_=0,max_state_age_ms_=0;
  std::string stage_="idle",last_error_;double stage_at_=0;std::thread worker_;
};
}  // namespace carcar_mapping_autonomy
