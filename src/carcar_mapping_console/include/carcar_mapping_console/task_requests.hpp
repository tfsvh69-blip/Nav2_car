#pragma once
#include <algorithm>
#include <deque>
#include <string>
#include <nlohmann/json.hpp>

namespace carcar_mapping_console {
// 请求账本保存每一端的关联结果；继续先短暂排队，暂停可以作废尚未下发的继续。
class TaskRequests {
public:
  struct Submission{bool forward=false,queued=false;std::string canonical,error;};
  void reset(const std::string&task){task_=task;++epoch_;records_.clear();}
  std::uint64_t epoch()const{return epoch_;}
  static bool stopping(const std::string&action){return action=="halt"||action=="pause";}
  static bool continuing(const std::string&action){return action=="start"||action=="resume";}
  bool continuation_submitted()const{
    return std::any_of(records_.begin(),records_.end(),[](const auto&r){return continuing(r.action)&&r.state=="submitted";});
  }
  void cancel_continuations(const std::string&reason){
    ++epoch_;
    for(auto&r:records_)if(r.state=="queued"||((r.action=="start"||r.action=="resume")&&r.state=="submitted")){
      r.state="cancelled";r.reason=reason;
    }
  }
  Submission submit(const std::string&id,const std::string&action,const nlohmann::json&extra,
    double now,const nlohmann::json&expected_epoch=nullptr,const std::string&ongoing={}){
    if(id.empty()||id.size()>128)return {false,false,{},"request_id无效，操作未提交"};
    for(const auto&r:records_)if(r.id==id){
      if(r.action!=action||r.extra!=extra)return {false,false,{},"同一request_id不能代表不同操作"};
      return {false,r.state=="queued",r.canonical,{}};
    }
    if(!stopping(action)&&!expected_epoch.is_null()&&
       (!expected_epoch.is_number_unsigned()||expected_epoch.get<std::uint64_t>()!=epoch_))
      return {false,false,{},"本次操作基于停车或任务切换前的旧状态，未下发；等待两端同步后重新选择"};
    if(stopping(action)){
      cancel_continuations("已被暂停／停车覆盖，不再等待继续结果；以停车反馈为准");
    }else{
      for(const auto&r:records_)if(r.epoch==epoch_&&(r.action==action||(continuing(r.action)&&continuing(action)))&&r.extra==extra&&
        (r.state=="queued"||r.state=="submitted"||(!ongoing.empty()&&r.canonical==ongoing))){
        const auto canonical=r.canonical;const auto state=r.state;const auto at=r.dispatch_at;
        add({id,canonical,action,state,"已合并同一操作，等待同一车端结果",extra,epoch_,at,now});
        return {false,state=="queued",canonical,{}};
      }
      if(!ongoing.empty())return {false,false,{},"重新检查尚未结束，本次未重复下发；等待当前检查完成"};
    }
    const bool queued=action=="start"||action=="resume";
    add({id,id,action,queued?"queued":"submitted",queued?"等待跨端停车优先窗口，尚未下发":"已提交车端，等待确认",extra,epoch_,now+.15,now});
    return {!queued,queued,id,{}};
  }
  nlohmann::json ready(double now){
    nlohmann::json result=nlohmann::json::array();
    for(auto&r:records_)if(r.id==r.canonical&&r.state=="queued"&&now>=r.dispatch_at){
      if(r.epoch!=epoch_){update(r.canonical,"cancelled","停车代次已改变，没有下发继续");continue;}
      result.push_back({{"request_id",r.id},{"action",r.action},{"extra",r.extra}});
      update(r.canonical,"submitted","已提交车端，等待确认");
    }
    return result;
  }
  void acknowledge(const std::string&id,const std::string&result,const std::string&reason){
    if(result=="accepted"||result=="rejected")update(id,result,reason);
  }
  nlohmann::json status()const{
    nlohmann::json values=nlohmann::json::array();
    for(const auto&r:records_)values.push_back({{"request_id",r.id},{"canonical_request_id",r.canonical},
      {"action",r.action},{"request_result",r.state},{"reason",r.reason},{"epoch",r.epoch}});
    return {{"protocol",1},{"task_id",task_},{"epoch",epoch_},{"requests",values}};
  }
private:
  struct Record{std::string id,canonical,action,state,reason;nlohmann::json extra;std::uint64_t epoch;double dispatch_at,created;};
  void add(Record value){
    records_.push_back(std::move(value));
    // 同一操作的主记录与各端别名共同淘汰，不能留下永远无法下发的排队别名。
    while(records_.size()>128){const auto oldest=records_.front().canonical;
      records_.erase(std::remove_if(records_.begin(),records_.end(),[&](const auto&r){return r.canonical==oldest;}),records_.end());}
  }
  void update(const std::string&canonical,const std::string&state,const std::string&reason){
    for(auto&r:records_)if(r.canonical==canonical&&!(r.state=="cancelled"&&(state=="accepted"||state=="rejected"))){r.state=state;r.reason=reason;}
  }
  std::deque<Record>records_;std::string task_;std::uint64_t epoch_=0;
};
}
