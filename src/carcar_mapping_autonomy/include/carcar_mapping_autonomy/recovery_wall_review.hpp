#pragma once
#include <algorithm>
#include <cmath>
#include "carcar_mapping_autonomy/task.hpp"
namespace carcar_mapping_autonomy {
// 人工复核使用独立总预算；反复进入/退出等待不能重置300秒上限。
class RecoveryWallReviewWait {
public:
  static constexpr double limit_seconds=300.;
  void reset(){elapsed_=0;last_=-1;active_=false;}
  void update(double now,bool eligible){
    if(!std::isfinite(now)||now<0){active_=false;return;}
    if(last_>=0&&active_)elapsed_+=std::max(0.,now-last_);
    last_=now;active_=eligible;
  }
  double elapsed(double now)const{
    return elapsed_+(active_&&last_>=0&&std::isfinite(now)?std::max(0.,now-last_):0.);
  }
  bool expired(double now)const{return elapsed(now)>limit_seconds;}
  bool active()const{return active_;}
private:double elapsed_=0,last_=-1;bool active_=false;
};
// 只接受本次恢复同task/任务代次且已明确停稳的监督器授权，不能由网页单字段延长恢复。
inline bool wall_review_authorized(const Json&state,const std::string&task,uint64_t generation){
  try{
    const auto&walls=state.at("walls");const auto&output=state.at("output");
    return state.value("task_id",std::string())==task&&state.value("generation",uint64_t{})==generation&&
      state.value("phase",std::string())=="RECOVERING"&&state.value("still",false)&&
      state.value("navigation_terminal",false)&&walls.value("review_allowed",false)&&
      walls.value("phase",std::string())=="REVIEW_REQUIRED"&&walls.contains("walls")&&
      walls["walls"].is_array()&&!walls["walls"].empty()&&output.is_array()&&output.size()==2&&
      output[0].is_number()&&output[1].is_number()&&std::isfinite(output[0].get<double>())&&
      std::isfinite(output[1].get<double>())&&std::abs(output[0].get<double>())<1e-9&&std::abs(output[1].get<double>())<1e-9;
  }catch(const std::exception&){return false;}
}
}  // namespace carcar_mapping_autonomy
