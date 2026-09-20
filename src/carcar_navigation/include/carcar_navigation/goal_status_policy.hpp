#pragma once
#include <string>
#include <sstream>
#include <iomanip>
#include <map>
#include <action_msgs/msg/goal_status_array.hpp>
namespace carcar_navigation {
inline std::string goal_uuid_text(const unique_identifier_msgs::msg::UUID & id) {
  std::ostringstream out;
  for (auto b : id.uuid) {out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);}
  return out.str();
}
// 先选择真正 EXECUTING 的最新目标，再处理整批历史终态；不依赖 status_list 顺序。
class GoalStatusPolicy {
public:
  std::string observe(const action_msgs::msg::GoalStatusArray & status) {
    for (const auto & entry : status.status_list) {
      if (entry.status!=action_msgs::msg::GoalStatus::STATUS_EXECUTING) {continue;}
      int64_t stamp=int64_t(entry.goal_info.stamp.sec)*1000000000+entry.goal_info.stamp.nanosec;
      const auto id=goal_uuid_text(entry.goal_info.goal_id);
      if (active_.empty() || stamp>stamp_ || id==active_) {active_=id;stamp_=stamp;}
    }
    return active_;
  }
  bool owns_terminal(const std::string & uuid) const {return !active_.empty() && uuid==active_;}
private:
  std::string active_;
  int64_t stamp_{-1};
};

// Action status 是累积数组；同一 UUID 的同一状态只记录一次，避免旧终态在每次发布时重复出现。
class ActionStatusDeduplicator {
public:
  bool changed(const std::string & uuid, int8_t status) {
    const auto previous=status_by_uuid_.find(uuid);
    if (previous!=status_by_uuid_.end() && previous->second==status) {return false;}
    status_by_uuid_[uuid]=status;return true;
  }
private:
  std::map<std::string,int8_t> status_by_uuid_;
};
}  // namespace carcar_navigation
