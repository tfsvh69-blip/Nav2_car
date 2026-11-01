#pragma once
#include <cmath>
#include <string>
#include <stdexcept>
#include <nlohmann/json.hpp>
namespace carcar_mapping_autonomy {
using SnapshotJson=nlohmann::json;
inline bool snapshot_identifier(const std::string&value){
  return !value.empty()&&value.size()<=128&&value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos;
}
inline void validate_snapshot_request(const SnapshotJson&request){
  for(const auto*key:{"task_id","request_id"})if(!snapshot_identifier(request.at(key).get<std::string>()))throw std::runtime_error("快照任务或请求标识无效");
  for(const auto*key:{"generation","navigation_generation"}){
    const auto&value=request.at(key);
    if(!value.is_number_integer()||(!value.is_number_unsigned()&&value.get<int64_t>()<0))throw std::runtime_error("快照代次无效");
  }
  const auto&origin=request.at("origin");if(!origin.is_array()||origin.size()!=3)throw std::runtime_error("快照任务原点无效");
  for(const auto&v:origin)if(!v.is_number()||!std::isfinite(v.get<double>()))throw std::runtime_error("快照任务原点无效");
}
inline bool snapshot_same_generation(const SnapshotJson&a,const SnapshotJson&b){
  for(const auto*key:{"task_id","generation","navigation_generation"})if(!a.contains(key)||!b.contains(key)||a.at(key)!=b.at(key))return false;
  return true;
}
} // namespace carcar_mapping_autonomy
