#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace carcar_mapping_console {
// 使用与现有录包器相同的2 GiB启动余量；保存保留256 MiB应急空间，不删除任何地图。
inline nlohmann::json storage_status(const std::filesystem::space_info&space){
  constexpr uintmax_t gib=1024ULL*1024*1024;
  return {{"valid",true},{"available_bytes",space.available},{"capacity_bytes",space.capacity},
    {"warning",space.available<5*gib},{"start_allowed",space.available>=2*gib},{"save_allowed",space.available>=gib/4},
    {"reason",space.available<gib/4?"磁盘余量不足256 MiB，保持停车；请归档日志后重试保存，已有地图保留":
      space.available<2*gib?"磁盘余量不足2 GiB，禁止新建或恢复运动；请先归档日志":
      space.available<5*gib?"磁盘剩余不足5 GiB，请及时归档录包与日志":""}};
}
inline nlohmann::json storage_status(const std::filesystem::path&root){
  std::error_code error;auto path=std::filesystem::absolute(root,error);
  if(error)return {{"valid",false},{"reason","无法解析地图存储路径"},{"start_allowed",false},{"save_allowed",false}};
  while(!std::filesystem::exists(path,error)&&!error&&path!=path.root_path())path=path.parent_path();
  const auto space=std::filesystem::space(path,error);
  if(error)return {{"valid",false},{"reason","无法读取磁盘可用空间："+error.message()},{"start_allowed",false},{"save_allowed",false}};
  return storage_status(space);
}
inline void require_storage(const std::filesystem::path&root,bool saving=false){
  const auto s=storage_status(root);if(!s.value(saving?"save_allowed":"start_allowed",false))
    throw std::runtime_error(s.value("reason",std::string("磁盘检查失败")));
}
}
