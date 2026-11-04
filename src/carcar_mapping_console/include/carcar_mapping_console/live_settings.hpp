#pragma once
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "carcar_mapping_console/common.hpp"

namespace carcar_mapping_console {
struct TuningRange {const char*name;double minimum,maximum,baseline;};
inline const std::array<TuningRange,4>&tuning_ranges(){
  static const std::array<TuningRange,4>ranges{{
    {"global_inflation_radius",.22,1.5,.5},{"global_cost_scaling_factor",.5,15,3},
    {"local_inflation_radius",.22,1.,.3},{"local_cost_scaling_factor",.5,15,5}}};
  return ranges;
}
inline json baseline_tuning_values(){json result=json::object();for(const auto&r:tuning_ranges())result[r.name]=r.baseline;return result;}
inline double live_number(const json&value,double minimum,double maximum,const std::string&name){
  if(!value.is_number())throw std::invalid_argument(name+"需要有限数值");
  const double number=value.get<double>();
  if(!std::isfinite(number)||number<minimum||number>maximum)throw std::invalid_argument(name+"超出允许范围");
  return number;
}
inline json validate_tuning_values(const json&values,bool complete=false){
  if(!values.is_object()||values.empty())throw std::invalid_argument("values需要非空参数对象");
  json checked=json::object();
  for(auto it=values.begin();it!=values.end();++it){
    bool found=false;for(const auto&r:tuning_ranges())if(it.key()==r.name){checked[r.name]=live_number(it.value(),r.minimum,r.maximum,r.name);found=true;break;}
    if(!found)throw std::invalid_argument("不支持的代价参数："+it.key());
  }
  if(complete&&checked.size()!=tuning_ranges().size())throw std::invalid_argument("默认配置缺少完整四项代价参数");
  return checked;
}
inline bool live_action(const std::string&action){
  return action=="set_tuning"||action=="restore_tuning"||action=="save_tuning"||action=="set_handoff"||
    action=="set_escape_enabled"||action=="set_walls"||action=="confirm_walls"||action=="set_wall_bypass";
}
inline bool online_live_action(const std::string&action){
  return action=="set_tuning"||action=="restore_tuning"||action=="save_tuning"||action=="set_handoff"||action=="set_escape_enabled";
}
// 网页字段使用固定白名单；墙和代价参数不能夹带未声明的硬件或安全门槛。
inline json validate_live_request(const std::string&action,const json&input){
  if(!live_action(action)||!input.is_object())throw std::invalid_argument("在线操作无效");
  std::set<std::string>allowed{"session","action","request_id","task_id","operation_epoch"};
  if(action=="set_tuning")allowed.insert("values");
  if(action=="set_handoff")allowed.insert("distance_m");
  if(action=="set_escape_enabled"||action=="set_wall_bypass")allowed.insert("enabled");
  if(action=="set_escape_enabled")allowed.insert("supervised_trial");
  if(action=="set_walls")allowed.insert("walls");
  if(action=="set_walls"||action=="confirm_walls"){allowed.insert("wall_revision");allowed.insert("navigation_generation");}
  for(auto it=input.begin();it!=input.end();++it)if(!allowed.count(it.key()))throw std::invalid_argument("在线操作包含未支持字段："+it.key());
  json result=json::object();
  if(action=="set_tuning")result["values"]=validate_tuning_values(input.at("values"));
  if(action=="set_handoff")result["distance_m"]=live_number(input.at("distance_m"),.15,1.5,"distance_m");
  if(action=="set_escape_enabled"||action=="set_wall_bypass"){
    if(!input.at("enabled").is_boolean())throw std::invalid_argument("enabled必须为布尔值");
    result["enabled"]=input["enabled"];
  }
  if(action=="set_escape_enabled"&&input.contains("supervised_trial")){
    if(!input["supervised_trial"].is_boolean())throw std::invalid_argument("supervised_trial必须为布尔值");
    result["supervised_trial"]=input["supervised_trial"];
  }
  if(action=="set_walls"||action=="confirm_walls")for(const auto*key:{"wall_revision","navigation_generation"}){
    if(!input.at(key).is_number_unsigned())throw std::invalid_argument(std::string(key)+"必须为非负整数");
    result[key]=input[key];
  }
  if(action=="set_walls"){
    const auto&walls=input.at("walls");if(!walls.is_array()||walls.size()>32)throw std::invalid_argument("人工墙须为不超过32条的列表");
    std::set<std::string>ids;size_t vertices=0;
    for(const auto&wall:walls){
      if(!wall.is_object()||wall.size()!=2||!wall.contains("id")||!wall["id"].is_string()||!wall.contains("points"))throw std::invalid_argument("人工墙仅接受id与points");
      const auto id=wall["id"].get<std::string>();
      if(id.empty()||id.size()>128||id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos||!ids.insert(id).second)throw std::invalid_argument("人工墙ID无效或重复");
      const auto&points=wall["points"];if(!points.is_array()||points.size()<2||points.size()>64)throw std::invalid_argument("每条人工墙需要2至64个顶点");
      vertices+=points.size();if(vertices>256)throw std::invalid_argument("人工墙顶点总数超过256");
      for(size_t i=0;i<points.size();++i){
        const auto&p=points[i];if(!p.is_array()||p.size()!=2)throw std::invalid_argument("人工墙顶点需要map中的[x米,y米]");
        const double x=live_number(p[0],-1e6,1e6,"墙x坐标"),y=live_number(p[1],-1e6,1e6,"墙y坐标");
        if(i&&std::hypot(x-points[i-1][0].get<double>(),y-points[i-1][1].get<double>())<1e-9)throw std::invalid_argument("人工墙相邻顶点不能重合");
      }
    }
    result["walls"]=walls;
  }
  return result;
}
inline std::filesystem::path default_tuning_file(){
  const char*home=std::getenv("HOME");if(!home||!*home)throw std::runtime_error("无法确定用户默认配置目录");
  return std::filesystem::path(home)/".local/state/carcar_mapping_console/tuning-defaults.json";
}
inline std::filesystem::path checked_tuning_path(const std::filesystem::path&value){
  const auto path=std::filesystem::absolute(value).lexically_normal();
  std::filesystem::path current;
  for(const auto&part:path){
    current/=part;std::error_code error;const auto state=std::filesystem::symlink_status(current,error);
    if(error&&error!=std::errc::no_such_file_or_directory)throw std::runtime_error("无法核对默认配置路径："+error.message());
    if(std::filesystem::is_symlink(state))throw std::runtime_error("默认配置路径不接受符号链接："+current.string());
  }
  return path;
}
inline json validate_tuning_defaults(const json&value){
  if(!value.is_object()||value.size()!=3||!value.contains("version")||!value["version"].is_number_integer()||value["version"]!=1||
     !value.contains("values")||!value.contains("handoff_distance_m"))throw std::invalid_argument("默认配置版本或字段不兼容，保留原文件并停止加载");
  return {{"version",1},{"values",validate_tuning_values(value["values"],true)},
    {"handoff_distance_m",live_number(value["handoff_distance_m"],.15,1.5,"handoff_distance_m")}};
}
inline json baseline_live_defaults(){return {{"version",1},{"values",baseline_tuning_values()},{"handoff_distance_m",.25}};}
// 不存在使用基线；损坏、不兼容、超大或链接文件均拒绝，不能静默回退后启动硬件。
inline json load_tuning_defaults(const std::filesystem::path&value){
  const auto path=checked_tuning_path(value);
  const int fd=::open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);
  if(fd<0){if(errno==ENOENT)return baseline_live_defaults();throw std::runtime_error("无法读取默认配置："+std::string(std::strerror(errno)));}
  struct stat info{};if(::fstat(fd,&info)||!S_ISREG(info.st_mode)||info.st_size<=0||info.st_size>16384){::close(fd);throw std::runtime_error("默认配置必须为不超过16KiB的普通有效文件");}
  std::string bytes;char buffer[4096];
  for(;;){const auto count=::read(fd,buffer,sizeof(buffer));if(count<0){if(errno==EINTR)continue;::close(fd);throw std::runtime_error("读取默认配置失败");}if(!count)break;bytes.append(buffer,static_cast<size_t>(count));if(bytes.size()>16384){::close(fd);throw std::runtime_error("默认配置超过16KiB");}}
  ::close(fd);return validate_tuning_defaults(json::parse(bytes));
}
inline json stable_tuning_defaults(const json&autonomy){
  const auto tuning=autonomy.value("tuning",json::object()),handoff=autonomy.value("handoff",json::object());
  if(tuning.value("phase",std::string())!="READY"||tuning.value("pending",true))throw std::runtime_error("参数尚未稳定读回，未保存默认配置");
  if(!tuning.value("ready",false)||!tuning.contains("age_seconds")||!tuning["age_seconds"].is_number())throw std::runtime_error("调参节点真实读回未就绪，未保存默认配置");
  const double age=tuning["age_seconds"].get<double>();
  if(!std::isfinite(age)||age<0||age>2.)throw std::runtime_error("调参节点真实读回已过期，未保存默认配置");
  const auto task=autonomy.value("task_id",std::string());
  if(task.empty()||tuning.value("task_id",std::string())!=task)throw std::runtime_error("调参读回不属于当前任务，未保存默认配置");
  if(handoff.value("phase",std::string())!="READY")throw std::runtime_error("提前衔接距离尚未稳定，未保存默认配置");
  return validate_tuning_defaults({{"version",1},{"values",tuning.at("confirmed")},{"handoff_distance_m",handoff.at("distance_m")}});
}
inline void save_tuning_defaults(const std::filesystem::path&value,const json&configuration){
  const auto checked=validate_tuning_defaults(configuration);
  const auto path=checked_tuning_path(value);
  // 已有不兼容文件必须留给操作者恢复，禁止用一次保存覆盖掉失败证据。
  (void)load_tuning_defaults(path);
  std::filesystem::create_directories(path.parent_path());(void)checked_tuning_path(path);
  const auto temporary=path.parent_path()/("."+path.filename().string()+".tmp-"+nonce());
  const int fd=::open(temporary.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
  if(fd<0)throw std::runtime_error("无法创建同目录默认配置临时文件");
  try{
    const auto bytes=checked.dump(2)+"\n";size_t written=0;
    while(written<bytes.size()){const auto count=::write(fd,bytes.data()+written,bytes.size()-written);if(count<0){if(errno==EINTR)continue;throw std::runtime_error("默认配置临时文件写入失败");}if(!count)throw std::runtime_error("默认配置写入未取得进展");written+=static_cast<size_t>(count);}
    if(::fsync(fd))throw std::runtime_error("默认配置临时文件同步失败");
    (void)checked_tuning_path(path);(void)load_tuning_defaults(path);
    if(::rename(temporary.c_str(),path.c_str()))throw std::runtime_error("默认配置原子替换失败");
    ::close(fd);
  }catch(...){::close(fd);std::error_code ignored;std::filesystem::remove(temporary,ignored);throw;}
}
}
