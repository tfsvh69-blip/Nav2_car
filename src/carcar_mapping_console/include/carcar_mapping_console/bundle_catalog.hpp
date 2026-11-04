#pragma once
#include <atomic>
#include <map>
#include <mutex>
#include <thread>
#include <sys/stat.h>
#include "carcar_mapping_console/publication.hpp"
namespace carcar_mapping_console {
// 目录校验在独立线程执行；HTTP请求只复制快照，避免大工程哈希阻塞停车和取消请求。
class SavedBundleCatalog {
public:
  explicit SavedBundleCatalog(std::filesystem::path root,std::chrono::milliseconds interval=std::chrono::milliseconds(250))
  :root_(std::move(root)),interval_(interval),worker_([this]{run();}){}
  ~SavedBundleCatalog(){running_=false;if(worker_.joinable())worker_.join();}
  json snapshot()const{std::lock_guard<std::mutex>lock(mutex_);return snapshot_;}
private:
  struct Entry{std::string signature;json value;};
  static std::string signature(const std::filesystem::path & directory){
    std::ostringstream out;
    for(const auto & name:{"map.yaml","map.pgm","graph.posegraph","graph.data","manifest.json","publication.json"}){
      struct stat value{};out<<name<<':';
      if(lstat((directory/name).c_str(),&value))out<<"missing;";
      else out<<value.st_dev<<','<<value.st_ino<<','<<value.st_mode<<','<<value.st_size<<','
        <<value.st_mtim.tv_sec<<','<<value.st_mtim.tv_nsec<<','<<value.st_ctim.tv_sec<<','<<value.st_ctim.tv_nsec<<';';
    }
    return out.str();
  }
  static json inspect(const std::filesystem::path & directory,const std::string & id){
    try{
      auto value=saved_map_status(directory/"map.yaml");
      const auto manifest=value.value("saved",false)?json::parse(read_file(directory/"manifest.json")):json::object();
      bool compatible=value.value("saved",false)&&value.value("navigation_allowed",false);std::string reason;
      if(compatible){try{validate_autonomy_manifest(manifest);}catch(const std::exception&e){compatible=false;reason=e.what();}}
      else reason=value.value("quality_reason",std::string());
      json publication;
      try{publication=bundle_publication(directory);}catch(const std::exception&e){publication={{"status","invalid"},{"error",e.what()}};}
      value.update({{"id",id},{"valid",value.value("saved",false)},{"source",manifest.value("source_bundle",std::string())},
        {"autonomy_compatible",compatible},{"autonomy_block_reason",reason},{"publication",publication},
        {"diagnostic_only",manifest.value("diagnostic_only",false)}});
      if(!value.value("saved",false))value["error"]=value.value("quality_reason",std::string("工程校验未通过"));
      return value;
    }catch(const std::exception&e){return {{"id",id},{"valid",false},{"saved",false},{"navigation_allowed",false},
      {"autonomy_compatible",false},{"quality","invalid"},{"quality_reason",e.what()},{"error",e.what()}};}
  }
  void run(){
    std::map<std::string,Entry> cache;uint64_t validations=0;
    while(running_){
      try{
        std::map<std::string,Entry> next;json items=json::array();
        if(std::filesystem::exists(root_))for(const auto & item:std::filesystem::directory_iterator(root_)){
          if(!running_)return;
          if(!item.is_directory()||item.is_symlink())continue;
          const auto id=item.path().filename().string(),key=signature(item.path());
          const auto old=cache.find(id);
          if(old!=cache.end()&&old->second.signature==key)next.emplace(id,old->second);
          else{
            auto value=inspect(item.path(),id);++validations;
            // 保存过程中的文件尚可能继续写入；签名变化的结果不进入稳定缓存。
            const auto after=signature(item.path());
            next.emplace(id,Entry{key==after?key:std::string(),std::move(value)});
          }
          items.push_back(next.at(id).value);
        }
        if(!running_)return;
        std::sort(items.begin(),items.end(),[](const json&a,const json&b){return a.at("id").get<std::string>()>b.at("id").get<std::string>();});
        cache=std::move(next);
        std::lock_guard<std::mutex>lock(mutex_);snapshot_={{"bundles",std::move(items)},{"loading",false},{"error",""},{"validations",validations}};
      }catch(const std::exception&e){std::lock_guard<std::mutex>lock(mutex_);snapshot_["error"]=std::string("已保存地图目录检查未完成：")+e.what();snapshot_["loading"]=false;}
      const auto until=std::chrono::steady_clock::now()+interval_;
      while(running_&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  std::filesystem::path root_;std::chrono::milliseconds interval_;std::atomic<bool>running_{true};
  mutable std::mutex mutex_;json snapshot_{{"bundles",json::array()},{"loading",true},{"error",""},{"validations",0}};
  std::thread worker_;
};
}  // namespace carcar_mapping_console
