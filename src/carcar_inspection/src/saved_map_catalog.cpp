#include "carcar_inspection/saved_map_catalog.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <mutex>
#include <thread>
#include <string>
#include <utility>
#include <vector>
#include <cerrno>
#include <sys/stat.h>
#include <nav2_map_server/map_io.hpp>
#include "carcar_mapping_console/publication.hpp"
#include "carcar_inspection/domain.hpp"

namespace carcar_inspection {
// 地图校验放在后台；控制循环只读取目录快照，不承担位姿图SHA计算。
class SavedMapCatalog::Impl {
public:
  using json=carcar_mapping_console::json;
  explicit Impl(std::filesystem::path root):root_(std::move(root)) {
    refresh();
    worker_=std::thread([this]{
      std::unique_lock<std::mutex> lock(wait_mutex_);
      while(!wake_.wait_for(lock,std::chrono::seconds(1),[this]{return stopped_;})) {
        lock.unlock();refresh();lock.lock();
      }
    });
  }
  ~Impl(){stop();}
  void stop() {
    {std::lock_guard<std::mutex> lock(wait_mutex_);stopped_=true;}
    wake_.notify_all();if(worker_.joinable())worker_.join();
  }
  Snapshot snapshot() const {std::lock_guard<std::mutex> lock(mutex_);return snapshot_;}
  static bool unchanged(const std::filesystem::path & yaml,const std::filesystem::path & image,const std::string & expected) {
    try{return signature(yaml,image)==expected;}catch(const std::exception &){return false;}
  }
private:
  struct Cached {std::string signature,yaml_signature;std::filesystem::path image;json status;};
  static std::string metadata(const std::filesystem::path & file) {
    struct stat value{};
    if(::lstat(file.c_str(),&value)!=0) {
      if(errno==ENOENT||errno==ENOTDIR)return file.string()+":missing";
      throw std::filesystem::filesystem_error("地图文件元信息读取失败",file,std::error_code(errno,std::generic_category()));
    }
    // 原子替换或保留mtime的恢复也必须失效；元信息检查不会读取地图内容。
    return file.string()+":"+std::to_string(value.st_dev)+":"+std::to_string(value.st_ino)+":"+std::to_string(value.st_mode)+
      ":"+std::to_string(value.st_size)+":"+std::to_string(value.st_mtim.tv_sec)+":"+std::to_string(value.st_mtim.tv_nsec)+
      ":"+std::to_string(value.st_ctim.tv_sec)+":"+std::to_string(value.st_ctim.tv_nsec);
  }
  // 内容或链接发生变化时重新验证；未变化的工程不重复读取大型图数据。
  static std::string signature(const std::filesystem::path & yaml,const std::filesystem::path & image) {
    std::vector<std::string> parts;
    // 只跟踪本图资产，旁边另一张地图的新增或保存不能让当前地图失效。
    const std::vector<std::filesystem::path> files{yaml,image,yaml.parent_path()/"manifest.json",yaml.parent_path()/"publication.json",
      yaml.parent_path()/"graph.data",yaml.parent_path()/"graph.posegraph"};
    for(const auto & file:files) {
      parts.push_back(metadata(file));
    }
    std::sort(parts.begin(),parts.end());std::string result;
    for(const auto & part:parts)result+=part+"\n";
    return result;
  }
  void refresh() {
    Snapshot next;std::map<std::string,Cached> cache;
    auto add=[&](const std::filesystem::path & yaml,const std::string & id,bool session) {
      try {
        if(!std::filesystem::is_regular_file(yaml)||std::filesystem::is_symlink(yaml))return;
        valid_id(id);
        const auto key=yaml.string(),yaml_stamp=metadata(yaml);
        const auto old=cache_.find(key);
        const auto image=old!=cache_.end()&&old->second.yaml_signature==yaml_stamp?old->second.image:
          std::filesystem::path(nav2_map_server::loadMapYaml(yaml.string()).image_file_name);
        if(!std::filesystem::is_regular_file(image)||std::filesystem::is_symlink(image)||std::filesystem::file_size(image)==0)return;
        const auto stamp=signature(yaml,image);
        const auto status=old!=cache_.end()&&old->second.signature==stamp?old->second.status:
          carcar_mapping_console::saved_map_status(yaml);
        cache[key]={stamp,yaml_stamp,image,status};
        if(!status.value("saved",false))return;
        const auto quality=status.value("quality",std::string("normal"));
        const auto suffix=quality=="diagnostic"?" · 建图异常，可查看":quality=="rejected"?" · 已标记不可用，可查看":quality=="invalid"?" · 状态异常，可查看":"";
        next.maps.push_back({{"id",id},{"name",id+suffix},{"saved",true},{"navigation_allowed",status.value("navigation_allowed",false)},
          {"quality",quality},{"quality_reason",status.value("quality_reason",std::string())},{"session",session}});
        next.paths[id]=yaml;next.images[id]=image;next.signatures[id]=stamp;
      }catch(const std::exception &){/* 写入中的文件下轮重试，不影响车辆控制。 */}
    };
    try {
      if(std::filesystem::is_directory(root_)) {
        for(const auto & entry:std::filesystem::directory_iterator(root_))
          if(entry.is_regular_file()&&entry.path().extension()==".yaml")add(entry.path(),entry.path().stem(),false);
        const auto sessions=root_/"mapping_sessions";
        if(std::filesystem::is_directory(sessions))for(const auto & entry:std::filesystem::directory_iterator(sessions))
          if(entry.is_directory()&&!entry.is_symlink())add(entry.path()/"map.yaml",entry.path().filename(),true);
      }
      std::sort(next.maps.begin(),next.maps.end(),[](const json & a,const json & b){
        if(a.at("session")!=b.at("session"))return a.at("session").get<bool>();
        return a.at("id").get<std::string>()>b.at("id").get<std::string>();
      });
      cache_=std::move(cache);
      std::lock_guard<std::mutex> lock(mutex_);snapshot_=std::move(next);
    }catch(const std::exception &){/* 短时目录异常保留最后快照，下轮继续检查。 */}
  }
  std::filesystem::path root_;std::map<std::string,Cached> cache_;Snapshot snapshot_;
  mutable std::mutex mutex_;std::mutex wait_mutex_;std::condition_variable wake_;bool stopped_=false;std::thread worker_;
};
SavedMapCatalog::SavedMapCatalog(std::filesystem::path root):impl_(std::make_unique<Impl>(std::move(root))){}
SavedMapCatalog::~SavedMapCatalog()=default;
void SavedMapCatalog::stop(){impl_->stop();}
SavedMapCatalog::Snapshot SavedMapCatalog::snapshot() const{return impl_->snapshot();}
bool SavedMapCatalog::unchanged(const std::filesystem::path & yaml,const std::filesystem::path & image,const std::string & expected){
  return Impl::unchanged(yaml,image,expected);
}
}  // namespace carcar_inspection
