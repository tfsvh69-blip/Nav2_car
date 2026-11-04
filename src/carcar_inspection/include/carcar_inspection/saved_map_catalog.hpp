#pragma once
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>

namespace carcar_inspection {
// 后台维护完整地图目录；前台只复制快照，不执行地图数据校验。
class SavedMapCatalog {
public:
  using json=nlohmann::json;
  struct Snapshot {
    json maps=json::array();
    std::map<std::string,std::filesystem::path> paths,images;
    std::map<std::string,std::string> signatures;
  };
  explicit SavedMapCatalog(std::filesystem::path root);
  ~SavedMapCatalog();
  void stop();
  Snapshot snapshot() const;
  static bool unchanged(const std::filesystem::path & yaml,const std::filesystem::path & image,const std::string & expected);
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace carcar_inspection
