#pragma once
#include <ctime>
#include "carcar_mapping_console/common.hpp"
namespace carcar_mapping_console {
inline std::filesystem::path validate_autonomy_bundle(const std::filesystem::path&root,const std::string&id){
  const auto directory=validate_bundle(root,id);
  const auto manifest=json::parse(read_file(directory/"manifest.json"));
  if(manifest.value("diagnostic_only",false))throw std::runtime_error("诊断工程不能继续写图；请选择人工核对可用的已保存工程");
  double range=manifest.value("range_m",0.0),resolution=manifest.value("resolution",0.0);
  std::string loss="None";
  if(manifest.contains("runtime_parameters")){
    const auto&p=manifest.at("runtime_parameters");
    if(p.contains("max_laser_range"))range=std::stod(p.at("max_laser_range").get<std::string>());
    if(p.contains("resolution"))resolution=std::stod(p.at("resolution").get<std::string>());
    loss=p.value("ceres_loss_function",std::string("None"));
  }
  if(!std::isfinite(range)||!std::isfinite(resolution)||std::abs(range-3.0)>1e-4||std::abs(resolution-.05)>1e-6||loss!="None")
    throw std::runtime_error("工程与本轮3米／5厘米／原损失基线不兼容或缺少参数；保留原入口核对，不混用专项候选");
  return directory;
}
// 状态与资产哈希分离；发布从不修改地图和位姿图。临时文件+rename避免半写状态。
inline void atomic_json(const std::filesystem::path & path,const json & value) {
  const auto temporary=path.string()+".tmp-"+nonce();
  {std::ofstream out(temporary,std::ios::binary);out<<value.dump(2)<<'\n';out.flush();
    if(!out)throw std::runtime_error("状态写入失败");}
  std::filesystem::rename(temporary,path);
}
inline json bundle_publication(const std::filesystem::path & directory) {
  const auto file=directory/"publication.json";
  if(!std::filesystem::exists(file))return {{"status","legacy"}};
  if(std::filesystem::is_symlink(file))throw std::runtime_error("发布状态不接受符号链接");
  return json::parse(read_file(file));
}
inline bool navigation_map_visible(const std::filesystem::path & yaml) {
  try {
    const auto state=bundle_publication(yaml.parent_path());
    if(state.value("status","")=="legacy"){
      const auto manifest_file=yaml.parent_path()/"manifest.json";
      if(std::filesystem::exists(manifest_file)&&json::parse(read_file(manifest_file)).value("review_required",false))return false;
      return true; // 已有接口保留，不隐式启用任何旧图。
    }
    if(state.value("status","")!="published")return false;
    const auto directory=validate_bundle(yaml.parent_path().parent_path(),yaml.parent_path().filename());
    const auto manifest=json::parse(read_file(directory/"manifest.json"));
    return state.at("asset_sha256")==manifest.at("sha256");
  }catch(...){return false;}
}
// 手动中途保存可以继续建图。自动候选和故障诊断保存后必须保持冻结，避免保存瞬间把新扫描写进已冻结的位姿图。
inline bool resume_scan_forwarding(bool finish_task,bool diagnostic,bool keep_writing){
  return keep_writing&&!finish_task&&!diagnostic;
}
inline json review_bundle(const std::filesystem::path & root,const std::string & id,bool approve,const std::string & note) {
  const auto directory=validate_bundle(root,id);
  auto manifest=json::parse(read_file(directory/"manifest.json"));
  if(approve&&manifest.value("diagnostic_only",false))throw std::runtime_error("故障快照只能诊断，不允许发布；请选可信工程重新建图");
  if(note.empty()||note.size()>1000)throw std::runtime_error("请填写现场核对结论（1～1000字节）");
  json state{{"status",approve?"published":"rejected"},{"review_note",note},{"asset_sha256",manifest.at("sha256")},
    {"reviewed_unix",std::time(nullptr)},{"bundle",id}};
  atomic_json(directory/"publication.json",state);return state;
}
}  // namespace carcar_mapping_console
