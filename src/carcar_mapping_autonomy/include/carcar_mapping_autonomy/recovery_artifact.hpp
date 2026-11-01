#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <openssl/evp.h>
#include <nlohmann/json.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace carcar_mapping_autonomy {
inline std::string recovery_sha256(const std::filesystem::path&path){
  std::ifstream input(path,std::ios::binary);if(!input)throw std::runtime_error("检查点文件缺失");
  auto*ctx=EVP_MD_CTX_new();if(!ctx)throw std::runtime_error("检查点摘要初始化失败");
  EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr);char buffer[65536];
  while(input){input.read(buffer,sizeof(buffer));EVP_DigestUpdate(ctx,buffer,input.gcount());}
  const bool bad=input.bad();unsigned char digest[EVP_MAX_MD_SIZE];unsigned count=0;
  EVP_DigestFinal_ex(ctx,digest,&count);EVP_MD_CTX_free(ctx);
  if(bad)throw std::runtime_error("检查点文件读取未完成");
  std::ostringstream text;for(unsigned i=0;i<count;++i)text<<std::hex<<std::setfill('0')<<std::setw(2)<<int(digest[i]);return text.str();
}
inline void recovery_write_map(const std::filesystem::path&directory,const nav_msgs::msg::OccupancyGrid&m){
  const size_t n=static_cast<size_t>(m.info.width)*m.info.height;
  if(!n||n!=m.data.size())throw std::runtime_error("检查点栅格尺寸不完整");
  std::ofstream pgm(directory/"map.pgm",std::ios::binary);
  pgm<<"P5\n"<<m.info.width<<' '<<m.info.height<<"\n255\n";
  for(int y=static_cast<int>(m.info.height)-1;y>=0;--y)for(unsigned x=0;x<m.info.width;++x){
    const int v=m.data[static_cast<size_t>(y)*m.info.width+x];
    const unsigned char pixel=v<0?205:(v>=65?0:254);pgm.write(reinterpret_cast<const char*>(&pixel),1);
  }
  pgm.close();if(!pgm)throw std::runtime_error("检查点栅格写入未完成");
  std::ofstream yaml(directory/"map.yaml");
  yaml<<std::setprecision(16)<<"image: map.pgm\nmode: trinary\nresolution: "<<m.info.resolution
    <<"\norigin: ["<<m.info.origin.position.x<<", "<<m.info.origin.position.y
    <<", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  yaml.close();if(!yaml)throw std::runtime_error("检查点元数据写入未完成");
}
inline nlohmann::json recovery_validate_artifact(const std::filesystem::path&directory){
  if(std::filesystem::is_symlink(directory))throw std::runtime_error("检查点目录不能是符号链接");
  std::ifstream stream(directory/"manifest.json");const auto manifest=nlohmann::json::parse(stream);
  if(!manifest.value("complete",false)||manifest.value("slam_version","")!="2.6.10"||
     manifest.value("base_frame","")!="base_footprint")throw std::runtime_error("检查点版本或完整性不匹配");
  for(const auto*name:{"graph.posegraph","graph.data","map.pgm","map.yaml"}){
    const auto p=directory/name;
    if(std::filesystem::is_symlink(p)||!std::filesystem::is_regular_file(p)||!std::filesystem::file_size(p)||
      manifest.at("sha256").at(name).get<std::string>()!=recovery_sha256(p))
      throw std::runtime_error("检查点校验失败："+std::string(name));
  }
  return manifest;
}
} // namespace carcar_mapping_autonomy
