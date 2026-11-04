#pragma once
#include <zip.h>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include "carcar_inspection/domain.hpp"
#include <carcar_mapping_console/common.hpp>
namespace carcar_inspection {
// HTTP工作线程生成归档；不占用ROS控制循环，也不修改原始照片。
inline std::vector<uint8_t> run_archive(const std::filesystem::path & root,const json & run) {
  static std::mutex mutex;
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
  require(lock.owns_lock(),"另一份归档正在生成，请稍后重试");
  const auto status=run.value("status","");
  require(status=="SUCCEEDED"||status=="FAILED"||status=="CANCELED"||status=="INTERRUPTED"||status=="LOCKED","本轮尚未结束，请结束后下载整轮照片");
  auto captures=run.value("captures",json::object());require(captures.size()<=100,"归档照片数量超限");
  uint64_t bytes=0;
  for(const auto & photo:captures){const auto id=photo.at("id").get<std::string>();valid_id(id);
    const auto path=root/"media"/(id+".jpg");
    require(!std::filesystem::is_symlink(path)&&std::filesystem::is_regular_file(path),"归档照片不存在或路径异常");
    require(photo.contains("sha256")&&carcar_mapping_console::sha256(path)==photo.at("sha256"),"原图校验失败，未生成不完整归档");
    bytes+=std::filesystem::file_size(path);
  }
  require(bytes<=128ULL*1024*1024,"本轮照片超过128MiB，请使用单张下载");
  require(std::filesystem::space(root).available>20ULL*1024*1024*1024+bytes+1024*1024,"归档空间不足20GiB预留及本次写入余量");
  std::string pattern=(root/".export-XXXXXX").string();std::vector<char> name(pattern.begin(),pattern.end());name.push_back(0);
  int fd=mkstemp(name.data());require(fd>=0,"无法创建归档临时文件");close(fd);
  struct Cleanup {std::filesystem::path path;~Cleanup(){std::error_code ec;std::filesystem::remove(path,ec);}} cleanup{name.data()};
  int error=0;zip_t * archive=zip_open(name.data(),ZIP_CREATE|ZIP_TRUNCATE,&error);require(archive!=nullptr,"无法创建ZIP归档");
  try {
    const auto manifest=run.dump(2);
    auto * source=zip_source_buffer(archive,manifest.data(),manifest.size(),0);require(source!=nullptr,"归档清单创建失败");
    if(zip_file_add(archive,"manifest.json",source,ZIP_FL_ENC_UTF_8)<0){zip_source_free(source);throw std::runtime_error("归档清单写入失败");}
    for(const auto & photo:captures){const auto id=photo.at("id").get<std::string>();const auto path=root/"media"/(id+".jpg");
      auto * file=zip_source_file(archive,path.c_str(),0,0);require(file!=nullptr,"归档照片读取失败");
      const auto index=zip_file_add(archive,("photos/"+id+".jpg").c_str(),file,ZIP_FL_ENC_UTF_8);
      if(index<0){zip_source_free(file);throw std::runtime_error("归档照片添加失败");}
      require(zip_set_file_compression(archive,index,ZIP_CM_STORE,0)==0,"归档压缩设置失败");
    }
    require(zip_close(archive)==0,"归档落盘失败");archive=nullptr;
  }catch(...){if(archive)zip_discard(archive);throw;}
  std::ifstream input(cleanup.path,std::ios::binary);
  return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
}
}
