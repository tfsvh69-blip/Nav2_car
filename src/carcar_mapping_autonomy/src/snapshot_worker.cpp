#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <tbb/global_control.h>
#include <karto_sdk/Mapper.h>
#include "carcar_mapping_autonomy/rasterization.hpp"
#include "carcar_mapping_autonomy/recovery_artifact.hpp"
#include "carcar_mapping_autonomy/recovery_core.hpp"
#include "carcar_mapping_autonomy/snapshot_protocol.hpp"
namespace ca=carcar_mapping_autonomy;
using Json=nlohmann::json;
namespace {
constexpr int graph_fd=198,data_fd=199,report_fd=197;
void report(const Json&state){const auto text=state.dump()+"\n";size_t offset=0;while(offset<text.size()){
  const auto count=write(report_fd,text.data()+offset,text.size()-offset);if(count<=0)return;offset+=size_t(count);}}
void sync_file(const std::filesystem::path&path){const int fd=open(path.c_str(),O_RDONLY|O_CLOEXEC);
  if(fd<0)throw std::runtime_error("快照文件无法同步");
  const int result=fsync(fd);close(fd);if(result)throw std::runtime_error("快照文件同步失败");}
void copy_archive(int source,const std::filesystem::path&path){
  struct stat st{};if(fstat(source,&st)||st.st_size<=0||st.st_size>256*1024*1024)throw std::runtime_error("快照内存档案长度无效");
  std::ofstream output(path,std::ios::binary);char buffer[65536];off_t offset=0;
  while(offset<st.st_size){const auto count=pread(source,buffer,std::min<off_t>(sizeof(buffer),st.st_size-offset),offset);
    if(count<=0)throw std::runtime_error("快照内存档案读取失败");
    output.write(buffer,count);offset+=count;}
  output.close();if(!output)throw std::runtime_error("快照档案写盘失败");sync_file(path);
}
nav_msgs::msg::OccupancyGrid make_map(const karto::OccupancyGrid&grid,const Json&meta){
  nav_msgs::msg::OccupancyGrid result;result.header.frame_id="map";
  const int64_t stamp=meta.at("capture_stamp_ns");result.header.stamp.sec=stamp/1000000000;result.header.stamp.nanosec=stamp%1000000000;
  result.info.resolution=meta.at("resolution");result.info.width=grid.GetWidth();result.info.height=grid.GetHeight();
  result.info.origin.orientation.w=1;const auto offset=grid.GetCoordinateConverter()->GetOffset();
  result.info.origin.position.x=offset.GetX();result.info.origin.position.y=offset.GetY();
  result.data.resize(size_t(result.info.width)*result.info.height);
  for(unsigned y=0;y<result.info.height;++y)for(unsigned x=0;x<result.info.width;++x){const auto value=grid.GetValue(karto::Vector2<int>(x,y));
    result.data[size_t(y)*result.info.width+x]=value==karto::GridStates_Occupied?100:(value==karto::GridStates_Free?0:-1);}
  return result;
}
}
// 独立进程只读封存档案；不连接ROS、不接触生产图或发布运动命令。
int main(int argc,char**argv){
  try{
    if(argc!=3)throw std::runtime_error("快照工作进程参数无效");
    auto meta=Json::parse(argv[2]);ca::validate_snapshot_request(meta);
    const std::filesystem::path root=argv[1];if(!root.is_absolute())throw std::runtime_error("快照根目录必须为绝对路径");
    const auto task=root/meta.at("task_id").get<std::string>();std::filesystem::create_directories(task);
    if(std::filesystem::is_symlink(task))throw std::runtime_error("快照任务目录不能是符号链接");
    const auto final=task/meta.at("request_id").get<std::string>();const auto partial=std::filesystem::path(final.string()+".partial");
    if(std::filesystem::exists(final)||!std::filesystem::create_directory(partial))throw std::runtime_error("快照事务目录已存在，拒绝覆盖");
    report({{"phase","WRITING"}});copy_archive(graph_fd,partial/"graph.posegraph");copy_archive(data_fd,partial/"graph.data");
    const tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
    karto::Mapper mapper;karto::Dataset dataset;
    mapper.LoadFromFile("/proc/self/fd/198");dataset.LoadFromFile("/proc/self/fd/199");
    for(auto*object:dataset.GetLasers())if(auto*sensor=dynamic_cast<karto::Sensor*>(object))karto::SensorManager::GetInstance()->RegisterSensor(sensor,true);
    report({{"phase","RASTERIZING"}});
    const auto scans=mapper.GetAllProcessedScans();auto*matched=static_cast<karto::LocalizedRangeScan*>(nullptr);
    for(auto*scan:scans)if(scan&&scan->GetUniqueId()==meta.at("scan_id").get<int>())matched=scan;
    if(!matched)throw std::runtime_error("捕获扫描不在封存位姿图内");
    auto grid=ca::rasterize_scans(scans,meta.at("resolution"),meta.at("min_pass_through"),meta.at("occupancy_threshold"));
    if(!grid)throw std::runtime_error("封存图不能生成有效栅格");
    auto map=make_map(*grid,meta);report({{"phase","VALIDATING"}});
    const auto base=matched->GetCorrectedPose(),sensor=matched->GetSensorPose();
    const auto pose=meta.at("scan_pose");
    if(pose.size()!=3||std::hypot(base.GetX()-pose.at(0).get<double>(),base.GetY()-pose.at(1).get<double>())>1e-8||
      std::abs(std::remainder(base.GetHeading()-pose.at(2).get<double>(),2*M_PI))>1e-8)throw std::runtime_error("快照扫描位姿与捕获元数据不一致");
    const double c=std::cos(base.GetHeading()),s=std::sin(base.GetHeading());
    const double ox=c*(sensor.GetX()-base.GetX())+s*(sensor.GetY()-base.GetY());
    const double oy=-s*(sensor.GetX()-base.GetX())+c*(sensor.GetY()-base.GetY());
    std::vector<ca::ReferenceMatcher::Ray> rays;const double max_range=meta.at("mapping_range_m");
    for(const auto&p:matched->GetPointReadings(false)){
      const double range=std::hypot(p.GetX()-sensor.GetX(),p.GetY()-sensor.GetY());if(!std::isfinite(range)||range<.05||range>=max_range)continue;
      rays.push_back({c*(p.GetX()-base.GetX())+s*(p.GetY()-base.GetY()),-s*(p.GetX()-base.GetX())+c*(p.GetY()-base.GetY()),ox,oy});
    }
    ca::ReferenceMatcher matcher(map);const auto geometry=matcher.score({base.GetX(),base.GetY(),base.GetHeading()},rays);
    const Json score{{"points",geometry.points},{"mean_residual_m",geometry.mean},{"coverage",geometry.coverage},{"hit_ratio",geometry.hit_ratio},{"free_conflict",geometry.free_conflict}};
    if(!geometry.acceptable())throw std::runtime_error("同一捕获状态的激光与栅格校验不通过："+score.dump());
    ca::recovery_write_map(partial,map);meta["complete"]=true;meta["slam_version"]="2.6.10";meta["base_frame"]="base_footprint";
    meta["pose"]={base.GetX(),base.GetY(),base.GetHeading()};meta["map_stamp_ns"]=meta.at("capture_stamp_ns");meta["geometry"]=score;
    for(const auto*name:{"graph.posegraph","graph.data","map.pgm","map.yaml"}){sync_file(partial/name);meta["sha256"][name]=ca::recovery_sha256(partial/name);}
    {std::ofstream output(partial/"manifest.json");output<<meta.dump(2)<<'\n';output.close();if(!output)throw std::runtime_error("快照manifest写盘失败");}
    sync_file(partial/"manifest.json");ca::recovery_validate_artifact(partial);sync_file(partial);
    if(syscall(SYS_renameat2,AT_FDCWD,partial.c_str(),AT_FDCWD,final.c_str(),RENAME_NOREPLACE))throw std::runtime_error("快照原子发布失败，旧快照保留");
    sync_file(task);report({{"phase","READY"},{"path",final.string()},{"manifest",meta}});return 0;
  }catch(const std::exception&e){report({{"phase","FAILED"},{"reason",e.what()}});return 1;}
}
