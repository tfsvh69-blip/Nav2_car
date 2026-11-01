#include <algorithm>
#include <fstream>
#include <iostream>
#include <sys/resource.h>
#include <filesystem>
#include <tbb/global_control.h>
#include <nlohmann/json.hpp>
#include "carcar_mapping_autonomy/graph_snapshot.hpp"
#include "carcar_mapping_autonomy/rasterization.hpp"
// 只读历史工程，不创建ROS节点或修改输入文件；独立进程隔离Karto全局传感器表。
int main(int argc,char **argv) {
  if(argc!=2&&argc!=3)return 2;
  const bool inspect=argc==3&&std::string(argv[1])=="--inspect";
  const std::string source=inspect?argv[2]:argv[1];
  try {
    const tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
    karto::Mapper mapper;karto::Dataset dataset;
    mapper.LoadFromFile(source+".posegraph");
    dataset.LoadFromFile(source+".data");
    for(auto * object:dataset.GetLasers())
      if(auto *sensor=dynamic_cast<karto::Sensor*>(object))karto::SensorManager::GetInstance()->RegisterSensor(sensor,true);
    if(inspect){
      const auto scans=mapper.GetAllProcessedScans();
      auto grid=carcar_mapping_autonomy::rasterize_scans(scans,.05,mapper.getParamMinPassThrough(),mapper.getParamOccupancyThreshold());
      if(!grid)throw std::runtime_error("恢复档案未能生成栅格");
      uint64_t hash=14695981039346656037ULL;size_t occupied=0;
      for(int y=0;y<grid->GetHeight();++y)for(int x=0;x<grid->GetWidth();++x){
        const auto value=*grid->GetDataPointer(karto::Vector2<int>(x,y));hash=(hash^value)*1099511628211ULL;
        if(value==karto::GridStates_Occupied)++occupied;
      }
      nlohmann::json poses=nlohmann::json::array();
      for(const auto*scan:scans){const auto&p=scan->GetCorrectedPose();poses.push_back({p.GetX(),p.GetY(),p.GetHeading()});}
      std::cout<<nlohmann::json{{"inspection",true},{"scans",scans.size()},{"data_scans",dataset.GetData().size()},
        {"width",grid->GetWidth()},{"height",grid->GetHeight()},{"occupied",occupied},
        {"grid_hash",std::to_string(hash)},{"poses",poses}}.dump()<<std::endl;
      return 0;
    }
    std::vector<double> samples,graph_times,data_times,prepare_times,seal_times;size_t bytes=0;
    for(int i=0;i<101;++i){auto prepared=carcar_mapping_autonomy::prepare_graph_snapshot();
      auto snapshot=carcar_mapping_autonomy::capture_graph(mapper,dataset,std::move(prepared));
      bytes=snapshot.graph->size()+snapshot.data->size();if(i){samples.push_back(snapshot.capture_ms);
        graph_times.push_back(snapshot.graph_ms);data_times.push_back(snapshot.data_ms);prepare_times.push_back(snapshot.prepare_ms);seal_times.push_back(snapshot.graph->finish_ms);}}
    std::sort(seal_times.begin(),seal_times.end());
    std::sort(prepare_times.begin(),prepare_times.end());
    std::sort(graph_times.begin(),graph_times.end());std::sort(data_times.begin(),data_times.end());
    std::sort(samples.begin(),samples.end());struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
    const bool passed=samples[98]<=100&&samples.back()<=200;
    if(argc==3){
      const std::string target=argv[2];
      if(std::filesystem::exists(target+".posegraph")||std::filesystem::exists(target+".data"))throw std::runtime_error("测试输出已存在，拒绝覆盖");
      auto prepared=carcar_mapping_autonomy::prepare_graph_snapshot();
      auto captured=carcar_mapping_autonomy::capture_graph(mapper,dataset,std::move(prepared));
      for(const auto&part:std::vector<std::pair<std::string,const carcar_mapping_autonomy::SnapshotMemoryBuffer*>>{
        {".posegraph",captured.graph.get()},{".data",captured.data.get()}}){
        std::ofstream output(target+part.first,std::ios::binary);const auto bytes=part.second->read();output.write(bytes.data(),bytes.size());
        output.close();if(!output)throw std::runtime_error("测试档案落盘失败");
      }
    }
    std::cout<<nlohmann::json{{"source",source},{"samples",samples.size()},{"bytes",bytes},
      {"p50_ms",samples[49]},{"p99_ms",samples[98]},{"max_ms",samples.back()},
      {"graph_p99_ms",graph_times[98]},{"data_p99_ms",data_times[98]},
      {"outside_lock_prepare_p99_ms",prepare_times[98]},
      {"graph_finalize_p99_ms",seal_times[98]},
      {"peak_rss_kib",usage.ru_maxrss},{"performance_passed",passed},{"hardware",false}}.dump()<<std::endl;
    return passed?0:1;
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}
}
