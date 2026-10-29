// 已关闭录包导出的地图CDR与任务日志回放；不初始化ROS、不发布话题、不读取活动数据库。
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include "carcar_mapping_autonomy/observation_selector.hpp"
using namespace carcar_mapping_autonomy;
static nav_msgs::msg::OccupancyGrid read(const char*path){
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("无法打开地图CDR");
  std::vector<uint8_t>bytes{std::istreambuf_iterator<char>(f),{}};
  rclcpp::SerializedMessage serialized(bytes.size());auto&raw=serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
  nav_msgs::msg::OccupancyGrid m;rclcpp::Serialization<nav_msgs::msg::OccupancyGrid>().deserialize_message(&serialized,&m);return m;
}
int main(int argc,char**argv){
  if(argc!=5)return 2;
  try{
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(read(argv[1]));auto recorded=read(argv[2]);
    auto expected=planning_keepout_mask(*map);if(!expected||!same_grid(*expected,recorded))throw std::runtime_error("当前地图生成的掩膜与现场掩膜不一致");
    ObservationSelector::Search search;ObservationSelector::Job job;job.map=map;job.revision=1;job.current={.01,0,0};
    Json result={{"hardware",false},{"mask_matches_recording",true},{"coordinate_note","初始车位map约[0.01,0]；前沿使用历史日志三位小数，只回放首次下发前的静态地图"},{"candidates",Json::array()}};
    const std::regex candidate(R"(dispatch=(\d+) frontier=\[([^,]+),([^\]]+)\])");
    std::ifstream log(argv[3]);if(!log)throw std::runtime_error("无法打开任务日志");std::string line;
    size_t rejected=0,changed=0,eligible=0;
    for(;std::getline(log,line);){
      if(line.find("event=dispatch ")!=std::string::npos)break;
      std::smatch match;if(!std::regex_search(line,match,candidate))continue;
      job.frontier={std::stod(match[2]),std::stod(match[3]),0};
      const auto begin=std::chrono::steady_clock::now();const auto selected=search.select(job);
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      const auto baseline=safe_observation(*map,job.frontier,job.task);
      Json item={{"dispatch",std::stoul(match[1])},{"frontier",{job.frontier.x,job.frontier.y}},{"selection_ms",ms}};
      item["baseline"]=baseline?Json{baseline->x,baseline->y}:Json(nullptr);
      item["reachable"]=selected?Json{selected->x,selected->y}:Json(nullptr);
      const bool old_precheck=baseline&&std::hypot(baseline->x-job.current.x,baseline->y-job.current.y)>.25;
      const bool new_precheck=selected&&std::hypot(selected->x-job.current.x,selected->y-job.current.y)>.25;
      item["baseline_precheck"]=old_precheck;item["reachable_precheck"]=new_precheck;
      if(old_precheck&&!new_precheck)++rejected;
      if(new_precheck){++eligible;if(old_precheck&&std::hypot(selected->x-baseline->x,selected->y-baseline->y)>.01)++changed;}
      result["candidates"].push_back(std::move(item));
    }
    result["previously_selected_now_rejected"]=rejected;result["reachable_alternatives"]=changed;result["eligible_candidates"]=eligible;
    if(result["candidates"].empty())throw std::runtime_error("没有可回放的历史候选");
    std::ofstream out(argv[4]);out<<result.dump(2)<<'\n';if(!out)throw std::runtime_error("无法保存回放结果");
    std::cout<<"候选="<<result["candidates"].size()<<" 静态断路拒绝="<<rejected<<" 可达替代="<<changed<<" 保留候选="<<eligible<<'\n';
    return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
