#include <fstream>
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tbb/global_control.h>
#include <karto_sdk/Mapper.h>
#include "carcar_mapping_autonomy/rasterization.hpp"
#include "carcar_mapping_autonomy/recovery_core.hpp"
#include "carcar_mapping_autonomy/loop_guard.hpp"
#include "carcar_mapping_autonomy/recovery_artifact.hpp"
using Json=nlohmann::json;
namespace ca=carcar_mapping_autonomy;
// 真实Karto/Ceres离线回放；隔离ROS域仅供插件参数使用，不订阅或发布硬件话题。
static Json pose(const karto::Pose2&p){return {p.GetX(),p.GetY(),p.GetHeading()};}
static void sensors(karto::Dataset&dataset){for(auto*object:dataset.GetLasers())if(auto*s=dynamic_cast<karto::Sensor*>(object))karto::SensorManager::GetInstance()->RegisterSensor(s,true);}
static nav_msgs::msg::OccupancyGrid grid(karto::Mapper&mapper){
  auto g=ca::rasterize_scans(mapper.GetAllProcessedScans(),.05,mapper.getParamMinPassThrough(),mapper.getParamOccupancyThreshold());
  nav_msgs::msg::OccupancyGrid m;m.info.width=g->GetWidth();m.info.height=g->GetHeight();m.info.resolution=.05;
  auto origin=g->GetCoordinateConverter()->GetOffset();m.info.origin.position.x=origin.GetX();m.info.origin.position.y=origin.GetY();m.info.origin.orientation.w=1;
  for(int y=0;y<g->GetHeight();++y)for(int x=0;x<g->GetWidth();++x){auto v=*g->GetDataPointer(karto::Vector2<int>(x,y));m.data.push_back(v==karto::GridStates_Occupied?100:v==karto::GridStates_Free?0:-1);}
  return m;
}
static Json score(karto::Mapper&mapper,karto::LocalizedRangeScan*scan){
  ca::ReferenceMatcher matcher(grid(mapper));const auto p=scan->GetCorrectedPose();const auto offset=scan->GetLaserRangeFinder()->GetOffsetPose();const auto readings=scan->GetRangeReadingsVector();
  const auto*laser=scan->GetLaserRangeFinder();std::vector<ca::ReferenceMatcher::Ray> rays;
  for(size_t i=0;i<readings.size();i+=std::max<size_t>(1,readings.size()/180)){
    const auto r=readings[i];if(!std::isfinite(r)||r<.15||r>laser->GetRangeThreshold()-.05)continue;
    const auto a=laser->GetMinimumAngle()+i*laser->GetAngularResolution()+offset.GetHeading();rays.push_back({offset.GetX()+r*std::cos(a),offset.GetY()+r*std::sin(a),offset.GetX(),offset.GetY()});}
  const auto s=matcher.score({p.GetX(),p.GetY(),p.GetHeading()},rays);return {{"hit",s.hit_ratio},{"mean",s.mean},{"coverage",s.coverage},{"acceptable",s.acceptable()}};
}
class Listener:public karto::MapperLoopClosureListener{
public:
  explicit Listener(karto::Mapper&m):mapper(m){}
  void LoopClosureCheck(const std::string&s)override{std::cout<<Json{{"loop_check",s}}<<std::endl;}
  void BeginLoopClosure(const std::string&)override{if(solver)guard.begin(mapper);std::cout<<Json{{"loop_before",score(mapper,current)},{"pose",pose(current->GetCorrectedPose())}}<<std::endl;}
  void EndLoopClosure(const std::string&)override{++loops;if(solver){guard.finish(mapper,*solver);std::cout<<guard.state()<<std::endl;}auto s=score(mapper,current);if(!s["acceptable"].get<bool>())bad=true;std::cout<<Json{{"loop_after",s},{"pose",pose(current->GetCorrectedPose())}}<<std::endl;}
  karto::Mapper&mapper;karto::LocalizedRangeScan*current=nullptr;unsigned loops=0;bool bad=false;
  ca::LoopGuard guard;karto::ScanSolver*solver=nullptr;
};
int main(int argc,char**argv){
  if(argc<4)return 2;
  const tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
  const std::string mode=argv[1],source=argv[2];
  if(mode=="dump"){
    karto::Mapper mapper;karto::Dataset dataset;mapper.LoadFromFile(source+".posegraph");dataset.LoadFromFile(source+".data");sensors(dataset);
    Json out=Json::array();for(auto*s:mapper.GetAllProcessedScans())out.push_back({{"id",s->GetUniqueId()},{"time",s->GetTime()},{"odom",pose(s->GetOdometricPose())},{"corrected",pose(s->GetCorrectedPose())},{"ranges",s->GetRangeReadingsVector()},{"sensor",s->GetSensorName().ToString()}});
    std::ofstream(argv[3])<<out.dump();
    if(argc>4){const std::filesystem::path output=argv[4];if(std::filesystem::exists(output))return 4;std::filesystem::create_directories(output);ca::recovery_write_map(output,grid(mapper));}
    return 0;
  }
  if(!std::getenv("ROS_DOMAIN_ID")||std::string(std::getenv("ROS_DOMAIN_ID"))!="208"||!std::getenv("ROS_LOCALHOST_ONLY")||std::string(std::getenv("ROS_LOCALHOST_ONLY"))!="1")return 3;
  rclcpp::init(0,nullptr);auto node=std::make_shared<rclcpp::Node>("recorded_graph_replay");node->declare_parameter("debug_logging",false);
  pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox","karto::ScanSolver");auto solver=loader.createSharedInstance("solver_plugins::CeresSolver");solver->Configure(node);
  karto::Mapper mapper;karto::Dataset dataset;mapper.LoadFromFile(source+".posegraph");dataset.LoadFromFile(source+".data");sensors(dataset);
  for(const auto&group:mapper.GetGraph()->GetVertices())for(const auto&entry:group.second)if(entry.second)solver->AddNode(entry.second);
  for(auto*edge:mapper.GetGraph()->GetEdges())if(edge)solver->AddConstraint(edge);
  mapper.SetScanSolver(solver.get());if(mode=="no-loop")mapper.setParamDoLoopClosing(false);
  Listener listener(mapper);if(mode=="guard")listener.solver=solver.get();mapper.AddListener(&listener);
  std::ifstream file(argv[3]);const auto inputs=Json::parse(file);int last=-1;for(auto*s:mapper.GetAllProcessedScans())last=std::max(last,s->GetUniqueId());
  for(const auto&j:inputs){if(j["id"].get<int>()<=last)continue;
    karto::RangeReadingsVector ranges;for(const auto&v:j["ranges"])ranges.push_back(v.is_null()?NAN:v.get<double>());
    auto*scan=new karto::LocalizedRangeScan(karto::Name(j["sensor"].get<std::string>()),ranges);scan->SetTime(j["time"]);
    const auto&p=j["odom"];scan->SetOdometricPose(karto::Pose2(p[0],p[1],p[2]));scan->SetCorrectedPose(scan->GetOdometricPose());listener.current=scan;
    if(mapper.Process(scan)){dataset.Add(scan);std::cout<<Json{{"id",scan->GetUniqueId()},{"pose",pose(scan->GetCorrectedPose())},{"geometry",score(mapper,scan)}}<<std::endl;}else delete scan;
  }
  mapper.RemoveListener(&listener);
  if(argc>4){
    const std::filesystem::path output=argv[4];
    if(std::filesystem::exists(output))throw std::runtime_error("回放输出目录已存在，拒绝覆盖");
    std::filesystem::create_directories(output);mapper.SaveToFile((output/"graph.posegraph").string());dataset.SaveToFile((output/"graph.data").string());
    ca::recovery_write_map(output,grid(mapper));
    Json poses=Json::array();for(auto*s:mapper.GetAllProcessedScans())poses.push_back({{"id",s->GetUniqueId()},{"pose",pose(s->GetCorrectedPose())}});
    std::ofstream(output/"replay.json")<<Json{{"mode",mode},{"source",source},{"hardware",false},{"trusted_checkpoint",false},{"poses",poses},{"loop_guard",listener.guard.state()}}.dump(2);
  }
  std::cout<<Json{{"loops",listener.loops},{"failed_geometry",listener.bad}}<<std::endl;rclcpp::shutdown();return listener.bad?1:0;
}
