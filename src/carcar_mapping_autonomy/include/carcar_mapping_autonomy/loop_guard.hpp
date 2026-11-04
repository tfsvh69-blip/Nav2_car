#pragma once
#include <chrono>
#include <set>
#include <sstream>
#include <iomanip>
#include <karto_sdk/Mapper.h>
#include "carcar_mapping_autonomy/rasterization.hpp"
#include "carcar_mapping_autonomy/recovery_core.hpp"
namespace carcar_mapping_autonomy {
// 在Karto持有建图锁的回环监听器内执行事务校验；不改变上游匹配阈值。
// 失败只撤销本次回环新约束和校正，顺序扫描及此前约束保留，随后由监督器停车复验。
class LoopGuard {
  struct Saved {karto::LocalizedRangeScan*scan;karto::Pose2 corrected,odometry;};
public:
  static ReferenceScore score(karto::Mapper&mapper,karto::LocalizedRangeScan*scan){
    auto raster=rasterize_scans(mapper.GetAllProcessedScans(),.05,
      mapper.getParamMinPassThrough(),mapper.getParamOccupancyThreshold());
    if(!raster||!scan)throw std::runtime_error("回环复核缺少地图或当前扫描");
    nav_msgs::msg::OccupancyGrid map;map.info.width=raster->GetWidth();map.info.height=raster->GetHeight();map.info.resolution=.05;
    const auto origin=raster->GetCoordinateConverter()->GetOffset();map.info.origin.position.x=origin.GetX();map.info.origin.position.y=origin.GetY();map.info.origin.orientation.w=1;
    map.data.reserve(size_t(map.info.width)*map.info.height);
    for(int y=0;y<raster->GetHeight();++y)for(int x=0;x<raster->GetWidth();++x){
      const auto value=*raster->GetDataPointer(karto::Vector2<int>(x,y));
      map.data.push_back(value==karto::GridStates_Occupied?100:value==karto::GridStates_Free?0:-1);
    }
    const auto*laser=scan->GetLaserRangeFinder();const auto offset=laser->GetOffsetPose();const auto values=scan->GetRangeReadingsVector();
    // 先筛选原始有效束，再均匀限量；避免整圈抽样丢掉短量程内已足够的独立证据。
    std::vector<size_t> valid_indices;valid_indices.reserve(values.size());
    for(size_t i=0;i<values.size();++i){
      const double r=values[i];
      if(std::isfinite(r)&&r>=std::max(.15,laser->GetMinimumRange())&&r<=laser->GetRangeThreshold()-.05)valid_indices.push_back(i);
    }
    const size_t sample_count=std::min<size_t>(180,valid_indices.size());
    std::vector<ReferenceMatcher::Ray> rays;rays.reserve(sample_count);
    for(size_t sample=0;sample<sample_count;++sample){
      // 保留有效束的角序及首尾，索引不重复；角度始终由原始束号计算。
      const size_t selected=sample_count==1?0:sample*(valid_indices.size()-1)/(sample_count-1);
      const size_t i=valid_indices[selected];const double r=values[i];
      const double a=laser->GetMinimumAngle()+i*laser->GetAngularResolution()+offset.GetHeading();
      rays.push_back({offset.GetX()+r*std::cos(a),offset.GetY()+r*std::sin(a),offset.GetX(),offset.GetY()});
    }
    const auto p=scan->GetCorrectedPose();return ReferenceMatcher(map).score({p.GetX(),p.GetY(),p.GetHeading()},rays);
  }
  void begin(karto::Mapper&mapper){
    started_=std::chrono::steady_clock::now();saved_.clear();current_=nullptr;
    for(auto*scan:mapper.GetAllProcessedScans())if(scan){
      saved_.push_back({scan,scan->GetCorrectedPose(),scan->GetOdometricPose()});
      if(!current_||scan->GetUniqueId()>current_->GetUniqueId())current_=scan;
    }
    edges_=mapper.GetGraph()->GetEdges().size();
    try{before_=score(mapper,current_);}catch(const std::exception&){before_=ReferenceScore{};}
  }
  bool finish(karto::Mapper&mapper,karto::ScanSolver&solver){
    if(saved_.empty()||!current_)throw std::runtime_error("回环事务缺少开始时的位姿快照");
    ReferenceScore after;std::string error;
    std::vector<std::string>constraints;bool quarantined=false;
    const auto&edges=mapper.GetGraph()->GetEdges();
    for(size_t i=edges_;i<edges.size();++i){
      const auto key=constraint_id(*edges[i]);constraints.push_back(key);
      quarantined=quarantined||rejected_constraints_.count(key)!=0;
    }
    try{after=score(mapper,current_);}catch(const std::exception&e){error=e.what();}
    const bool valid=!quarantined&&error.empty()&&current_->GetUniqueId()!=rejected_scan_id_&&after.acceptable()&&
      (!before_.acceptable()||(after.mean<=before_.mean+.04&&after.hit_ratio>=before_.hit_ratio-.15));
    double largest=0;for(const auto&s:saved_)largest=std::max(largest,std::hypot(s.corrected.GetX()-s.scan->GetCorrectedPose().GetX(),s.corrected.GetY()-s.scan->GetCorrectedPose().GetY()));
    last_={{"accepted",valid},{"scan_id",current_->GetUniqueId()},{"before",json(before_)},{"proposed",json(after)},
      {"constraint_ids",constraints},{"quarantined",quarantined},{"largest_translation_m",largest},{"new_constraints",mapper.GetGraph()->GetEdges().size()-edges_},{"error",error}};
    if(!valid){
      // 上游2.6.10只追加回环边，不改原边。撤销两端邻接表再释放边，避免留下悬空指针。
      auto*graph=mapper.GetGraph();
      solver.Reset();
      while(graph->GetEdges().size()>edges_){
        const auto index=graph->GetEdges().size()-1;auto*edge=graph->GetEdges()[index];
        for(auto*vertex:{edge->GetSource(),edge->GetTarget()}){
          const auto&adjacent=vertex->GetEdges();const auto found=std::find(adjacent.begin(),adjacent.end(),edge);
          if(found!=adjacent.end())vertex->RemoveEdge(int(found-adjacent.begin()));
        }
        graph->RemoveEdge(int(index));delete edge;
      }
      for(const auto&s:saved_){s.scan->SetOdometricPose(s.odometry);s.scan->SetCorrectedPoseAndUpdate(s.corrected);}
      // Ceres的内部参数也必须恢复，不能只改显示位姿后继续使用错误优化状态。
      for(const auto&group:graph->GetVertices())for(const auto&entry:group.second)if(entry.second)solver.AddNode(entry.second);
      for(auto*edge:graph->GetEdges())if(edge)solver.AddConstraint(edge);
      // 同一任务恢复检查点后保留约束隔离；重复撤销不消耗一次新的自动恢复预算。
      // 时间戳参与标识，防止重新载图后复用整数ID误伤新的扫描。
      for(const auto&key:constraints)rejected_constraints_.insert(key);
      if(quarantined)++quarantine_hits_;else ++rejections_;
      rejected_scan_id_=current_->GetUniqueId();
    }
    last_["quarantine_hits"]=quarantine_hits_;last_["quarantine_size"]=rejected_constraints_.size();
    last_["rejections"]=rejections_;last_["rolled_back"]=!valid;
    last_["validation_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started_).count();
    saved_.clear();current_=nullptr;return valid;
  }
  const Json&state()const{return last_;}
  void reset_graph(){saved_.clear();current_=nullptr;rejected_scan_id_=-1;last_["rolled_back"]=false;}
private:
  static std::string constraint_id(const karto::Edge<karto::LocalizedRangeScan>&edge){
    auto identify=[](const karto::LocalizedRangeScan*scan){
      std::ostringstream out;out<<scan->GetSensorName().ToString()<<':'<<scan->GetUniqueId()<<':'<<std::setprecision(17)<<scan->GetTime();
      return out.str();
    };
    auto a=identify(edge.GetSource()->GetObject()),b=identify(edge.GetTarget()->GetObject());
    if(b<a)std::swap(a,b);
    return a+"|"+b;
  }
  std::set<std::string>rejected_constraints_;unsigned quarantine_hits_=0;
  static Json json(const ReferenceScore&s){return {{"points",s.points},{"mean_residual_m",s.mean},{"coverage",s.coverage},{"hit_ratio",s.hit_ratio},{"free_conflict",s.free_conflict}};}
  std::vector<Saved>saved_;karto::LocalizedRangeScan*current_=nullptr;size_t edges_=0;unsigned rejections_=0;
  int rejected_scan_id_=-1;ReferenceScore before_;Json last_=Json::object();std::chrono::steady_clock::time_point started_;
};
}
