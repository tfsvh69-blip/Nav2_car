#pragma once
#include <chrono>
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
    std::vector<ReferenceMatcher::Ray> rays;
    for(size_t i=0;i<values.size();i+=std::max<size_t>(1,values.size()/180)){
      const double r=values[i];if(!std::isfinite(r)||r<std::max(.15,laser->GetMinimumRange())||r>laser->GetRangeThreshold()-.05)continue;
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
    try{after=score(mapper,current_);}catch(const std::exception&e){error=e.what();}
    const bool valid=error.empty()&&current_->GetUniqueId()!=rejected_scan_id_&&after.acceptable()&&
      (!before_.acceptable()||(after.mean<=before_.mean+.04&&after.hit_ratio>=before_.hit_ratio-.15));
    double largest=0;for(const auto&s:saved_)largest=std::max(largest,std::hypot(s.corrected.GetX()-s.scan->GetCorrectedPose().GetX(),s.corrected.GetY()-s.scan->GetCorrectedPose().GetY()));
    last_={{"accepted",valid},{"scan_id",current_->GetUniqueId()},{"before",json(before_)},{"proposed",json(after)},
      {"largest_translation_m",largest},{"new_constraints",mapper.GetGraph()->GetEdges().size()-edges_},{"error",error}};
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
      ++rejections_;rejected_scan_id_=current_->GetUniqueId();
    }
    last_["rejections"]=rejections_;last_["rolled_back"]=!valid;
    last_["validation_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started_).count();
    saved_.clear();current_=nullptr;return valid;
  }
  const Json&state()const{return last_;}
  void reset_graph(){saved_.clear();current_=nullptr;rejected_scan_id_=-1;last_["rolled_back"]=false;}
private:
  static Json json(const ReferenceScore&s){return {{"points",s.points},{"mean_residual_m",s.mean},{"coverage",s.coverage},{"hit_ratio",s.hit_ratio},{"free_conflict",s.free_conflict}};}
  std::vector<Saved>saved_;karto::LocalizedRangeScan*current_=nullptr;size_t edges_=0;unsigned rejections_=0;
  int rejected_scan_id_=-1;ReferenceScore before_;Json last_=Json::object();std::chrono::steady_clock::time_point started_;
};
}
