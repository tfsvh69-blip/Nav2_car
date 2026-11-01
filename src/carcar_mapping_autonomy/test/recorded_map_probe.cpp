#include <fstream>
#include <iostream>
#include "carcar_mapping_autonomy/recovery_initializer.hpp"
// 离线原始扫描回放，只调用实际几何检查和相关匹配，不创建ROS节点。
int main(int argc,char**argv){
  if(argc!=2&&argc!=3)return 2;
  std::ifstream input(argv[1]);const auto data=nlohmann::json::parse(input);bool valid=true,multiframe_passed=false;
  carcar_mapping_autonomy::MultiFrameVerification multiframe,direct_frames;bool direct_passed=false;
  for(const auto&j:data){
    sensor_msgs::msg::LaserScan scan;const auto&s=j["scan"];
    scan.angle_min=s["angle_min"];scan.angle_increment=s["angle_increment"];scan.range_min=s["range_min"];scan.range_max=s["range_max"];
    for(const auto&r:s["ranges"])scan.ranges.push_back(r.is_null()?NAN:r.get<float>());
    const auto pose=[&](const char*k){return carcar_mapping_autonomy::Pose{j[k][0],j[k][1],j[k][2]};};
    const auto offset=pose("laser_offset");
    std::vector<carcar_mapping_autonomy::ReferenceMatcher::Ray> rays;
    for(size_t i=0;i<scan.ranges.size();i+=std::max<size_t>(1,scan.ranges.size()/180)){
      const double r=scan.ranges[i];if(!std::isfinite(r)||r<std::max(.15,double(scan.range_min))||r>scan.range_max-.05)continue;
      const double a=scan.angle_min+i*scan.angle_increment+offset.yaw;
      rays.push_back({offset.x+r*std::cos(a),offset.y+r*std::sin(a),offset.x,offset.y});
    }
    for(const auto*map_key:{"map","current_map"}){
      const auto&m=j[map_key];nav_msgs::msg::OccupancyGrid map;map.info.width=m["width"];map.info.height=m["height"];map.info.resolution=m["resolution"];
      map.info.origin.position.x=m["origin"][0];map.info.origin.position.y=m["origin"][1];map.info.origin.orientation.w=1;map.data=m["data"].get<std::vector<int8_t>>();
      carcar_mapping_autonomy::ReferenceMatcher matcher(map);
      for(const auto*key:{"aligned","anchored"}){
        const auto p=pose(key);const auto score=matcher.score(p,rays);
        nlohmann::json result{{"time",j["time"]},{"map",map_key},{"prior",key},{"hit",score.hit_ratio},{"mean",score.mean},{"coverage",score.coverage}};
        const auto direct=matcher.verify(p,rays);result["direct_unique"]=direct.unique;
        auto candidate=p;
        try{
          candidate=carcar_mapping_autonomy::recovery_initial_pose(map,scan,p,offset);
          const auto check=matcher.verify(candidate,rays);
          result["candidate"]={candidate.x,candidate.y,candidate.yaw};result["accepted"]=check.unique&&check.score.acceptable();
          result["candidate_hit"]=check.score.hit_ratio;result["candidate_mean"]=check.score.mean;
          result["unique"]=check.unique;
        }catch(const std::exception&e){result["accepted"]=false;result["error"]=e.what();}
        if(std::string(map_key)=="map"&&std::string(key)=="aligned"){
          const bool accepted=result["accepted"].get<bool>();if(!accepted)valid=false;
          multiframe_passed=multiframe.observe(candidate,j["scan_stamp_ns"].get<int64_t>(),j["time"].get<double>(),accepted);
          direct_passed=direct_frames.observe(p,j["scan_stamp_ns"].get<int64_t>(),j["time"].get<double>(),direct.unique&&direct.score.acceptable());
        }
        std::cout<<result.dump()<<std::endl;
      }
    }
  }
  std::cout<<nlohmann::json{{"multiframe_passed",multiframe_passed},{"frames",multiframe.count},{"direct_passed",direct_passed},{"direct_frames",direct_frames.count}}<<std::endl;
  return valid&&(argc==2||multiframe_passed)?0:1;
}
