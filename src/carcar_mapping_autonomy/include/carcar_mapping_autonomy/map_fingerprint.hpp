#pragma once
#include <cstdint>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace carcar_mapping_autonomy {
// 只用于关联地图内容与规划掩膜，几何需另行比较；资产完整性仍使用SHA256。
inline uint64_t map_content_fingerprint(const nav_msgs::msg::OccupancyGrid & map){
  uint64_t value=14695981039346656037ULL;
  for(const auto cell:map.data){value^=static_cast<uint8_t>(cell);value*=1099511628211ULL;}
  return value;
}
inline bool same_map_geometry(const nav_msgs::msg::OccupancyGrid & a,const nav_msgs::msg::OccupancyGrid & b){
  return a.header.frame_id==b.header.frame_id&&a.info.width==b.info.width&&a.info.height==b.info.height&&
    a.info.resolution==b.info.resolution&&a.info.origin==b.info.origin;
}
}
