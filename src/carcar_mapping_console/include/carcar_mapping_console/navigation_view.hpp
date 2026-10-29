#pragma once
#include <mutex>
#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <tf2_ros/buffer.h>

namespace carcar_mapping_console {
// 只读显示缓存：全局／局部代价图，以及 /plan、/received_global_plan、/lookahead_collision_arc。
// 代价图大约每 0.4 秒编码一帧；路径只保留显示用的抽样点。坐标在读取时换成 map。
class NavigationView {
 public:
  struct Image {std::vector<unsigned char> bytes; nlohmann::json meta = nlohmann::json::object();};
  void grid(size_t index, const nav_msgs::msg::OccupancyGrid & value, double at);
  void path(size_t index, const nav_msgs::msg::Path & value, double at);
  nlohmann::json snapshot(tf2_ros::Buffer & buffer, double at, double ros_time) const;
  Image image(size_t index, tf2_ros::Buffer & buffer, double at, double ros_time) const;
 private:
  struct Grid {
    bool received = false; double at = 0, stamp = 0; uint64_t sequence = 0;
    std::string frame; nav_msgs::msg::MapMetaData info; std::vector<unsigned char> png;
  };
  struct PathLayer {
    bool received = false; double at = 0, stamp = 0; uint64_t sequence = 0;
    std::string frame; nlohmann::json points = nlohmann::json::array();
  };
  nlohmann::json grid_info(const Grid & layer, tf2_ros::Buffer & buffer, double at, double ros_time) const;
  nlohmann::json path_info(const PathLayer & layer, tf2_ros::Buffer & buffer, double at, double ros_time) const;
  mutable std::mutex mutex_;
  Grid grids_[2];
  PathLayer paths_[3];
};
}  // namespace carcar_mapping_console
