// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/reader.hpp"

namespace fs = std::filesystem;

namespace {

struct Area {
  double min_x;
  double max_x;
  double min_y;
  double max_y;

  bool contains(double x, double y) const
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
  }
};

struct Counts {
  size_t cells{0};
  size_t unknown{0};
  size_t free{0};
  size_t inflated{0};
  size_t lethal{0};
  int max_cost{0};
  uint64_t cost_sum{0};

  void add(int8_t cost)
  {
    ++cells;
    if (cost < 0) {
      ++unknown;
    } else if (cost == 0) {
      ++free;
    } else {
      max_cost = std::max(max_cost, static_cast<int>(cost));
      cost_sum += static_cast<uint8_t>(cost);
      if (cost >= 100) {
        ++lethal;
      } else {
        ++inflated;
      }
    }
  }
};

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

std::unique_ptr<rosbag2_cpp::Reader> open_reader(const fs::path & bag_path)
{
  bool compressed = false;
  for (const auto & entry : fs::directory_iterator(bag_path)) {
    if (entry.path().extension() == ".zstd") {
      compressed = true;
      break;
    }
  }
  auto reader = compressed ?
    std::make_unique<rosbag2_cpp::Reader>(
    std::make_unique<rosbag2_compression::SequentialCompressionReader>()) :
    std::make_unique<rosbag2_cpp::Reader>();
  reader->open(bag_path.string());
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/costmap/costmap"};
  reader->set_filter(filter);
  return reader;
}

void audit(const fs::path & bag_path, const fs::path & output_path, const Area & focus,
  double margin)
{
  if (!fs::is_directory(bag_path)) {
    throw std::runtime_error("rosbag 目录不存在：" + bag_path.string());
  }
  if (fs::exists(output_path)) {
    throw std::runtime_error("输出文件已存在，拒绝覆盖：" + output_path.string());
  }
  const auto cells_path = output_path.parent_path() /
    (output_path.stem().string() + "_occupied_cells.csv");
  if (fs::exists(cells_path)) {
    throw std::runtime_error("逐格输出文件已存在，拒绝覆盖：" + cells_path.string());
  }
  const Area expanded{
    focus.min_x - margin, focus.max_x + margin,
    focus.min_y - margin, focus.max_y + margin};
  auto reader = open_reader(bag_path);
  std::ofstream csv(output_path);
  std::ofstream cells_csv(cells_path);
  if (!csv || !cells_csv) {throw std::runtime_error("无法创建输出 CSV");}
  csv << "stamp_ns,focus_cells,focus_unknown,focus_free,focus_inflated,"
         "focus_lethal,focus_max_cost,focus_cost_sum,"
         "margin_cells,margin_unknown,margin_free,margin_inflated,"
         "margin_lethal,margin_max_cost,margin_cost_sum\n";
  cells_csv << "stamp_ns,region,x,y,cost\n";
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> codec;
  size_t frames = 0;
  while (reader->has_next()) {
    const auto bag_message = reader->read_next();
    if (bag_message->topic_name != "/costmap/costmap") {continue;}
    rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
    nav_msgs::msg::OccupancyGrid map;
    codec.deserialize_message(&serialized, &map);
    const auto & info = map.info;
    const auto expected = static_cast<size_t>(info.width) * info.height;
    if (expected == 0 || map.data.size() != expected ||
      !std::isfinite(info.resolution) || info.resolution <= 0.0F ||
      std::abs(info.origin.orientation.x) > 1e-5 ||
      std::abs(info.origin.orientation.y) > 1e-5 ||
      std::abs(info.origin.orientation.z) > 1e-5 ||
      std::abs(info.origin.orientation.w - 1.0) > 1e-5)
    {
      throw std::runtime_error("二维地图尺寸、分辨率或原点方向无效");
    }
    const double map_min_x = info.origin.position.x;
    const double map_min_y = info.origin.position.y;
    const double map_max_x = map_min_x + info.width * info.resolution;
    const double map_max_y = map_min_y + info.height * info.resolution;
    if (expanded.min_x < map_min_x || expanded.max_x > map_max_x ||
      expanded.min_y < map_min_y || expanded.max_y > map_max_y)
    {
      throw std::runtime_error("目标区域或周边带超出二维地图，拒绝部分统计");
    }
    Counts inner, outer;
    for (size_t row = 0; row < info.height; ++row) {
      const double y = info.origin.position.y + (row + 0.5) * info.resolution;
      if (y < expanded.min_y || y > expanded.max_y) {continue;}
      for (size_t column = 0; column < info.width; ++column) {
        const double x = info.origin.position.x + (column + 0.5) * info.resolution;
        if (!expanded.contains(x, y)) {continue;}
        const auto cost = map.data[row * info.width + column];
        if (focus.contains(x, y)) {
          inner.add(cost);
          if (cost != 0) {
            cells_csv << stamp_ns(map.header.stamp) << ",focus," << x << ',' << y << ','
                      << static_cast<int>(cost) << '\n';
          }
        } else {
          outer.add(cost);
          if (cost != 0) {
            cells_csv << stamp_ns(map.header.stamp) << ",margin," << x << ',' << y << ','
                      << static_cast<int>(cost) << '\n';
          }
        }
      }
    }
    if (inner.cells == 0) {
      throw std::runtime_error("目标区域不在二维地图内");
    }
    csv << stamp_ns(map.header.stamp) << ','
        << inner.cells << ',' << inner.unknown << ',' << inner.free << ','
        << inner.inflated << ',' << inner.lethal << ',' << inner.max_cost << ','
        << inner.cost_sum << ',' << outer.cells << ',' << outer.unknown << ','
        << outer.free << ',' << outer.inflated << ',' << outer.lethal << ','
        << outer.max_cost << ',' << outer.cost_sum << '\n';
    ++frames;
  }
  if (frames == 0) {throw std::runtime_error("bag 中没有 /costmap/costmap");}
  std::cout << "二维地图审计完成：" << frames << " 帧，输出 " << output_path << " 和 "
            << cells_path << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 7 && argc != 8) {
    std::cerr << "用法：depth_map_audit <rosbag目录> <全新CSV文件> "
                 "<xmin> <xmax> <ymin> <ymax> [周边宽度m，默认0.20]\n";
    return 2;
  }
  try {
    const Area focus{std::stod(argv[3]), std::stod(argv[4]),
      std::stod(argv[5]), std::stod(argv[6])};
    const double margin = argc == 8 ? std::stod(argv[7]) : 0.20;
    if (!std::isfinite(focus.min_x) || !std::isfinite(focus.max_x) ||
      !std::isfinite(focus.min_y) || !std::isfinite(focus.max_y) ||
      focus.min_x >= focus.max_x || focus.min_y >= focus.max_y ||
      !std::isfinite(margin) || margin < 0.0)
    {
      throw std::invalid_argument("区域边界或周边宽度无效");
    }
    audit(argv[1], argv[2], focus, margin);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "二维地图审计失败：" << error.what() << '\n';
    return 1;
  }
}
