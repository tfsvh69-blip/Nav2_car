// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

namespace fs = std::filesystem;
namespace {

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

template<typename MessageT>
MessageT decode(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  rclcpp::Serialization<MessageT> codec;
  MessageT message;
  codec.deserialize_message(&serialized, &message);
  return message;
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
  return reader;
}

struct Point {
  float x;
  float y;
  float z;
};

struct CloudFrame {
  int64_t stamp{0};
  Point sensor_origin{0.0F, 0.0F, 0.0F};
  std::vector<Point> points;
};

bool segment_crosses_voxel(
  const Point & origin, const Point & endpoint, const Point & center,
  const Point & resolution, double usable_length)
{
  const double dx = endpoint.x - origin.x;
  const double dy = endpoint.y - origin.y;
  const double dz = endpoint.z - origin.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (length <= 0.0 || usable_length <= 0.0) {return false;}
  double enter = 0.0;
  double leave = std::min(1.0, usable_length / length);
  const double from[3] = {origin.x, origin.y, origin.z};
  const double direction[3] = {dx, dy, dz};
  const double middle[3] = {center.x, center.y, center.z};
  const double size[3] = {resolution.x, resolution.y, resolution.z};
  for (size_t axis = 0; axis < 3; ++axis) {
    const double lower = middle[axis] - size[axis] * 0.5;
    const double upper = middle[axis] + size[axis] * 0.5;
    if (std::abs(direction[axis]) < 1e-9) {
      if (from[axis] < lower || from[axis] > upper) {return false;}
      continue;
    }
    const double first = (lower - from[axis]) / direction[axis];
    const double second = (upper - from[axis]) / direction[axis];
    enter = std::max(enter, std::min(first, second));
    leave = std::min(leave, std::max(first, second));
    if (enter > leave) {return false;}
  }
  return true;
}

struct MapFrame {
  int64_t stamp{0};
  nav_msgs::msg::OccupancyGrid message;
};

struct FocusArea {
  double min_x;
  double max_x;
  double min_y;
  double max_y;

  bool contains(double x, double y) const
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
  }
};

const MapFrame * nearest_map(const std::vector<MapFrame> & maps, int64_t stamp)
{
  if (maps.empty()) {return nullptr;}
  auto next = std::lower_bound(maps.begin(), maps.end(), stamp,
      [](const MapFrame & map, int64_t target) {return map.stamp < target;});
  if (next == maps.begin()) {return &*next;}
  if (next == maps.end()) {return &maps.back();}
  const auto & previous = *(next - 1);
  return std::abs(next->stamp - stamp) < std::abs(previous.stamp - stamp) ?
         &*next : &previous;
}

int cost_at(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  const auto & metadata = map.info;
  if (metadata.resolution <= 0.0F) {return -1;}
  const auto mx = static_cast<int64_t>(std::floor(
      (x - metadata.origin.position.x) / metadata.resolution));
  const auto my = static_cast<int64_t>(std::floor(
      (y - metadata.origin.position.y) / metadata.resolution));
  if (mx < 0 || my < 0 || mx >= metadata.width || my >= metadata.height) {return -1;}
  const auto index = static_cast<size_t>(my) * metadata.width + mx;
  return index < map.data.size() ? map.data[index] : -1;
}

std::unordered_set<uint64_t> obstacle_cells(
  const CloudFrame & frame, const nav2_msgs::msg::VoxelGrid & grid)
{
  std::unordered_set<uint64_t> result;
  if (grid.resolutions.x <= 0.0F || grid.resolutions.y <= 0.0F ||
    grid.resolutions.z <= 0.0F) {return result;}
  for (const auto & point : frame.points) {
    const auto x = static_cast<int64_t>(std::floor(
      (point.x - grid.origin.x) / grid.resolutions.x));
    const auto y = static_cast<int64_t>(std::floor(
      (point.y - grid.origin.y) / grid.resolutions.y));
    const auto z = static_cast<int64_t>(std::floor(
      (point.z - grid.origin.z) / grid.resolutions.z));
    if (x < 0 || y < 0 || z < 0 || x >= grid.size_x || y >= grid.size_y ||
      z >= std::min(grid.size_z, 16U)) {continue;}
    result.insert(
      (static_cast<uint64_t>(z) * grid.size_y + y) * grid.size_x + x);
  }
  return result;
}

CloudFrame summarize_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & topic,
  int64_t received_ns, tf2_ros::Buffer & buffer, std::ofstream & output)
{
  CloudFrame frame;
  frame.stamp = stamp_ns(cloud.header.stamp);
  tf2::Transform base_from_sensor;
  try {
    tf2::fromMsg(buffer.tf2::BufferCore::lookupTransform(
        "base_footprint", cloud.header.frame_id,
        tf2::TimePointZero).transform, base_from_sensor);
    frame.sensor_origin = {static_cast<float>(base_from_sensor.getOrigin().x()),
      static_cast<float>(base_from_sensor.getOrigin().y()),
      static_cast<float>(base_from_sensor.getOrigin().z())};
  } catch (const tf2::TransformException & error) {
    output << topic << ',' << frame.stamp << ',' << received_ns << ",TF_UNAVAILABLE,"
           << cloud.header.frame_id << ",0,0,0,0,0,0,0,0,0,0\n";
    std::cerr << "点云 TF 不可用：" << error.what() << '\n';
    return frame;
  }

  size_t valid = 0, roi = 0, below_zero = 0, floor_band = 0, obstacle_band = 0;
  size_t above = 0, invalid = 0;
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z <= 0.0F) {
        ++invalid;
        continue;
      }
      ++valid;
      const auto point = base_from_sensor * tf2::Vector3(*x, *y, *z);
      if (topic == "/camera/depth/points" ||
        topic == "/camera/navigation/obstacles" ||
        topic == "/camera/navigation/clearing") {
        frame.points.push_back({static_cast<float>(point.x()),
          static_cast<float>(point.y()), static_cast<float>(point.z())});
      }
      if (point.x() < 0.3 || point.x() > 1.2 || std::abs(point.y()) > 0.15) {continue;}
      ++roi;
      min_z = std::min(min_z, point.z());
      max_z = std::max(max_z, point.z());
      if (point.z() < 0.0) {++below_zero;}
      else if (point.z() < 0.03) {++floor_band;}
      else if (point.z() <= 0.26) {++obstacle_band;}
      else {++above;}
    }
  } catch (const std::runtime_error & error) {
    output << topic << ',' << frame.stamp << ',' << received_ns << ",INVALID_FORMAT,"
           << cloud.header.frame_id << ",0,0,0,0,0,0,0,0,0,0\n";
    std::cerr << "点云格式错误：" << error.what() << '\n';
    return CloudFrame{};
  }
  output << topic << ',' << frame.stamp << ',' << received_ns << ",OK,"
         << cloud.header.frame_id << ',' << valid << ',' << invalid << ',' << roi << ','
         << below_zero << ',' << floor_band << ',' << obstacle_band << ',' << above << ','
         << (roi ? min_z : 0.0) << ',' << (roi ? max_z : 0.0) << '\n';
  return frame;
}

void audit(
  const fs::path & bag_path, const fs::path & output_path,
  const FocusArea & focus)
{
  if (!fs::is_directory(bag_path)) {
    throw std::runtime_error("rosbag 目录不存在：" + bag_path.string());
  }
  if (fs::exists(output_path) && !fs::is_empty(output_path)) {
    throw std::runtime_error("输出目录已有内容，拒绝覆盖：" + output_path.string());
  }
  fs::create_directories(output_path);
  std::ofstream cloud_csv(output_path / "cloud_frames.csv");
  std::ofstream voxel_csv(output_path / "marked_voxels.csv");
  std::ofstream grid_csv(output_path / "grid_frames.csv");
  std::ofstream ray_csv(output_path / "focus_clearing_rays.csv");
  std::ofstream profile_csv(output_path / "raw_spatial_profile.csv");
  std::ofstream endpoint_csv(output_path / "clearing_endpoints.csv");
  std::ofstream health_csv(output_path / "health_frames.csv");
  std::ofstream diagnostic_csv(output_path / "diagnostic_frames.csv");
  if (!cloud_csv || !voxel_csv || !grid_csv || !ray_csv || !profile_csv ||
    !endpoint_csv || !health_csv || !diagnostic_csv) {
    throw std::runtime_error("无法创建分析结果文件");
  }
  cloud_csv << "topic,header_ns,receive_ns,status,frame_id,valid,invalid,roi,"
               "z_negative,z_0_to_3cm,z_3_to_26cm,z_above_26cm,min_z,max_z\n";
  voxel_csv << "grid_ns,x,y,z,blue_same_voxel,blue_age_s,clear_age_s,"
               "occupancy_cost,map_age_s\n";
  grid_csv << "grid_ns,marked_voxels,blue_same_voxel,lethal_columns,"
              "focus_marked,focus_blue,focus_lethal_columns,"
              "blue_age_s,clear_age_s,map_age_s\n";
  ray_csv << "grid_ns,x,y,z,recent_clear_frames,all_depth_ray_hits,"
             "height_eligible_full_range_hits,raytrace_2m_hits,"
             "shortened_2m_hits,ground_clipped_hits,endpoint_min_z,endpoint_mean_z,endpoint_max_z,"
             "endpoint_mean_x,endpoint_mean_y,endpoint_mean_range,latest_clear_age_s\n";
  profile_csv << "x_min,x_max,y_min,y_max,raw_frames,low_points,mean_z,min_z,max_z,"
                 "fraction_below_minus3cm\n";
  endpoint_csv << "header_ns,receive_ns,frame_id,valid_points,ground_endpoints,min_z,max_z\n";
  health_csv << "receive_ns,healthy\n";
  diagnostic_csv << "header_ns,receive_ns,level,message,reason_code,raw_points,valid_measurements\n";

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer transforms(clock);
  std::vector<MapFrame> maps;
  {
    auto reader = open_reader(bag_path);
    while (reader->has_next()) {
      const auto bag_message = reader->read_next();
      if (bag_message->topic_name == "/tf_static") {
        for (const auto & transform : decode<tf2_msgs::msg::TFMessage>(*bag_message).transforms) {
          transforms.setTransform(transform, "depth_costmap_audit", true);
        }
      } else if (bag_message->topic_name == "/costmap/costmap") {
        auto map = decode<nav_msgs::msg::OccupancyGrid>(*bag_message);
        maps.push_back({stamp_ns(map.header.stamp), std::move(map)});
      }
    }
  }
  std::sort(maps.begin(), maps.end(),
    [](const MapFrame & lhs, const MapFrame & rhs) {return lhs.stamp < rhs.stamp;});

  CloudFrame latest_blue;
  std::deque<CloudFrame> recent_clears;
  std::deque<CloudFrame> recent_raw;
  std::vector<Point> final_focus_voxels;
  Point final_resolution{0.0F, 0.0F, 0.0F};
  int64_t final_grid_stamp = 0;
  size_t grid_count = 0, marked_total = 0;
  auto reader = open_reader(bag_path);
  while (reader->has_next()) {
    const auto bag_message = reader->read_next();
    const auto & topic = bag_message->topic_name;
    if (topic == "/costmap/clearing_endpoints") {
      const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
      size_t valid_points = 0, ground_endpoints = 0;
      double min_z = std::numeric_limits<double>::infinity();
      double max_z = -std::numeric_limits<double>::infinity();
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
          std::hypot(*x, *y) < 0.30F) {continue;}
        ++valid_points;
        min_z = std::min(min_z, static_cast<double>(*z));
        max_z = std::max(max_z, static_cast<double>(*z));
        if (std::abs(*z) <= 0.002F) {++ground_endpoints;}
      }
      endpoint_csv << stamp_ns(cloud.header.stamp) << ',' << bag_message->time_stamp << ','
                   << cloud.header.frame_id << ',' << valid_points << ',' << ground_endpoints
                   << ',' << (valid_points ? min_z : NAN) << ','
                   << (valid_points ? max_z : NAN) << '\n';
      continue;
    }
    if (topic == "/camera/navigation/healthy") {
      health_csv << bag_message->time_stamp << ',' <<
        static_cast<int>(decode<std_msgs::msg::Bool>(*bag_message).data) << '\n';
      continue;
    }
    if (topic == "/camera/navigation/diagnostics") {
      const auto diagnostic = decode<diagnostic_msgs::msg::DiagnosticArray>(*bag_message);
      for (const auto & status : diagnostic.status) {
        std::string reason, raw_points, valid_measurements;
        for (const auto & value : status.values) {
          if (value.key == "reason_code") {reason = value.value;}
          if (value.key == "raw_points") {raw_points = value.value;}
          if (value.key == "valid_measurements") {valid_measurements = value.value;}
        }
        diagnostic_csv << stamp_ns(diagnostic.header.stamp) << ',' << bag_message->time_stamp
                       << ',' << static_cast<unsigned int>(status.level) << ',' << status.message
                       << ',' << reason << ',' << raw_points << ',' << valid_measurements << '\n';
      }
      continue;
    }
    if (topic == "/camera/depth/points" ||
      topic == "/camera/navigation/obstacles" ||
      topic == "/camera/navigation/clearing")
    {
      auto frame = summarize_cloud(
        decode<sensor_msgs::msg::PointCloud2>(*bag_message), topic,
        bag_message->time_stamp, transforms, cloud_csv);
      if (topic == "/camera/depth/points") {
        recent_raw.push_back(std::move(frame));
        if (recent_raw.size() > 3) {recent_raw.pop_front();}
      }
      if (topic == "/camera/navigation/obstacles") {latest_blue = std::move(frame);}
      if (topic == "/camera/navigation/clearing") {
        recent_clears.push_back(std::move(frame));
        if (recent_clears.size() > 12) {recent_clears.pop_front();}
      }
      continue;
    }
    if (topic != "/costmap/voxel_grid") {continue;}
    const auto grid = decode<nav2_msgs::msg::VoxelGrid>(*bag_message);
    const auto grid_stamp = stamp_ns(grid.header.stamp);
    // 停止 bag 的 /clock 后，部分 Costmap 墙钟定时器会重复发布同一仿真时刻。
    if (grid_stamp <= final_grid_stamp) {continue;}
    final_grid_stamp = grid_stamp;
    final_focus_voxels.clear();
    final_resolution = {static_cast<float>(grid.resolutions.x),
      static_cast<float>(grid.resolutions.y), static_cast<float>(grid.resolutions.z)};
    const auto expected = static_cast<size_t>(grid.size_x) * grid.size_y;
    if (grid.data.size() != expected || grid.size_z > 16 || grid.size_z == 0 ||
      grid.resolutions.x <= 0 || grid.resolutions.y <= 0 || grid.resolutions.z <= 0) {
      throw std::runtime_error("体素地图尺寸或分辨率无效");
    }
    const auto * map = nearest_map(maps, grid_stamp);
    const double map_age = map ? (grid_stamp - map->stamp) / 1e9 : NAN;
    const double blue_age = latest_blue.stamp ? (grid_stamp - latest_blue.stamp) / 1e9 : NAN;
    const double clear_age = !recent_clears.empty() && recent_clears.back().stamp ?
      (grid_stamp - recent_clears.back().stamp) / 1e9 : NAN;
    const bool blue_current = std::isfinite(blue_age) &&
      blue_age >= -0.10 && blue_age <= 0.50;
    const auto blue_cells = blue_current ? obstacle_cells(latest_blue, grid) :
      std::unordered_set<uint64_t>{};
    size_t marked = 0, supported = 0, lethal_columns = 0;
    size_t focus_marked = 0, focus_blue = 0, focus_lethal_columns = 0;
    for (size_t column = 0; column < expected; ++column) {
      const auto word = grid.data[column];
      const auto marked_bits = static_cast<uint16_t>((word >> 16) & word);
      if (!marked_bits) {continue;}
      const auto x_index = column % grid.size_x;
      const auto y_index = column / grid.size_x;
      const double x = grid.origin.x + (x_index + 0.5) * grid.resolutions.x;
      const double y = grid.origin.y + (y_index + 0.5) * grid.resolutions.y;
      const int cost = map && std::abs(map_age) <= 0.75 ? cost_at(map->message, x, y) : -1;
      if (cost == 100) {++lethal_columns;}
      const bool inside_focus = focus.contains(x, y);
      if (inside_focus && cost == 100) {++focus_lethal_columns;}
      for (unsigned int z_index = 0; z_index < grid.size_z; ++z_index) {
        if (!(marked_bits & (1U << z_index))) {continue;}
        const double z = grid.origin.z + (z_index + 0.5) * grid.resolutions.z;
        const auto key = (static_cast<uint64_t>(z_index) * grid.size_y + y_index) *
          grid.size_x + x_index;
        const bool has_blue = blue_current && blue_cells.count(key) > 0;
        ++marked;
        supported += has_blue;
        if (inside_focus) {
          ++focus_marked;
          focus_blue += has_blue;
          final_focus_voxels.push_back({static_cast<float>(x),
            static_cast<float>(y), static_cast<float>(z)});
        }
        voxel_csv << grid_stamp << ',' << x << ',' << y << ',' << z << ','
                  << (blue_current ? static_cast<int>(has_blue) : -1) << ','
                  << blue_age << ',' << clear_age << ',' << cost << ',' << map_age << '\n';
      }
    }
    grid_csv << grid_stamp << ',' << marked << ',' << supported << ',' << lethal_columns
             << ',' << focus_marked << ',' << focus_blue << ',' << focus_lethal_columns
             << ',' << blue_age << ',' << clear_age << ',' << map_age << '\n';
    ++grid_count;
    marked_total += marked;
  }
  size_t usable_clear_frames = 0;
  for (const auto & frame : recent_clears) {
    if (frame.stamp > 0 && frame.stamp <= final_grid_stamp &&
      final_grid_stamp - frame.stamp <= 2000000000LL) {++usable_clear_frames;}
  }
  const double latest_clear_age = !recent_clears.empty() && recent_clears.back().stamp ?
    (final_grid_stamp - recent_clears.back().stamp) / 1e9 : NAN;
  for (const auto & voxel : final_focus_voxels) {
    size_t all_depth_hits = 0, height_full_hits = 0;
    size_t raytrace_hits = 0, shortened_hits = 0, ground_clipped_hits = 0;
    double endpoint_min_z = std::numeric_limits<double>::infinity();
    double endpoint_max_z = -std::numeric_limits<double>::infinity();
    double endpoint_sum_z = 0.0, endpoint_sum_x = 0.0, endpoint_sum_y = 0.0;
    double endpoint_sum_range = 0.0;
    for (const auto & frame : recent_clears) {
      if (frame.stamp <= 0 || frame.stamp > final_grid_stamp ||
        final_grid_stamp - frame.stamp > 2000000000LL) {continue;}
      for (const auto & point : frame.points) {
        const double dx = point.x - frame.sensor_origin.x;
        const double dy = point.y - frame.sensor_origin.y;
        const double dz = point.z - frame.sensor_origin.z;
        const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!segment_crosses_voxel(
            frame.sensor_origin, point, voxel, final_resolution, length)) {continue;}
        ++all_depth_hits;
        endpoint_min_z = std::min(endpoint_min_z, static_cast<double>(point.z));
        endpoint_max_z = std::max(endpoint_max_z, static_cast<double>(point.z));
        endpoint_sum_z += point.z;
        endpoint_sum_x += point.x;
        endpoint_sum_y += point.y;
        endpoint_sum_range += length;
        if (point.z < 0.001F && frame.sensor_origin.z > 0.001F) {
          const double fraction =
            (frame.sensor_origin.z - 0.001) / (frame.sensor_origin.z - point.z);
          const Point clipped{
            static_cast<float>(frame.sensor_origin.x + fraction * dx),
            static_cast<float>(frame.sensor_origin.y + fraction * dy), 0.001F};
          const double clipped_length = length * fraction;
          if (segment_crosses_voxel(
              frame.sensor_origin, clipped, voxel, final_resolution,
              std::min(clipped_length, 2.0) - 0.04)) {
            ++ground_clipped_hits;
          }
        }
        // ObservationBuffer 在本轮配置中先剔除低于 -3 cm 的清障端点。
        if (point.z < -0.03F || point.z > 2.0F) {continue;}
        ++height_full_hits;
        const double ray_length = std::min(length, 2.0);
        if (segment_crosses_voxel(
            frame.sensor_origin, point, voxel, final_resolution, ray_length)) {
          ++raytrace_hits;
        }
        // 4 cm 只作为 Nav2 端点缩短量级的保守几何对照，不模拟栅格遍历。
        if (segment_crosses_voxel(
            frame.sensor_origin, point, voxel, final_resolution,
            ray_length - 0.04)) {
          ++shortened_hits;
        }
      }
    }
    ray_csv << final_grid_stamp << ',' << voxel.x << ',' << voxel.y << ',' << voxel.z
            << ',' << usable_clear_frames << ',' << all_depth_hits << ','
            << height_full_hits << ',' << raytrace_hits << ',' << shortened_hits
            << ',' << ground_clipped_hits
            << ',' << (all_depth_hits ? endpoint_min_z : NAN)
            << ',' << (all_depth_hits ? endpoint_sum_z / all_depth_hits : NAN)
            << ',' << (all_depth_hits ? endpoint_max_z : NAN)
            << ',' << (all_depth_hits ? endpoint_sum_x / all_depth_hits : NAN)
            << ',' << (all_depth_hits ? endpoint_sum_y / all_depth_hits : NAN)
            << ',' << (all_depth_hits ? endpoint_sum_range / all_depth_hits : NAN)
            << ',' << latest_clear_age << '\n';
  }
  struct ProfileBin {
    size_t count{0};
    size_t below_minus3{0};
    double sum_z{0.0};
    double min_z{std::numeric_limits<double>::infinity()};
    double max_z{-std::numeric_limits<double>::infinity()};
  };
  constexpr int x_bins = 15;
  constexpr int y_bins = 10;
  std::vector<ProfileBin> profile(x_bins * y_bins);
  for (const auto & frame : recent_raw) {
    for (const auto & point : frame.points) {
      // 近地面候选；不声称这些点全属于同一个实体地面。
      if (point.z < -0.5F || point.z > 0.05F) {continue;}
      const int ix = static_cast<int>(std::floor((point.x - 0.3F) / 0.25F));
      const int iy = static_cast<int>(std::floor((point.y + 1.0F) / 0.2F));
      if (ix < 0 || ix >= x_bins || iy < 0 || iy >= y_bins) {continue;}
      auto & bin = profile[iy * x_bins + ix];
      ++bin.count;
      bin.below_minus3 += point.z < -0.03F;
      bin.sum_z += point.z;
      bin.min_z = std::min(bin.min_z, static_cast<double>(point.z));
      bin.max_z = std::max(bin.max_z, static_cast<double>(point.z));
    }
  }
  for (int iy = 0; iy < y_bins; ++iy) {
    for (int ix = 0; ix < x_bins; ++ix) {
      const auto & bin = profile[iy * x_bins + ix];
      if (bin.count == 0) {continue;}
      const double x_min = 0.3 + ix * 0.25;
      const double y_min = -1.0 + iy * 0.2;
      profile_csv << x_min << ',' << x_min + 0.25 << ',' << y_min << ','
                  << y_min + 0.2 << ',' << recent_raw.size() << ',' << bin.count
                  << ',' << bin.sum_z / bin.count << ',' << bin.min_z << ','
                  << bin.max_z << ',' << static_cast<double>(bin.below_minus3) / bin.count
                  << '\n';
    }
  }
  std::ofstream summary(output_path / "结果说明.md");
  summary << "# 静止深度 Costmap 离线检查\n\n"
          << "- 输入 bag：`" << bag_path.string() << "`。\n"
          << "- 体素帧：" << grid_count << "；逐帧已标记体素总和：" << marked_total
          << "（同一体素跨帧重复计数）。\n"
          << "- 二维 OccupancyGrid 地图帧：" << maps.size() << "；100 为致命代价，"
          << "-1 为缺失、超时或超出地图。\n"
          << "- 重点区域：x=" << focus.min_x << "～" << focus.max_x
          << "m，y=" << focus.min_y << "～" << focus.max_y << "m；"
          << "`grid_frames.csv` 的 focus 列对该区域逐帧计数。\n"
          << "- `blue_same_voxel=-1` 表示没有时间差合格的蓝点帧；0 只表示当前"
          << "对应体素没有同格蓝点，不证明没有其他障碍或有效清障射线。\n"
          << "- 地面 ROI 仅为 x=0.3～1.2m、|y|≤0.15m 的诊断区。"
          << "架空车体或面对桌面时不得按地面零位解释。\n"
          << "- 时间差保留在 CSV；异步 Costmap/点云不能靠最近帧推定严格因果。"
          << " 清障端点、遮挡和 Nav2 射线裁剪仍须结合原始 bag 检查。\n";
  summary << "- `focus_clearing_rays.csv` 只检查末帧残留体素与最近 2 s、最多 12 帧"
             "清障端点的连续线段几何相交；依次比较全部有效深度线段、"
             "端点通过 -3 cm 高度筛选、2 m 射线限制，以及再缩短 4 cm 的对照。"
             "ground_clipped 为将低于地平面的有效端点沿原测量射线收缩至 z=1 mm"
             "后的几何候选数，只用于离线评估，不代表已发布或已清除。"
             "它不模拟 Nav2 栅格遍历或证明体素已被清除；0 条候选射线"
             "只说明这些已记录点在相应边界下没有几何穿过该体素。\n";
  summary << "- `raw_spatial_profile.csv` 汇总 bag 末尾最多 3 帧、z=-0.5～0.05 m"
             "近地面候选的水平分箱；它描述深度几何，不证明每箱均为同一平整地面。\n";
  summary << "- 点云坐标仅使用 bag 内 `/tf_static`；本工具只适用于固定平视 TF。"
             "动态舵机姿态必须另行分析，不能将本结果当成动态 TF 验证。\n";
  if (grid_count == 0) {
    throw std::runtime_error("bag 中没有 /costmap/voxel_grid，无法分析体素残留");
  }
  std::cout << "分析完成：" << (output_path / "结果说明.md") << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 7) {
    std::cerr << "用法：depth_costmap_audit <rosbag目录> <全新输出目录> "
                 "[重点区xmin xmax ymin ymax]\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  try {
    FocusArea focus{0.3, 1.2, -0.15, 0.15};
    if (argc == 7) {
      focus = {std::stod(argv[3]), std::stod(argv[4]),
        std::stod(argv[5]), std::stod(argv[6])};
      if (!std::isfinite(focus.min_x) || !std::isfinite(focus.max_x) ||
        !std::isfinite(focus.min_y) || !std::isfinite(focus.max_y) ||
        focus.min_x >= focus.max_x || focus.min_y >= focus.max_y)
      {
        throw std::invalid_argument("重点区域边界无效");
      }
    }
    audit(argv[1], argv[2], focus);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "分析失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
