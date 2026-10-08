// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.
//
// NAV-015 阶段二离线取证工具：回答“黄色体素闪烁是不是输入端噪点”。
// 只读 rosbag，不回放、不发布、不修改任何 ROS 参数。
//
// 输出：
//   obstacle_spatial_hist.csv  标障/ROR 点云在 base_footprint 下的水平分格计数
//   z_profile_by_x.csv         纵向高度剖面（判断地面在远处是否抬升）
//   z_profile_by_y.csv         横向高度剖面（判断地面是否随左右倾斜）
//   voxel_frames.csv           每帧黄色体素数，用于观察整体闪烁节奏
//   voxel_flash_all.csv        每个标记体素的出现统计与同格标障点数
//   voxel_flash_flicker.csv    仅中途消失过的体素，附 0/1 游程编码
//   summary.md                 中文结论摘要

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

#include "carcar_camera/flicker_logic.hpp"

namespace fs = std::filesystem;
namespace {

using carcar_camera::VoxelKey;

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

// 高度剖面：按给定轴分箱，统计该箱内点的高度分布，用于识别“地面随距离/横向抬升”。
struct HeightProfile {
  struct Bin {
    std::vector<float> z;
  };
  std::map<int, Bin> bins;

  void add(int bin, float z_value)
  {
    if (bin != carcar_camera::kInvalidBin) {bins[bin].z.push_back(z_value);}
  }

  void write(std::ofstream & out, const std::string & axis, double origin, double width) const
  {
    out << axis << "_min," << axis << "_max,points,mean_z,p05_z,p50_z,p95_z,"
           "frac_z_lt_0p01,frac_z_lt_0p03\n";
    for (const auto & [index, bin] : bins) {
      if (bin.z.empty()) {continue;}
      std::vector<float> sorted = bin.z;
      std::sort(sorted.begin(), sorted.end());
      const double mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
      auto pct = [&sorted](double q) {
          return sorted[static_cast<size_t>(q * (sorted.size() - 1))];
        };
      const size_t low = static_cast<size_t>(
        std::count_if(sorted.begin(), sorted.end(), [](float v) {return v < 0.01F;}));
      const size_t band = static_cast<size_t>(
        std::count_if(sorted.begin(), sorted.end(), [](float v) {return v < 0.03F;}));
      out << std::fixed << std::setprecision(4)
          << (origin + index * width) << ',' << (origin + (index + 1) * width) << ','
          << sorted.size() << ',' << mean << ',' << pct(0.05) << ',' << pct(0.50) << ','
          << pct(0.95) << ','
          << (static_cast<double>(low) / sorted.size()) << ','
          << (static_cast<double>(band) / sorted.size()) << '\n';
    }
  }
};

struct Options {
  fs::path bag;
  fs::path out;
  double voxel_resolution{0.02};
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "用法: depth_flicker_audit <bag_dir> <out_dir> [voxel_resolution]\n";
    return 2;
  }
  Options options;
  options.bag = argv[1];
  options.out = argv[2];
  if (argc >= 4) {options.voxel_resolution = std::stod(argv[3]);}
  if (!fs::is_directory(options.bag)) {
    std::cerr << "rosbag 目录不存在：" << options.bag << '\n';
    return 2;
  }
  if (!carcar_camera::valid_resolution(options.voxel_resolution)) {
    std::cerr << "体素分辨率无效\n";
    return 2;
  }
  if (fs::exists(options.out) && !fs::is_empty(options.out)) {
    std::cerr << "输出目录已有内容，拒绝覆盖：" << options.out << '\n';
    return 2;
  }
  fs::create_directories(options.out);

  rclcpp::init(argc, argv);
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

  const double res = options.voxel_resolution;

  // 水平分格计数：key = (x 格, y 格, 高度带)，高度带 0=3~8cm,1=8~15cm,2=15~26cm
  auto band_of = [](double z) -> int {
      if (z < 0.08) {return 0;}
      if (z < 0.15) {return 1;}
      return 2;
    };
  std::map<std::tuple<int, int, int>, int64_t> obstacle_hist;
  std::map<std::tuple<int, int, int>, int64_t> ror_hist;

  HeightProfile obstacle_by_x, obstacle_by_y, ror_by_x, ror_by_y;
  const double profile_width = 0.10;

  std::map<VoxelKey, std::vector<int64_t>> voxel_samples;
  std::map<VoxelKey, int64_t> obstacle_hits, ror_hits;
  std::vector<std::pair<int64_t, size_t>> voxel_frame_counts;

  // 残留判据：某一帧被标为黄格的体素，若前后时间窗内没有任何标障输入点落在同格，
  // 说明它已经不对应任何当前观测，属于射线没能清掉的残留。
  std::map<VoxelKey, std::vector<int64_t>> obstacle_hit_frames;
  std::vector<int64_t> obstacle_frame_stamps;
  std::vector<std::pair<int64_t, std::vector<VoxelKey>>> voxel_frame_sets;

  // 二维代价栅格：只有它参与规划和碰撞判定，因此必须单独统计是否也在闪烁。
  std::map<VoxelKey, std::vector<int64_t>> cell_samples;
  std::vector<std::pair<int64_t, size_t>> cell_frame_counts;
  int64_t cell_sample_index = 0;

  // 反事实：用体素云还原不同 mark_threshold 下的二维致命格。
  // Nav2 的 mark_threshold 是“一列里至少有多少个体素被占用，该列才算致命”，
  // 所以可以直接从 voxel_marked_cloud 按列计数复算，不必重录。
  std::map<int, int64_t> cost_value_histogram;
  const std::vector<int> thresholds{1, 2, 3};
  std::vector<std::map<std::pair<int32_t, int32_t>, std::vector<int64_t>>> threshold_samples(
    thresholds.size());

  int64_t voxel_sample_index = 0;
  int64_t obstacle_frames = 0, ror_frames = 0;
  size_t obstacle_points = 0, ror_points = 0;

  // 第一遍只收集 /tf_static。bag 按接收时刻排序，静态 TF 可能晚于最早的点云帧，
  // 若边读边设 TF，开头若干帧会因查不到 base_footprint 而整帧丢失。
  try {
    auto pre_scan = open_reader(options.bag);
    while (pre_scan->has_next()) {
      const auto bag_message = pre_scan->read_next();
      if (bag_message->topic_name != "/tf_static") {continue;}
      for (const auto & transform : decode<tf2_msgs::msg::TFMessage>(*bag_message).transforms) {
        transforms.setTransform(transform, "depth_flicker_audit", true);
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "预读 /tf_static 失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }

  size_t tf_failures = 0;
  try {
    auto reader = open_reader(options.bag);
    while (reader->has_next()) {
      const auto bag_message = reader->read_next();
      const auto & topic = bag_message->topic_name;

      if (topic == "/costmap/voxel_marked_cloud") {
        const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
        size_t marked = 0;
        std::vector<VoxelKey> frame_keys;
        std::map<std::pair<int32_t, int32_t>, int> column_counts;
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
          if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
          const auto key = carcar_camera::voxel_key(*x, *y, *z, res);
          voxel_samples[key].push_back(voxel_sample_index);
          frame_keys.push_back(key);
          ++column_counts[{key.x, key.y}];
          ++marked;
        }
        voxel_frame_sets.emplace_back(stamp_ns(cloud.header.stamp), std::move(frame_keys));
        for (size_t t = 0; t < thresholds.size(); ++t) {
          for (const auto & [column, count] : column_counts) {
            if (count >= thresholds[t]) {
              threshold_samples[t][column].push_back(voxel_sample_index);
            }
          }
        }
        voxel_frame_counts.emplace_back(stamp_ns(cloud.header.stamp), marked);
        ++voxel_sample_index;
        continue;
      }

      if (topic == "/costmap/costmap") {
        const auto grid = decode<nav_msgs::msg::OccupancyGrid>(*bag_message);
        const double resolution = grid.info.resolution;
        if (!(resolution > 0.0) ||
          grid.data.size() < static_cast<size_t>(grid.info.width) * grid.info.height) {
          continue;
        }
        size_t lethal = 0;
        for (uint32_t row = 0; row < grid.info.height; ++row) {
          for (uint32_t column = 0; column < grid.info.width; ++column) {
            // Nav2 发布到 OccupancyGrid 时已映射为 0-100：100=致命，99=内切膨胀。
            // 只统计真正的致命格，否则会把膨胀环也算成障碍。
            const int8_t value = grid.data[row * grid.info.width + column];
            ++cost_value_histogram[value];
            if (value < 100) {continue;}
            // 用世界坐标分格，滚动窗口原点若有变化也不会错位。
            const double wx = grid.info.origin.position.x + (column + 0.5) * resolution;
            const double wy = grid.info.origin.position.y + (row + 0.5) * resolution;
            cell_samples[carcar_camera::voxel_key(wx, wy, 0.0, res)].push_back(cell_sample_index);
            ++lethal;
          }
        }
        cell_frame_counts.emplace_back(stamp_ns(grid.header.stamp), lethal);
        ++cell_sample_index;
        continue;
      }

      const bool is_obstacle = topic == "/camera/navigation/obstacles";
      const bool is_ror = topic == "/camera/navigation/obstacles_ror_shadow";
      if (!is_obstacle && !is_ror) {continue;}

      const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
      tf2::Transform base_from_sensor;
      try {
        tf2::fromMsg(transforms.tf2::BufferCore::lookupTransform(
            "base_footprint", cloud.header.frame_id, tf2::TimePointZero).transform,
          base_from_sensor);
      } catch (const tf2::TransformException & error) {
        ++tf_failures;
        if (tf_failures <= 3) {
          std::cerr << "点云 TF 不可用：" << error.what() << '\n';
        }
        continue;
      }

      auto & hist = is_obstacle ? obstacle_hist : ror_hist;
      auto & by_x = is_obstacle ? obstacle_by_x : ror_by_x;
      auto & by_y = is_obstacle ? obstacle_by_y : ror_by_y;
      auto & hits = is_obstacle ? obstacle_hits : ror_hits;
      if (is_obstacle) {
        ++obstacle_frames;
        obstacle_frame_stamps.push_back(stamp_ns(cloud.header.stamp));
      } else {
        ++ror_frames;
      }

      size_t count = 0;
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z <= 0.0F) {
          continue;
        }
        const auto point = base_from_sensor * tf2::Vector3(*x, *y, *z);
        ++count;
        hist[{carcar_camera::axis_index(point.x(), profile_width),
          carcar_camera::axis_index(point.y(), profile_width),
          band_of(point.z())}] += 1;
        by_x.add(carcar_camera::bin_index(point.x(), 0.0, profile_width),
          static_cast<float>(point.z()));
        by_y.add(carcar_camera::bin_index(point.y(), 0.0, profile_width),
          static_cast<float>(point.z()));
        const auto key = carcar_camera::voxel_key(point.x(), point.y(), point.z(), res);
        hits[key] += 1;
        if (is_obstacle) {
          obstacle_hit_frames[key].push_back(static_cast<int64_t>(obstacle_frames) - 1);
        }
      }
      if (is_obstacle) {obstacle_points += count;} else {ror_points += count;}
    }
  } catch (const std::exception & error) {
    std::cerr << "分析失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();

  if (voxel_sample_index == 0) {
    std::cerr << "bag 中没有 /costmap/voxel_marked_cloud，无法分析体素闪烁\n";
    return 1;
  }
  if (obstacle_frames == 0) {
    std::cerr << "bag 中没有 /camera/navigation/obstacles，无法分析标障输入\n";
    return 1;
  }

  // ---- 水平分格 ----
  {
    std::ofstream out(options.out / "obstacle_spatial_hist.csv");
    out << "source,bx_min,bx_max,by_min,by_max,z_band,count\n";
    auto dump = [&out](const char * source, const std::map<std::tuple<int, int, int>,
        int64_t> & hist) {
        for (const auto & [key, count] : hist) {
          const auto [bx, by, band] = key;
          out << source << ',' << std::fixed << std::setprecision(2)
              << (bx * 0.10) << ',' << ((bx + 1) * 0.10) << ','
              << (by * 0.10) << ',' << ((by + 1) * 0.10) << ','
              << (band == 0 ? "0.03-0.08" : band == 1 ? "0.08-0.15" : "0.15-0.26")
              << ',' << count << '\n';
        }
      };
    dump("obstacles", obstacle_hist);
    dump("ror", ror_hist);
  }

  // ---- 高度剖面 ----
  {
    std::ofstream out(options.out / "z_profile_by_x.csv");
    obstacle_by_x.write(out, "x", 0.0, profile_width);
  }
  {
    std::ofstream out(options.out / "z_profile_by_y.csv");
    obstacle_by_y.write(out, "y", 0.0, profile_width);
  }

  // ---- 每帧体素数 ----
  {
    std::ofstream out(options.out / "voxel_frames.csv");
    out << "sample,stamp_ns,marked_voxels\n";
    int64_t index = 0;
    for (const auto & [stamp, marked] : voxel_frame_counts) {
      out << index++ << ',' << stamp << ',' << marked << '\n';
    }
  }

  // ---- 体素闪烁统计 ----
  struct Row {
    VoxelKey key;
    size_t samples;
    int runs;
    int transitions;
    double on_ratio;
    int64_t obstacle_hit;
    int64_t ror_hit;
  };
  std::vector<Row> rows;
  rows.reserve(voxel_samples.size());
  for (const auto & [key, samples] : voxel_samples) {
    const auto obstacle_it = obstacle_hits.find(key);
    const auto ror_it = ror_hits.find(key);
    rows.push_back(Row{key, samples.size(),
      carcar_camera::count_runs(samples),
      carcar_camera::count_transitions(samples, voxel_sample_index),
      static_cast<double>(samples.size()) / voxel_sample_index,
      obstacle_it == obstacle_hits.end() ? 0 : obstacle_it->second,
      ror_it == ror_hits.end() ? 0 : ror_it->second});
  }
  std::sort(rows.begin(), rows.end(), [](const Row & a, const Row & b) {
      if (a.runs != b.runs) {return a.runs > b.runs;}
      return a.samples > b.samples;
    });

  auto row_line = [&](std::ofstream & out, const Row & row) {
      out << std::fixed << std::setprecision(3)
          << carcar_camera::voxel_center(row.key.x, res) << ','
          << carcar_camera::voxel_center(row.key.y, res) << ','
          << carcar_camera::voxel_center(row.key.z, res) << ','
          << row.samples << ',' << voxel_sample_index << ',' << row.runs << ','
          << row.transitions << ',' << std::setprecision(3) << row.on_ratio << ','
          << row.obstacle_hit << ',' << row.ror_hit;
    };

  size_t flicker_count = 0;
  {
    std::ofstream all(options.out / "voxel_flash_all.csv");
    all << "x,y,z,samples_present,total_samples,runs,transitions,on_ratio,"
           "obstacle_points_same_voxel,ror_points_same_voxel\n";
    for (const auto & row : rows) {
      row_line(all, row);
      all << '\n';
      if (row.runs > 1) {++flicker_count;}
    }
  }
  {
    std::ofstream flicker(options.out / "voxel_flash_flicker.csv");
    flicker << "x,y,z,samples_present,total_samples,runs,transitions,on_ratio,"
               "obstacle_points_same_voxel,ror_points_same_voxel,rle\n";
    for (const auto & row : rows) {
      if (row.runs <= 1) {continue;}
      row_line(flicker, row);
      flicker << ',' << carcar_camera::run_length_encode(
        voxel_samples[row.key], voxel_sample_index) << '\n';
    }
  }

  // ---- 二维代价栅格闪烁 ----
  size_t cell_flicker = 0;
  std::ofstream cell_out(options.out / "costmap_cell_flash.csv");
  cell_out << "x,y,samples_lethal,total_samples,runs,transitions,on_ratio,rle\n";
  {
    std::vector<std::pair<VoxelKey, std::vector<int64_t>>> ordered(
      cell_samples.begin(), cell_samples.end());
    std::sort(ordered.begin(), ordered.end(), [](const auto & a, const auto & b) {
        return carcar_camera::count_runs(a.second) > carcar_camera::count_runs(b.second);
      });
    for (const auto & [key, samples] : ordered) {
      const int runs = carcar_camera::count_runs(samples);
      if (runs > 1) {++cell_flicker;}
      cell_out << std::fixed << std::setprecision(3)
               << carcar_camera::voxel_center(key.x, res) << ','
               << carcar_camera::voxel_center(key.y, res) << ','
               << samples.size() << ',' << cell_sample_index << ',' << runs << ','
               << carcar_camera::count_transitions(samples, cell_sample_index) << ','
               << std::setprecision(3)
               << (static_cast<double>(samples.size()) / std::max<int64_t>(1, cell_sample_index))
               << ',' << carcar_camera::run_length_encode(samples, cell_sample_index) << '\n';
    }
  }
  {
    std::ofstream out(options.out / "costmap_frames.csv");
    out << "sample,stamp_ns,lethal_cells\n";
    int64_t index = 0;
    for (const auto & [stamp, lethal] : cell_frame_counts) {
      out << index++ << ',' << stamp << ',' << lethal << '\n';
    }
  }
  {
    std::ofstream out(options.out / "costmap_value_histogram.csv");
    out << "value,cell_samples\n";
    for (const auto & [value, count] : cost_value_histogram) {
      out << static_cast<int>(value) << ',' << count << '\n';
    }
  }

  // ---- 残留体素：当前被标为黄格但窗口内没有任何标障输入 ----
  const int64_t stale_window_ns = 500000000LL;  // ±0.5 s
  int64_t stale_occurrences = 0, total_occurrences = 0;
  std::map<VoxelKey, int64_t> stale_counts;
  std::vector<std::pair<int64_t, std::pair<size_t, size_t>>> stale_frame_counts;
  for (const auto & [stamp, keys] : voxel_frame_sets) {
    size_t stale_here = 0;
    for (const auto & key : keys) {
      ++total_occurrences;
      const auto it = obstacle_hit_frames.find(key);
      bool backed = false;
      if (it != obstacle_hit_frames.end()) {
        // obstacle_hit_frames 按帧序递增，逐条比较时间戳即可。
        for (const int64_t frame_index : it->second) {
          const int64_t delta = obstacle_frame_stamps[frame_index] - stamp;
          if (delta > stale_window_ns) {break;}
          if (delta >= -stale_window_ns) {backed = true; break;}
        }
      }
      if (!backed) {
        ++stale_here;
        ++stale_counts[key];
      }
    }
    stale_occurrences += static_cast<int64_t>(stale_here);
    stale_frame_counts.emplace_back(stamp, std::make_pair(keys.size(), stale_here));
  }
  {
    std::ofstream out(options.out / "voxel_residue_frames.csv");
    out << "stamp_ns,marked_voxels,residue_voxels\n";
    for (const auto & [stamp, counts] : stale_frame_counts) {
      out << stamp << ',' << counts.first << ',' << counts.second << '\n';
    }
  }
  {
    std::ofstream out(options.out / "voxel_residue.csv");
    out << "x,y,z,residue_samples\n";
    std::vector<std::pair<VoxelKey, int64_t>> ordered(stale_counts.begin(), stale_counts.end());
    std::sort(ordered.begin(), ordered.end(), [](const auto & a, const auto & b) {
        return a.second > b.second;
      });
    for (const auto & [key, count] : ordered) {
      out << std::fixed << std::setprecision(3)
          << carcar_camera::voxel_center(key.x, res) << ','
          << carcar_camera::voxel_center(key.y, res) << ','
          << carcar_camera::voxel_center(key.z, res) << ',' << count << '\n';
    }
  }

  // ---- mark_threshold 反事实对照 ----
  std::vector<size_t> threshold_columns(thresholds.size(), 0);
  std::vector<size_t> threshold_flicker(thresholds.size(), 0);
  {
    std::ofstream out(options.out / "mark_threshold_counterfactual.csv");
    out << "mark_threshold,distinct_lethal_columns,flickering_columns,flicker_pct\n";
    for (size_t t = 0; t < thresholds.size(); ++t) {
      size_t flickering = 0;
      for (const auto & [column, samples] : threshold_samples[t]) {
        if (carcar_camera::count_runs(samples) > 1) {++flickering;}
      }
      threshold_columns[t] = threshold_samples[t].size();
      threshold_flicker[t] = flickering;
      const double pct = threshold_samples[t].empty() ? 0.0 :
        100.0 * static_cast<double>(flickering) / static_cast<double>(threshold_samples[t].size());
      out << thresholds[t] << ',' << threshold_samples[t].size() << ',' << flickering << ','
          << std::fixed << std::setprecision(2) << pct << '\n';
    }
  }

  // ---- 摘要 ----
  size_t flicker_with_input = 0, flicker_without_input = 0;
  int64_t flicker_obstacle_points = 0, flicker_ror_points = 0;
  for (const auto & row : rows) {
    if (row.runs <= 1) {continue;}
    if (row.obstacle_hit > 0) {
      ++flicker_with_input;
      flicker_obstacle_points += row.obstacle_hit;
      flicker_ror_points += row.ror_hit;
    } else {
      ++flicker_without_input;
    }
  }
  {
    std::ofstream out(options.out / "summary.md");
    out << "# 黄色体素闪烁离线审计\n\n";
    out << "- bag：" << options.bag << "\n";
    out << "- 体素分辨率：" << res << " m\n";
    out << "- 体素观测样本：" << voxel_sample_index << " 帧\n";
    out << "- 标障点云帧：" << obstacle_frames << " 帧，累计 " << obstacle_points << " 点\n";
    out << "- ROR 点云帧：" << ror_frames << " 帧，累计 " << ror_points << " 点\n";
    out << "- 因 TF 不可用被跳过的点云帧：" << tf_failures << "（应为 0，否则结论不可用）\n\n";
    out << "## 结论要点\n\n";
    out << "- 出现过的标记体素总数：" << rows.size() << "，其中中途消失过（runs>1）："
        << flicker_count << "\n";
    out << "- 闪烁体素中，同格存在标障输入点的：" << flicker_with_input
        << "；完全无输入点的：" << flicker_without_input << "\n";
    out << "- 闪烁体素同格标障点累计 " << flicker_obstacle_points
        << "，ROR 点累计 " << flicker_ror_points << "\n";
    out << "- 标障点云总点数 " << obstacle_points << "，ROR 点云总点数 " << ror_points
        << "，差值 " << (static_cast<int64_t>(obstacle_points) -
      static_cast<int64_t>(ror_points)) << "（ROR 实际删除量）\n";
    out << "\n## 残留体素（射线未清除）\n\n";
    out << "- 判定窗口：体素帧前后 ±0.5 s\n";
    out << "- 黄格出现总次数：" << total_occurrences << "，其中**窗口内无任何标障输入**的："
        << stale_occurrences;
    if (total_occurrences > 0) {
      out << "（" << std::fixed << std::setprecision(2)
          << (100.0 * static_cast<double>(stale_occurrences) /
        static_cast<double>(total_occurrences)) << "%）";
    }
    out << "\n- 涉及的不同体素：" << stale_counts.size() << "\n";
    out << "\n判读：该比例高说明有大量黄格已经不对应任何当前观测，"
           "是清障射线没有覆盖到的历史标记，前端滤波无法消除。\n";
    out << "\n## 二维代价栅格\n\n";
    out << "- 代价栅格观测样本：" << cell_sample_index << " 帧\n";
    out << "- 出现过致命代价（>=99）的格子：" << cell_samples.size()
        << "，其中中途变回可通行的（runs>1）：" << cell_flicker << "\n";
    out << "\n判读：二维代价栅格是规划和碰撞的唯一判据。若二维格子不闪烁，"
           "三维黄色体素的明暗变化只是 RViz 诊断显示，不影响导航安全。\n";
    out << "\n## mark_threshold 反事实对照\n\n";
    out << "| mark_threshold | 致命列数 | 闪烁列数 | 闪烁比例 |\n|---|---|---|---|\n";
    for (size_t t = 0; t < thresholds.size(); ++t) {
      const double pct = threshold_columns[t] == 0 ? 0.0 :
        100.0 * static_cast<double>(threshold_flicker[t]) /
        static_cast<double>(threshold_columns[t]);
      out << "| " << thresholds[t] << " | " << threshold_columns[t] << " | "
          << threshold_flicker[t] << " | " << std::fixed << std::setprecision(2) << pct
          << "% |\n";
    }
    out << "\n提高 mark_threshold 会同时减少致命列总数（等于把障碍边缘向内收），"
           "因此必须同时看“闪烁比例下降”与“致命列损失”，不能只看前者。\n";
    out << "\n判读：若“完全无输入点”的数量可观，说明黄色来自体素层自身未清除的历史格，"
           "前端滤波不可能消除；若闪烁格普遍有输入点，才需要前端空间/时间滤波。\n";
  }

  std::cout << "审计完成：" << options.out << "\n"
            << "体素样本帧=" << voxel_sample_index << " 体素总数=" << rows.size()
            << " 闪烁体素=" << flicker_count
            << " 其中同格有标障输入=" << flicker_with_input
            << " 无输入=" << flicker_without_input << "\n"
            << "残留黄格=" << stale_occurrences << "/" << total_occurrences
            << " 涉及体素=" << stale_counts.size() << "\n"
            << "二维样本帧=" << cell_sample_index << " 致命格=" << cell_samples.size()
            << " 闪烁致命格=" << cell_flicker << "\n"
            << "标障点=" << obstacle_points << " ROR点=" << ror_points << "\n";
  return 0;
}
