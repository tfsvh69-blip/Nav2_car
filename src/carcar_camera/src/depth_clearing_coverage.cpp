// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.
//
// 清障射线覆盖审计：回答“黄色残留体素到底有没有被清障射线穿过”。
// 只读 bag，不回放、不发布、不改任何 ROS 参数。
//
// 口径：
//   covered   从相机原点( base_footprint )向每个 /costmap/clearing_endpoints 终点做体素级 DDA
//             所经过的全部 0.02 m 体素并集。
//   marked    最后一帧 /costmap/voxel_marked_cloud 里仍然标黄的体素。
//   残留未被覆盖 = marked \ covered。这些格子“本来应该被射线清掉却没有”，
//   属于射线几何或阈值语义的问题；反之 marked ∩ covered 里的格子若仍标黄，
//   说明清障根本没有生效。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

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

// 三维 DDA（Amanatides-Woo）：与 Nav2 clearVoxelLineInMap 一样按体素步进，
// 保证“射线穿过哪些格子”的判定与体素分辨率一致。
void trace_line(
  const tf2::Vector3 & origin, const tf2::Vector3 & endpoint, double resolution,
  std::set<VoxelKey> & visited, size_t max_steps = 2000)
{
  const double dx = endpoint.x() - origin.x();
  const double dy = endpoint.y() - origin.y();
  const double dz = endpoint.z() - origin.z();

  int32_t ix = carcar_camera::axis_index(origin.x(), resolution);
  int32_t iy = carcar_camera::axis_index(origin.y(), resolution);
  int32_t iz = carcar_camera::axis_index(origin.z(), resolution);
  const int32_t ex = carcar_camera::axis_index(endpoint.x(), resolution);
  const int32_t ey = carcar_camera::axis_index(endpoint.y(), resolution);
  const int32_t ez = carcar_camera::axis_index(endpoint.z(), resolution);

  const int32_t step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
  const int32_t step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
  const int32_t step_z = dz > 0.0 ? 1 : (dz < 0.0 ? -1 : 0);

  auto next_boundary = [resolution](int32_t index, int32_t step) {
      return step > 0 ? (index + 1) * resolution : index * resolution;
    };
  const double inf = std::numeric_limits<double>::infinity();
  const double t_max_x = step_x == 0 ? inf :
    (next_boundary(ix, step_x) - origin.x()) / dx;
  const double t_max_y = step_y == 0 ? inf :
    (next_boundary(iy, step_y) - origin.y()) / dy;
  const double t_max_z = step_z == 0 ? inf :
    (next_boundary(iz, step_z) - origin.z()) / dz;
  const double t_delta_x = step_x == 0 ? inf : std::abs(resolution / dx);
  const double t_delta_y = step_y == 0 ? inf : std::abs(resolution / dy);
  const double t_delta_z = step_z == 0 ? inf : std::abs(resolution / dz);

  double t_max_x_running = t_max_x, t_max_y_running = t_max_y, t_max_z_running = t_max_z;
  size_t steps = 0;
  while (steps++ < max_steps) {
    visited.insert(VoxelKey{ix, iy, iz});
    if (ix == ex && iy == ey && iz == ez) {break;}
    if (t_max_x_running <= t_max_y_running && t_max_x_running <= t_max_z_running) {
      if (t_max_x_running > 1.0) {break;}
      ix += step_x;
      t_max_x_running += t_delta_x;
    } else if (t_max_y_running <= t_max_z_running) {
      if (t_max_y_running > 1.0) {break;}
      iy += step_y;
      t_max_y_running += t_delta_y;
    } else {
      if (t_max_z_running > 1.0) {break;}
      iz += step_z;
      t_max_z_running += t_delta_z;
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "用法: depth_clearing_coverage <bag_dir> <out_dir> [voxel_resolution] "
                 "[frame_stride]\n";
    return 2;
  }
  const fs::path bag_path = argv[1];
  const fs::path out_path = argv[2];
  const double resolution = argc >= 4 ? std::stod(argv[3]) : 0.02;
  const int frame_stride = argc >= 5 ? std::max(1, std::stoi(argv[4])) : 5;
  if (!fs::is_directory(bag_path)) {
    std::cerr << "rosbag 目录不存在：" << bag_path << '\n';
    return 2;
  }
  if (!carcar_camera::valid_resolution(resolution)) {
    std::cerr << "体素分辨率无效\n";
    return 2;
  }
  if (fs::exists(out_path) && !fs::is_empty(out_path)) {
    std::cerr << "输出目录已有内容，拒绝覆盖：" << out_path << '\n';
    return 2;
  }
  fs::create_directories(out_path);

  rclcpp::init(argc, argv);
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  try {
    auto pre_scan = open_reader(bag_path);
    while (pre_scan->has_next()) {
      const auto bag_message = pre_scan->read_next();
      if (bag_message->topic_name != "/tf_static") {continue;}
      for (const auto & transform : decode<tf2_msgs::msg::TFMessage>(*bag_message).transforms) {
        transforms.setTransform(transform, "depth_clearing_coverage", true);
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "预读 /tf_static 失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }

  tf2::Vector3 sensor_origin(0.0, 0.0, 0.0);
  try {
    tf2::Transform base_from_camera;
    tf2::fromMsg(transforms.tf2::BufferCore::lookupTransform(
        "base_footprint", "camera_depth_optical_frame", tf2::TimePointZero).transform,
      base_from_camera);
    sensor_origin = base_from_camera.getOrigin();
  } catch (const tf2::TransformException & error) {
    std::cerr << "相机原点 TF 不可用：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }

  std::set<VoxelKey> covered;
  std::set<VoxelKey> marked_union;
  std::set<VoxelKey> marked_last;
  size_t endpoint_frames = 0, marked_frames = 0;
  int64_t endpoints_used = 0;
  const double raytrace_min_range = 0.30;
  const double raytrace_max_range = 2.00;  // 与 depth_costmap_clearing_candidate.yaml 一致

  // 反事实：Nav2 只用 raytrace_max_range 截断射线，端点方向由清障点云给出。
  // 因此“把清障背景看得更远”会加宽可清障楔形区。这里用原始清障点云按不同
  // 端点距离上限复算覆盖，用来判断放宽 max_clearing_range 是否真的有收益。
  const std::vector<double> counterfactual_ranges{2.0, 3.0, 4.0};
  std::vector<std::set<VoxelKey>> counterfactual_covered(counterfactual_ranges.size());

  try {
    auto reader = open_reader(bag_path);
    while (reader->has_next()) {
      const auto bag_message = reader->read_next();
      const auto & topic = bag_message->topic_name;

      if (topic == "/costmap/clearing_endpoints") {
        const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
        if (endpoint_frames % frame_stride != 0) {
          ++endpoint_frames;
          continue;
        }
        ++endpoint_frames;
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
          if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
          const tf2::Vector3 endpoint(*x, *y, *z);
          if ((endpoint - sensor_origin).length() < raytrace_min_range) {continue;}
          trace_line(sensor_origin, endpoint, resolution, covered);
          ++endpoints_used;
        }
        continue;
      }

      if (topic == "/camera/navigation/clearing") {
        const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
        if (endpoint_frames % frame_stride != 0) {continue;}
        tf2::Transform base_from_camera;
        try {
          tf2::fromMsg(transforms.tf2::BufferCore::lookupTransform(
              "base_footprint", cloud.header.frame_id, tf2::TimePointZero).transform,
            base_from_camera);
        } catch (const tf2::TransformException &) {
          continue;
        }
        sensor_msgs::PointCloud2ConstIterator<float> cx(cloud, "x"), cy(cloud, "y"),
          cz(cloud, "z");
        for (; cx != cx.end(); ++cx, ++cy, ++cz) {
          if (!std::isfinite(*cx) || !std::isfinite(*cy) || !std::isfinite(*cz) ||
            *cz <= 0.0F)
          {
            continue;
          }
          const tf2::Vector3 source(*cx, *cy, *cz);
          const double range = source.length();
          if (range < raytrace_min_range) {continue;}
          tf2::Vector3 endpoint = base_from_camera * source;
          // Nav2 只把射线画到 raytrace_max_range；方向由端点决定，长度被截断。
          const tf2::Vector3 offset = endpoint - sensor_origin;
          const double offset_length = offset.length();
          if (offset_length > raytrace_max_range) {
            endpoint = sensor_origin + offset * (raytrace_max_range / offset_length);
          }
          for (size_t r = 0; r < counterfactual_ranges.size(); ++r) {
            if (range > counterfactual_ranges[r]) {continue;}
            trace_line(sensor_origin, endpoint, resolution, counterfactual_covered[r]);
          }
        }
        continue;
      }

      if (topic != "/costmap/voxel_marked_cloud") {continue;}
      const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
      marked_last.clear();
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
        const auto key = carcar_camera::voxel_key(*x, *y, *z, resolution);
        marked_last.insert(key);
        marked_union.insert(key);
      }
      ++marked_frames;
    }
  } catch (const std::exception & error) {
    std::cerr << "覆盖分析失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();

  if (marked_frames == 0) {
    std::cerr << "bag 中没有 /costmap/voxel_marked_cloud\n";
    return 1;
  }
  if (endpoint_frames == 0) {
    std::cerr << "bag 中没有 /costmap/clearing_endpoints\n";
    return 1;
  }

  size_t last_covered = 0, last_uncovered = 0;
  size_t union_covered = 0, union_uncovered = 0;
  std::map<int, std::pair<size_t, size_t>> by_z;  // z 分箱 -> (covered, uncovered)
  {
    std::ofstream out(out_path / "uncovered_marked_voxels.csv");
    out << "x,y,z,uncovered_scope\n";
    for (const auto & key : marked_last) {
      if (covered.count(key) != 0) {
        ++last_covered;
        by_z[carcar_camera::axis_index(
            carcar_camera::voxel_center(key.z, resolution), 0.02)].first++;
        continue;
      }
      ++last_uncovered;
      by_z[carcar_camera::axis_index(
          carcar_camera::voxel_center(key.z, resolution), 0.02)].second++;
      out << std::fixed << std::setprecision(3)
          << carcar_camera::voxel_center(key.x, resolution) << ','
          << carcar_camera::voxel_center(key.y, resolution) << ','
          << carcar_camera::voxel_center(key.z, resolution) << ",last_frame\n";
    }
    for (const auto & key : marked_union) {
      if (covered.count(key) != 0) {++union_covered;} else {
        ++union_uncovered;
        out << std::fixed << std::setprecision(3)
            << carcar_camera::voxel_center(key.x, resolution) << ','
            << carcar_camera::voxel_center(key.y, resolution) << ','
            << carcar_camera::voxel_center(key.z, resolution) << ",union\n";
      }
    }
  }
  {
    std::ofstream out(out_path / "coverage_by_z.csv");
    out << "z_min,z_max,marked_covered,marked_uncovered,uncovered_pct\n";
    for (const auto & [bin, counts] : by_z) {
      const size_t total = counts.first + counts.second;
      out << std::fixed << std::setprecision(2) << (bin * 0.02) << ',' << ((bin + 1) * 0.02) << ','
          << counts.first << ',' << counts.second << ','
          << (total == 0 ? 0.0 : 100.0 * static_cast<double>(counts.second) / total) << '\n';
    }
  }
  {
    std::ofstream out(out_path / "coverage_summary.md");
    out << "# 清障射线覆盖审计\n\n";
    out << "- bag：" << bag_path << "\n";
    out << "- 体素分辨率：" << resolution << " m\n";
    out << "- 相机原点(base_footprint)：" << std::fixed << std::setprecision(3)
        << sensor_origin.x() << ", " << sensor_origin.y() << ", " << sensor_origin.z() << "\n";
    out << "- 清障终点帧：" << endpoint_frames << "，按 stride=" << frame_stride
        << " 使用其中 " << endpoints_used << " 条射线\n";
    out << "- 黄色体素帧：" << marked_frames << "\n";
    out << "- 射线覆盖到的体素总数：" << covered.size() << "\n\n";
    out << "## 最后一帧仍然标黄的体素\n\n";
    out << "- 总数：" << marked_last.size() << "\n";
    out << "- 其中**被射线覆盖过**（本应被清掉却仍标黄）：" << last_covered << "\n";
    out << "- 其中**从未被任何射线覆盖**：" << last_uncovered << "（"
        << (marked_last.empty() ? 0.0 : 100.0 * static_cast<double>(last_uncovered) /
        marked_last.size()) << "%）\n\n";
    out << "## 全 bag 出现过的标黄体素\n\n";
    out << "- 总数：" << marked_union.size() << "\n";
    out << "- 被覆盖过：" << union_covered << "；从未被覆盖：" << union_uncovered << "\n\n";
    out << "判读：若“被覆盖却仍标黄”占多数，说明清障射线没有真正生效（阈值/语义问题）；"
           "若“从未被覆盖”占多数，说明是射线几何或遮挡导致该区域根本没有清障射线。\n";
    out << "\n## 清障背景距离反事实（用 /camera/navigation/clearing 原始端点复算）\n\n";
    out << "射线长度按 raytrace_max_range=" << raytrace_max_range
        << " m 截断，只改变允许的端点距离。\n\n";
    out << "| 端点距离上限 | 覆盖体素 | 最后帧标黄中未被覆盖 | 未覆盖率 |\n|---|---|---|---|\n";
    for (size_t r = 0; r < counterfactual_ranges.size(); ++r) {
      size_t uncovered = 0;
      for (const auto & key : marked_last) {
        if (counterfactual_covered[r].count(key) == 0) {++uncovered;}
      }
      out << "| " << counterfactual_ranges[r] << " m | " << counterfactual_covered[r].size()
          << " | " << uncovered << " | "
          << (marked_last.empty() ? 0.0 : 100.0 * static_cast<double>(uncovered) /
      marked_last.size()) << "% |\n";
    }
    out << "\n判读：若未覆盖率随端点距离上限明显下降，说明放宽清障背景距离确实能加宽可清障楔形区；"
           "若基本不变，说明残留主要是物体自身表面被遮挡，放宽距离无用。\n";
  }
  for (size_t r = 0; r < counterfactual_ranges.size(); ++r) {
    size_t uncovered = 0;
    for (const auto & key : marked_last) {
      if (counterfactual_covered[r].count(key) == 0) {++uncovered;}
    }
    std::cout << "  反事实 端点<=" << counterfactual_ranges[r] << " m: 覆盖="
              << counterfactual_covered[r].size() << " 未覆盖=" << uncovered << "\n";
  }

  std::cout << "覆盖审计完成：" << out_path << "\n"
            << "相机原点=" << sensor_origin.x() << "," << sensor_origin.y() << "," <<
    sensor_origin.z() << " 覆盖体素=" << covered.size() << "\n"
            << "最后帧标黄=" << marked_last.size() << " 覆盖过却仍标黄=" << last_covered
            << " 从未被覆盖=" << last_uncovered << "\n"
            << "全bag标黄=" << marked_union.size() << " 覆盖过=" << union_covered
            << " 从未覆盖=" << union_uncovered << "\n";
  return 0;
}
