// Copyright 2026 carcar maintainers
// Licensed under the Apache License, Version 2.0.
//
// 离线参数扫描：在真实录包的标障点云上跑多组 PCL 半径滤波参数，
// 回答“前端 ROR 能不能消掉黄色体素，代价是不是把真实细障碍一起削掉”。
// 只读 bag，不回放、不发布、不改 ROS 参数。
//
// 判定口径：
//   removed_pct        被删点占全部输入点的比例。接近 0 说明这些点本来就是稠密表面点。
//   voxels_lost        整个 bag 内“滤波后不再有任何点”的 0.02 m 体素数。
//   lost_on_objects    其中落在 |y|>0.30 m（真实家具/垃圾桶一侧）的数量。
//                      该值大意味着真实障碍被削，即使 removed_pct 很低也不能接受。

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
#include <sstream>
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
#include "carcar_camera/radius_outlier_shadow.hpp"

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

struct SweepConfig {
  double radius;
  int min_neighbors;
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "用法: depth_ror_sweep <bag_dir> <out_dir>\n";
    return 2;
  }
  const fs::path bag_path = argv[1];
  const fs::path out_path = argv[2];
  if (!fs::is_directory(bag_path)) {
    std::cerr << "rosbag 目录不存在：" << bag_path << '\n';
    return 2;
  }
  if (fs::exists(out_path) && !fs::is_empty(out_path)) {
    std::cerr << "输出目录已有内容，拒绝覆盖：" << out_path << '\n';
    return 2;
  }
  fs::create_directories(out_path);

  // 覆盖从保守到激进的范围；0.05/2 是项目当前影子节点默认值。
  const std::vector<SweepConfig> configs{
    {0.05, 2}, {0.03, 2}, {0.02, 2}, {0.02, 3}, {0.03, 3}, {0.03, 5}, {0.05, 5}, {0.08, 3}};
  const double resolution = 0.02;

  rclcpp::init(argc, argv);
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

  try {
    auto pre_scan = open_reader(bag_path);
    while (pre_scan->has_next()) {
      const auto bag_message = pre_scan->read_next();
      if (bag_message->topic_name != "/tf_static") {continue;}
      for (const auto & transform : decode<tf2_msgs::msg::TFMessage>(*bag_message).transforms) {
        transforms.setTransform(transform, "depth_ror_sweep", true);
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "预读 /tf_static 失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }

  struct Accumulator {
    int64_t input_points{0};
    int64_t removed_points{0};
    int64_t frames{0};
    std::set<VoxelKey> before;
    std::set<VoxelKey> after;
    int64_t lost_on_objects{0};
    int64_t lost_off_objects{0};
  };
  std::vector<Accumulator> acc(configs.size());

  try {
    auto reader = open_reader(bag_path);
    while (reader->has_next()) {
      const auto bag_message = reader->read_next();
      if (bag_message->topic_name != "/camera/navigation/obstacles") {continue;}
      const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*bag_message);
      tf2::Transform base_from_sensor;
      try {
        tf2::fromMsg(transforms.tf2::BufferCore::lookupTransform(
            "base_footprint", cloud.header.frame_id, tf2::TimePointZero).transform,
          base_from_sensor);
      } catch (const tf2::TransformException &) {
        continue;
      }

      auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<tf2::Vector3> base_points;
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
        input->emplace_back(*x, *y, *z);
        base_points.emplace_back(base_from_sensor * tf2::Vector3(*x, *y, *z));
      }
      input->width = static_cast<uint32_t>(input->size());
      input->height = 1;
      input->is_dense = true;
      if (input->empty()) {continue;}

      for (size_t i = 0; i < configs.size(); ++i) {
        auto & a = acc[i];
        ++a.frames;
        a.input_points += static_cast<int64_t>(input->size());
        for (const auto & point : base_points) {
          a.before.insert(carcar_camera::voxel_key(point.x(), point.y(), point.z(), resolution));
        }
        std::vector<bool> mask;
        try {
          mask = carcar_camera::radius_outlier_shadow_mask(input, configs[i].radius,
              configs[i].min_neighbors);
        } catch (const std::exception & error) {
          std::cerr << "ROR 失败：" << error.what() << '\n';
          continue;
        }
        for (size_t k = 0; k < input->size(); ++k) {
          if (!mask[k]) {
            ++a.removed_points;
            continue;
          }
          const auto & base = base_points[k];
          a.after.insert(carcar_camera::voxel_key(base.x(), base.y(), base.z(), resolution));
        }
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "扫描失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();

  std::ofstream out(out_path / "ror_sweep.csv");
  out << "radius_m,min_neighbors,frames,input_points,removed_points,removed_pct,"
         "voxels_before,voxels_after,voxels_lost,lost_on_objects,lost_off_objects\n";
  for (size_t i = 0; i < configs.size(); ++i) {
    const auto & a = acc[i];
    int64_t lost_on = 0, lost_off = 0;
    for (const auto & key : a.before) {
      if (a.after.count(key) != 0) {continue;}
      const double center_y = carcar_camera::voxel_center(key.y, resolution);
      if (std::abs(center_y) > 0.30) {++lost_on;} else {++lost_off;}
    }
    const double removed_pct = a.input_points > 0 ?
      100.0 * static_cast<double>(a.removed_points) / static_cast<double>(a.input_points) : 0.0;
    out << configs[i].radius << ',' << configs[i].min_neighbors << ',' << a.frames << ','
        << a.input_points << ',' << a.removed_points << ',' << std::fixed << std::setprecision(4)
        << removed_pct << ',' << a.before.size() << ',' << a.after.size() << ','
        << (static_cast<int64_t>(a.before.size()) - static_cast<int64_t>(a.after.size())) << ','
        << lost_on << ',' << lost_off << '\n';
    std::cout << "r=" << configs[i].radius << " min=" << configs[i].min_neighbors
              << " 删除率=" << removed_pct << "% 丢失体素="
              << (static_cast<int64_t>(a.before.size()) - static_cast<int64_t>(a.after.size()))
              << "（真实障碍侧 " << lost_on << "）\n";
  }
  std::cout << "扫描完成：" << out_path << '\n';
  return 0;
}
