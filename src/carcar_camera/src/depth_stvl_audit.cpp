#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include "carcar_camera/stvl_roi.hpp"

// 只读已封存、未压缩的静止STVL包；输出完整体素ROI逐帧计数，不判定物理空闲。
int main(int argc, char ** argv)
{
  if (argc != 9) {
    std::cerr << "用法：depth_stvl_audit <bag目录> <全新CSV> <xmin> <xmax> <ymin> <ymax> <zmin> <zmax>\n";
    return 2;
  }
  try {
    if (std::filesystem::exists(argv[2])) {throw std::runtime_error("输出已存在，拒绝覆盖");}
    const std::filesystem::path output_path(argv[2]);
    const auto points_path = output_path.parent_path() /
      (output_path.stem().string() + "_roi_voxels.csv");
    if (std::filesystem::exists(points_path)) {
      throw std::runtime_error("体素坐标输出已存在，拒绝覆盖");
    }
    std::array<double, 6> bounds{};
    for (size_t i = 0; i < bounds.size(); ++i) {
      const std::string arg(argv[i + 3]);
      size_t parsed = 0;
      bounds[i] = std::stod(arg, &parsed);
      if (parsed != arg.size()) {throw std::invalid_argument("边界含非数值字符");}
    }
    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    bool found = false;
    for (const auto & topic : reader.get_all_topics_and_types()) {
      if (topic.name == "/costmap/voxel_grid") {
        if (topic.type != "sensor_msgs/msg/PointCloud2") {
          throw std::runtime_error("体素话题不是STVL PointCloud2，拒绝误解码");
        }
        found = true;
      }
    }
    if (!found) {throw std::runtime_error("包中没有真实STVL体素话题");}
    rosbag2_storage::StorageFilter filter;
    filter.topics = {"/costmap/voxel_grid"};
    reader.set_filter(filter);
    std::ofstream out(argv[2]);
    std::ofstream points(points_path);
    if (!out || !points) {throw std::runtime_error("无法写入CSV");}
    out << "stamp_ns,receive_ns,roi_voxels,total_voxels\n";
    points << std::setprecision(9) << "stamp_ns,receive_ns,x,y,z\n";
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> codec;
    size_t frames = 0;
    while (reader.has_next()) {
      const auto message = reader.read_next();
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      sensor_msgs::msg::PointCloud2 cloud;
      codec.deserialize_message(&serialized, &cloud);
      const int64_t stamp = int64_t(cloud.header.stamp.sec) * 1000000000LL +
        cloud.header.stamp.nanosec;
      const auto count = carcar_camera::visit_stvl_roi(cloud, bounds,
        [&](const auto & xyz) {
          points << stamp << ',' << message->time_stamp << ','
                 << xyz[0] << ',' << xyz[1] << ',' << xyz[2] << '\n';
        });
      out << stamp << ',' << message->time_stamp << ',' << count << ',' << cloud.width << '\n';
      ++frames;
    }
    out.flush();
    points.flush();
    if (!frames || !out.good() || !points.good()) {throw std::runtime_error("无帧或CSV写入失败");}
    std::cout << "完整STVL体素审计：" << frames << "帧；不代表物理空闲或健康通过\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "STVL审计失败：" << error.what() << '\n';
    return 1;
  }
}
