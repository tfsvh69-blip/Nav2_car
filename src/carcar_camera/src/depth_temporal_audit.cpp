// Copyright 2026 carcar maintainers. Apache-2.0.
// 只读未压缩静止包，对未启用时间确认的过滤输入做影子对照；不发布、不连接硬件。
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include "carcar_camera/temporal_confirmation.hpp"

template<typename Message>
Message decode(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

int main(int argc, char ** argv)
{
  if (argc != 9) {
    std::cerr << "用法：depth_temporal_audit <未压缩bag> <全新CSV> "
      "<xmin> <xmax> <ymin> <ymax> <zmin> <zmax>\n";
    return 2;
  }
  try {
    if (std::filesystem::exists(argv[2])) {throw std::runtime_error("输出已存在，拒绝覆盖");}
    std::array<double, 6> bounds{};
    for (size_t i = 0; i < bounds.size(); ++i) {
      size_t parsed = 0;
      const std::string value(argv[i + 3]);
      bounds[i] = std::stod(value, &parsed);
      if (parsed != value.size() || !std::isfinite(bounds[i])) {
        throw std::invalid_argument("ROI边界无效");
      }
    }
    for (size_t i = 0; i < 3; ++i) {
      if (bounds[2 * i] >= bounds[2 * i + 1]) {throw std::invalid_argument("ROI上下界无效");}
    }
    tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    bool found = false;
    for (const auto & topic : reader.get_all_topics_and_types()) {
      if (topic.name == "/camera/navigation/obstacles") {
        if (topic.type != "sensor_msgs/msg/PointCloud2") {
          throw std::runtime_error("标障话题类型错误");
        }
        found = true;
      }
    }
    if (!found) {throw std::runtime_error("没有标障输入话题");}
    rosbag2_storage::StorageFilter filter;
    filter.topics = {"/tf_static"};
    reader.set_filter(filter);
    while (reader.has_next()) {
      for (const auto & transform : decode<tf2_msgs::msg::TFMessage>(*reader.read_next()).transforms) {
        if (!transforms.setTransform(transform, "depth_temporal_audit", true)) {
          throw std::runtime_error("静态TF无效");
        }
      }
    }
    rosbag2_cpp::Reader clouds;
    clouds.open(argv[1]);
    filter.topics = {"/camera/navigation/obstacles"};
    clouds.set_filter(filter);
    carcar_camera::TemporalConfirmation confirmation(0.02, 3, 2);
    std::ofstream out(argv[2]);
    if (!out) {throw std::runtime_error("无法创建CSV");}
    out << "stamp_ns,receive_ns,roi_before,roi_after,total_before,total_after,max_gap_s\n";
    out << std::setprecision(12);
    int64_t frame = 0, previous_stamp = 0;
    double max_gap = 0;
    while (clouds.has_next()) {
      const auto message = clouds.read_next();
      const auto cloud = decode<sensor_msgs::msg::PointCloud2>(*message);
      if (cloud.header.frame_id.empty() || cloud.is_bigendian || cloud.height != 1 ||
        cloud.point_step < 12 || cloud.row_step != size_t(cloud.width) * cloud.point_step ||
        cloud.data.size() != cloud.row_step)
      {throw std::runtime_error("输入点云缓冲无效");}
      for (const auto * name : {"x", "y", "z"}) {
        bool valid = false;
        for (const auto & field : cloud.fields) {
          if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            field.count == 1 && size_t(field.offset) + sizeof(float) <= cloud.point_step)
          {valid = true;}
        }
        if (!valid) {throw std::runtime_error("XYZ字段无效");}
      }
      const int64_t stamp = int64_t(cloud.header.stamp.sec) * 1000000000LL + cloud.header.stamp.nanosec;
      if (previous_stamp && stamp <= previous_stamp) {throw std::runtime_error("输入时间戳非递增");}
      if (previous_stamp) {max_gap = std::max(max_gap, (stamp - previous_stamp) / 1e9);}
      previous_stamp = stamp;
      tf2::Transform base_from_sensor;
      tf2::fromMsg(transforms.lookupTransform("base_footprint", cloud.header.frame_id,
        tf2::TimePointZero).transform, base_from_sensor);
      confirmation.begin_frame(++frame);
      size_t before = 0, after = 0, total_after = 0;
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          throw std::runtime_error("标障输入包含非有限点");
        }
        const auto base = base_from_sensor * tf2::Vector3(*x, *y, *z);
        const bool inside = base.x() >= bounds[0] && base.x() <= bounds[1] &&
          base.y() >= bounds[2] && base.y() <= bounds[3] && base.z() >= bounds[4] && base.z() <= bounds[5];
        const bool accepted = confirmation.accept(base.x(), base.y(), base.z(), frame);
        before += inside;
        after += inside && accepted;
        total_after += accepted;
      }
      out << stamp << ',' << message->time_stamp << ',' << before << ',' << after << ','
        << cloud.width << ',' << total_after << ',' << max_gap << '\n';
    }
    out.flush();
    if (!frame || !out.good()) {throw std::runtime_error("无帧或写入失败");}
    std::cout << "时间确认影子审计：" << frame << "帧；0.02 m / 3帧 / 2次命中。"
      "仅静止未确认输入适用，不证明实机健康、STVL结果或移动目标保留。\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "时间确认审计失败：" << error.what() << '\n';
    return 1;
  }
}
