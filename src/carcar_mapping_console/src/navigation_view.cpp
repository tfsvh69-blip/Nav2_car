#include "carcar_mapping_console/navigation_view.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

namespace carcar_mapping_console {
namespace {
double stamp_of(const builtin_interfaces::msg::Time & value) {return value.sec + value.nanosec * 1e-9;}
double heading(const geometry_msgs::msg::Quaternion & q) {return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));}
struct Pose2 {double x = 0, y = 0, yaw = 0; bool ok = false; std::string error;};
Pose2 lookup(tf2_ros::Buffer & buffer, const std::string & frame) {
  if (frame == "map") return {};
  try {
    if (frame.empty()) throw std::runtime_error("消息缺少坐标系");
    const auto transform = buffer.lookupTransform("map", frame, tf2::TimePointZero);
    return {transform.transform.translation.x, transform.transform.translation.y, heading(transform.transform.rotation), true, {}};
  } catch (const std::exception & error) {return {0, 0, 0, false, std::string("坐标转换未就绪：") + error.what()};}
}
std::array<double, 2> apply(double x, double y, const Pose2 & pose) {
  if (!pose.ok && pose.error.empty()) return {x, y};
  return {pose.x + std::cos(pose.yaw) * x - std::sin(pose.yaw) * y, pose.y + std::sin(pose.yaw) * x + std::cos(pose.yaw) * y};
}
}

void NavigationView::grid(size_t index, const nav_msgs::msg::OccupancyGrid & value, double at) {
  if (index >= 2 || !value.info.width || !value.info.height || !std::isfinite(value.info.resolution) || value.info.resolution <= 0 ||
    static_cast<uint64_t>(value.info.width) * value.info.height > 4000000 ||
    value.data.size() != static_cast<size_t>(value.info.width) * value.info.height) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (grids_[index].received && at - grids_[index].at < 0.4) return;
  }
  cv::Mat pixels(value.info.height, value.info.width, CV_8UC4, cv::Scalar(0, 0, 0, 0));
  for (uint32_t y = 0; y < value.info.height; ++y) for (uint32_t x = 0; x < value.info.width; ++x) {
    const auto cost = value.data[y * value.info.width + x];
    if (cost <= 0) continue;
    auto & pixel = pixels.at<cv::Vec4b>(value.info.height - 1 - y, x);
    pixel = cost >= 99 ? cv::Vec4b(55, 45, 225, 210) : index == 0 ?
      cv::Vec4b(210, 60, 135, static_cast<uint8_t>(45 + cost)) :
      cv::Vec4b(25, 145, 245, static_cast<uint8_t>(45 + cost));
  }
  std::vector<unsigned char> encoded;
  if (!cv::imencode(".png", pixels, encoded, {cv::IMWRITE_PNG_COMPRESSION, 1})) return;
  std::lock_guard<std::mutex> lock(mutex_);
  auto & layer = grids_[index];
  layer.received = true; layer.at = at; layer.stamp = stamp_of(value.header.stamp); ++layer.sequence;
  layer.frame = value.header.frame_id; layer.info = value.info; layer.png = std::move(encoded);
}

void NavigationView::path(size_t index, const nav_msgs::msg::Path & value, double at) {
  if (index >= 3) return;
  nlohmann::json points = nlohmann::json::array();
  const size_t stride = std::max<size_t>(1, (value.poses.size() + 399) / 400);
  for (size_t i = 0; i < value.poses.size(); i += stride) {
    const auto & pose = value.poses[i];
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != value.header.frame_id) ||
      !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) return;
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  if (!value.poses.empty()) {
    const auto & back = value.poses.back().pose.position;
    if (std::isfinite(back.x) && std::isfinite(back.y)) points.push_back({back.x, back.y});
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto & layer = paths_[index];
  layer.received = true; layer.at = at; layer.stamp = stamp_of(value.header.stamp); ++layer.sequence;
  layer.frame = value.header.frame_id; layer.points = std::move(points);
}

nlohmann::json NavigationView::grid_info(const Grid & layer, tf2_ros::Buffer & buffer, double at, double ros_time) const {
  nlohmann::json info{{"available", layer.received}, {"source_frame", layer.frame}, {"frame", "map"},
    {"sequence", layer.sequence}, {"age_seconds", layer.received ? at - layer.at : 0.0},
    {"source_age_seconds", layer.received ? ros_time - layer.stamp : 0.0}, {"error", layer.received ? "" : "尚未收到数据"}};
  if (!layer.received) return info;
  const auto pose = lookup(buffer, layer.frame);
  if (!layer.frame.empty() && layer.frame != "map" && !pose.ok) {info["available"] = false; info["error"] = pose.error; return info;}
  const auto origin = apply(layer.info.origin.position.x, layer.info.origin.position.y, pose);
  info["width"] = layer.info.width; info["height"] = layer.info.height; info["resolution"] = layer.info.resolution;
  info["origin"] = {{"x", origin[0]}, {"y", origin[1]}, {"yaw", heading(layer.info.origin.orientation) + pose.yaw}};
  return info;
}

nlohmann::json NavigationView::path_info(const PathLayer & layer, tf2_ros::Buffer & buffer, double at, double ros_time) const {
  const bool fresh = layer.received && at - layer.at <= 1.5;
  nlohmann::json info{{"available", fresh}, {"source_frame", layer.frame}, {"frame", "map"},
    {"sequence", layer.sequence}, {"age_seconds", layer.received ? at - layer.at : 0.0},
    {"source_age_seconds", layer.received ? ros_time - layer.stamp : 0.0},
    {"error", !layer.received ? "尚未收到数据" : fresh ? "" : "路径已结束或超过1.5秒没有更新"}, {"points", nlohmann::json::array()}};
  if (!fresh) return info;
  const auto pose = lookup(buffer, layer.frame);
  if (!layer.frame.empty() && layer.frame != "map" && !pose.ok) {info["available"] = false; info["error"] = pose.error; return info;}
  for (const auto & point : layer.points) {
    const auto mapped = apply(point[0], point[1], pose);
    info["points"].push_back({mapped[0], mapped[1]});
  }
  return info;
}

nlohmann::json NavigationView::snapshot(tf2_ros::Buffer & buffer, double at, double ros_time) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const char * grid_names[] = {"global_costmap", "local_costmap"};
  const char * path_names[] = {"global_path", "local_path", "prediction"};
  nlohmann::json result;
  for (size_t i = 0; i < 2; ++i) result[grid_names[i]] = grid_info(grids_[i], buffer, at, ros_time);
  for (size_t i = 0; i < 3; ++i) result[path_names[i]] = path_info(paths_[i], buffer, at, ros_time);
  return result;
}

NavigationView::Image NavigationView::image(size_t index, tf2_ros::Buffer & buffer, double at, double ros_time) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= 2) return {};
  return {grids_[index].png, grid_info(grids_[index], buffer, at, ros_time)};
}
}  // namespace carcar_mapping_console
