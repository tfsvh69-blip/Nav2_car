#pragma once
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>
#include "carcar_mapping_autonomy/task.hpp"

namespace carcar_mapping_autonomy {
// SLAM位姿只随接纳扫描更新；用两时刻的轮式TF对齐当前扫描，不能复用旧位姿直接评分。
// 历史端可使用随SLAM位姿保存的同时间戳TF，避免长时间静止后历史滑出缓存。
// 未保存的历史或当前扫描TF缺失、时间倒序均拒绝，不回退到最新TF。
inline Pose pose_at_scan(const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const builtin_interfaces::msg::Time & scan_stamp, tf2::BufferCore & transforms,
  const geometry_msgs::msg::TransformStamped * accepted_odometry=nullptr)
{
  const auto ns=[](const auto & stamp){return int64_t(stamp.sec)*1000000000LL+stamp.nanosec;};
  const auto pose_ns=ns(pose.header.stamp),scan_ns=ns(scan_stamp);
  if(pose.header.frame_id!="map"||pose_ns<=0||scan_ns<pose_ns)
    throw std::runtime_error("检查点扫描与SLAM位姿的坐标系或时间顺序无效");
  if(accepted_odometry&&(accepted_odometry->header.frame_id!="odom"||accepted_odometry->child_frame_id!="base_footprint"||
    ns(accepted_odometry->header.stamp)!=pose_ns))
    throw std::runtime_error("检查点缓存轮式TF与SLAM位姿的时间或坐标系不匹配");
  const auto before=accepted_odometry?*accepted_odometry:
    transforms.lookupTransform("odom","base_footprint",tf2::TimePoint(std::chrono::nanoseconds(pose_ns)));
  const auto after=transforms.lookupTransform("odom","base_footprint",tf2::TimePoint(std::chrono::nanoseconds(scan_ns)));
  const auto as_tf=[](const auto & transform){const auto & q=transform.rotation;const auto & t=transform.translation;
    return tf2::Transform(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(t.x,t.y,t.z));};
  const auto & p=pose.pose.pose.position;const auto & q=pose.pose.pose.orientation;
  const tf2::Transform accepted(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(p.x,p.y,p.z));
  const auto aligned=accepted*as_tf(before.transform).inverse()*as_tf(after.transform);
  const auto & v=aligned.getOrigin();const auto & r=aligned.getRotation();
  Pose result{v.x(),v.y(),std::atan2(2*(r.w()*r.z()+r.x()*r.y()),1-2*(r.y()*r.y()+r.z()*r.z()))};
  if(!std::isfinite(result.x)||!std::isfinite(result.y)||!std::isfinite(result.yaw))
    throw std::runtime_error("检查点扫描时间位姿包含非有限数值");
  return result;
}
}  // namespace carcar_mapping_autonomy
