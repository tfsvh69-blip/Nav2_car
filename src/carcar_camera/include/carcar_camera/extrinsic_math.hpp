#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace carcar_camera {

inline double rotation_error(const tf2::Quaternion & a, const tf2::Quaternion & b)
{
  tf2::Quaternion delta = a.inverse() * b;
  delta.normalize();
  return std::abs(delta.getAngleShortestPath());
}

inline tf2::Transform average_transforms(const std::vector<tf2::Transform> & transforms)
{
  if (transforms.empty()) {
    throw std::invalid_argument("不能平均空变换集合");
  }
  tf2::Vector3 translation(0, 0, 0);
  tf2::Quaternion reference = transforms.front().getRotation();
  reference.normalize();
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
  for (const auto & transform : transforms) {
    translation += transform.getOrigin();
    tf2::Quaternion q = transform.getRotation();
    q.normalize();
    if (reference.dot(q) < 0.0) {
      q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
    }
    qx += q.x();
    qy += q.y();
    qz += q.z();
    qw += q.w();
  }
  translation /= static_cast<double>(transforms.size());
  tf2::Quaternion rotation(qx, qy, qz, qw);
  rotation.normalize();
  return tf2::Transform(rotation, translation);
}

inline tf2::Transform transform_from_xyz_rpy(
  const std::vector<double> & xyz, const std::vector<double> & rpy)
{
  if (xyz.size() != 3 || rpy.size() != 3) {
    throw std::invalid_argument("XYZ/RPY 必须各包含 3 个数");
  }
  tf2::Quaternion rotation;
  rotation.setRPY(rpy[0], rpy[1], rpy[2]);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
}

inline tf2::Transform transform_from_reference_points(
  const std::vector<double> & points, double board_width, double board_height,
  double max_size_error)
{
  if (points.size() != 9) {
    throw std::invalid_argument("board_reference_points 必须包含 9 个数");
  }
  for (const double coordinate : points) {
    if (!std::isfinite(coordinate)) {
      throw std::invalid_argument("board_reference_points 只能包含有限的米制坐标");
    }
  }
  const tf2::Vector3 origin(points[0], points[1], points[2]);
  tf2::Vector3 x_axis = tf2::Vector3(points[3], points[4], points[5]) - origin;
  tf2::Vector3 y_hint = tf2::Vector3(points[6], points[7], points[8]) - origin;
  const double x_length = x_axis.length();
  const double y_length = y_hint.length();
  if (std::abs(x_length - board_width) > max_size_error ||
    std::abs(y_length - board_height) > max_size_error ||
    x_length < 1e-6 || y_length < 1e-6)
  {
    throw std::invalid_argument("三个基准点得到的板宽或板高与配置不符");
  }
  x_axis /= x_length;
  y_hint /= y_length;
  if (std::abs(x_axis.dot(y_hint)) > 0.10) {
    throw std::invalid_argument("板 X/Y 基准边不近似垂直，请复核三个非共线点");
  }
  tf2::Vector3 z_axis = x_axis.cross(y_hint);
  z_axis.normalize();
  const tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
  tf2::Matrix3x3 rotation(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());
  tf2::Quaternion quaternion;
  rotation.getRotation(quaternion);
  quaternion.normalize();
  return tf2::Transform(quaternion, origin);
}

inline tf2::Transform solve_base_from_camera(
  const tf2::Transform & base_from_board,
  const tf2::Transform & camera_from_board)
{
  return base_from_board * camera_from_board.inverse();
}

inline tf2::Transform optical_result_to_camera_link(
  const tf2::Transform & parent_from_base,
  const tf2::Transform & base_from_optical,
  const tf2::Transform & optical_from_camera_link)
{
  return parent_from_base * base_from_optical * optical_from_camera_link;
}

}  // namespace carcar_camera
