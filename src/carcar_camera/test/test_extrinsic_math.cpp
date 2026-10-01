#include <gtest/gtest.h>
#include <limits>

#include "carcar_camera/extrinsic_math.hpp"

namespace {

tf2::Transform make_transform(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  return carcar_camera::transform_from_xyz_rpy({x, y, z}, {roll, pitch, yaw});
}

void expect_transform_near(
  const tf2::Transform & actual, const tf2::Transform & expected, double tolerance = 1e-9)
{
  EXPECT_NEAR((actual.getOrigin() - expected.getOrigin()).length(), 0.0, tolerance);
  EXPECT_NEAR(
    carcar_camera::rotation_error(actual.getRotation(), expected.getRotation()),
    0.0, tolerance);
}

}  // namespace

TEST(ExtrinsicMath, ThreeMeasuredBoardPointsDefineFullPose)
{
  const auto pose = carcar_camera::transform_from_reference_points(
    {0.5, -0.105, 0.3, 0.5, 0.105, 0.3, 0.5, -0.105, 0.15},
    0.210, 0.150, 0.010);
  EXPECT_NEAR(pose.getOrigin().x(), 0.5, 1e-9);
  EXPECT_NEAR(pose.getBasis().getColumn(0).y(), 1.0, 1e-9);
  EXPECT_NEAR(pose.getBasis().getColumn(1).z(), -1.0, 1e-9);
  EXPECT_NEAR(pose.getBasis().getColumn(2).x(), -1.0, 1e-9);
}

TEST(ExtrinsicMath, RejectsWrongBoardSizeAndNearlyParallelEdges)
{
  EXPECT_THROW(
    carcar_camera::transform_from_reference_points(
      {0.0, 0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 0.15, 0.0},
      0.210, 0.150, 0.010),
    std::invalid_argument);
  EXPECT_THROW(
    carcar_camera::transform_from_reference_points(
      {0.0, 0.0, 0.0, 0.21, 0.0, 0.0, 0.149, 0.017, 0.0},
      0.210, 0.150, 0.010),
    std::invalid_argument);
}

TEST(ExtrinsicMath, RejectsNonFiniteMeasuredCoordinates)
{
  EXPECT_THROW(
    carcar_camera::transform_from_reference_points(
      {0.0, 0.0, 0.0, 0.21, 0.0, 0.0, 0.0, 0.15,
        std::numeric_limits<double>::quiet_NaN()},
      0.210, 0.150, 0.010),
    std::invalid_argument);
}

TEST(ExtrinsicMath, InvertsCameraFromBoardInTheCorrectDirection)
{
  const auto expected_base_from_camera = make_transform(0.13, 0.033, 0.14, 3.1, 0.08, -0.04);
  const auto base_from_board = make_transform(0.55, -0.08, 0.24, 0.2, -0.3, 2.8);
  const auto camera_from_board = expected_base_from_camera.inverse() * base_from_board;
  expect_transform_near(
    carcar_camera::solve_base_from_camera(base_from_board, camera_from_board),
    expected_base_from_camera);
}

TEST(ExtrinsicMath, ConvertsOpticalResultBackToInvertedCameraLink)
{
  const auto parent_from_base = make_transform(0.0, 0.0, -0.03, 0.0, 0.0, 0.0);
  const auto expected_parent_from_link = make_transform(0.13, 0.033, 0.14, 3.141592653589793, 0.05, -0.02);
  const auto link_from_optical = make_transform(
    0.01, 0.0, 0.0, -1.5707963267948966, 0.0, -1.5707963267948966);
  const auto optical_from_link = link_from_optical.inverse();
  const auto base_from_optical =
    parent_from_base.inverse() * expected_parent_from_link * link_from_optical;
  expect_transform_near(
    carcar_camera::optical_result_to_camera_link(
      parent_from_base, base_from_optical, optical_from_link),
    expected_parent_from_link);
}
