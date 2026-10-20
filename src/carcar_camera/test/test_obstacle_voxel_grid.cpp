#include <algorithm>
#include <limits>
#include <gtest/gtest.h>
#include "carcar_camera/obstacle_voxel_grid.hpp"

using carcar_camera::downsample_obstacles;

TEST(ObstacleVoxelGrid, UsesCentroidAndKeepsSparseCells)
{
  const auto output = downsample_obstacles(
    {{0.501F, 0.001F, 0.101F}, {0.503F, 0.003F, 0.103F}, {0.551F, 0.001F, 0.101F}}, 0.01);
  ASSERT_EQ(output.size(), 2U);
  const auto centroid = std::min_element(output.begin(), output.end(),
    [](const auto & a, const auto & b) {return a.x < b.x;});
  EXPECT_NEAR(centroid->x, 0.502, 1e-6);
  EXPECT_NEAR(centroid->y, 0.002, 1e-6);
  EXPECT_NEAR(centroid->z, 0.102, 1e-6);
}

TEST(ObstacleVoxelGrid, KeepsNegativeAndPositiveCellsSeparate)
{
  EXPECT_EQ(downsample_obstacles({{-0.001F, 0, 1}, {0.001F, 0, 1}}, 0.01).size(), 2U);
}

TEST(ObstacleVoxelGrid, EmptyFrameStaysEmptyAndInvalidLeafIsRejected)
{
  EXPECT_TRUE(downsample_obstacles({}, 0.01).empty());
  for (const double leaf : {0.0, -0.01, 0.0001, 0.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(downsample_obstacles({}, leaf), std::invalid_argument);
  }
}
