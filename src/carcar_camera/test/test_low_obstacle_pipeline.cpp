#include <gtest/gtest.h>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include "carcar_camera/depth_logic.hpp"

TEST(LowObstaclePipeline, TwoCentimetresRequiresLowerMarkingHeight)
{
  carcar_camera::FilterLimits limits;
  EXPECT_FALSE(carcar_camera::classify_point(0, 0, 0.7F, 0.8, 0, 0.02, limits).obstacle);
  limits.min_obstacle_height = 0.01;
  limits.max_range = 0.85;
  EXPECT_TRUE(carcar_camera::valid_limits(limits));
  EXPECT_TRUE(carcar_camera::classify_point(0, 0, 0.7F, 0.8, 0, 0.02, limits).obstacle);
  EXPECT_FALSE(carcar_camera::classify_point(0, 0, 0.7F, 0.8, 0, 0.0091, limits).obstacle);
}

TEST(LowObstaclePipeline, RealNav2SingleVoxelColumnNeedsZeroThreshold)
{
  // 使用已安装Humble原生VoxelGrid，证明只有一层体素的低物体不会被threshold=1放行。
  nav2_voxel_grid::VoxelGrid grid(1, 1, 16);
  EXPECT_FALSE(grid.markVoxelInMap(0, 0, 1, 1));
  EXPECT_FALSE(grid.markVoxelInMap(0, 0, 1, 1));
  EXPECT_TRUE(grid.markVoxelInMap(0, 0, 1, 0));
  EXPECT_TRUE(grid.markVoxelInMap(0, 0, 2, 1));
}

TEST(LowObstaclePipeline, NearMarkingLimitKeepsMeasuredBackgroundForClearing)
{
  carcar_camera::FilterLimits limits;
  limits.min_obstacle_height = 0.01;
  limits.max_range = 0.85;
  limits.max_clearing_range = 2.0;
  auto far_noise = carcar_camera::classify_point(0, 0, 1.6F, 1.7, 0, 0.024, limits);
  EXPECT_TRUE(far_noise.valid_for_clearing);
  EXPECT_FALSE(far_noise.obstacle);
  EXPECT_FALSE(carcar_camera::classify_point(0, 0, 2.1F, 2.2, 0, 0, limits).valid_for_clearing);
}
