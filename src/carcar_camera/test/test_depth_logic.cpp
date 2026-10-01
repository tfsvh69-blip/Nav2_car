#include <gtest/gtest.h>
#include "carcar_camera/depth_logic.hpp"

TEST(DepthFilter, SeparatesMarkingClearingAndSelf)
{
  carcar_camera::FilterLimits l;
  auto obstacle = carcar_camera::classify_point(0.0F, 0.0F, 1.0F, 0.8, 0.0, 0.10, l);
  EXPECT_TRUE(obstacle.valid_for_clearing);
  EXPECT_TRUE(obstacle.obstacle);
  auto table = carcar_camera::classify_point(0.0F, 0.0F, 1.0F, 0.8, 0.0, 0.70, l);
  EXPECT_TRUE(table.valid_for_clearing);
  EXPECT_FALSE(table.obstacle);
  auto self = carcar_camera::classify_point(0.0F, 0.0F, 0.5F, 0.1, 0.0, 0.10, l);
  EXPECT_TRUE(self.valid_for_clearing);
  EXPECT_FALSE(self.obstacle);
}

TEST(DepthFilter, RejectsInvalidAndOutOfRange)
{
  carcar_camera::FilterLimits l;
  EXPECT_FALSE(carcar_camera::classify_point(NAN, 0, 1, 1, 0, 0.1, l).valid_for_clearing);
  EXPECT_FALSE(carcar_camera::classify_point(0, 0, 0.2F, 0.2, 0, 0.1, l).valid_for_clearing);
  EXPECT_FALSE(carcar_camera::classify_point(0, 0, 2.1F, 2.1, 0, 0.1, l).valid_for_clearing);
}

TEST(DepthFilter, DistinguishesValidEmptySceneFromInvalidDepth)
{
  EXPECT_TRUE(carcar_camera::valid_depth_measurement(0.0F, 0.0F, 3.0F));
  EXPECT_FALSE(carcar_camera::valid_depth_measurement(NAN, 0.0F, 1.0F));
  EXPECT_FALSE(carcar_camera::valid_depth_measurement(0.0F, 0.0F, 0.0F));
}

TEST(DepthHealth, RequiresContinuousRecoveryWindow)
{
  carcar_camera::HealthStateMachine sm(3.0);
  EXPECT_EQ(sm.update(false, 0.0), carcar_camera::FusionState::LASER_ONLY);
  EXPECT_EQ(sm.update(true, 1.0), carcar_camera::FusionState::RECOVERING);
  EXPECT_EQ(sm.update(true, 3.9), carcar_camera::FusionState::RECOVERING);
  EXPECT_EQ(sm.update(true, 4.0), carcar_camera::FusionState::FUSED);
  EXPECT_EQ(sm.update(false, 4.1), carcar_camera::FusionState::LASER_ONLY);
  EXPECT_EQ(sm.update(true, 4.2), carcar_camera::FusionState::RECOVERING);
}

TEST(DepthFilter, FarMeasuredBackgroundClearsWithoutExtendingObstacleRange)
{
  carcar_camera::FilterLimits limits;
  limits.max_clearing_range = 4.0;
  const auto far = carcar_camera::classify_point(0, 0, 3.0F, 3.0, 0, 0.10, limits);
  EXPECT_TRUE(far.valid_for_clearing);
  EXPECT_FALSE(far.obstacle);
  EXPECT_FALSE(carcar_camera::classify_point(0, 0, 4.1F, 4.1, 0, 0, limits).valid_for_clearing);
  EXPECT_FALSE(carcar_camera::classify_point(NAN, 0, 3, 3, 0, 0, limits).valid_for_clearing);
  const auto floor = carcar_camera::classify_point(0, 0, 1, 1, 0, -0.01, limits);
  EXPECT_TRUE(floor.valid_for_clearing);
  EXPECT_FALSE(floor.obstacle);
  limits.max_clearing_range = 1.0;
  EXPECT_FALSE(carcar_camera::valid_limits(limits));
}
