#include <limits>

#include "gtest/gtest.h"
#include "carcar_navigation/imu_gyro_bias.hpp"

TEST(ImuGyroBias, SubtractsOnlyConfiguredBias)
{
  const auto result=carcar_navigation::correct_gyro_z(.12,.02,2.0);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.corrected_z,.10,1e-12);
}

TEST(ImuGyroBias, RejectsNonfiniteAndOutOfRangeValues)
{
  EXPECT_FALSE(carcar_navigation::correct_gyro_z(
    std::numeric_limits<double>::quiet_NaN(),0.0,2.0).valid);
  EXPECT_FALSE(carcar_navigation::correct_gyro_z(3.0,0.0,2.0).valid);
  EXPECT_FALSE(carcar_navigation::correct_gyro_z(0.0,0.0,0.0).valid);
}
