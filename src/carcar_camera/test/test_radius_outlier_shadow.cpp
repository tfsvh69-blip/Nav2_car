#include <gtest/gtest.h>

#include "carcar_camera/radius_outlier_shadow.hpp"

TEST(RadiusOutlierShadow, RemovesIsolatedPointAndKeepsThinLeg)
{
  auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 10; ++i) {
    input->emplace_back(0.8F, 0.2F, 0.02F * i);
  }
  input->emplace_back(0.5F, -0.2F, 0.1F);
  input->width = static_cast<uint32_t>(input->size());
  input->height = 1;
  input->is_dense = true;

  const auto kept = carcar_camera::radius_outlier_shadow(input, 0.05, 2);
  EXPECT_EQ(kept.size(), 10U);
  for (const auto & point : kept) {
    EXPECT_FLOAT_EQ(point.x, 0.8F);
  }
}

TEST(RadiusOutlierShadow, RejectsInvalidParameter)
{
  auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  EXPECT_THROW(carcar_camera::radius_outlier_shadow(input, 0.0, 2), std::invalid_argument);
  EXPECT_THROW(carcar_camera::radius_outlier_shadow(input, 0.04, 0), std::invalid_argument);
}
