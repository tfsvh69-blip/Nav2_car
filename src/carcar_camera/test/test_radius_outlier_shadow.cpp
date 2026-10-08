#include <algorithm>
#include <stdexcept>

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
  EXPECT_THROW(carcar_camera::radius_outlier_shadow_mask(input, 0.05, 0), std::invalid_argument);
  EXPECT_THROW(carcar_camera::radius_outlier_shadow_mask(input, 0.30, 2), std::invalid_argument);
}

TEST(RadiusOutlierShadow, MaskMarksExactlyTheRemovedPointAndKeepsOrder)
{
  auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 10; ++i) {
    input->emplace_back(0.8F, 0.2F, 0.02F * i);
  }
  // 孤立点放在中间，用来确认掩码下标与输入顺序一致，而不是被重新排序。
  input->emplace_back(0.5F, -0.2F, 0.1F);
  for (int i = 0; i < 10; ++i) {
    input->emplace_back(1.2F, 0.2F, 0.02F * i);
  }
  input->width = static_cast<uint32_t>(input->size());
  input->height = 1;
  input->is_dense = true;

  const auto mask = carcar_camera::radius_outlier_shadow_mask(input, 0.05, 2);
  ASSERT_EQ(mask.size(), input->size());
  EXPECT_FALSE(mask[10]);
  for (size_t i = 0; i < mask.size(); ++i) {
    if (i != 10U) {EXPECT_TRUE(mask[i]) << "下标 " << i << " 不应被删除";}
  }

  const auto kept = carcar_camera::radius_outlier_shadow(input, 0.05, 2);
  const size_t kept_in_mask = static_cast<size_t>(std::count(mask.begin(), mask.end(), true));
  EXPECT_EQ(kept_in_mask, kept.size());
}

TEST(RadiusOutlierShadow, MaskKeepsDenseSurfaceEntirely)
{
  // 稠密平面：每个点都有很多邻居，任何合理参数都不应删除，对应实车墙面/垃圾桶表面。
  auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      input->emplace_back(1.0F + 0.005F * i, -0.4F + 0.005F * j, 0.05F);
    }
  }
  input->width = static_cast<uint32_t>(input->size());
  input->height = 1;
  input->is_dense = true;

  const auto mask = carcar_camera::radius_outlier_shadow_mask(input, 0.05, 2);
  EXPECT_EQ(static_cast<size_t>(std::count(mask.begin(), mask.end(), true)), input->size());
}
