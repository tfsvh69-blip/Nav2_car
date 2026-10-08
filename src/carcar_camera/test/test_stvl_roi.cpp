#include <limits>
#include <vector>
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "carcar_camera/stvl_roi.hpp"

TEST(StvlRoi, CountsOnlyFullThreeDimensionalRegionAndAcceptsEmptyUpdate)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_footprint";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(4);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (int i = 0; i < 4; ++i, ++x, ++y, ++z) {
    *x = i == 1 ? 1.5F : 1.15F;
    *y = i == 2 ? 0.5F : 0.0F;
    *z = i == 3 ? 0.01F : 0.15F;
  }
  const std::array<double, 6> roi{1.07, 1.24, -0.07, 0.12, 0.05, 0.26};
  EXPECT_EQ(carcar_camera::count_stvl_roi(cloud, roi), 1U);
  std::vector<std::array<float, 3>> visited;
  EXPECT_EQ(carcar_camera::visit_stvl_roi(cloud, roi,
    [&](const auto & xyz) {visited.push_back(xyz);}), 1U);
  ASSERT_EQ(visited.size(), 1U);
  EXPECT_FLOAT_EQ(visited.front()[0], 1.15F);
  EXPECT_FLOAT_EQ(visited.front()[1], 0.0F);
  EXPECT_FLOAT_EQ(visited.front()[2], 0.15F);
  auto malformed = cloud;
  malformed.data.pop_back();
  EXPECT_THROW(carcar_camera::count_stvl_roi(malformed, roi), std::runtime_error);
  malformed = cloud;
  malformed.header.frame_id = "camera_depth_optical_frame";
  EXPECT_THROW(carcar_camera::count_stvl_roi(malformed, roi), std::runtime_error);
  malformed = cloud;
  malformed.fields.front().offset = cloud.point_step;
  EXPECT_THROW(carcar_camera::count_stvl_roi(malformed, roi), std::runtime_error);
  sensor_msgs::PointCloud2Iterator<float> invalid_x(cloud, "x");
  *invalid_x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(carcar_camera::count_stvl_roi(cloud, roi), std::runtime_error);
  modifier.resize(0);
  EXPECT_EQ(carcar_camera::count_stvl_roi(cloud, roi), 0U);
  auto invalid_roi = roi;
  invalid_roi[5] = invalid_roi[4];
  EXPECT_THROW(carcar_camera::count_stvl_roi(cloud, invalid_roi), std::invalid_argument);
}
