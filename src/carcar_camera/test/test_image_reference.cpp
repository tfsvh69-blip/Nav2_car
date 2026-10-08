#include <gtest/gtest.h>
#include "carcar_camera/image_reference.hpp"

TEST(ImageReference, DepthPaddingInvalidRangeAndHeaderArePreserved)
{
  sensor_msgs::msg::Image input;
  input.width = 4; input.height = 1; input.step = 10; input.encoding = "16UC1";
  input.header.frame_id = "camera_depth_optical_frame";
  input.header.stamp.sec = 42;
  const uint16_t values[] = {0, 500, 1500, 4000, 123};
  input.data.resize(10); std::memcpy(input.data.data(), values, 10);
  const auto result = carcar_camera::image_reference(input, true, 4, 3.0, 70);
  const auto decoded = cv::imdecode(result.data, cv::IMREAD_UNCHANGED);
  ASSERT_EQ(decoded.type(), CV_8UC1);
  EXPECT_EQ(decoded.at<uint8_t>(0, 0), 0);
  EXPECT_GT(decoded.at<uint8_t>(0, 1), decoded.at<uint8_t>(0, 2));
  EXPECT_EQ(decoded.at<uint8_t>(0, 3), 0);
  EXPECT_EQ(result.header.frame_id, input.header.frame_id);
  EXPECT_EQ(result.header.stamp.sec, 42);
  EXPECT_EQ(input.data.size(), 10U);
  input.data.pop_back();
  EXPECT_THROW(carcar_camera::image_reference(input, true, 4, 3.0, 70), std::invalid_argument);
}

TEST(ImageReference, RGBChannelOrderAndResizeUseActualPixels)
{
  sensor_msgs::msg::Image input;
  input.width = 8; input.height = 4; input.step = 24; input.encoding = "rgb8";
  input.data.assign(96, 0);
  for (size_t i = 0; i < 96; i += 3) {input.data[i] = 255;}
  const auto result = carcar_camera::image_reference(input, false, 4, 3.0, 95);
  const auto decoded = cv::imdecode(result.data, cv::IMREAD_COLOR);
  ASSERT_EQ(decoded.cols, 4); ASSERT_EQ(decoded.rows, 2);
  EXPECT_GT(decoded.at<cv::Vec3b>(0, 0)[2], 240);
  EXPECT_LT(decoded.at<cv::Vec3b>(0, 0)[0], 15);
  EXPECT_EQ(input.data[0], 255);
  input.encoding = "mono16";
  EXPECT_THROW(carcar_camera::image_reference(input, false, 4, 3.0, 70), std::invalid_argument);
}
