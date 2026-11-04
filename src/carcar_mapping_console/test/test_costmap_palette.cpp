#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <cstdlib>
#include "carcar_mapping_console/navigation_view.hpp"

TEST(CostmapPalette, RvizColorsSurvivePngEncoding) {
  carcar_mapping_console::NavigationView view;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 101; grid.info.height = 2; grid.info.resolution = 0.05;
  grid.info.origin.orientation.w = 1;
  grid.data.assign(202, -1);
  for (int cost = 0; cost <= 100; ++cost) grid.data[cost] = cost;
  for (size_t layer = 0; layer < 2; ++layer) {
    view.grid(layer, grid, 10);
    const auto png = view.image(layer, buffer, 10, 10);
    const auto pixels = cv::imdecode(png.bytes, cv::IMREAD_UNCHANGED);
    ASSERT_EQ(pixels.rows, 2);
    const int alpha = layer == 0 ? 102 : 140;
    // 低到高代价必须改变色相，不能仅改变同一颜色的透明度。
    for (int cost = 1; cost <= 98; ++cost) {
      EXPECT_EQ(pixels.at<cv::Vec4b>(1, cost), cv::Vec4b(255 - 255 * cost / 100, 0, 255 * cost / 100, alpha));
    }
    EXPECT_EQ(pixels.at<cv::Vec4b>(1, 99), cv::Vec4b(255, 255, 0, alpha));
    EXPECT_EQ(pixels.at<cv::Vec4b>(1, 100), cv::Vec4b(255, 0, 255, alpha));
    EXPECT_EQ(pixels.at<cv::Vec4b>(1, 0)[3], 0);
    // 未知区域沿用底图，不能伪造为高代价红块。
    EXPECT_EQ(pixels.at<cv::Vec4b>(0, 0)[3], 0);
  }
}

TEST(CostmapPalette, RecordedGridReplay) {
  const char * input = std::getenv("CARCAR_COSTMAP_GRID");
  if (!input) GTEST_SKIP() << "未指定现场只读栅格快照";
  std::ifstream stream(input);
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map"; grid.info.resolution = 0.05;
  grid.info.origin.orientation.w = 1;
  std::string line; bool data = false;
  while (std::getline(stream, line)) {
    if (line.rfind("  width: ", 0) == 0) grid.info.width = std::stoul(line.substr(9));
    if (line.rfind("  height: ", 0) == 0) grid.info.height = std::stoul(line.substr(10));
    if (line == "data:") data = true;
    else if (data && line.rfind("- ", 0) == 0) grid.data.push_back(std::stoi(line.substr(2)));
  }
  ASSERT_GT(grid.info.width, 0u);
  ASSERT_EQ(grid.data.size(), grid.info.width * grid.info.height);
  carcar_mapping_console::NavigationView view;
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  view.grid(0, grid, 10);
  const auto image = view.image(0, buffer, 10, 10);
  ASSERT_FALSE(image.bytes.empty());
  if (const char * output = std::getenv("CARCAR_COSTMAP_PNG")) {
    std::ofstream png(output, std::ios::binary);
    png.write(reinterpret_cast<const char *>(image.bytes.data()), image.bytes.size());
    ASSERT_TRUE(png.good());
  }
}

TEST(CostmapPalette, IndependentLayersPreserveObstacleAndMaskIdentity) {
  carcar_mapping_console::NavigationView view;
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id="map";grid.info.width=3;grid.info.height=1;grid.info.resolution=.05;grid.info.origin.orientation.w=1;
  grid.data={100,100,100};view.grid(0,grid,10);view.grid(3,grid,10);
  grid.data={100,40,0};view.grid(2,grid,10);
  auto image=view.image(2,buffer,10,10);
  auto decoded=cv::imdecode(image.bytes,cv::IMREAD_UNCHANGED);
  EXPECT_EQ(decoded.at<cv::Vec4b>(0,0),cv::Vec4b(255,0,255,102));
  EXPECT_EQ(decoded.at<cv::Vec4b>(0,1),cv::Vec4b(153,0,102,102));
  EXPECT_EQ(decoded.at<cv::Vec4b>(0,2)[3],0);
  // 静态掩膜事件不能被0.4秒连续图片限频丢弃。
  grid.data={0,100,0};view.grid(3,grid,10.01);
  auto mask=cv::imdecode(view.image(3,buffer,10.02,10.02).bytes,cv::IMREAD_UNCHANGED);
  EXPECT_EQ(mask.at<cv::Vec4b>(0,0)[3],0);EXPECT_GT(mask.at<cv::Vec4b>(0,1)[3],0);
  auto final=cv::imdecode(view.image(0,buffer,10,10).bytes,cv::IMREAD_UNCHANGED);
  EXPECT_EQ(final.at<cv::Vec4b>(0,2),cv::Vec4b(255,0,255,102));
  EXPECT_TRUE(view.snapshot(buffer,10,10)["obstacle_costmap"]["available"]);
  EXPECT_TRUE(view.snapshot(buffer,10,10)["keepout_mask"]["available"]);
}
