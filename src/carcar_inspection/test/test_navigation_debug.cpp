#include <chrono>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "carcar_inspection/navigation_debug.hpp"
namespace {
class NavigationDebugTest:public ::testing::Test {
 protected:
  void SetUp()override {rclcpp::init(0,nullptr);buffer=std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));}
  void TearDown()override{buffer.reset();rclcpp::shutdown();}
  std::unique_ptr<tf2_ros::Buffer> buffer;
  carcar_inspection::NavigationDebug display;
  nav_msgs::msg::OccupancyGrid grid(){nav_msgs::msg::OccupancyGrid value;value.header.frame_id="odom";value.header.stamp.sec=8;
    value.info.width=2;value.info.height=3;value.info.resolution=.1;value.info.origin.position.x=1;value.info.origin.position.y=2;
    value.info.origin.orientation.w=1;value.data={100,0,50,-1,0,0};return value;}
  void transform(){geometry_msgs::msg::TransformStamped value;value.header.frame_id="map";value.child_frame_id="odom";
    value.transform.translation.x=10;value.transform.translation.y=-3;value.transform.rotation.z=std::sin(M_PI/4);value.transform.rotation.w=std::cos(M_PI/4);
    ASSERT_TRUE(buffer->setTransform(value,"offline_test",true));}
};
TEST_F(NavigationDebugTest, RotatesLocalGridAndPairsImageGeometryWithPixels){
  transform();display.grid(1,grid(),10);ASSERT_TRUE(display.wait_for(1,std::chrono::seconds(2)));const auto image=display.image(1,*buffer,11,12);
  ASSERT_TRUE(image.second.at("available"));EXPECT_NEAR(image.second["origin"]["x"].get<double>(),8,1e-8);
  EXPECT_NEAR(image.second["origin"]["y"].get<double>(),-2,1e-8);EXPECT_NEAR(image.second["origin"]["yaw"].get<double>(),M_PI/2,1e-8);
  EXPECT_EQ(image.second["source_frame"],"odom");EXPECT_NEAR(image.second["source_age_seconds"].get<double>(),4,1e-8);
  const auto pixels=cv::imdecode(image.first,cv::IMREAD_UNCHANGED);ASSERT_EQ(pixels.rows,3);ASSERT_EQ(pixels.cols,2);ASSERT_EQ(pixels.channels(),4);
  EXPECT_GT(pixels.at<cv::Vec4b>(2,0)[3],0);EXPECT_EQ(pixels.at<cv::Vec4b>(2,1)[3],0);EXPECT_EQ(pixels.at<cv::Vec4b>(1,1)[3],0);
  EXPECT_EQ(display.snapshot(*buffer,11,12)["local_costmap"]["sequence"],image.second["sequence"]);
}
TEST_F(NavigationDebugTest, MissingTransformNeverPresentsOdomAsMap){
  display.grid(0,grid(),10);ASSERT_TRUE(display.wait_for(0,std::chrono::seconds(2)));const auto value=display.snapshot(*buffer,11,12);
  EXPECT_FALSE(value["global_costmap"]["available"]);EXPECT_FALSE(value["local_path"]["available"]);
  EXPECT_NE(value["global_costmap"]["error"].get<std::string>().find("坐标转换"),std::string::npos);
}
TEST_F(NavigationDebugTest, LargeGridReturnsBeforeEncodingFinishes){
  transform();auto value=grid();value.info.width=1200;value.info.height=1000;value.data.assign(1200u*1000u,0);
  for(size_t i=0;i<value.data.size();i+=17)value.data[i]=static_cast<int8_t>(50+(i%50));
  const auto started=std::chrono::steady_clock::now();display.grid(0,value,10);
  EXPECT_LT(std::chrono::steady_clock::now()-started,std::chrono::milliseconds(80));
  ASSERT_TRUE(display.wait_for(0,std::chrono::seconds(5)));
  const auto image=display.image(0,*buffer,11,12);
  ASSERT_TRUE(image.second.at("available"));
  EXPECT_EQ(image.second["width"],1200);EXPECT_EQ(image.second["height"],1000);
  EXPECT_NEAR(image.second["origin"]["x"].get<double>(),8,1e-6);
  EXPECT_NEAR(image.second["origin"]["y"].get<double>(),-2,1e-6);
  EXPECT_FALSE(image.first.empty());
  EXPECT_EQ(display.snapshot(*buffer,11,12)["global_costmap"]["sequence"],image.second["sequence"]);
}
TEST_F(NavigationDebugTest, ConvertsPathEndpointsWithoutInventingMissingPaths){
  transform();nav_msgs::msg::Path path;path.header.frame_id="odom";path.header.stamp.sec=9;
  for(int i=0;i<1200;++i){geometry_msgs::msg::PoseStamped pose;pose.pose.position.x=1+i*.001;pose.pose.position.y=2;path.poses.push_back(pose);}
  display.path(1,path,10);const auto result=display.snapshot(*buffer,11,12);const auto points=result["local_path"]["points"];
  ASSERT_LE(points.size(),401U);EXPECT_NEAR(points.front()[0].get<double>(),8,1e-8);EXPECT_NEAR(points.front()[1].get<double>(),-2,1e-8);
  EXPECT_NEAR(points.back()[1].get<double>(),-.801,1e-8);EXPECT_FALSE(result["global_path"]["available"]);
}
}  // namespace
