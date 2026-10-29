#include <gtest/gtest.h>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_filters/speckle_filter.h>
class LaserDistance:public testing::Test {
protected:
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_F(LaserDistance, PluginRemovesIsolatedReturnWithoutInventingFreeRay){
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("filter1.name","distance"),rclcpp::Parameter("filter1.type","laser_filters/LaserScanSpeckleFilter"),
    rclcpp::Parameter("filter1.params.filter_type",0),rclcpp::Parameter("filter1.params.max_range",3.0),
    rclcpp::Parameter("filter1.params.max_range_difference",.10),rclcpp::Parameter("filter1.params.filter_window",2)});
  auto node=std::make_shared<rclcpp::Node>("distance_test",options);
  filters::FilterChain<sensor_msgs::msg::LaserScan> chain("sensor_msgs::msg::LaserScan");
  ASSERT_TRUE(chain.configure("",node->get_node_logging_interface(),node->get_node_parameters_interface()));
  sensor_msgs::msg::LaserScan input,output;input.header.frame_id="laser_frame";input.header.stamp.sec=12;input.header.stamp.nanosec=34;
  input.angle_min=-1;input.angle_increment=.01;input.time_increment=.001;input.scan_time=.1;
  input.ranges={1,1.02,NAN,1,.3,1,1.01,INFINITY,INFINITY};input.intensities.assign(9,5);
  ASSERT_TRUE(chain.update(input,output));
  EXPECT_TRUE(std::isnan(output.ranges[4]));
  EXPECT_FALSE(std::isinf(output.ranges[4]));
  EXPECT_FLOAT_EQ(output.ranges[0],1);
  EXPECT_FLOAT_EQ(output.ranges[1],1.02);
  EXPECT_FLOAT_EQ(output.ranges[5],1);
  EXPECT_EQ(output.header,input.header);
  EXPECT_EQ(output.intensities,input.intensities);
  EXPECT_EQ(output.angle_increment,input.angle_increment);
  EXPECT_EQ(output.time_increment,input.time_increment);
}
TEST_F(LaserDistance, TinyScanFailsRatherThanPublishingLastGoodData){
  rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("filter1.name","distance"),rclcpp::Parameter("filter1.type","laser_filters/LaserScanSpeckleFilter"),rclcpp::Parameter("filter1.params.filter_type",0),rclcpp::Parameter("filter1.params.max_range",3.0),rclcpp::Parameter("filter1.params.max_range_difference",.1),rclcpp::Parameter("filter1.params.filter_window",2)});
  auto n=std::make_shared<rclcpp::Node>("tiny_filter",options);filters::FilterChain<sensor_msgs::msg::LaserScan> chain("sensor_msgs::msg::LaserScan");
  ASSERT_TRUE(chain.configure("",n->get_node_logging_interface(),n->get_node_parameters_interface()));sensor_msgs::msg::LaserScan input,out;input.ranges={1,1};
  EXPECT_FALSE(chain.update(input,out));
}
TEST_F(LaserDistance, RadiusValidatorChecksEdgeBeforeRead){
  laser_filters::RadiusOutlierWindowValidator validator;laser_filters::WindowValidator&base=validator;
  sensor_msgs::msg::LaserScan input;input.ranges={1,NAN,1};input.angle_increment=.01;
  EXPECT_FALSE(base.checkWindowValid(input,0,4,.1));
  EXPECT_FALSE(base.checkWindowValid(input,2,4,.1));
}
