// 使用真实TF缓存验证扫描时间配准，不初始化ROS或连接硬件。
#include <gtest/gtest.h>
#include "carcar_mapping_autonomy/scan_pose.hpp"
namespace ca=carcar_mapping_autonomy;
static builtin_interfaces::msg::Time stamp(int seconds){builtin_interfaces::msg::Time t;t.sec=seconds;return t;}
static void odom(tf2::BufferCore & buffer,int seconds,double x,double y,double yaw){
  geometry_msgs::msg::TransformStamped t;t.header.frame_id="odom";t.child_frame_id="base_footprint";
  t.header.stamp=stamp(seconds);t.transform.translation.x=x;t.transform.translation.y=y;
  t.transform.rotation.z=std::sin(yaw/2);t.transform.rotation.w=std::cos(yaw/2);
  ASSERT_TRUE(buffer.setTransform(t,"fixture",false));
}
TEST(ScanPose, AdvancesOldSlamPoseUsingRelativeWheelMotionInMapCoordinates){
  tf2::BufferCore buffer;odom(buffer,100,5,2,M_PI/2);odom(buffer,107,5,2.1,M_PI/2+.1);
  geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.frame_id="map";p.header.stamp=stamp(100);
  p.pose.pose.position.x=-8;p.pose.pose.position.y=1;p.pose.pose.orientation.w=1;
  const auto result=ca::pose_at_scan(p,stamp(107),buffer);
  EXPECT_NEAR(result.x,-7.9,1e-9);EXPECT_NEAR(result.y,1,1e-9);EXPECT_NEAR(result.yaw,.1,1e-9);
}
TEST(ScanPose, RejectsMissingHistoryInsteadOfUsingLatestTransform){
  tf2::BufferCore buffer;odom(buffer,107,0,0,0);
  geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.frame_id="map";p.header.stamp=stamp(100);p.pose.pose.orientation.w=1;
  EXPECT_THROW(ca::pose_at_scan(p,stamp(107),buffer),tf2::TransformException);
  odom(buffer,100,0,0,0);
  EXPECT_THROW(ca::pose_at_scan(p,stamp(108),buffer),tf2::TransformException);
}
TEST(ScanPose, RejectsWrongFrameUnsetStampAndReversedTime){
  tf2::BufferCore buffer;geometry_msgs::msg::PoseWithCovarianceStamped p;p.pose.pose.orientation.w=1;
  p.header.frame_id="map";EXPECT_THROW(ca::pose_at_scan(p,stamp(107),buffer),std::runtime_error);
  p.header.stamp=stamp(100);EXPECT_THROW(ca::pose_at_scan(p,stamp(99),buffer),std::runtime_error);
  p.header.frame_id="odom";EXPECT_THROW(ca::pose_at_scan(p,stamp(107),buffer),std::runtime_error);
}
TEST(ScanPose, KeepsExactAcceptedOdometryAcrossLongStationaryPeriodsAndRejectsMismatchedCache){
  tf2::BufferCore buffer;odom(buffer,100,5,2,0);
  auto accepted=buffer.lookupTransform("odom","base_footprint",tf2::TimePoint(std::chrono::seconds(100)));
  odom(buffer,140,5.1,2,0);
  odom(buffer,141,5.1,2,0); // 持续轮式更新才会实际触发旧缓存回收。
  geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.frame_id="map";p.header.stamp=stamp(100);p.pose.pose.orientation.w=1;
  EXPECT_THROW(ca::pose_at_scan(p,stamp(140),buffer),tf2::TransformException);
  const auto aligned=ca::pose_at_scan(p,stamp(140),buffer,&accepted);
  EXPECT_NEAR(aligned.x,.1,1e-9);EXPECT_NEAR(aligned.y,0,1e-9);
  accepted.header.stamp=stamp(101);
  EXPECT_THROW(ca::pose_at_scan(p,stamp(140),buffer,&accepted),std::runtime_error);
}
