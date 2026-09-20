// 离线执行真实插件周期，不启动硬件；同时观察实际零速/负速输出。
#include <gtest/gtest.h>
#include "carcar_navigation/protected_backup.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

class BackupHarness : public carcar_navigation::ProtectedBackUp {
public:
  void inputs(const carcar_navigation::SensorSnapshot & data,
    const nav_msgs::msg::Odometry::SharedPtr & odom) {
    safety_data_=data;
    on_scan(std::make_shared<sensor_msgs::msg::LaserScan>(*data.scan));
    on_odom(odom);
  }
  void expire() {deadline_=carcar_navigation::Steady::now();}
};

TEST(ProtectedBackupExecution, FrontEscapeRearObstacleExpiryAndSignedProgress)
{
  using namespace carcar_navigation;
  using nav2_behaviors::Status;
  rclcpp::init(0,nullptr);
  {
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("backup_execution_test");
    node->declare_parameter("global_frame",std::string("map"));
    node->declare_parameter("robot_base_frame",std::string("base_footprint"));
    node->declare_parameter("cycle_frequency",10.0);
    // 本测试直接注入 TF，没有监听线程，不使用等待超时。
    node->declare_parameter("transform_tolerance",0.0);
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped body;
    body.header.frame_id="map";body.child_frame_id="base_footprint";body.transform.rotation.w=1;
    tf->setTransform(body,"test",true);
    BackupHarness backup;backup.configure(node,"test_backup",tf,nullptr);backup.activate();
    auto observer=std::make_shared<rclcpp::Node>("backup_velocity_observer");
    double velocity=99;
    auto sub=observer->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,
      [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) {velocity=msg->linear.x;});
    const auto connected=after(2);
    while (sub->get_publisher_count()==0 && Steady::now()<connected) {
      rclcpp::spin_some(observer);std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto cm=std::make_shared<nav2_msgs::msg::Costmap>();
    cm->header.frame_id="map";cm->metadata.resolution=.05;
    cm->metadata.size_x=cm->metadata.size_y=60;
    cm->metadata.origin.position.x=cm->metadata.origin.position.y=-1.5;
    cm->metadata.origin.orientation.w=1;cm->data.assign(3600,0);
    auto fp=std::make_shared<geometry_msgs::msg::PolygonStamped>();fp->header.frame_id="base_footprint";
    for (auto p : {std::pair<float,float>{-.14,-.13},{-.14,.13},{.14,.13},{.14,-.13}}) {
      geometry_msgs::msg::Point32 point;point.x=p.first;point.y=p.second;fp->polygon.points.push_back(point);
    }
    auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();scan->header=fp->header;
    scan->angle_min=-3.14;scan->angle_increment=.01;scan->range_min=.05;scan->range_max=10;
    scan->ranges.assign(629,5);
    auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.orientation.w=1;
    auto refresh=[&] {
      cm->header.stamp=fp->header.stamp=scan->header.stamp=odom->header.stamp=node->now();
      SensorSnapshot data;data.costmap=cm;data.footprint=fp;data.scan=scan;
      data.costmap_received=data.footprint_received=data.scan_received=Steady::now();
      backup.inputs(data,odom);
    };
    auto goal=std::make_shared<BackUpAction::Goal>();goal->target.x=-.2;goal->speed=.05;
    goal->time_allowance=rclcpp::Duration::from_seconds(6);
    auto expect_velocity=[&](double expected) {
      const auto until=after(.3);
      do {rclcpp::spin_some(observer);std::this_thread::sleep_for(std::chrono::milliseconds(5));}
      while (Steady::now()<until);
      EXPECT_NEAR(velocity,expected,1e-8); // Action.speed 是 float32。
    };
    cm->data[30*60+32]=254; // 车头边缘 x=0.10～0.15。
    refresh();ASSERT_EQ(backup.onRun(goal),Status::SUCCEEDED);expect_velocity(0);
    refresh();EXPECT_EQ(backup.onCycleUpdate(),Status::RUNNING);expect_velocity(-.05);
    scan->header.stamp=node->now()+rclcpp::Duration::from_nanoseconds(8000000);
    SensorSnapshot jitter;jitter.scan=scan;jitter.costmap=cm;jitter.footprint=fp;
    jitter.scan_received=jitter.costmap_received=jitter.footprint_received=Steady::now();
    backup.inputs(jitter,odom);
    EXPECT_EQ(backup.onCycleUpdate(),Status::RUNNING);expect_velocity(-.05);
    cm->data[30*60+25]=254;refresh();
    EXPECT_EQ(backup.onCycleUpdate(),Status::FAILED);expect_velocity(0);
    cm->data[30*60+25]=0;refresh();ASSERT_EQ(backup.onRun(goal),Status::SUCCEEDED);
    backup.expire();EXPECT_EQ(backup.onCycleUpdate(),Status::FAILED);expect_velocity(0);
    refresh();ASSERT_EQ(backup.onRun(goal),Status::SUCCEEDED);
    scan->header.stamp=node->now()-rclcpp::Duration::from_seconds(1);
    SensorSnapshot stale;stale.scan=scan;stale.costmap=cm;stale.footprint=fp;
    stale.scan_received=stale.costmap_received=stale.footprint_received=Steady::now();
    backup.inputs(stale,odom);
    EXPECT_EQ(backup.onCycleUpdate(),Status::FAILED);expect_velocity(0);
    refresh();ASSERT_EQ(backup.onRun(goal),Status::SUCCEEDED);
    body.transform.translation.x=.21;tf->setTransform(body,"test",true);
    EXPECT_EQ(backup.onCycleUpdate(),Status::FAILED); // 前进不能当作倒车完成。
    body.transform.translation.x=0;tf->setTransform(body,"test",true);
    refresh();ASSERT_EQ(backup.onRun(goal),Status::SUCCEEDED);
    body.transform.translation.x=-.21;tf->setTransform(body,"test",true);
    EXPECT_EQ(backup.onCycleUpdate(),Status::SUCCEEDED);expect_velocity(0);
    backup.deactivate();backup.cleanup();
  }
  rclcpp::shutdown();
}
