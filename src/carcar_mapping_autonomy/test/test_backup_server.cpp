#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <thread>
#include <nav2_behaviors/behavior_server.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "carcar_mapping_autonomy/backup_motion.hpp"

namespace ca=carcar_mapping_autonomy;
class RealBackupServer:public testing::Test {
protected:
  static void SetUpTestSuite(){
    ASSERT_NE(std::getenv("ROS_DOMAIN_ID"),nullptr);ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"225");
    ASSERT_NE(std::getenv("ROS_LOCALHOST_ONLY"),nullptr);ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite(){rclcpp::shutdown();}
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<behavior_server::BehaviorServer> server;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  std::unique_ptr<ca::BackupMotion> motion;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub;
  ca::Pose wheel;
  double last_time=0,zero_since=0,command=0,applied=0,max_negative=0;
  size_t negative_commands=0,zero_commands=0,nonstraight_commands=0;
  bool obstacle=false;
  static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void SetUp()override{
    node=std::make_shared<rclcpp::Node>("real_backup_software_io");
    tf=std::make_unique<tf2_ros::TransformBroadcaster>(node);
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    map_pub=node->create_publisher<nav2_msgs::msg::Costmap>("/backup_test/costmap_raw",qos);
    footprint_pub=node->create_publisher<geometry_msgs::msg::PolygonStamped>("/backup_test/footprint",qos);
    command_sub=node->create_subscription<geometry_msgs::msg::Twist>("/backup_test/command",10,
      [this](const geometry_msgs::msg::Twist&m){command=m.linear.x;
        if(m.linear.x< -1e-8){++negative_commands;max_negative=std::max(max_negative,-m.linear.x);}
        if(std::abs(m.linear.x)<1e-8)++zero_commands;
        if(std::abs(m.linear.y)>1e-8||std::abs(m.angular.z)>1e-8)++nonstraight_commands;});
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args","-r","cmd_vel:=/backup_test/command"});
    options.parameter_overrides({{"behavior_plugins",std::vector<std::string>{"backup"}},
      {"backup.plugin","nav2_behaviors/BackUp"},{"cycle_frequency",20.0},
      {"global_frame","odom"},{"robot_base_frame","base_footprint"},{"transform_tolerance",.1},
      {"costmap_topic","/backup_test/costmap_raw"},{"footprint_topic","/backup_test/footprint"},
      {"simulate_ahead_time",1.0},{"backup.acceleration_limit",.20},{"backup.deceleration_limit",-.20},
      {"backup.minimum_speed",.001},{"bond_heartbeat_period",0.0}});
    server=std::make_shared<behavior_server::BehaviorServer>(options);
    executor.add_node(node);executor.add_node(server->get_node_base_interface());
    ASSERT_EQ(server->configure().label(),"inactive");ASSERT_EQ(server->activate().label(),"active");
    motion=std::make_unique<ca::BackupMotion>(*node);last_time=zero_since=steady();
    ASSERT_TRUE(until([this]{return motion->available();},2));
    run_for(.3);
  }
  void TearDown()override{
    if(motion){motion->cancel("真实BackUp隔离测试收尾");until([this]{return motion->terminal();},1);motion.reset();}
    if(server){server->deactivate();server->cleanup();executor.remove_node(server->get_node_base_interface());server.reset();}
    command_sub.reset();tf.reset();map_pub.reset();footprint_pub.reset();executor.remove_node(node);node.reset();
  }
  void publish_inputs(){
    const auto now=node->now();
    geometry_msgs::msg::TransformStamped tr;tr.header.stamp=now;tr.header.frame_id="odom";
    tr.child_frame_id="base_footprint";tr.transform.translation.x=wheel.x;tr.transform.rotation.w=1;tf->sendTransform(tr);
    geometry_msgs::msg::PolygonStamped footprint;footprint.header.stamp=now;footprint.header.frame_id="base_footprint";
    for(const auto&p:std::vector<std::pair<float,float>>{{-.14f,-.13f},{-.14f,.13f},{.14f,.13f},{.14f,-.13f}}){
      geometry_msgs::msg::Point32 point;point.x=p.first;point.y=p.second;footprint.polygon.points.push_back(point);}
    footprint_pub->publish(footprint);
    nav2_msgs::msg::Costmap map;map.header.frame_id="odom";map.header.stamp=now;
    map.metadata.layer="master";map.metadata.size_x=100;map.metadata.size_y=100;map.metadata.resolution=.02;
    map.metadata.origin.position.x=-1;map.metadata.origin.position.y=-1;map.metadata.origin.orientation.w=1;
    map.metadata.update_time=now;map.data.assign(10000,0);
    if(obstacle){
      const int x=std::floor((wheel.x-.14+1)/.02);
      for(int ix=x-1;ix<=x+1;++ix)for(int y=49;y<=50;++y)if(ix>=0&&ix<100)map.data[y*100+ix]=254;
    }
    map_pub->publish(map);
  }
  void pump(){
    const double now=steady(),dt=std::min(.02,std::max(0.,now-last_time));last_time=now;
    // 软件轮式模型模拟既有监督器的动作互斥输出门；取消及VERIFYING阶段立即施加零输出。
    const bool moving=motion&&!motion->failed()&&motion->status().value("phase",std::string())=="MOVING";
    applied=moving?command:0;
    wheel.x+=applied*dt;
    if(std::abs(applied)>1e-8)zero_since=now;
    publish_inputs();executor.spin_some();
    if(motion)motion->tick(now,wheel,true,now-zero_since>=.3);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool until(const std::function<bool()>&predicate,double seconds){
    const auto end=steady()+seconds;
    while(steady()<end){pump();if(predicate())return true;}return predicate();
  }
  void run_for(double seconds){until([]{return false;},seconds);}
  void start(double distance,double speed=.02){
    ASSERT_TRUE(motion->start(distance,speed,10,wheel,steady(),"real-backup-software"));
  }
};

TEST_F(RealBackupServer, OneMillimeterRequestRunsActualNav2AndVerifiesStoppedWheel){
  start(.001,.01);ASSERT_TRUE(until([this]{return motion->terminal();},4))<<motion->status().dump();
  EXPECT_TRUE(motion->succeeded())<<motion->status().dump();EXPECT_GT(negative_commands,0U);
  EXPECT_GT(zero_commands,0U);EXPECT_EQ(nonstraight_commands,0U);EXPECT_GE(-wheel.x,.0009);
  EXPECT_LT(-wheel.x,.004);EXPECT_LE(max_negative,.010001);EXPECT_DOUBLE_EQ(applied,0);
}
TEST_F(RealBackupServer, ThreeCentimeterRequestProducesBoundedReverseAndWheelFeedback){
  start(.03);ASSERT_TRUE(until([this]{return motion->terminal();},5))<<motion->status().dump();
  EXPECT_TRUE(motion->succeeded())<<motion->status().dump();EXPECT_GE(-wheel.x,.0295);EXPECT_LE(-wheel.x,.034);
  EXPECT_LE(max_negative,.020001);EXPECT_EQ(nonstraight_commands,0U);EXPECT_DOUBLE_EQ(applied,0);
}
TEST_F(RealBackupServer, OperatorCancelReachesRealCanceledResultAndRemainsStopped){
  start(.05);ASSERT_TRUE(until([this]{return wheel.x<-.002;},3))<<motion->status().dump();
  motion->cancel("操作者暂停真实BackUp隔离动作");const double stopped=wheel.x;
  ASSERT_TRUE(until([this]{return motion->terminal();},2))<<motion->status().dump();
  EXPECT_TRUE(motion->failed());EXPECT_FALSE(motion->succeeded());EXPECT_EQ(motion->status()["result"],"CANCELED");
  run_for(.2);EXPECT_DOUBLE_EQ(wheel.x,stopped);EXPECT_DOUBLE_EQ(applied,0);EXPECT_GT(zero_commands,0U);
}
TEST_F(RealBackupServer, CurrentFootprintObstacleRefusesRealBackupWithoutWheelMotion){
  obstacle=true;run_for(.2);start(.03);
  ASSERT_TRUE(until([this]{return motion->terminal();},3))<<motion->status().dump();
  EXPECT_TRUE(motion->failed());EXPECT_FALSE(motion->succeeded());EXPECT_EQ(motion->status()["result"],"ABORTED");
  EXPECT_EQ(negative_commands,0U);EXPECT_DOUBLE_EQ(wheel.x,0);EXPECT_GT(zero_commands,0U);
}
TEST_F(RealBackupServer, NewObstacleAbortsActualNav2ReverseBeforeFurtherProgress){
  start(.05);ASSERT_TRUE(until([this]{return wheel.x<-.002;},3))<<motion->status().dump();
  obstacle=true;
  ASSERT_TRUE(until([this]{return motion->terminal();},2))<<motion->status().dump();
  EXPECT_TRUE(motion->failed());EXPECT_EQ(motion->status()["result"],"ABORTED");
  const double stopped=wheel.x;run_for(.2);EXPECT_DOUBLE_EQ(wheel.x,stopped);EXPECT_DOUBLE_EQ(applied,0);
}
