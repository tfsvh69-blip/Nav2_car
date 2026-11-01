#include <fstream>
#include <filesystem>
#include <thread>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "carcar_mapping_autonomy/subcell_departure.hpp"
extern char **environ;
namespace ca=carcar_mapping_autonomy;
namespace {
struct NavigationProcess {
  pid_t pid=-1;
  ~NavigationProcess(){
    if(pid<=0)return;kill(-pid,SIGINT);
    for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid)return;std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    kill(-pid,SIGKILL);waitpid(pid,nullptr,0);
  }
  bool start(const std::string&log){
    std::vector<std::string>args={"/opt/ros/humble/bin/ros2","launch",NAVIGATION_LAUNCH};
    std::vector<char*>argv;for(auto&s:args)argv.push_back(s.data());argv.push_back(nullptr);
    posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    const int fd=open(log.c_str(),O_CREAT|O_TRUNC|O_WRONLY,0644);if(fd<0)return false;
    posix_spawn_file_actions_adddup2(&actions,fd,1);posix_spawn_file_actions_adddup2(&actions,fd,2);posix_spawn_file_actions_addclose(&actions,fd);
    const int result=posix_spawn(&pid,argv[0],&actions,&attr,argv.data(),environ);
    close(fd);posix_spawn_file_actions_destroy(&actions);posix_spawnattr_destroy(&attr);return result==0;
  }
};
// 真实插件加载、Nav2行为树、周期重规划及速度平滑器；只在隔离域积分虚拟轮式运动。
TEST(SubcellNavigation, RecordedStopReplansAndReachesObservationWithRealNav2) {
  ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):""),"212");
  ASSERT_EQ(std::string(std::getenv("ROS_LOCALHOST_ONLY")?std::getenv("ROS_LOCALHOST_ONLY"):""),"1");
  rclcpp::init(0,nullptr);struct Shutdown{~Shutdown(){rclcpp::shutdown();}}shutdown;
  const auto directory=std::filesystem::current_path()/"subcell_navigation";std::filesystem::create_directories(directory);
  auto node=std::make_shared<rclcpp::Node>("subcell_virtual_odometry");NavigationProcess child;
  const std::string prior=std::getenv("LD_LIBRARY_PATH")?std::getenv("LD_LIBRARY_PATH"):"";
  const auto compat=ament_index_cpp::get_package_prefix("carcar_tf2_compat")+"/lib/carcar_tf2_compat";
  setenv("LD_LIBRARY_PATH",(compat+":"+prior).c_str(),1);
  ASSERT_TRUE(child.start((directory/"nav2.log").string()));setenv("LD_LIBRARY_PATH",prior.c_str(),1);
  tf2_ros::TransformBroadcaster tf(*node);tf2_ros::StaticTransformBroadcaster fixed(*node);
  geometry_msgs::msg::TransformStamped odom_tf;odom_tf.header.frame_id="map";odom_tf.child_frame_id="odom";odom_tf.transform.rotation.w=1;
  geometry_msgs::msg::TransformStamped laser_tf;laser_tf.header.frame_id="base_footprint";laser_tf.child_frame_id="laser_frame";laser_tf.transform.rotation.w=1;
  fixed.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{odom_tf,laser_tf});
  auto maps=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
  auto odoms=node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
  auto scans=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
  auto filtered=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS());
  std::ifstream file(SUBCELL_FIXTURE);const auto j=ca::Json::parse(file);nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id="map";map.info.width=j["info"]["width"];map.info.height=j["info"]["height"];map.info.resolution=j["info"]["resolution"];
  map.info.origin.position.x=j["info"]["origin"]["position"]["x"];map.info.origin.position.y=j["info"]["origin"]["position"]["y"];
  map.info.origin.orientation.w=1;map.data=j["data"].get<std::vector<int8_t>>();
  ca::Pose current{.04239974986262743,-.012599852844464556,-.41358128558844526};
  geometry_msgs::msg::Twist velocity;double command_at=-1e9;bool unsafe=false,finished=false;size_t moving=0;
  std::ofstream trace(directory/"trace.jsonl");
  auto commands=node->create_subscription<geometry_msgs::msg::Twist>("/mapping/auto_cmd_vel",10,[&](const geometry_msgs::msg::Twist&m){velocity=m;command_at=node->now().seconds();});
  auto feed=[&]{
    const auto now=node->now();if(now.seconds()-command_at>.3||unsafe)velocity=geometry_msgs::msg::Twist();
    const ca::Pose next{current.x+velocity.linear.x*std::cos(current.yaw)*.05,current.y+velocity.linear.x*std::sin(current.yaw)*.05,current.yaw+velocity.angular.z*.05};
    if(!ca::free_connector(map,current,next))unsafe=true;else current=next;
    if(std::abs(velocity.linear.x)>.01)++moving;
    nav_msgs::msg::Odometry odom;odom.header.stamp=now;odom.header.frame_id="odom";odom.child_frame_id="base_footprint";
    odom.pose.pose.position.x=current.x;odom.pose.pose.position.y=current.y;
    odom.pose.pose.orientation.z=std::sin(current.yaw/2);odom.pose.pose.orientation.w=std::cos(current.yaw/2);odom.twist.twist=velocity;odoms->publish(odom);
    geometry_msgs::msg::TransformStamped tr;tr.header=odom.header;tr.child_frame_id="base_footprint";tr.transform.translation.x=current.x;tr.transform.translation.y=current.y;tr.transform.rotation=odom.pose.pose.orientation;tf.sendTransform(tr);
    map.header.stamp=now;maps->publish(map);
    // 已录原图负责静态障碍；本场景无额外动态物体，合法扫描保持无近场碰撞。
    sensor_msgs::msg::LaserScan scan;scan.header.stamp=now;scan.header.frame_id="laser_frame";
    scan.angle_min=-M_PI;scan.angle_max=M_PI;scan.angle_increment=2*M_PI/360;scan.range_min=.05;scan.range_max=12;
    scan.ranges.assign(361,std::numeric_limits<float>::infinity());scans->publish(scan);filtered->publish(scan);
    trace<<ca::Json{{"pose",{current.x,current.y,current.yaw}},{"output",{velocity.linear.x,velocity.angular.z}},{"safe",!unsafe}}.dump()<<'\n';
  };
  auto timer=node->create_wall_timer(std::chrono::milliseconds(50),feed);
  auto until=[&](auto done,double seconds){
    const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);if(done())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return done();
  };
  using Nav=nav2_msgs::action::NavigateToPose;auto nav=rclcpp_action::create_client<Nav>(node,"/navigate_to_pose");
  ASSERT_TRUE(until([&]{return nav->action_server_is_ready();},25));until([]{return false;},2);
  Nav::Goal goal;goal.pose.header.frame_id="map";goal.pose.pose.position.x=.47661976260876737;goal.pose.pose.position.y=.0038987502426766074;goal.pose.pose.orientation.w=1;
  goal.behavior_tree=EXPLORATION_TREE;rclcpp_action::ResultCode result=rclcpp_action::ResultCode::UNKNOWN;
  rclcpp_action::Client<Nav>::SendGoalOptions options;options.result_callback=[&](const auto&r){finished=true;result=r.code;};
  auto sent=nav->async_send_goal(goal,options);ASSERT_TRUE(until([&]{return sent.wait_for(std::chrono::seconds(0))==std::future_status::ready;},5));ASSERT_TRUE(sent.get());
  EXPECT_TRUE(until([&]{return finished||unsafe;},35));
  if(unsafe||!finished){nav->async_cancel_all_goals();until([&]{return finished;},2);}
  EXPECT_FALSE(unsafe);EXPECT_EQ(result,rclcpp_action::ResultCode::SUCCEEDED);EXPECT_GT(moving,20U);
  EXPECT_LE(std::hypot(current.x-goal.pose.pose.position.x,current.y-goal.pose.pose.position.y),.16);
  std::ofstream(directory/"result.json")<<ca::Json{{"hardware",false},{"real_nav2",true},{"safe",!unsafe},{"result",int(result)},
    {"moving_samples",moving},{"last_pose",{current.x,current.y,current.yaw}}}.dump(2);
}
}  // namespace
