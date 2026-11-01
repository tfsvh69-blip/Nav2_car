#include <gtest/gtest.h>
#include <algorithm>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nlohmann/json.hpp>
extern char**environ;
namespace {
struct NavigationProcess {
 pid_t pid=-1;
 ~NavigationProcess(){if(pid<=0)return;kill(-pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid)return;std::this_thread::sleep_for(std::chrono::milliseconds(20));}kill(-pid,SIGKILL);waitpid(pid,nullptr,0);}
 bool start(const std::string&log){
   std::vector<std::string>args={"/opt/ros/humble/bin/ros2","launch",NAVIGATION_LAUNCH,std::string("bt_params:=")+CONTINUOUS_PARAMS};
   std::vector<char*>argv;for(auto&s:args)argv.push_back(s.data());argv.push_back(nullptr);
   posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
   posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);const int fd=open(log.c_str(),O_CREAT|O_TRUNC|O_WRONLY,0644);
   if(fd<0)return false;
   posix_spawn_file_actions_adddup2(&actions,fd,1);posix_spawn_file_actions_adddup2(&actions,fd,2);posix_spawn_file_actions_addclose(&actions,fd);
   const int rc=posix_spawn(&pid,argv[0],&actions,&attr,argv.data(),environ);
   close(fd);posix_spawn_file_actions_destroy(&actions);posix_spawnattr_destroy(&attr);return rc==0;
 }
};
}
// 真实Nav2规划/行为树/RPP/平滑器；地图与轮式运动为软件真值，不加载硬件或SLAM。
TEST(ContinuousNavigation, TenViaPointsUseOneActionAndNoIntermediateTerminal){
 ASSERT_EQ(std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):""),"226");
 rclcpp::init(0,nullptr);struct Shutdown{~Shutdown(){rclcpp::shutdown();}} shutdown;
 const std::filesystem::path evidence="log/nav018_r26_snapshot/fixed-route-preemption";std::filesystem::create_directories(evidence);
 auto node=std::make_shared<rclcpp::Node>("continuous_navigation_truth");NavigationProcess child;
 const std::string prior=std::getenv("LD_LIBRARY_PATH")?std::getenv("LD_LIBRARY_PATH"):"";
 const auto compat=ament_index_cpp::get_package_prefix("carcar_tf2_compat")+"/lib/carcar_tf2_compat";
 // 与正式建图包装入口保持同一TF2兼容库环境，避免测试入口遗漏既有锁序修复。
 setenv("LD_LIBRARY_PATH",(compat+":"+std::string(CHECKED_PLUGIN_DIRECTORY)+":"+prior).c_str(),1);ASSERT_TRUE(child.start((evidence/"nav2.log").string()));setenv("LD_LIBRARY_PATH",prior.c_str(),1);
 tf2_ros::TransformBroadcaster tf(*node);tf2_ros::StaticTransformBroadcaster fixed(*node);
 geometry_msgs::msg::TransformStamped static_map;static_map.header.frame_id="map";static_map.child_frame_id="odom";static_map.transform.rotation.w=1;
 geometry_msgs::msg::TransformStamped laser;laser.header.frame_id="base_footprint";laser.child_frame_id="laser_frame";laser.transform.rotation.w=1;
 fixed.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{static_map,laser});
 auto maps=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
 auto states=node->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
 auto odoms=node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
 auto scans=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
 auto filtered=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS());
 auto limits=node->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit",1);
 nav_msgs::msg::OccupancyGrid map;map.header.frame_id="map";map.info.width=160;map.info.height=120;map.info.resolution=.05;
 map.info.origin.position.x=-4;map.info.origin.position.y=-3;map.info.origin.orientation.w=1;map.data.assign(160*120,0);
 for(int y=0;y<120;++y)for(int x=0;x<160;++x)if(x==0||x==159||y==0||y==119)map.data[y*160+x]=100;
 double x=0,y=0,yaw=0,last_command=-1e9,previous_v=0,maximum_accel=0;size_t zero_during_middle=0;
 double limited_max=0,restored_max=0;size_t limited_samples=0;
 geometry_msgs::msg::Twist velocity;std::vector<bool> passed(10,false);bool running=false,finished=false;
 std::ofstream trace(evidence/"velocity.jsonl");
 auto command=node->create_subscription<geometry_msgs::msg::Twist>("/mapping/auto_cmd_vel",10,[&](const geometry_msgs::msg::Twist&m){
   const double now=node->now().seconds();if(last_command>0&&now>last_command&&running)maximum_accel=std::max(maximum_accel,std::abs(m.linear.x-previous_v)/(now-last_command));
   previous_v=m.linear.x;velocity=m;last_command=now;
   if(running&&x>.4&&x<2.9&&std::abs(m.linear.x)+std::abs(m.angular.z)<1e-5)++zero_during_middle;
   if(running&&x>.85&&x<1.2){limited_max=std::max(limited_max,m.linear.x);++limited_samples;}
   if(running&&x>1.6&&x<2.4)restored_max=std::max(restored_max,m.linear.x);
   trace<<nlohmann::json{{"t",now},{"vx",m.linear.x},{"wz",m.angular.z},{"x",x},{"y",y}}.dump()<<'\n';
 });
 auto feed=[&]{
   const auto now=node->now();if(now.seconds()-last_command>.3)velocity=geometry_msgs::msg::Twist();
   x+=velocity.linear.x*std::cos(yaw)*.05;y+=velocity.linear.x*std::sin(yaw)*.05;yaw+=velocity.angular.z*.05;
   nav_msgs::msg::Odometry odom;odom.header.stamp=now;odom.header.frame_id="odom";odom.child_frame_id="base_footprint";
   odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.orientation.z=std::sin(yaw/2);odom.pose.pose.orientation.w=std::cos(yaw/2);odom.twist.twist=velocity;odoms->publish(odom);
   geometry_msgs::msg::TransformStamped tr;tr.header=odom.header;tr.child_frame_id="base_footprint";tr.transform.translation.x=x;tr.transform.translation.y=y;tr.transform.rotation=odom.pose.pose.orientation;tf.sendTransform(tr);
   map.header.stamp=now;maps->publish(map);std_msgs::msg::String state;state.data="{\"phase\":\"EXPLORING\",\"origin\":[0,0,0],\"radius_m\":20,\"generation\":1}";states->publish(state);
   sensor_msgs::msg::LaserScan scan;scan.header.stamp=now;scan.header.frame_id="laser_frame";scan.angle_min=-M_PI;scan.angle_increment=2*M_PI/360;scan.angle_max=M_PI;scan.range_min=.05;scan.range_max=12;scan.ranges.resize(361);
   for(size_t i=0;i<scan.ranges.size();++i){const auto a=yaw+scan.angle_min+i*scan.angle_increment;const auto dx=std::cos(a),dy=std::sin(a);double r=12;
     if(dx>1e-6)r=std::min(r,(3.95-x)/dx);else if(dx< -1e-6)r=std::min(r,(-4-x)/dx);
     if(dy>1e-6)r=std::min(r,(2.95-y)/dy);else if(dy< -1e-6)r=std::min(r,(-3-y)/dy);scan.ranges[i]=r;}
   scans->publish(scan);filtered->publish(scan);
   // 监督器限速决策另有集成测试；这里验证相同接口确实穿过RPP与平滑器。
   nav2_msgs::msg::SpeedLimit limit;limit.header.stamp=now;limit.percentage=true;
   limit.speed_limit=x>.7&&x<1.3?60.:0.;limits->publish(limit);
   for(size_t i=0;i<10;++i)if(std::hypot(x-.3*(i+1),y)<.15)passed[i]=true;
 };
 auto timer=node->create_wall_timer(std::chrono::milliseconds(50),feed);
 auto spin_until=[&](auto done,double seconds){const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
   while(std::chrono::steady_clock::now()<deadline){rclcpp::spin_some(node);if(done())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return done();};
 using Navigate=nav2_msgs::action::NavigateThroughPoses;auto nav=rclcpp_action::create_client<Navigate>(node,"/navigate_through_poses");
 ASSERT_TRUE(spin_until([&]{return nav->action_server_is_ready();},30));
 // 生命周期Action发现不代表代价图已经收到数据，先保持静止形成完整地图与TF。
 spin_until([]{return false;},2);
 Navigate::Goal goal;goal.behavior_tree=CONTINUOUS_TREE;
 // 前十个是中间点，第十一个才是允许按到点容差停车的终点。
 for(int i=1;i<=11;++i){geometry_msgs::msg::PoseStamped point;point.header.frame_id="map";point.header.stamp=node->now();point.pose.position.x=.3*i;point.pose.orientation.w=1;goal.poses.push_back(point);}
 rclcpp_action::ResultCode result=rclcpp_action::ResultCode::UNKNOWN;int remaining=11;
 rclcpp_action::Client<Navigate>::SendGoalOptions options;options.feedback_callback=[&](auto,auto feedback){remaining=feedback->number_of_poses_remaining;};
 options.result_callback=[&](const auto&r){finished=true;result=r.code;running=false;};
 auto accepted=nav->async_send_goal(goal,options);ASSERT_TRUE(spin_until([&]{return accepted.wait_for(std::chrono::seconds(0))==std::future_status::ready;},5));ASSERT_TRUE(accepted.get());running=true;
 EXPECT_TRUE(spin_until([&]{return finished;},70));EXPECT_EQ(result,rclcpp_action::ResultCode::SUCCEEDED);
 const auto count=std::count(passed.begin(),passed.end(),true);EXPECT_EQ(count,10);EXPECT_EQ(remaining,1);EXPECT_EQ(zero_during_middle,0U);
 EXPECT_LE(maximum_accel,.30); // 平滑器0.20m/s²，考虑DDS接收时间测量误差，轨迹另存供核查。
 EXPECT_GT(limited_samples,10U);EXPECT_LE(limited_max,.065);EXPECT_GE(restored_max,.095);
 std::ofstream(evidence/"result.json")<<nlohmann::json{{"hardware",false},{"real_nav2",true},{"real_slam",false},{"passed_points",count},{"intermediate_zero_commands",zero_during_middle},{"max_receive_time_accel",maximum_accel},{"result",int(result)},{"last_pose",{x,y,yaw}}}.dump(2);
 // 在旧终点即将完成时续接转弯点；旧FollowPath的成功不能替代新末点的实际到达。
 const double end_x=x+.30,end_y=y+1.;goal.poses.resize(1);goal.poses[0].pose.position.x=end_x;goal.poses[0].pose.position.y=y;
 finished=false;result=rclcpp_action::ResultCode::UNKNOWN;
 auto short_goal=nav->async_send_goal(goal);
 ASSERT_TRUE(spin_until([&]{return short_goal.wait_for(std::chrono::seconds(0))==std::future_status::ready;},5));ASSERT_TRUE(short_goal.get());
 ASSERT_TRUE(spin_until([&]{return x>end_x-.20;},10));
 goal.poses.push_back(goal.poses.front());goal.poses.back().pose.position.y=end_y;
 auto continued=nav->async_send_goal(goal,options);
 ASSERT_TRUE(spin_until([&]{return continued.wait_for(std::chrono::seconds(0))==std::future_status::ready;},5));ASSERT_TRUE(continued.get());
 EXPECT_TRUE(spin_until([&]{return finished;},30));EXPECT_EQ(result,rclcpp_action::ResultCode::SUCCEEDED);
 EXPECT_LE(std::hypot(x-end_x,y-end_y),.20)<<"旧终点附近返回的新路线成功属于错误终态";
 std::ofstream(evidence/"preemption.json")<<nlohmann::json{{"hardware",false},{"result",int(result)},{"last_pose",{x,y,yaw}},{"endpoint",{end_x,end_y}},{"endpoint_distance_m",std::hypot(x-end_x,y-end_y)}}.dump(2);
}
