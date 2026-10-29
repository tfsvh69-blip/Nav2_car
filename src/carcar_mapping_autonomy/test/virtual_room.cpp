// 完整SLAM/Nav2/explore_lite的软件房间；仅隔离域203，无任何硬件设备入口。
#include <chrono>
#include <iostream>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
extern char**environ;
int main(int argc,char**argv){
  if(std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"")!="203"){std::cerr<<"只允许隔离域203\n";return 2;}
  rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("virtual_room");
  const bool complete=std::getenv("CARCAR_ROOM_COMPLETE")!=nullptr;
  const bool managed=std::getenv("CARCAR_ROOM_MANAGED")!=nullptr;
  const bool observe=managed||complete||std::getenv("CARCAR_ROOM_OBSERVE")!=nullptr;
  const double west=complete?-1.5:-2.5,east=complete?1.5:3.5,south=complete?-1.5:-2.,north=complete?1.5:2.;
  tf2_ros::TransformBroadcaster tf(*n);tf2_ros::StaticTransformBroadcaster fixed(*n);
  geometry_msgs::msg::TransformStamped f;f.header.frame_id="base_footprint";f.child_frame_id="laser_frame";f.transform.translation.x=-.05;f.transform.translation.z=.217;f.transform.rotation.w=1;fixed.sendTransform(f);
  auto scan=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
  auto odom=n->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
  auto commands=n->create_publisher<std_msgs::msg::String>("/mapping/autonomy/request",10);
  geometry_msgs::msg::Twist velocity;double velocity_at=-1e9,x=complete?0:-1.2,y=0,a=0,path=0;int nonzero=0,relays=0;bool fault=false,started=false,paused=false;
  nlohmann::json state=nlohmann::json::object();const auto began=std::chrono::steady_clock::now();
  auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();};
  auto sub=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,[&](const geometry_msgs::msg::Twist&v){velocity=v;velocity_at=elapsed();if(std::abs(v.linear.x)+std::abs(v.angular.z)>1e-5)++nonzero;});
  auto states=n->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){state=nlohmann::json::parse(m.data);if(state.value("phase","")=="EXPLORING")started=true;if(state.value("phase","")=="PAUSED")paused=true;if(state.value("phase","")=="FAULT"){std::cerr<<state.dump()<<'\n';fault=true;}});
  auto relay=n->create_subscription<sensor_msgs::msg::LaserScan>("/mapping/slam_scan",rclcpp::SensorDataQoS(),[&](const sensor_msgs::msg::LaserScan&){++relays;});
  auto timer=n->create_wall_timer(std::chrono::milliseconds(50),[&]{
    if(elapsed()-velocity_at>.3)velocity=geometry_msgs::msg::Twist();
    const double step=.05;const double nextx=x+velocity.linear.x*std::cos(a)*step,nexty=y+velocity.linear.x*std::sin(a)*step;
    if(nextx>west+.3&&nextx<east-.3&&nexty>south+.3&&nexty<north-.3){path+=std::hypot(nextx-x,nexty-y);x=nextx;y=nexty;}
    a+=velocity.angular.z*step;
    nav_msgs::msg::Odometry o;o.header.stamp=n->now();o.header.frame_id="odom";o.child_frame_id="base_footprint";o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.z=std::sin(a/2);o.pose.pose.orientation.w=std::cos(a/2);o.twist.twist=velocity;odom->publish(o);
    geometry_msgs::msg::TransformStamped t;t.header=o.header;t.child_frame_id="base_footprint";t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=o.pose.pose.orientation;tf.sendTransform(t);
    sensor_msgs::msg::LaserScan s;s.header.stamp=o.header.stamp;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_max=M_PI;s.angle_increment=2*M_PI/360;s.scan_time=.05;s.time_increment=0;s.range_min=.05;s.range_max=12;s.ranges.resize(361);s.intensities.assign(361,10);
    const double sx=x-.05*std::cos(a),sy=y-.05*std::sin(a);
    for(size_t i=0;i<s.ranges.size();++i){const double theta=a+s.angle_min+i*s.angle_increment,dx=std::cos(theta),dy=std::sin(theta);double r=12;
      if(dx>1e-6)r=std::min(r,(east-sx)/dx);else if(dx< -1e-6)r=std::min(r,(west-sx)/dx);
      if(dy>1e-6)r=std::min(r,(north-sy)/dy);else if(dy< -1e-6)r=std::min(r,(south-sy)/dy);
      s.ranges[i]=static_cast<float>(r);}
    scan->publish(s);
  });
  char root[]="/tmp/carcar-mapping-room-XXXXXX";if(!mkdtemp(root))return 3;
  std::string map_root=std::string("map_root:=")+root;
  char ros2[]="/opt/ros/humble/bin/ros2",launch[]="launch",pkg[]="carcar_mapping_console",file[]="mapping_autonomous.launch.xml",hw[]="start_hardware:=false",mode[]="mode:=new",control[]="control_mode:=automatic",port[]="port:=18098",record[]="record_raw_bag:=false";
  char*args[]={ros2,launch,pkg,file,hw,mode,control,port,record,map_root.data(),nullptr};pid_t pid=-1;
  posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
  if(!std::getenv("CARCAR_ROOM_EXTERNAL")&&posix_spawn(&pid,ros2,nullptr,&attr,args,environ)!=0)return 3;
  posix_spawnattr_destroy(&attr);
  bool pause_sent=false;double last_report=0,review_at=-1;
  while(rclcpp::ok()&&elapsed()<(managed?240:100)&&!fault){rclcpp::spin_some(n);
    if(elapsed()-last_report>2){last_report=elapsed();std::cerr<<"ROOM_STATUS "<<nlohmann::json{{"t",elapsed()},{"path_m",path},{"yaw",a},{"relays",relays},{"state",state}}.dump()<<'\n';}
    if(!observe&&started&&nonzero>10&&path>.15&&!pause_sent){std_msgs::msg::String m;m.data=nlohmann::json{{"task_id",state.value("task_id","")},{"request_id","virtual-room-pause"},{"action","pause"}}.dump();commands->publish(m);pause_sent=true;}
    if(observe&&state.value("phase","")=="REVIEW_REQUIRED"&&review_at<0)review_at=elapsed();
    if((!observe&&paused)||(!managed&&review_at>=0&&elapsed()-review_at>3))break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  const bool success=started&&!fault&&(complete?(state.value("phase","")=="REVIEW_REQUIRED"&&state.value("outcome","")=="complete"):
    (nonzero>10&&path>.15&&(managed|| (observe?state.value("phase","")=="REVIEW_REQUIRED":paused))));
  std::cout<<"VIRTUAL_ROOM "<<nlohmann::json{{"started",started},{"nonzero_commands",nonzero},{"path_m",path},{"paused",paused},{"fault",fault},{"phase",state.value("phase","")},{"hardware",false},{"success",success}}.dump()<<'\n';
  if(pid>0){kill(-pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}if(pid>0){kill(-pid,SIGKILL);waitpid(pid,nullptr,0);}}
  rclcpp::shutdown();return success?0:1;
}
