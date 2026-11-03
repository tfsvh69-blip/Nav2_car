// 四种固定软件真值场景；真实SLAM/Nav2/监督器闭环，绝不连接硬件。
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <thread>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nlohmann/json.hpp>
extern char**environ;
using Json=nlohmann::json;
struct Rectangle{double left,right,bottom,top;};
class World{
public:
 explicit World(const std::string&name){
  if(name=="corridor")rooms={{-.6,4.8,-.8,.8}};
  else if(name=="rooms")rooms={{-.6,5.4,-.65,.65},{1,3.2,1.05,3.25},{1.55,2.45,.60,1.10},
    {3.2,5.4,-3.25,-1.05},{3.8,4.7,-1.10,-.60}};
  else if(name=="dead_end")rooms={{-.6,4.8,-.7,.7},{2,3.3,.6,3.7}};
  else if(name=="symmetric")rooms={{-3.2,3.2,-.7,.7},{-.7,.7,-3.2,3.2}};
  else throw std::runtime_error("场景必须为corridor、rooms、dead_end或symmetric");
  for(int y=0;y<height;++y)for(int x=0;x<width;++x)if(free((x+.5)*resolution-4,(y+.5)*resolution-4))++total;
 }
 bool free(double x,double y)const{for(const auto&r:rooms)if(x>r.left&&x<r.right&&y>r.bottom&&y<r.top)return true;return false;}
 double ray(double x,double y,double angle)const{
  const double dx=std::cos(angle),dy=std::sin(angle);double best=12.;
  // 解析求墙面交点；不能把2.5厘米步进量化误差当成理想激光真值。
  auto candidate=[&](double t){if(t>=0&&t<best&&!free(x+(t+1e-7)*dx,y+(t+1e-7)*dy))best=t;};
  for(const auto&r:rooms){
    if(std::abs(dx)>1e-12)for(double wall:{r.left,r.right}){const double t=(wall-x)/dx,hit=y+t*dy;if(hit>=r.bottom&&hit<=r.top)candidate(t);}
    if(std::abs(dy)>1e-12)for(double wall:{r.bottom,r.top}){const double t=(wall-y)/dy,hit=x+t*dx;if(hit>=r.left&&hit<=r.right)candidate(t);}
  }
  return best;
 }
 bool safe(double x,double y)const{if(!free(x,y))return false;for(int i=0;i<36;++i)if(!free(x+.195*std::cos(i*M_PI/18),y+.195*std::sin(i*M_PI/18)))return false;return true;}
 void observe(double x,double y,double angle,double range){
  const double dx=std::cos(angle),dy=std::sin(angle);for(double d=.025;d<std::min(3.,range);d+=.025){
   const double px=x+d*dx,py=y+d*dy;const int gx=int(std::floor((px+4)/resolution)),gy=int(std::floor((py+4)/resolution));
   if(gx>=0&&gy>=0&&gx<width&&gy<height&&free((gx+.5)*resolution-4,(gy+.5)*resolution-4))seen.insert(gy*width+gx);
  }
 }
 static constexpr double resolution=.05;static constexpr int width=220,height=160;
 std::vector<Rectangle>rooms;std::set<int>seen;size_t total=0;
};
class Child{
public:
 pid_t pid=-1;
 ~Child(){stop();}
 bool start(std::vector<std::string>args){std::vector<char*>argv;for(auto&a:args)argv.push_back(a.data());argv.push_back(nullptr);
  posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
  const int code=posix_spawn(&pid,argv[0],nullptr,&attr,argv.data(),environ);posix_spawnattr_destroy(&attr);return code==0;}
 void stop(){if(pid<=0)return;kill(-pid,SIGINT);for(int i=0;i<150;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;return;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  kill(-pid,SIGKILL);waitpid(pid,nullptr,0);pid=-1;}
};
int run_evaluation(int argc,char**argv){
 const std::string domain=std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"";
 const std::string local=std::getenv("ROS_LOCALHOST_ONLY")?std::getenv("ROS_LOCALHOST_ONLY"):"";
 if(argc!=4||(domain!="231"&&domain!="232")||local!="1"){
  std::cerr<<"仅允许本机隔离域231或232；参数为场景、legacy、ranked或continuous、新证据目录\n";return 2;
 }
 const std::string scenario=argv[1],mode=argv[2];if(mode!="legacy"&&mode!="ranked"&&mode!="continuous")return 2;
 const std::filesystem::path evidence=argv[3];if(std::filesystem::exists(evidence)){std::cerr<<"证据目录已存在，拒绝覆盖\n";return 2;}
 std::filesystem::create_directories(evidence);World world(scenario);rclcpp::init(0,nullptr);
 auto node=std::make_shared<rclcpp::Node>("exploration_evaluation_truth");
 tf2_ros::TransformBroadcaster tf(*node);tf2_ros::StaticTransformBroadcaster fixed(*node);
 geometry_msgs::msg::TransformStamped laser;laser.header.frame_id="base_footprint";laser.child_frame_id="laser_frame";laser.transform.translation.x=-.05;laser.transform.translation.z=.217;laser.transform.rotation.w=1;fixed.sendTransform(laser);
 auto scans=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
 auto imu=node->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",rclcpp::SensorDataQoS());
 auto odom=node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
 auto requests=node->create_publisher<std_msgs::msg::String>("/mapping/autonomy/request",10);
 const auto began=std::chrono::steady_clock::now();auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();};
 double x=0,y=0,yaw=0,path=0,repeated=0,window_path=0,velocity_at=-1e9,started_at=-1;size_t initial_seen=0,window_seen=0;
 geometry_msgs::msg::Twist velocity;Json state=Json::object();bool contact=false,started=false;std::set<std::string>checkpoints;
 std::ofstream trace(evidence/"trace.jsonl");
 std::ofstream command_trace(evidence/"commands.jsonl");
 std::ofstream rank_trace(evidence/"ranking.jsonl");
 auto ranking=node->create_subscription<std_msgs::msg::String>("/mapping/exploration/rank_result",10,
   [&](const std_msgs::msg::String&m){try{rank_trace<<Json{{"time_s",elapsed()},{"result",Json::parse(m.data)}}.dump()<<'\n';rank_trace.flush();}catch(const std::exception&){}});
 nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map;
 auto map_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),
   [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){latest_map=std::move(m);});

 auto commands=node->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,[&](const geometry_msgs::msg::Twist&m){
  velocity=m;velocity_at=elapsed();
  // 保存真正进入软件轮式积分的最终命令，后续可核对停顿与状态，不能用目标速度冒充反馈。
  command_trace<<Json{{"time_s",velocity_at},{"vx",m.linear.x},{"wz",m.angular.z},{"phase",state.value("phase","")},
   {"healthy",state.value("healthy",false)},{"route_id",state.value("continuous_exploration",Json::object()).value("route_id",0)}}.dump()<<'\n';
 });
 auto states=node->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){
  state=Json::parse(m.data);if(!started&&state.value("phase","")=="EXPLORING"){started=true;started_at=elapsed();initial_seen=window_seen=world.seen.size();}
  const auto id=state.value("recovery",Json::object()).value("checkpoint_id",std::string());if(!id.empty())checkpoints.insert(id);
 });
 auto timer=node->create_wall_timer(std::chrono::milliseconds(50),[&]{
  if(elapsed()-velocity_at>.3)velocity=geometry_msgs::msg::Twist();
  const double dx=velocity.linear.x*std::cos(yaw)*.05,dy=velocity.linear.x*std::sin(yaw)*.05;
  if(world.safe(x+dx,y+dy)){path+=std::hypot(dx,dy);window_path+=std::hypot(dx,dy);x+=dx;y+=dy;}
  else if(std::hypot(dx,dy)>.00001)contact=true;
  yaw+=velocity.angular.z*.05;const auto stamp=node->now();
  nav_msgs::msg::Odometry o;o.header.stamp=stamp;o.header.frame_id="odom";o.child_frame_id="base_footprint";
  o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.z=std::sin(yaw/2);o.pose.pose.orientation.w=std::cos(yaw/2);o.twist.twist=velocity;odom->publish(o);
  geometry_msgs::msg::TransformStamped t;t.header=o.header;t.child_frame_id="base_footprint";t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=o.pose.pose.orientation;tf.sendTransform(t);
  sensor_msgs::msg::LaserScan scan;scan.header.stamp=stamp;scan.header.frame_id="laser_frame";scan.angle_min=-M_PI;scan.angle_max=M_PI;scan.angle_increment=M_PI/180;scan.range_min=.05;scan.range_max=12;scan.scan_time=.05;
  const double sx=x-.05*std::cos(yaw),sy=y-.05*std::sin(yaw);
  for(int i=0;i<=360;++i){const double angle=yaw-M_PI+i*M_PI/180;const double range=world.ray(sx,sy,angle);scan.ranges.push_back(range);world.observe(sx,sy,angle,range);}scans->publish(scan);
  sensor_msgs::msg::Imu im;im.header.stamp=stamp;im.header.frame_id="imu_link";im.orientation_covariance[0]=-1;im.linear_acceleration.z=9.81;im.angular_velocity.z=velocity.angular.z;imu->publish(im);
  // 以0.25米路段聚合新增面积，避免把栅格量化造成的逐帧零增长当成重复路程。
  if(started&&window_path>=.25){if((world.seen.size()-window_seen)*.0025<.02)repeated+=window_path;window_path=0;window_seen=world.seen.size();}
 });
 char temporary[]="/tmp/carcar-exploration-eval-XXXXXX";if(!mkdtemp(temporary))return 2;Child child;
 std::vector<std::string>args{"/opt/ros/humble/bin/ros2","launch","carcar_mapping_console","mapping_autonomous.launch.xml","start_hardware:=false","record_raw_bag:=false","control_mode:=automatic",std::string("port:=")+std::to_string(18127+std::stoi(domain)-231),"task_id:=exploration-evaluation","explore_seconds:=240.0",std::string("map_root:=")+temporary,
  std::string("checkpoint_root:=")+temporary+"/checkpoints",std::string("tuning_defaults_file:=")+temporary+"/isolated-tuning-defaults.json",std::string("continuous_exploration:=")+(mode=="continuous"?"true":"false"),std::string("online_snapshots:=true")};
 if(mode!="continuous"){
  args.push_back(std::string("path_aware_ranking:=")+(mode=="ranked"?"true":"false"));
  args.push_back("prefer_observation_clearance:=true");args.push_back("auto_execution_sweep:=true");args.push_back("obstacle_wait_enabled:=true");
  args.push_back(std::string("controller_params:=")+EVALUATION_SHIM_PARAMS);args.push_back(std::string("cost_params:=")+EVALUATION_CLEARANCE_PARAMS);
 }
 if(mode=="continuous")args.push_back(std::string("bt_params:=")+EVALUATION_BT_PARAMS);
 if(!child.start(args))return 2;
 std::string stop="budget";double last_trace=-1;std::string last_phase;Json stop_reasons=Json::array();
 while(rclcpp::ok()&&elapsed()<285){rclcpp::spin_some(node);const auto phase=state.value("phase","");
  if(phase!=last_phase){stop_reasons.push_back({{"time_s",elapsed()},{"phase",phase},{"reason",state.value("reason","")}});last_phase=phase;}
  if(elapsed()-last_trace>=1){last_trace=elapsed();trace<<Json{{"time_s",elapsed()},{"pose",{x,y,yaw}},{"path_m",path},{"seen_cells",world.seen.size()},{"phase",phase},{"reason",state.value("reason","")},{"goal",state.value("goal",Json())},
    {"frontier_goal",state.value("frontier_goal",Json())},{"navigation_remaining_m",state.value("navigation_remaining_m",-1.)},
    {"continuous",state.value("continuous_exploration",Json::object())},{"snapshot",state.value("snapshot_maintenance",Json::object()).value("phase","")}}.dump()<<'\n';trace.flush();}
  if(contact){stop="contact";break;}if(phase=="FAULT"){stop="fault";break;}if(started&&phase=="PAUSED"){stop="paused";break;}if(started&&(phase=="RETURNING"||phase=="REVIEW_REQUIRED")){stop=state.value("outcome",phase);break;}
  if(started&&elapsed()-started_at>=240)break;
  if(!started&&elapsed()>45){stop="startup_timeout";break;}
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
 }
 if(!rclcpp::ok())stop="interrupted";
 const Json result{{"hardware",false},{"same_controller_and_soft_costs",true},{"online_snapshots",true},{"scenario",scenario},{"mode",mode},{"speed_mps",.10},{"budget_seconds",240},{"started",started},{"stop",stop},{"contact",contact},
  {"wall_seconds",elapsed()},{"exploration_wall_seconds",started?elapsed()-started_at:0.},{"initial_observable_m2",initial_seen*.0025},{"total_observable_m2",world.total*.0025},
  {"coverage",double(world.seen.size())/world.total},{"new_observable_m2",(world.seen.size()-initial_seen)*.0025},{"path_m",path},{"gain_m2_per_m",path>0?(world.seen.size()-initial_seen)*.0025/path:0.},
  {"no_gain_distance_m",repeated},{"snapshot_count",checkpoints.size()},{"stop_reasons",stop_reasons},{"final_state",state},{"artifacts",temporary}};
 if(latest_map){const auto&m=*latest_map;
  std::ofstream(evidence/"final_map.json")<<Json{{"resolution",m.info.resolution},{"width",m.info.width},{"height",m.info.height},
    {"origin",{m.info.origin.position.x,m.info.origin.position.y}},{"data",m.data}}.dump()<<'\n';
 }
 std::ofstream(evidence/"result.json")<<result.dump(2)<<'\n';std::cout<<"EXPLORATION_EVALUATION "<<result.dump()<<'\n';
 // 先请求真实监督器停车并继续提供新鲜数据，再退出自有子进程组。
 if(rclcpp::ok()){
 std_msgs::msg::String pause;pause.data=Json{{"task_id","exploration-evaluation"},{"request_id","evaluation-stop"},{"action","pause"}}.dump();requests->publish(pause);
 const double until=elapsed()+3;while(rclcpp::ok()&&elapsed()<until){rclcpp::spin_some(node);if(state.value("still",false)&&state.value("navigation_terminal",false))break;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
 }
 child.stop();rclcpp::shutdown();return started&&!contact&&stop!="fault"&&stop!="interrupted"&&stop!="paused"&&stop!="budget"&&stop!="startup_timeout"&&result.at("coverage").get<double>()>=.95?0:1;
}
int main(int argc,char**argv){
 // 异常必须先展开场景栈、回收自有子进程；避免中断时ROS上下文异常跳过清理。
 try{return run_evaluation(argc,argv);}catch(const std::exception&e){
  std::cerr<<"软件场景中止："<<e.what()<<'\n';rclcpp::shutdown();return 2;
 }
}
