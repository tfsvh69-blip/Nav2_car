// 隔离软件场景：8m走廊、两个3x3m房间、0.9m门。真实SLAM/Nav2/探索器，模拟扫描及轮式运动。
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <thread>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include "carcar_mapping_autonomy/map_geometry.hpp"
extern char **environ;
using Json=nlohmann::json;
struct Wall{double x1,y1,x2,y2;};
static bool inside(double x,double y){return (x>-.5&&x<7.5&&std::abs(y)<.6)||
 (x>1&&x<4&&y>=.6&&y<3.6)||(x>4.5&&x<7.5&&y<=-.6&&y> -3.6);}
int main(int argc,char**argv){
 const std::string domain=std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"";
 if(domain!="203"&&domain!="204")return 2;
 const bool blocked=std::getenv("CARCAR_CORRIDOR_BLOCKED")!=nullptr;
 const bool featured=std::getenv("CARCAR_CORRIDOR_FEATURED")!=nullptr;
 const bool controller_regression=std::getenv("CARCAR_CORRIDOR_RPP_STARTUP")!=nullptr;
 if((blocked&&controller_regression)||(domain=="204"&&!controller_regression))return 2;
 const double angular_deadzone=std::getenv("CARCAR_CORRIDOR_ROTATION_DEADZONE")?.08:0.;
 const double doorway=blocked?.15:.45;
 std::vector<Wall>walls={{-.5,-.6,-.5,.6},{7.5,-.6,7.5,.6},{-.5,.6,2.05,.6},{2.95,.6,7.5,.6},
 {-.5,-.6,6-doorway,-.6},{6+doorway,-.6,7.5,-.6},
 {1,.6,2.05,.6},{2.95,.6,4,.6},{1,.6,1,3.6},{4,.6,4,3.6},{1,3.6,4,3.6},
 {4.5,-.6,6-doorway,-.6},{6+doorway,-.6,7.5,-.6},{4.5,-.6,4.5,-3.6},{7.5,-.6,7.5,-3.6},{4.5,-3.6,7.5,-3.6}};
 // 办公区结构对照：两处0.20m宽、0.15m深的墙面凸起。原平直墙场景保留。
 if(featured){walls.insert(walls.end(),{{4,.45,4,.6},{4,.45,4.2,.45},{4.2,.45,4.2,.6},
   {5,-.6,5,-.45},{5,-.45,5.2,-.45},{5.2,-.6,5.2,-.45}});}
 auto inside_world=[&](double px,double py){return inside(px,py)&&(!featured||
   !((px>4&&px<4.2&&py>.45&&py<.6)||(px>5&&px<5.2&&py>-.6&&py<-.45)));};
 auto ray=[&](double x,double y,double angle){const double dx=std::cos(angle),dy=std::sin(angle);double best=12.;
  for(const auto&w:walls){const double sx=w.x2-w.x1,sy=w.y2-w.y1,den=dx*sy-dy*sx;if(std::abs(den)<1e-9)continue;
   const double qx=w.x1-x,qy=w.y1-y,t=(qx*sy-qy*sx)/den,u=(qx*dy-qy*dx)/den;
   if(t>=0&&u>=0&&u<=1)best=std::min(best,t);}
  return best;};
 rclcpp::init(argc,argv);auto n=std::make_shared<rclcpp::Node>("corridor_truth");
 tf2_ros::TransformBroadcaster tf(*n);tf2_ros::StaticTransformBroadcaster fixed(*n);tf2_ros::Buffer buffer(n->get_clock());tf2_ros::TransformListener listener(buffer);
 geometry_msgs::msg::TransformStamped f;f.header.frame_id="base_footprint";f.child_frame_id="laser_frame";f.transform.translation.x=-.05;f.transform.translation.z=.217;f.transform.rotation.w=1;fixed.sendTransform(f);
 auto scan=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());auto odom=n->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
 Json state=Json::object();geometry_msgs::msg::Twist velocity;nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
 const auto began=std::chrono::steady_clock::now();auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();};
 // 死区回归从背向走廊开始，确保先建立原地转向；不能靠恰好直走来绕过所测缺陷。
 const double initial_yaw=angular_deadzone>0?M_PI:0.;
 double x=0,y=0,a=initial_yaw,path=0,turn=0,velocity_at=-1e9,max_linear=0,max_angular=0;bool entered1=false,entered2=false,started=false,contact=false;
 size_t keep_events=0,revalidation_cancels=0,loop_cancels=0,route_cancels=0,unexplained_cancels=0;std::string last_event;
 auto vsub=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,[&](const geometry_msgs::msg::Twist&v){velocity=v;velocity_at=elapsed();max_linear=std::max(max_linear,std::abs(v.linear.x));max_angular=std::max(max_angular,std::abs(v.angular.z));});
 auto ssub=n->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),[&](const std_msgs::msg::String&m){state=Json::parse(m.data);if(state.value("phase","")=="EXPLORING")started=true;
  const auto reason=state.value("goal_change_reason","");if(reason!=last_event){last_event=reason;if(reason.find("keep_goal")==0)++keep_events;if(reason.find("revalidate_cancel")==0){
   ++revalidation_cancels;if(reason.find("SLAM闭环")!=std::string::npos)++loop_cancels;
   else if(reason.find("净空失效")!=std::string::npos)++route_cancels;else ++unexplained_cancels;}}});
 auto msub=n->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map=m;});
 auto request=n->create_publisher<std_msgs::msg::String>("/mapping/autonomy/request",rclcpp::QoS(1).reliable());
 auto timer=n->create_wall_timer(std::chrono::milliseconds(50),[&]{
  if(elapsed()-velocity_at>.3)velocity=geometry_msgs::msg::Twist();
  const double nx=x+velocity.linear.x*std::cos(a)*.05,ny=y+velocity.linear.x*std::sin(a)*.05;
  bool safe=inside_world(nx,ny);for(int i=0;i<36&&safe;++i)if(ray(nx,ny,i*2*M_PI/36)<.192)safe=false;
  if(safe){path+=std::hypot(nx-x,ny-y);x=nx;y=ny;}else if(std::abs(velocity.linear.x)>.001)contact=true;
  // 仅隔离回归的合成静摩擦模型，0.08并非实物标定。平移时不施加该原地转向死区。
  auto measured=velocity;if(angular_deadzone>0&&std::abs(velocity.linear.x)<.001)
   measured.angular.z=std::copysign(std::max(0.,std::abs(velocity.angular.z)-angular_deadzone),velocity.angular.z);
  a+=measured.angular.z*.05;turn+=std::abs(measured.angular.z*.05);
  // 整车0.22m包络已越过0.6m门线，再加0.03m判定余量。
  entered1=entered1||(x>1.25&&x<3.75&&y>.85);entered2=entered2||(x>4.75&&x<7.25&&y< -.85);
  const auto now=n->now();nav_msgs::msg::Odometry o;o.header.stamp=now;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.z=std::sin(a/2);o.pose.pose.orientation.w=std::cos(a/2);o.twist.twist=measured;odom->publish(o);
  geometry_msgs::msg::TransformStamped t;t.header=o.header;t.child_frame_id="base_footprint";t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=o.pose.pose.orientation;tf.sendTransform(t);
  sensor_msgs::msg::LaserScan s;s.header.stamp=now;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_max=M_PI;s.angle_increment=2*M_PI/540;s.scan_time=.05;s.range_min=.05;s.range_max=12;s.ranges.resize(541);
  const double sx=x-.05*std::cos(a),sy=y-.05*std::sin(a);for(size_t i=0;i<s.ranges.size();++i)s.ranges[i]=ray(sx,sy,a+s.angle_min+i*s.angle_increment);scan->publish(s);
 });
 char root[]="/tmp/carcar-corridor-XXXXXX";if(!mkdtemp(root))return 3;std::string maps="map_root:="+std::string(root);
 std::vector<std::string> command={"/opt/ros/humble/bin/ros2","launch","carcar_mapping_console","mapping_autonomous.launch.xml","start_hardware:=false","mode:=new","control_mode:=automatic",domain=="204"?"port:=18100":"port:=18098","record_raw_bag:=false",maps,blocked?"explore_seconds:=180.0":"explore_seconds:=2400.0"};
 if(const auto*params=std::getenv("CARCAR_CORRIDOR_CONTROLLER_PARAMS"))command.push_back(std::string("controller_params:=")+params);
 std::vector<char*> args;for(auto&arg:command)args.push_back(arg.data());args.push_back(nullptr);
 pid_t pid=-1;posix_spawnattr_t attr;posix_spawnattr_init(&attr);posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
 if(posix_spawn(&pid,command[0].c_str(),nullptr,&attr,args.data(),environ)!=0)return 3;
 posix_spawnattr_destroy(&attr);
 double last_report=0;bool finished=false,pause_requested=false;const double limit=controller_regression?60:(blocked?600:1500);
 while(rclcpp::ok()&&elapsed()<limit){rclcpp::spin_some(n);
  if(elapsed()-last_report>10){last_report=elapsed();std::cerr<<"CORRIDOR_STATUS "<<Json{{"t",elapsed()},{"truth",{x,y,a}},{"path",path},{"entered",{entered1,entered2}},{"state",state}}.dump()<<'\n';}
  if(!started&&elapsed()>45){std::cerr<<"启动45秒内未就绪，结束本轮并保留证据\n";break;}
  if(controller_regression&&angular_deadzone>0&&started&&elapsed()>25&&path<.02&&turn<.02&&max_angular>.01){std::cerr<<"合成死区下有转向命令但没有建立运动，保留失败\n";break;}
  const auto phase=state.value("phase","");if(phase=="FAULT"||phase=="REVIEW_REQUIRED"){finished=true;break;}
  if(controller_regression){
   if(!pause_requested&&phase=="EXPLORING"&&path>=.50&&(angular_deadzone==0||turn>=1.)&&state.value("accepted_sequence",0)>=7){
    std_msgs::msg::String m;m.data=Json{{"task_id",state.at("task_id")},{"request_id","rpp-startup-regression-pause"},{"action","pause"}}.dump();request->publish(m);pause_requested=true;
   }
   if(pause_requested&&phase=="PAUSED"&&state.value("navigation_terminal",false)&&state.value("still",false)&&
      state.value("output",std::vector<double>{1,1})==std::vector<double>{0,0}){finished=true;break;}
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
 }
 double coverage=0;size_t denominator=0,known=0;
 if(map){try{const auto tr=buffer.lookupTransform("map","odom",tf2::TimePointZero);const auto&q=tr.transform.rotation;const double angle=std::atan2(2*q.w*q.z,1-2*q.z*q.z);
  for(double gy=-3.5;gy<3.6;gy+=.05)for(double gx=-.4;gx<7.5;gx+=.05){if(!inside_world(gx,gy))continue;
   double wall_distance=12;for(int i=0;i<36;++i)wall_distance=std::min(wall_distance,ray(gx,gy,i*2*M_PI/36));if(wall_distance<.10)continue;
   ++denominator;const double px=tr.transform.translation.x+std::cos(angle)*gx-std::sin(angle)*gy,py=tr.transform.translation.y+std::sin(angle)*gx+std::cos(angle)*gy;
   const double ma=carcar_mapping_autonomy::grid_yaw(*map),dx=px-map->info.origin.position.x,dy=py-map->info.origin.position.y;
   int ix=std::floor((std::cos(ma)*dx+std::sin(ma)*dy)/map->info.resolution),iy=std::floor((-std::sin(ma)*dx+std::cos(ma)*dy)/map->info.resolution);
   if(ix>=0&&iy>=0&&ix<(int)map->info.width&&iy<(int)map->info.height){const auto v=map->data[iy*map->info.width+ix];if(v>=0&&v<50)++known;}
  }
  coverage=denominator?static_cast<double>(known)/denominator:0;
 }catch(const tf2::TransformException&e){std::cerr<<e.what()<<'\n';}}
 const bool returned=state.value("return_result","")=="succeeded";
 // 闭环／真实净空失效本就要求取消重验；不能将这类安全取消冒充普通扩边抢断。
 // 普通安全地图更新保持UUID由真实监督器回归另行确认；此处要求所有重验均有已知原因。
 const bool success=started&&finished&&!contact&&unexplained_cancels==0&&max_linear<=.100001&&max_angular<=.360001&&
  (controller_regression?(pause_requested&&revalidation_cancels==0&&path>=.50&&(angular_deadzone==0||turn>=1.)&&state.value("accepted_sequence",0)>=7):
   (blocked?((state.value("outcome","")=="partial_unreachable"||state.value("outcome","")=="time_budget")&&entered1&&!entered2&&returned):(entered1&&entered2&&coverage>=.95&&returned&&state.value("outcome","")=="complete")));
 Json result={{"success",success},{"hardware",false},{"controller_startup_regression",controller_regression},{"synthetic_angular_deadzone",angular_deadzone},{"featured_geometry",featured},{"blocked",blocked},{"seconds",elapsed()},{"entered_rooms",{entered1,entered2}},{"coverage",coverage},{"known_cells",known},{"reference_cells",denominator},{"path_m",path},{"turn_radians",turn},{"contact",contact},{"max_linear",max_linear},{"max_angular",max_angular},{"kept_goals",keep_events},{"revalidation_cancels",revalidation_cancels},{"map_root",root},{"final",state}};
 result["loop_revalidation_cancels"]=loop_cancels;result["route_revalidation_cancels"]=route_cancels;
 result["initial_yaw"]=initial_yaw;
 result["unexplained_revalidation_cancels"]=unexplained_cancels;
 std::cout<<"CORRIDOR_RESULT "<<result.dump()<<std::endl;
 if(pid>0){kill(-pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}if(pid>0){kill(-pid,SIGKILL);waitpid(pid,nullptr,0);}}
 rclcpp::shutdown();return success?0:1;
}
