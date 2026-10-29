// 仅软件诊断：把闭包原始costmap/path/pose交给本机真实Nav2插件，不发布速度或启动硬件。
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include <nlohmann/json.hpp>
using Json=nlohmann::json;
namespace fs=std::filesystem;
template<class T>T read_message(const fs::path&p){
 std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error("missing CDR: "+p.string());
 const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(f),{}};
 rclcpp::SerializedMessage serialized(bytes.size());auto&raw=serialized.get_rcl_serialized_message();
 std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
 T message;rclcpp::Serialization<T>().deserialize_message(&serialized,&message);return message;
}
geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw,const std::string&frame){
 geometry_msgs::msg::PoseStamped p;p.header.frame_id=frame;p.pose.position.x=x;p.pose.position.y=y;
 p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);return p;
}
auto costmap(const nav2_msgs::msg::Costmap&m,const std::string&name){
 if(m.data.size()!=static_cast<size_t>(m.metadata.size_x)*m.metadata.size_y)throw std::runtime_error("invalid raw costmap");
 auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>(name);
 const std::vector<rclcpp::Parameter> params={
  {"global_frame",m.header.frame_id},{"robot_base_frame","base_footprint"},
  {"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
  {"rolling_window",false},{"track_unknown_space",true},{"resolution",m.metadata.resolution},
  {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}};
 for(const auto&r:cm->set_parameters(params))if(!r.successful)throw std::runtime_error(r.reason);
 cm->configure();if(cm->get_current_state().label()!="inactive")throw std::runtime_error("costmap configure failed");
 auto*grid=cm->getCostmap();grid->resizeMap(m.metadata.size_x,m.metadata.size_y,m.metadata.resolution,
  m.metadata.origin.position.x,m.metadata.origin.position.y);std::copy(m.data.begin(),m.data.end(),grid->getCharMap());
 return cm;
}
void transforms(const Json&j,const std::shared_ptr<tf2_ros::Buffer>&tf){
 for(const auto&edge:std::vector<std::tuple<std::string,std::string,std::string>>{{"map","odom","map_odom"},{"odom","base_footprint","odom_base"}}){
  const auto&v=j.at(std::get<2>(edge));geometry_msgs::msg::TransformStamped t;t.header.frame_id=std::get<0>(edge);t.child_frame_id=std::get<1>(edge);
  t.transform.translation.x=v.at("x");t.transform.translation.y=v.at("y");const double a=v.at("yaw");t.transform.rotation.z=std::sin(a/2);t.transform.rotation.w=std::cos(a/2);tf->setTransform(t,"closed_bag",true);
 }
}
// 这里只向实际RPP提供固定的既有到点容差；不模拟控制器、路径变换或碰撞检测。
class Tolerances:public nav2_core::GoalChecker{
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,const std::string&,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override{}
 void reset()override{}
 bool isGoalReached(const geometry_msgs::msg::Pose&,const geometry_msgs::msg::Pose&,const geometry_msgs::msg::Twist&)override{return false;}
 bool getTolerances(geometry_msgs::msg::Pose&p,geometry_msgs::msg::Twist&)override{p.position.x=p.position.y=.15;p.orientation.z=std::sin(.35/2);p.orientation.w=std::cos(.35/2);return true;}
};
// 仅暴露本机插件的观察入口；所有路径变换、转向决策与碰撞判定仍由Nav2执行。
class ObservedController:public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController{
public:
 Json inspect(const geometry_msgs::msg::PoseStamped&p,const geometry_msgs::msg::Twist&speed){
  const auto transformed=transformGlobalPlan(p);
  const double lookahead=getLookAheadDistance(speed);
  const auto carrot=getLookAheadPoint(lookahead,transformed);double angle=0;
  const bool rotate_path=shouldRotateToPath(carrot,angle);
  const double yaw=2*std::atan2(p.pose.orientation.z,p.pose.orientation.w);
  return {{"lookahead_m",lookahead},{"carrot_robot",{carrot.pose.position.x,carrot.pose.position.y}},
   {"angle_to_path",angle},{"rotate_path",rotate_path},{"rotate_goal",shouldRotateToGoalHeading(carrot)},
   {"current_footprint_collision",inCollision(p.pose.position.x,p.pose.position.y,yaw)}};
 }
};
int main(int argc,char**argv){
 if(!std::getenv("ROS_DOMAIN_ID")||std::string(std::getenv("ROS_DOMAIN_ID"))!="209"||argc<2)return 2;
 rclcpp::init(argc,argv);Json result={{"hardware",false},{"snapshot",argv[1]}};bool pass=false;
 try{
  const fs::path dir=argv[1];std::ifstream input(dir/"snapshot.json");Json meta;input>>meta;
  auto global=costmap(read_message<nav2_msgs::msg::Costmap>(dir/"global_costmap_costmap_raw.cdr"),"snapshot_global");
  auto local=costmap(read_message<nav2_msgs::msg::Costmap>(dir/"local_costmap_costmap_raw.cdr"),"snapshot_local");
  transforms(meta,global->getTfBuffer());transforms(meta,local->getTfBuffer());
  const auto&start=meta.at("pose_map");const auto&origin=meta.at("state").at("origin");const auto&ob=meta.at("odom_base");
  std::vector<rclcpp::Parameter> pp={{"GridBased.tolerance",.10},{"GridBased.allow_unknown",false},{"GridBased.max_planning_time",1.0}};
  auto pn=std::make_shared<rclcpp_lifecycle::LifecycleNode>("snapshot_planner",rclcpp::NodeOptions().parameter_overrides(pp).automatically_declare_parameters_from_overrides(true));
  nav2_smac_planner::SmacPlanner2D planner;planner.configure(pn,"GridBased",global->getTfBuffer(),global);
  bool planned=false;try{const auto path=planner.createPlan(pose(start[0],start[1],0,"map"),pose(origin[0],origin[1],origin[2],"map"));planned=!path.poses.empty();result["planner"]={{"success",planned},{"poses",path.poses.size()}};}
  catch(const std::exception&e){result["planner"]={{"success",false},{"error",e.what()}};}
  planner.cleanup();auto path=read_message<nav_msgs::msg::Path>(dir/"plan.cdr");path.header.stamp=rclcpp::Time(0);for(auto&p:path.poses)p.header.stamp=rclcpp::Time(0);
  const auto odom=read_message<nav_msgs::msg::Odometry>(dir/"wheel_odometry.cdr");Tolerances checker;size_t index=0;bool baseline=false;
  const double original=meta.at("rpp_parameters").at("rotate_to_heading_min_angle");
  std::vector<double> angles{original};if(argc>2&&std::string(argv[2])=="--sweep")angles.insert(angles.end(),{.52,.35});
  for(const double angle:angles){
   std::vector<rclcpp::Parameter> cp={{"controller_frequency",10.0}};
   for(auto it=meta.at("rpp_parameters").begin();it!=meta.at("rpp_parameters").end();++it){const auto name="FollowPath."+it.key();
    if(it->is_boolean())cp.emplace_back(name,it->get<bool>());else if(it->is_number())cp.emplace_back(name,it->get<double>());else if(it->is_string())cp.emplace_back(name,it->get<std::string>());
   }
   for(auto&p:cp)if(p.get_name()=="FollowPath.rotate_to_heading_min_angle")p=rclcpp::Parameter(p.get_name(),angle);
   auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("snapshot_controller_"+std::to_string(index++),rclcpp::NodeOptions().parameter_overrides(cp).automatically_declare_parameters_from_overrides(true));
   ObservedController controller;controller.configure(node,"FollowPath",local->getTfBuffer(),local);controller.activate();controller.setPlan(path);
   Json row={{"rotate_to_heading_min_angle",angle}};
   try{const auto v=controller.computeVelocityCommands(pose(ob["x"],ob["y"],ob["yaw"],"odom"),odom.twist.twist,&checker);row["success"]=true;row["output"]={v.twist.linear.x,v.twist.angular.z};if(index==1)baseline=true;}
   catch(const std::exception&e){row["success"]=false;row["error"]=e.what();}
   try{row["diagnostic"]=controller.inspect(pose(ob["x"],ob["y"],ob["yaw"],"odom"),odom.twist.twist);}
   catch(const std::exception&e){row["diagnostic_error"]=e.what();}
   result["controller"].push_back(row);controller.deactivate();controller.cleanup();
  }
  pass=planned&&baseline;local->cleanup();global->cleanup();
 }catch(const std::exception&e){result["error"]=e.what();}
 result["baseline_pass"]=pass;std::cout<<"NAVIGATION_REPLAY "<<result.dump()<<std::endl;rclcpp::shutdown();return pass?0:1;
}
