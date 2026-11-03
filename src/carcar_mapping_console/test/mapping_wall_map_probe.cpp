// 只读隔离软件地图快照；不创建发布者、不调用参数或控制服务，不接受生产ROS域。
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
using Json=nlohmann::json;
static double yaw(const geometry_msgs::msg::Quaternion&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
int main(int argc,char**argv){
  if(argc!=2||std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"")!="227"||
     std::string(std::getenv("ROS_LOCALHOST_ONLY")?std::getenv("ROS_LOCALHOST_ONLY"):"")!="1"){
    std::cerr<<"只读地图probe仅接受域227/localhost及一个新输出文件\n";return 2;
  }
  const auto output=std::filesystem::absolute(argv[1]);
  if(std::filesystem::exists(output)||std::filesystem::is_symlink(output)){std::cerr<<"输出已存在，拒绝覆盖\n";return 2;}
  rclcpp::init(0,nullptr);auto node=std::make_shared<rclcpp::Node>("mapping_wall_map_probe");
  tf2_ros::Buffer buffer(node->get_clock());tf2_ros::TransformListener listener(buffer);
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr raw,local,global;
  auto qos=rclcpp::QoS(1).reliable().transient_local();
  auto raw_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",qos,[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){raw=m;});
  auto local_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap",qos,[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){local=m;});
  auto global_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",qos,[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){global=m;});
  const auto began=std::chrono::steady_clock::now();std::string reason;
  auto encode=[&](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr&m,double max_age){
    if(!m||m->header.frame_id.empty()||!m->info.width||!m->info.height||m->info.resolution<=0||
       uint64_t(m->info.width)*m->info.height!=m->data.size()||m->data.size()>4000000)throw std::runtime_error("地图缺失或几何无效");
    const auto stamp=rclcpp::Time(m->header.stamp);const double age=(node->now()-stamp).seconds();
    if(stamp.nanoseconds()<=0||age<-.05||age>max_age)throw std::runtime_error("地图消息时刻过期或处于未来："+m->header.frame_id);
    Json to_map={0.,0.,0.};int64_t transform_stamp=stamp.nanoseconds();
    if(m->header.frame_id!="map"){
      const auto transform=buffer.lookupTransform("map",m->header.frame_id,stamp,rclcpp::Duration::from_seconds(0.));
      to_map={transform.transform.translation.x,transform.transform.translation.y,yaw(transform.transform.rotation)};
      transform_stamp=rclcpp::Time(transform.header.stamp).nanoseconds();
    }
    return Json{{"frame_id",m->header.frame_id},{"stamp_ns",stamp.nanoseconds()},{"source_age_seconds",age},
      {"width",m->info.width},{"height",m->info.height},{"resolution",m->info.resolution},
      {"origin",{m->info.origin.position.x,m->info.origin.position.y,yaw(m->info.origin.orientation)}},
      {"to_map",to_map},{"transform_stamp_ns",transform_stamp},{"data",m->data}};
  };
  bool success=false;
  while(rclcpp::ok()&&std::chrono::steady_clock::now()-began<std::chrono::seconds(12)){
    rclcpp::spin_some(node);
    if(std::chrono::steady_clock::now()-began>std::chrono::milliseconds(1200))try{
      const Json result{{"read_only",true},{"domain",227},{"coordinate_reference","map；局部地图按各自消息时刻TF转换，不使用最新TF冒充历史对齐"},
        {"raw",encode(raw,10.)},{"local",encode(local,2.5)},{"global",encode(global,2.5)}};
      std::ofstream file(output);file<<result.dump()<<'\n';file.flush();if(!file)throw std::runtime_error("快照输出写入失败");
      success=true;break;
    }catch(const std::exception&e){reason=e.what();}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if(!success)std::cerr<<"有界只读快照未完成："<<reason<<" raw="<<bool(raw)<<" local="<<bool(local)<<" global="<<bool(global)<<'\n';
  node.reset();rclcpp::shutdown();return success?0:1;
}
