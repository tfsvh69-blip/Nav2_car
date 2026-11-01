#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "carcar_mapping_autonomy/planning_departure.hpp"
#include "carcar_mapping_autonomy/map_fingerprint.hpp"
// 复用Humble KeepoutFilter，原图和足迹不变。独立进程计算，不占用速度循环。
class KnownSpace:public rclcpp::Node {
public:
 KnownSpace():Node("mapping_known_space"),tf_(get_clock()),listener_(tf_){
  const auto qos=rclcpp::QoS(1).reliable().transient_local();
  masks_=create_publisher<nav_msgs::msg::OccupancyGrid>("/mapping/known_space_mask",qos);
  info_=create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/mapping/known_space_filter_info",qos);
  status_=create_publisher<std_msgs::msg::String>("/mapping/known_space/state",qos);
  generation_sub_=create_subscription<std_msgs::msg::UInt64>("/mapping/navigation_generation",qos,
    [this](const std_msgs::msg::UInt64&m){generation_=m.data;dirty_=true;publish();});
  nav2_msgs::msg::CostmapFilterInfo info;info.header.frame_id="map";info.header.stamp=now();info.type=0;
  info.filter_mask_topic="/mapping/known_space_mask";info.base=0;info.multiplier=1;info_->publish(info);
  maps_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",qos,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
   if(!carcar_mapping_autonomy::grid_valid(*m))return;
   if(previous_&&carcar_mapping_autonomy::same_grid(*previous_,*m))return;
   const auto start=std::chrono::steady_clock::now();
   auto mask=carcar_mapping_autonomy::planning_keepout_mask(*m);
   if(!mask){RCLCPP_ERROR(get_logger(),"规划掩膜拒绝旋转或无效地图；检查SLAM坐标系");return;}
   base_=std::move(mask);previous_=m;fingerprint_=carcar_mapping_autonomy::map_content_fingerprint(*m);dirty_=true;publish();
   RCLCPP_INFO(get_logger(),"规划净空掩膜已更新 cells=%zu elapsed_ms=%.2f；原图及0.22米审查保持",
    m->data.size(),std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  });
  timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{publish();});
 }
private:
 void publish(){
  if(!base_||!previous_)return;
  auto mask=*base_;carcar_mapping_autonomy::Pose p;bool recovery=false;
  try{
   const auto transform=tf_.lookupTransform("map","base_footprint",tf2::TimePointZero);
   const double age=(now()-rclcpp::Time(transform.header.stamp)).seconds();
   if(age>=-.1&&age<=.5){p={transform.transform.translation.x,transform.transform.translation.y,0};
    recovery=carcar_mapping_autonomy::allow_planning_departure(*previous_,p,mask);}
  }catch(const tf2::TransformException&){}
  if(dirty_||recovery!=recovering_||(recovery&&std::hypot(p.x-last_.x,p.y-last_.y)>previous_->info.resolution*.5)){
   masks_->publish(mask);dirty_=false;recovering_=recovery;last_=p;
   std_msgs::msg::String state;state.data=carcar_mapping_autonomy::Json{{"source_fingerprint",std::to_string(fingerprint_)},
     {"navigation_generation",generation_},{"source_map_stamp_ns",rclcpp::Time(previous_->header.stamp).nanoseconds()}}.dump();status_->publish(state);
   if(recovery)RCLCPP_INFO(get_logger(),"当前位置仍满足0.22米净空；允许在三格邻域离开额外规划余量区 x=%.3f y=%.3f",p.x,p.y);
  }
 }
 tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr masks_;
 rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr info_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr generation_sub_;
 uint64_t generation_=0,fingerprint_=0;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_;
 rclcpp::TimerBase::SharedPtr timer_;
 nav_msgs::msg::OccupancyGrid::ConstSharedPtr previous_;
 std::optional<nav_msgs::msg::OccupancyGrid>base_;
 bool dirty_=false,recovering_=false;carcar_mapping_autonomy::Pose last_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<KnownSpace>());rclcpp::shutdown();}
