#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include "carcar_mapping_autonomy/task_walls.hpp"

namespace ca=carcar_mapping_autonomy;
class MappingTaskScene:public rclcpp::Node {
public:
  MappingTaskScene():Node("mapping_task_scene"){
    const auto task=declare_parameter("task_id",std::string());
    if(!task.empty())scene_=std::make_unique<ca::TaskScene>(task);
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    maps_=create_publisher<nav_msgs::msg::OccupancyGrid>("/mapping/navigation_map",qos);
    masks_=create_publisher<nav_msgs::msg::OccupancyGrid>("/mapping/walls/mask",qos);
    info_=create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/mapping/walls/filter_info",qos);
    states_=create_publisher<std_msgs::msg::String>("/mapping/walls/state",qos);
    source_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map){
        map_received_=Clock::now();cached_map_=std::move(map);
        if(scene_)scene_->map(cached_map_);
        publish();});
    generation_=create_subscription<std_msgs::msg::UInt64>("/mapping/navigation_generation",qos,
      [this](const std_msgs::msg::UInt64&value){cached_generation_=value.data;have_generation_=true;
        if(scene_)scene_->generation(value.data);
        publish();});
    if(task.empty())autonomy_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",qos,
      [this](const std_msgs::msg::String&message){try{
        const auto state=ca::Json::parse(message.data);const auto id=state.value("task_id",std::string());
        if(scene_||!ca::task_wall_id_valid(id))return;
        // 默认XML没有task_id时只绑定首次有效任务，不从磁盘继承，也不切换到后续任务。
        scene_=std::make_unique<ca::TaskScene>(id);
        if(have_generation_)scene_->generation(cached_generation_);
        if(cached_map_){scene_->map(cached_map_);
          if(std::chrono::duration<double>(Clock::now()-map_received_).count()>10)scene_->stale();}
        publish();
      }catch(const std::exception&){};});
    requests_=create_subscription<std_msgs::msg::String>("/mapping/walls/request",rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String&value){
        if(!scene_){publish();return;}
        if(value.data.size()>65536)scene_->invalid_request("人工墙请求超过64KiB，未应用");
        else try{scene_->request(ca::Json::parse(value.data,[](int depth,ca::Json::parse_event_t,ca::Json&){
          if(depth>16)throw std::invalid_argument("人工墙请求嵌套过深");
          return true;}));}
        catch(const std::exception&){scene_->invalid_request("人工墙请求JSON无效，未应用");}
        publish();
      });
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{
      if(scene_&&map_received_!=Clock::time_point{}&&std::chrono::duration<double>(Clock::now()-map_received_).count()>10)scene_->stale();
      publish();
    });
    nav2_msgs::msg::CostmapFilterInfo filter;filter.header.frame_id="map";filter.header.stamp=now();
    filter.type=0;filter.base=0;filter.multiplier=1;filter.filter_mask_topic="/mapping/walls/mask";info_->publish(filter);
    publish();
  }
private:
  using Clock=std::chrono::steady_clock;
  void publish(){
    // 先发布约束图及墙mask，最后确认同一revision可用；停稳与跨节点应用屏障由监督器掌握。
    if(scene_&&scene_->ready()&&scene_->output_sequence()!=published_){
      maps_->publish(scene_->output()->navigation);masks_->publish(scene_->output()->mask);published_=scene_->output_sequence();
    }
    auto state=scene_?scene_->state():ca::Json{{"task_id",""},{"phase","FAILED"},{"map_ready",false},
      {"error","等待首次有效建图任务标识；未继承任何人工墙"},{"walls",ca::Json::array()}};
    state["source_age_seconds"]=map_received_==Clock::time_point{}?
      ca::Json(nullptr):ca::Json(std::chrono::duration<double>(Clock::now()-map_received_).count());
    std_msgs::msg::String message;message.data=state.dump();states_->publish(message);
  }
  std::unique_ptr<ca::TaskScene>scene_;Clock::time_point map_received_{};uint64_t published_=0,cached_generation_=0;
  bool have_generation_=false;nav_msgs::msg::OccupancyGrid::ConstSharedPtr cached_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr maps_,masks_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr states_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr source_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr generation_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requests_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomy_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<MappingTaskScene>());rclcpp::shutdown();return 0;}
  catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("mapping_task_scene"),"本次任务人工墙节点退出：%s",e.what());
    if(rclcpp::ok())rclcpp::shutdown();
    return 1;}
}
