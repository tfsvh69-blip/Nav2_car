#include <fstream>
#include <thread>
#include <gtest/gtest.h>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include "carcar_mapping_autonomy/subcell_smac_planner.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
using Json=nlohmann::json;
nav_msgs::msg::OccupancyGrid recorded_map(const Json&j){
  const auto&m=j.at("map");nav_msgs::msg::OccupancyGrid map;map.header.frame_id="map";
  map.info.width=m["info"]["width"];map.info.height=m["info"]["height"];map.info.resolution=m["info"]["resolution"];
  map.info.origin.position.x=m["info"]["origin"]["position"]["x"];map.info.origin.position.y=m["info"]["origin"]["position"]["y"];
  map.info.origin.orientation.w=1;map.data=m["data"].get<std::vector<int8_t>>();return map;
}
geometry_msgs::msg::PoseStamped pose(double x,double y,double a){
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(a/2);p.pose.orientation.w=std::cos(a/2);return p;
}
Json points(const nav_msgs::msg::Path&path){Json result=Json::array();for(const auto&p:path.poses)result.push_back({p.pose.position.x,p.pose.position.y});return result;}
Json clearance(const nav_msgs::msg::OccupancyGrid&map,const nav_msgs::msg::Path&path){
  std::vector<double>distances;double length=0;bool safe=true;ca::Pose previous;
  for(size_t i=0;i<path.poses.size();++i){const auto&p=path.poses[i].pose.position;ca::Pose here{p.x,p.y,0};
    if(i){length+=std::hypot(here.x-previous.x,here.y-previous.y);safe=safe&&ca::free_connector(map,previous,here);}
    previous=here;double nearest=2.;const double r=map.info.resolution;
    const int gx=std::floor((p.x-map.info.origin.position.x)/r),gy=std::floor((p.y-map.info.origin.position.y)/r),reach=std::ceil(2/r);
    for(int y=std::max(0,gy-reach);y<std::min(int(map.info.height),gy+reach+1);++y)
      for(int x=std::max(0,gx-reach);x<std::min(int(map.info.width),gx+reach+1);++x){
        if(map.data[size_t(y)*map.info.width+x]<50)continue;
        const double left=map.info.origin.position.x+x*r,bottom=map.info.origin.position.y+y*r;
        nearest=std::min(nearest,std::hypot(p.x-std::clamp(p.x,left,left+r),p.y-std::clamp(p.y,bottom,bottom+r)));
      }
    distances.push_back(nearest);
  }
  std::sort(distances.begin(),distances.end());
  return {{"length_m",length},{"safe",safe},{"points",path.poses.size()},
    {"known_obstacle_min_m",distances.empty()?0:distances.front()},
    {"known_obstacle_p10_m",distances.empty()?0:distances[distances.size()/10]},
    {"known_obstacle_median_m",distances.empty()?0:distances[distances.size()/2]}};
}
TEST(PlanningClearance, SameRecordedMapGoalAndHardMaskSeparateSoftFieldFromSmoothing){
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"210");ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
  ASSERT_NE(std::getenv("CARCAR_RECORDED_FIXTURE"),nullptr);rclcpp::init(0,nullptr);
  std::ifstream file(std::getenv("CARCAR_RECORDED_FIXTURE"));const auto fixture=Json::parse(file);const auto raw=recorded_map(fixture);
  const auto&initial=fixture["initial_pose"];auto start=pose(initial[0],initial[1],initial[2]);const auto&last=fixture["route"]["poses"].back();
  auto goal=pose(last["x"],last["y"],2*std::atan2(last["qz"].get<double>(),last["qw"].get<double>()));
  Json results=Json::array();
  for(const auto profile:std::vector<std::pair<double,double>>{{.30,1.},{.50,1.},{.50,2.}}){
    const auto radius=profile.first,weight=profile.second;
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("clearance_costmap");
    for(const auto&r:map->set_parameters({{"plugins",std::vector<std::string>{"inflation_layer"}},{"filters",std::vector<std::string>{}},
      {"global_frame","map"},{"robot_base_frame","base_footprint"},{"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
    map->declare_parameter("inflation_layer.plugin",std::string("nav2_costmap_2d::InflationLayer"));
    map->declare_parameter("inflation_layer.inflation_radius",radius);map->declare_parameter("inflation_layer.cost_scaling_factor",3.);
    map->configure();map->getLayeredCostmap()->resizeMap(raw.info.width,raw.info.height,raw.info.resolution,raw.info.origin.position.x,raw.info.origin.position.y);
    auto*master=map->getCostmap();
    for(size_t i=0;i<raw.data.size();++i)master->getCharMap()[i]=raw.data[i]<0?255:raw.data[i]>=50?254:0;
    auto inflation=std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(map->getLayeredCostmap()->getPlugins()->front());ASSERT_TRUE(inflation);
    double minx=1e9,miny=1e9,maxx=-1e9,maxy=-1e9;inflation->updateBounds(start.pose.position.x,start.pose.position.y,0,&minx,&miny,&maxx,&maxy);
    inflation->updateCosts(*master,0,0,raw.info.width,raw.info.height);
    auto mask=ca::planning_keepout_mask(raw);ASSERT_TRUE(mask);ca::allow_planning_departure(raw,{initial[0],initial[1],initial[2]},*mask);
    size_t free=0,soft=0;for(size_t i=0;i<raw.data.size();++i){if(mask->data[i])master->getCharMap()[i]=254;else{++free;if(master->getCharMap()[i]>0&&master->getCharMap()[i]<253)++soft;}}
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server",rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,"--params-file",MAPPING_CONFIG}).automatically_declare_parameters_from_overrides(true));
    for(const auto*name:{"GridBased","ExplorationGrid"})node->declare_parameter(std::string(name)+".cost_travel_multiplier",weight);
    auto io=std::make_shared<rclcpp::Node>("clearance_fixture");auto pub=io->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::Path unsmoothed;auto sub=io->create_subscription<nav_msgs::msg::Path>("/unsmoothed_plan",10,[&](const nav_msgs::msg::Path&m){unsmoothed=m;});
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node->get_node_base_interface());executor.add_node(io);
    for(const auto*name:{"GridBased","ExplorationGrid"}){
      ca::SubcellSmacPlanner planner;planner.configure(node,name,map->getTfBuffer(),map);planner.activate();pub->publish(raw);
      for(int i=0;i<30;++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
      start.header.stamp=goal.header.stamp=node->now();unsmoothed=nav_msgs::msg::Path();nav_msgs::msg::Path planned;std::string error;
      try{planned=planner.createPlan(start,goal);}catch(const std::exception&e){error=e.what();}
      for(int i=0;i<20&&unsmoothed.poses.empty();++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
      const auto stats=clearance(raw,planned),before=clearance(raw,unsmoothed);
      results.push_back({{"planner",name},{"inflation_radius",radius},{"cost_scaling_factor",3.},{"cost_travel_multiplier",node->get_parameter(std::string(name)+".cost_travel_multiplier").as_double()},
        {"eligible_cells",free},{"eligible_soft_cells",soft},{"soft_fraction",free?double(soft)/free:0.},
        {"raw_path",points(unsmoothed)},{"path",points(planned)},{"before",before},{"after",stats},{"error",error}});
      EXPECT_TRUE(error.empty())<<name<<":"<<radius<<":"<<weight<<":"<<error;
      EXPECT_FALSE(planned.poses.empty());EXPECT_FALSE(unsmoothed.poses.empty());EXPECT_TRUE(stats["safe"]);
      planner.deactivate();planner.cleanup();
    }
    executor.remove_node(node->get_node_base_interface());executor.remove_node(io);map->cleanup();
  }
  const char*out=std::getenv("CARCAR_PLANNING_OUTPUT");ASSERT_NE(out,nullptr);
  std::ofstream(out)<<Json{{"goal_id",fixture["goal_id"]},{"hardware",false},{"replanned_from_recording",true},{"results",results}}.dump(2);
  for(auto j:results){j.erase("raw_path");j.erase("path");std::cout<<"PLANNING_CLEARANCE "<<j.dump()<<std::endl;}
  rclcpp::shutdown();
}
}
