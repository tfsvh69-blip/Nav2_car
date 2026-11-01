#include <fstream>
#include <thread>
#include <gtest/gtest.h>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include "carcar_mapping_autonomy/subcell_smac_planner.hpp"
#include "carcar_mapping_autonomy/observation_selector.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
nav_msgs::msg::OccupancyGrid grid(const ca::Json&j) {
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";
  m.info.width=j["width"];m.info.height=j["height"];m.info.resolution=j["resolution"];
  m.info.origin.position.x=j["origin"][0];m.info.origin.position.y=j["origin"][1];
  m.info.origin.orientation.w=1;m.data=j["data"].get<std::vector<int8_t>>();return m;
}
geometry_msgs::msg::PoseStamped pose(const ca::Json&j) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
  p.pose.position.x=j[0];p.pose.position.y=j[1];const double a=j[2];
  p.pose.orientation.z=std::sin(a/2);p.pose.orientation.w=std::cos(a/2);return p;
}
class RecordedLayer:public nav2_costmap_2d::ObstacleLayer {
public:RecordedLayer(){enabled_=true;}
};
TEST(RecordedStartBlock, ReturnFromStoppedPoseWithObservedScan) {
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"213");
  ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");rclcpp::init(0,nullptr);
  std::ifstream file(START_BLOCK_FIXTURE);auto j=ca::Json::parse(file);
  auto raw=grid(j["raw"]),dynamic=grid(j["dynamic"]);auto start=pose(j["pose"]),goal=pose(j["origin"]);
  const ca::Pose current{start.pose.position.x,start.pose.position.y,0};
  std::cout<<"原图圆检查="<<ca::free_footprint(raw,current)<<" 当前扫描圆检查="<<ca::free_connector(dynamic,current,current)<<std::endl;
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("recorded_start_costmap");
  for(const auto&r:map->set_parameters({{"plugins",std::vector<std::string>{}},
    {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
    {"rolling_window",false},{"track_unknown_space",true},
    {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
    {"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
  map->configure();map->getCostmap()->resizeMap(raw.info.width,raw.info.height,raw.info.resolution,
    raw.info.origin.position.x,raw.info.origin.position.y);
  auto costs=j["global"]["data"].get<std::vector<unsigned char>>();
  std::copy(costs.begin(),costs.end(),map->getCostmap()->getCharMap());
  auto obstacle=std::make_shared<RecordedLayer>();obstacle->resizeMap(raw.info.width,raw.info.height,
    raw.info.resolution,raw.info.origin.position.x,raw.info.origin.position.y);
  for(size_t i=0;i<dynamic.data.size();++i)obstacle->getCharMap()[i]=dynamic.data[i]?254:0;
  map->getLayeredCostmap()->getPlugins()->push_back(obstacle);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("recorded_start_planner");
  node->declare_parameter("GridBased.allow_unknown",false);node->declare_parameter("GridBased.max_planning_time",1.);
  ca::SubcellSmacPlanner planner;planner.configure(node,"GridBased",map->getTfBuffer(),map);
  auto io=std::make_shared<rclcpp::Node>("recorded_start_input");
  auto pub=io->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node->get_node_base_interface());executor.add_node(io);
  pub->publish(raw);
  for(int i=0;i<30;++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  try {const auto path=planner.createPlan(start,goal);EXPECT_FALSE(path.poses.empty());}
  catch(const std::exception&e){ADD_FAILURE()<<e.what();}
  planner.cleanup();map->getLayeredCostmap()->getPlugins()->clear();map->cleanup();rclcpp::shutdown();
}
}
