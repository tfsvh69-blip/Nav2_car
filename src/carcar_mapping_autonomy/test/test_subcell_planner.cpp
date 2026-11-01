#include <fstream>
#include <thread>
#include <gtest/gtest.h>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_controller/plugins/position_goal_checker.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include "carcar_mapping_autonomy/subcell_smac_planner.hpp"
namespace ca=carcar_mapping_autonomy;
namespace {
nav_msgs::msg::OccupancyGrid fixture() {
  std::ifstream file(SUBCELL_FIXTURE);auto j=ca::Json::parse(file);
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";
  m.info.width=j["info"]["width"];m.info.height=j["info"]["height"];m.info.resolution=j["info"]["resolution"];
  m.info.origin.position.x=j["info"]["origin"]["position"]["x"];
  m.info.origin.position.y=j["info"]["origin"]["position"]["y"];
  m.info.origin.orientation.w=1;m.data=j["data"].get<std::vector<int8_t>>();return m;
}
geometry_msgs::msg::PoseStamped pose(double x,double y) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1;return p;
}
class RecordedObstacleLayer:public nav2_costmap_2d::ObstacleLayer {
public:
  RecordedObstacleLayer(){enabled_=true;}
};
class SubcellPlanner:public testing::Test {
protected:
  static void SetUpTestSuite(){
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"213");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite(){rclcpp::shutdown();}
  nav_msgs::msg::OccupancyGrid raw;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr io;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  ca::SubcellSmacPlanner planner;
  const geometry_msgs::msg::PoseStamped stopped=pose(.04239974986262743,-.012599852844464556);
  const geometry_msgs::msg::PoseStamped target=pose(.47661976260876737,.0038987502426766074);
  void feed(){
    raw.header.stamp=io->now();publisher->publish(raw);
    const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
    while(std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  }
  void SetUp()override {
    raw=fixture();map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("subcell_map");
    for(const auto&r:map->set_parameters({{"plugins",std::vector<std::string>{}},
      {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
      {"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
      {"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
    map->configure();ASSERT_EQ(map->get_current_state().label(),"inactive");
    map->getCostmap()->resizeMap(raw.info.width,raw.info.height,raw.info.resolution,
      raw.info.origin.position.x,raw.info.origin.position.y);
    const auto mask=ca::planning_keepout_mask(raw);ASSERT_TRUE(mask);
    for(size_t i=0;i<raw.data.size();++i)map->getCostmap()->getCharMap()[i]=mask->data[i]?254:0;
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("subcell_planner");
    node->declare_parameter("ExplorationGrid.allow_unknown",false);
    node->declare_parameter("ExplorationGrid.max_planning_time",1.);
    node->declare_parameter("ExplorationGrid.use_final_approach_orientation",true);
    planner.configure(node,"ExplorationGrid",map->getTfBuffer(),map);
    io=std::make_shared<rclcpp::Node>("subcell_input");
    publisher=io->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
    executor.add_node(node->get_node_base_interface());executor.add_node(io);feed();
  }
  void TearDown()override {
    planner.cleanup();map->getLayeredCostmap()->getPlugins()->clear();map->cleanup();
    executor.remove_node(node->get_node_base_interface());executor.remove_node(io);
    publisher.reset();io.reset();node.reset();map.reset();
  }
};
TEST_F(SubcellPlanner, HistoricalFailureBecomesContinuousCheckedPathWithoutClearingMask) {
  const auto before=std::vector<unsigned char>(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+raw.data.size());
  nav2_smac_planner::SmacPlanner2D baseline;baseline.configure(node,"Legacy",map->getTfBuffer(),map);
  EXPECT_TRUE(baseline.createPlan(stopped,target).poses.empty());baseline.cleanup();
  const auto path=planner.createPlan(stopped,target);ASSERT_GT(path.poses.size(),2U);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x,stopped.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y,stopped.pose.position.y);
  for(size_t i=1;i<path.poses.size();++i) {
    const auto&a=path.poses[i-1].pose.position;const auto&b=path.poses[i].pose.position;
    EXPECT_TRUE(ca::free_connector(raw,{a.x,a.y,0},{b.x,b.y,0}));
  }
  EXPECT_LT(std::hypot(path.poses.back().pose.position.x-target.pose.position.x,path.poses.back().pose.position.y-target.pose.position.y),.1);
  EXPECT_EQ(before,std::vector<unsigned char>(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+raw.data.size()));
  EXPECT_EQ(map->getCostmap()->getCost(34,31),254) << "不安全格心继续禁行";
}
TEST_F(SubcellPlanner, ActualUnknownFootprintIsNeverBypassed) {
  raw.data[31*raw.info.width+34]=-1;feed();EXPECT_THROW(planner.createPlan(stopped,target),std::runtime_error);
}
TEST_F(SubcellPlanner, DynamicObstacleAtRobotIsNeverBypassed) {
  auto obstacle=std::make_shared<RecordedObstacleLayer>();
  obstacle->resizeMap(raw.info.width,raw.info.height,raw.info.resolution,raw.info.origin.position.x,raw.info.origin.position.y);
  std::fill_n(obstacle->getCharMap(),raw.data.size(),0);obstacle->setCost(34,31,254);
  map->getLayeredCostmap()->getPlugins()->push_back(obstacle);
  EXPECT_THROW(planner.createPlan(stopped,target),std::runtime_error);
}
TEST_F(SubcellPlanner, MapCostmapMisalignmentRefusesRatherThanUsingWrongCoordinates) {
  raw.info.origin.position.x+=.05;feed();EXPECT_THROW(planner.createPlan(stopped,target),std::runtime_error);
}
TEST_F(SubcellPlanner, NoSafeEntryDoesNotOpenLethalCells) {
  std::fill_n(map->getCostmap()->getCharMap(),raw.data.size(),254);
  EXPECT_THROW(planner.createPlan(stopped,target),std::runtime_error);
  EXPECT_TRUE(std::all_of(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+raw.data.size(),[](auto v){return v==254;}));
}
TEST_F(SubcellPlanner, RepeatedPlanningAtRecordedMovingPosesKeepsWorking) {
  for(const auto&start:{pose(0,0),pose(.003045935,-.000634294),stopped}) {
    const auto path=planner.createPlan(start,target);ASSERT_FALSE(path.poses.empty());
    EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x,start.pose.position.x);
  }
}
TEST_F(SubcellPlanner, RealRppAndPeriodicReplanningLeaveRecordedStopWithoutCrossingUnknown) {
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("subcell_local");
  for(const auto&r:local->set_parameters({{"plugins",std::vector<std::string>{}},
    {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
    {"rolling_window",false},{"track_unknown_space",true},
    {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
  local->configure();local->getCostmap()->resizeMap(raw.info.width,raw.info.height,raw.info.resolution,
    raw.info.origin.position.x,raw.info.origin.position.y);
  for(size_t i=0;i<raw.data.size();++i)local->getCostmap()->getCharMap()[i]=raw.data[i]<0?255:raw.data[i]>=50?254:0;
  auto controller_node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",
    rclcpp::NodeOptions().arguments({"--ros-args","--params-file",RPP_CONFIG}).automatically_declare_parameters_from_overrides(true));
  controller_node->declare_parameter("exploration_goal_checker.xy_goal_tolerance",.15);
  nav2_controller::PositionGoalChecker checker;checker.initialize(controller_node,"exploration_goal_checker",local);
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController controller;
  controller.configure(controller_node,"FollowPath",local->getTfBuffer(),local);controller.activate();
  ca::Pose current{stopped.pose.position.x,stopped.pose.position.y,-.41358128558844526};
  geometry_msgs::msg::Twist velocity;bool reached=false;size_t replans=0;
  ca::Json trace=ca::Json::array();
  for(int step=0;step<900;++step) {
    auto here=pose(current.x,current.y);here.header.stamp=controller_node->now();
    here.pose.orientation.z=std::sin(current.yaw/2);here.pose.orientation.w=std::cos(current.yaw/2);
    geometry_msgs::msg::TransformStamped tf;tf.header=here.header;tf.child_frame_id="base_footprint";
    tf.transform.translation.x=current.x;tf.transform.translation.y=current.y;tf.transform.rotation=here.pose.orientation;
    local->getTfBuffer()->setTransform(tf,"subcell_software_odometry",true);
    if(checker.isGoalReached(here.pose,target.pose,velocity)){reached=true;break;}
    if(step%20==0){controller.setPlan(planner.createPlan(here,target));++replans;}
    const auto requested=controller.computeVelocityCommands(here,velocity,&checker).twist;
    // 沿用正式平滑器的0.20m/s²与0.80rad/s²界限，避免理想瞬时转向掩盖起步切角。
    const double dv=requested.linear.x-velocity.linear.x,dw=requested.angular.z-velocity.angular.z;
    double scale=1.;if(std::abs(dv)>.01)scale=std::min(scale,.01/std::abs(dv));
    if(std::abs(dw)>.04)scale=std::min(scale,.04/std::abs(dw));
    velocity.linear.x+=dv*scale;velocity.angular.z+=dw*scale;
    const ca::Pose next{current.x+velocity.linear.x*std::cos(current.yaw)*.05,
      current.y+velocity.linear.x*std::sin(current.yaw)*.05,current.yaw+velocity.angular.z*.05};
    trace.push_back({{"step",step},{"pose",{next.x,next.y,next.yaw}},{"velocity",{velocity.linear.x,velocity.angular.z}}});
    if(!ca::free_connector(raw,current,next)) {
      ADD_FAILURE()<<"控制轨迹越过0.22米净空边界 step="<<step<<" pose="<<next.x<<","<<next.y;break;
    }
    current=next;
  }
  std::ofstream("subcell_rpp_trace.json")<<trace.dump(2);
  EXPECT_TRUE(reached);EXPECT_GE(replans,2U);
  controller.deactivate();controller.cleanup();local->cleanup();
}
}  // namespace
