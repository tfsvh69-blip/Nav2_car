#include <fstream>
#include <deque>
#include "carcar_mapping_autonomy/auto_motion.hpp"
#include "carcar_mapping_autonomy/mapping_rotation_shim.hpp"
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
// 真实控制器、周期重规划、与正式相同的平滑约束及执行延迟；仅离线积分。
TEST_F(SubcellPlanner, KnownNarrowCorridorAndCornerWithHeadingAndDelay) {
 const bool corner=std::getenv("CLEARANCE_CORNER")!=nullptr;
 const double yaw=std::getenv("CLEARANCE_YAW")?std::stod(std::getenv("CLEARANCE_YAW")):0;
 const int delay=std::getenv("CLEARANCE_DELAY")?std::stoi(std::getenv("CLEARANCE_DELAY")):0;
 const double offset=std::getenv("CLEARANCE_OFFSET")?std::stod(std::getenv("CLEARANCE_OFFSET")):0;
 raw.info.width=100;raw.info.height=90;raw.info.resolution=.05;
 raw.info.origin.position.x=-2.025;raw.info.origin.position.y=-1.275;
 raw.data.assign(9000,100);
 for(int y=20;y<=30;++y)for(int x=10;x<(corner?66:90);++x)raw.data[y*100+x]=0;
 if(corner)for(int y=20;y<=65;++y)for(int x=55;x<=65;++x)raw.data[y*100+x]=0;
 auto*global=map->getCostmap();global->resizeMap(100,90,.05,-2.025,-1.275);
 const auto mask=ca::planning_keepout_mask(raw);ASSERT_TRUE(mask);
 for(size_t i=0;i<raw.data.size();++i)global->getCharMap()[i]=mask->data[i]?254:0;
 feed();
 auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("narrow_local");
 for(const auto&r:local->set_parameters({{"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
   {"global_frame","map"},{"robot_base_frame","base_footprint"},{"rolling_window",false},{"track_unknown_space",true},
   {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
 local->configure();local->getCostmap()->resizeMap(100,90,.05,-2.025,-1.275);
 for(size_t i=0;i<raw.data.size();++i)local->getCostmap()->getCharMap()[i]=raw.data[i]>=50?254:0;
 auto cn=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",rclcpp::NodeOptions().arguments(
   {"--ros-args","--params-file",RPP_CONFIG,"--params-file",SHIM_CONFIG}).automatically_declare_parameters_from_overrides(true));
 cn->declare_parameter("exploration_goal_checker.xy_goal_tolerance",.10);
 nav2_controller::PositionGoalChecker checker;checker.initialize(cn,"exploration_goal_checker",local);
 ca::MappingRotationShim controller;controller.configure(cn,"FollowPath",local->getTfBuffer(),local);controller.activate();
 ca::Pose current{0,offset,yaw};auto goal=pose(corner?1.:1.5,corner?1.5:0);
 geometry_msgs::msg::Twist actual,filtered;std::deque<geometry_msgs::msg::Twist>queue(delay,actual);
 bool reached=false,safe=true;size_t replans=0;ca::Json trace=ca::Json::array();
 for(int step=0;step<1800;++step){
   auto here=pose(current.x,current.y);here.header.stamp=cn->now();here.pose.orientation.z=std::sin(current.yaw/2);here.pose.orientation.w=std::cos(current.yaw/2);
   geometry_msgs::msg::TransformStamped tf;tf.header=here.header;tf.child_frame_id="base_footprint";
   tf.transform.translation.x=current.x;tf.transform.translation.y=current.y;tf.transform.rotation=here.pose.orientation;
   local->getTfBuffer()->setTransform(tf,"offline",true);
   if(checker.isGoalReached(here.pose,goal.pose,actual)){reached=true;break;}
   if(step%20==0){controller.setPlan(planner.createPlan(here,goal));++replans;feed();}
   const auto request=controller.computeVelocityCommands(here,actual,&checker).twist;
   const double dv=request.linear.x-filtered.linear.x,dw=request.angular.z-filtered.angular.z;
   double scale=1.;if(std::abs(dv)>.01)scale=std::min(scale,.01/std::abs(dv));if(std::abs(dw)>.04)scale=std::min(scale,.04/std::abs(dw));
   filtered.linear.x=std::clamp(filtered.linear.x+dv*scale,0.,.10);filtered.angular.z=std::clamp(filtered.angular.z+dw*scale,-.36,.36);
   const auto guarded=ca::auto_motion_check(raw,ca::Task(),current,{},filtered,actual);
   queue.push_back(guarded.velocity);actual=queue.front();queue.pop_front();
   const double angle=actual.angular.z*.05;
   ca::Pose next{current.x+actual.linear.x*.05*std::cos(current.yaw+angle/2),current.y+actual.linear.x*.05*std::sin(current.yaw+angle/2),current.yaw+angle};
   safe=ca::free_connector(raw,current,next,.22);
   trace.push_back({{"step",step},{"pose",{next.x,next.y,next.yaw}},{"velocity",{actual.linear.x,actual.angular.z}},{"safe",safe}});
   if(!safe){ADD_FAILURE()<<"窄道连续基础净空越界 step="<<step;break;}current=next;
 }
 if(const char*out=std::getenv("CLEARANCE_TRACE"))std::ofstream(out)<<trace.dump(2);
 EXPECT_TRUE(safe);EXPECT_TRUE(reached);EXPECT_GT(replans,0u);
 controller.deactivate();controller.cleanup();local->cleanup();
}

}
