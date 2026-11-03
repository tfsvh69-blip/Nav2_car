// 使用本机真实RPP及配置检查折返路径顺序，不启动硬件、不发布速度命令。
// 夹具是R27软件场景的Smac重建路径，不作为故障时原路径录包或实物轨迹。
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
namespace {
class ObservedRpp:public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
 size_t remaining(const geometry_msgs::msg::PoseStamped&p){transformGlobalPlan(p);return global_plan_.poses.size();}
};
class RppRouteOrder:public testing::Test {
protected:
 static void SetUpTestSuite(){
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"","214");
  ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY")?std::getenv("ROS_LOCALHOST_ONLY"):"","1");
  rclcpp::init(0,nullptr);
 }
 static void TearDownTestSuite(){rclcpp::shutdown();}
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
 rclcpp_lifecycle::LifecycleNode::SharedPtr node;
 ObservedRpp rpp;nav_msgs::msg::Path path;size_t via=0;std::vector<double> start;
 geometry_msgs::msg::PoseStamped at(double x,double y,double angle=0){
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(angle/2);p.pose.orientation.w=std::cos(angle/2);
  geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";
  t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=p.pose.orientation;
  map->getTfBuffer()->setTransform(t,"route_order_test",true);return p;
 }
 void SetUp()override{
  std::ifstream file(FOLDED_ROUTE_FIXTURE);const auto j=nlohmann::json::parse(file);via=j.at("via_index");start=j.at("start").get<std::vector<double>>();
  map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("route_order_map");
  for(const auto&r:map->set_parameters({{"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
    {"global_frame","map"},{"robot_base_frame","base_footprint"},{"rolling_window",false},{"track_unknown_space",false}}))ASSERT_TRUE(r.successful);
  map->configure();ASSERT_EQ(map->get_current_state().label(),"inactive");
  // 与正式局部代价图同为3米，保留未配置时1.5米的原始搜索默认值。
  map->getCostmap()->resizeMap(60,60,.05,start[0]-1.5,start[1]-1.5);std::fill_n(map->getCostmap()->getCharMap(),3600,0);
  path.header.frame_id="map";for(const auto&p:j.at("path"))path.poses.push_back(at(p[0],p[1]));
  node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",rclcpp::NodeOptions().arguments({"--ros-args","--params-file",RPP_CONFIG}).automatically_declare_parameters_from_overrides(true));
  rpp.configure(node,"FollowPath",map->getTfBuffer(),map);rpp.activate();rpp.setPlan(path);
 }
 void TearDown()override{rpp.deactivate();rpp.cleanup();map->cleanup();}
};
TEST_F(RppRouteOrder, ConfiguredControllerKeepsUnreachedViaBeforeNearbyReturnLeg){
 const auto current=at(start[0],start[1],start[2]);const auto&target=path.poses.at(via).pose.position;
 ASSERT_GT(std::hypot(current.pose.position.x-target.x,current.pose.position.y-target.y),.15);
 const auto removed=path.poses.size()-rpp.remaining(current);
 EXPECT_LE(removed,via)<<"尚未到达首个目标，不能因返程路径更近而跳过它";
}
TEST_F(RppRouteOrder, ForwardMotionStillPrunesPassedPointsAndEventuallyPassesVia){
 size_t removed=0;
 for(size_t i=0;i<=via+3;++i){
  const auto&p=path.poses[i].pose.position;const auto next=path.poses.size()-rpp.remaining(at(p.x,p.y));
  EXPECT_GE(next,removed);EXPECT_LE(next,i)<<"不能裁掉当前车位之后尚未经过的路径";removed=next;
 }
 EXPECT_GT(removed,via)<<"搜索范围缩短后仍须正常经过首个目标，不能冻结裁剪";
}
}  // namespace
