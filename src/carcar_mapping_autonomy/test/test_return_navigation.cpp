// 仅软件回归：合成地图、真实Nav2插件和虚拟轮式运动，不启动驱动、不发布速度命令。
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include "carcar_mapping_autonomy/return_selector.hpp"

namespace {
using carcar_mapping_autonomy::Pose;
using carcar_mapping_autonomy::ReturnSelector;
using Json = nlohmann::json;

geometry_msgs::msg::PoseStamped stamped(const Pose & p) {
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id="map";
  result.pose.position.x=p.x;result.pose.position.y=p.y;
  result.pose.orientation.z=std::sin(p.yaw/2);result.pose.orientation.w=std::cos(p.yaw/2);
  return result;
}

rclcpp::NodeOptions options() {
  return rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,
    "--params-file",MAPPING_CONFIG,"--params-file",CONTROLLER_CONFIG})
    .automatically_declare_parameters_from_overrides(true);
}

ReturnSelector::Job scene() {
  ReturnSelector::Job job;
  job.operation=1;job.generation=2;job.revision=3;job.request_id="real-nav2-offline";
  job.current={1.8,.3,2.7};job.task.anchored=true;job.task.origin={0,0,0};
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id="map";map->info.width=map->info.height=100;map->info.resolution=.05;
  map->info.origin.position.x=map->info.origin.position.y=-2.5;map->info.origin.orientation.w=1;
  map->data.assign(10000,0);job.map=map;return job;
}

// Smac使用与选点一致的保守禁行栅格；RPP使用原地图上的矩形足迹碰撞检查。
// 此处不替代生产代价图图层、激光动态障碍、行为树或监督器集成验收。
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap(
  const nav_msgs::msg::OccupancyGrid & source,const std::string & name) {
  auto result=std::make_shared<nav2_costmap_2d::Costmap2DROS>(name);
  for(const auto & parameter:result->set_parameters({
      {"global_frame","map"},{"robot_base_frame","base_footprint"},
      {"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
      {"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
      {"footprint_padding",0.0}})) {
    if(!parameter.successful)throw std::runtime_error(parameter.reason);
  }
  result->configure();
  if(result->get_current_state().label()!="inactive")throw std::runtime_error("costmap配置未完成");
  auto * grid=result->getCostmap();
  grid->resizeMap(source.info.width,source.info.height,source.info.resolution,
    source.info.origin.position.x,source.info.origin.position.y);
  for(size_t index=0;index<source.data.size();++index) {
    const auto value=source.data[index];
    grid->getCharMap()[index]=value<0?255:(value>=50?254:0);
  }
  return result;
}

class RealNavigation {
public:
  explicit RealNavigation(const ReturnSelector::Job & job) {
    auto mask=carcar_mapping_autonomy::planning_keepout_mask(*job.map);
    if(!mask)throw std::runtime_error("规划安全掩码不可用");
    carcar_mapping_autonomy::allow_planning_departure(*job.map,job.current,*mask);
    global=costmap(*mask,"return_test_global");local=costmap(*job.map,"return_test_local");
    planner_node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server",options());
    controller_node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",options());
    planner.configure(planner_node,"GridBased",global->getTfBuffer(),global);
    checker.initialize(controller_node,"general_goal_checker",local);
    controller.configure(controller_node,"FollowPath",local->getTfBuffer(),local);
    controller.activate();
  }
  ~RealNavigation() {
    controller.deactivate();controller.cleanup();planner.cleanup();local->cleanup();global->cleanup();
  }
  nav_msgs::msg::Path plan(const Pose & current,const Pose & target) {
    return planner.createPlan(stamped(current),stamped(target));
  }
  bool can_plan(const Pose & current,const Pose & target) {
    last_plan_error.clear();
    try{return !plan(current,target).poses.empty();}
    catch(const std::exception & error){last_plan_error=error.what();return false;}
  }
  void update_transform(const Pose & pose) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="map";transform.child_frame_id="base_footprint";
    transform.header.stamp=controller_node->now();
    transform.transform.translation.x=pose.x;transform.transform.translation.y=pose.y;
    transform.transform.rotation=stamped(pose).pose.orientation;
    local->getTfBuffer()->setTransform(transform,"virtual_wheel_odometry",false);
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global,local;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> planner_node,controller_node;
  nav2_smac_planner::SmacPlanner2D planner;
  nav2_controller::SimpleGoalChecker checker;
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController controller;
  std::string last_plan_error;
};

class ReturnNavigation : public testing::Test {
protected:
  static void SetUpTestSuite() {
    // 域与本机限制由测试入口和CTest双重约束，误用生产环境立即拒绝。
    if(std::string(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"")!="231"||
       std::string(std::getenv("ROS_LOCALHOST_ONLY")?std::getenv("ROS_LOCALHOST_ONLY"):"")!="1")
      throw std::runtime_error("仅允许ROS_DOMAIN_ID=231和ROS_LOCALHOST_ONLY=1的软件测试");
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite(){rclcpp::shutdown();}
};

TEST_F(ReturnNavigation, OccupiedOriginSelectsNearbyAndRealPluginsDriveToSafeStoppedPose) {
  auto job=scene();auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*job.map);
  map->data[50*100+50]=100;job.map=map;
  ASSERT_FALSE(carcar_mapping_autonomy::free_footprint(*map,job.task.origin));
  const auto selection=ReturnSelector::Search::select(job);
  ASSERT_FALSE(selection.candidates.empty())<<selection.reason;
  RealNavigation navigation(job);
  // 原实现只试受阻起点；真实Smac仍应拒绝，不能通过放大容差伪造原点成功。
  EXPECT_FALSE(navigation.can_plan(job.current,job.task.origin));
  const auto target=selection.candidates.front();
  const auto path=navigation.plan(job.current,target);
  ASSERT_GT(path.poses.size(),2U);
  ASSERT_LE(std::hypot(path.poses.back().pose.position.x-target.x,
    path.poses.back().pose.position.y-target.y),.15);
  // 控制器接收本次Smac的同一条路径，不能用另一条历史路径替代候选目标验证。
  navigation.controller.setPlan(path);
  Pose current=job.current;
  geometry_msgs::msg::Twist measured,command;
  bool reached=false;int moving_steps=0,stationary_steps=0;
  double travelled=0;int safe_footprint_steps=0;
  constexpr double dt=.05;
  Json trace=Json::array();
  for(int step=0;step<4000;++step) {
    navigation.update_transform(current);
    const auto pose=stamped(current);
    if(navigation.checker.isGoalReached(pose.pose,stamped(target).pose,measured))reached=true;
    // 与controller_server相同：到点后停止计算运动命令，发送零；虚拟车立即执行命令。
    command=reached?geometry_msgs::msg::Twist():
      navigation.controller.computeVelocityCommands(pose,measured,&navigation.checker).twist;
    ASSERT_TRUE(std::isfinite(command.linear.x));ASSERT_TRUE(std::isfinite(command.angular.z));
    ASSERT_GE(command.linear.x,-1e-9);
    ASSERT_LE(command.linear.x,.1001);ASSERT_LE(std::abs(command.angular.z),.3601);
    if(command.linear.x>1e-5)++moving_steps;
    const auto previous=current;
    current.x+=command.linear.x*std::cos(current.yaw)*dt;
    current.y+=command.linear.x*std::sin(current.yaw)*dt;
    current.yaw=std::remainder(current.yaw+command.angular.z*dt,2*M_PI);
    travelled+=std::hypot(current.x-previous.x,current.y-previous.y);measured=command;
    ASSERT_TRUE(carcar_mapping_autonomy::free_footprint(*map,current))<<"连续运动中的0.22米足迹越界";
    ++safe_footprint_steps;
    if(step%20==0)trace.push_back({{"step",step},{"pose",{current.x,current.y,current.yaw}},
      {"command",{command.linear.x,command.angular.z}}});
    if(reached&&std::abs(measured.linear.x)+std::abs(measured.angular.z)<1e-9)++stationary_steps;
    else stationary_steps=0;
    if(stationary_steps>=10)break;
  }
  const double origin_distance=std::hypot(current.x-job.task.origin.x,current.y-job.task.origin.y);
  const double target_distance=std::hypot(current.x-target.x,current.y-target.y);
  EXPECT_TRUE(reached);EXPECT_GE(stationary_steps,10);EXPECT_GT(moving_steps,50);EXPECT_GT(travelled,1.0);
  EXPECT_LE(origin_distance,.8);EXPECT_LE(target_distance,.15);
  EXPECT_LE(std::abs(std::remainder(current.yaw-target.yaw,2*M_PI)),.175);
  EXPECT_TRUE(navigation.checker.isGoalReached(stamped(current).pose,stamped(target).pose,measured));
  EXPECT_DOUBLE_EQ(command.linear.x,0);EXPECT_DOUBLE_EQ(command.angular.z,0);
  std::cout<<"RETURN_NAVIGATION_REPLAY "<<Json{{"hardware",false},{"real_nav2_plugins",true},
    {"synthetic_map",true},{"closed_bag_replay",false},{"virtual_motion",true},
    {"real_controller_server",false},{"virtual_immediate_stop",true},{"goal_checker_reached",reached},
    {"origin",{job.task.origin.x,job.task.origin.y,job.task.origin.yaw}},
    {"selected_target",{target.x,target.y,target.yaw}},{"smac_path_poses",path.poses.size()},
    {"same_smac_path_to_rpp",true},{"last_pose",{current.x,current.y,current.yaw}},
    {"origin_distance_m",origin_distance},{"target_distance_m",target_distance},
    {"stationary_steps",stationary_steps},{"travelled_m",travelled},
    {"final_command",{command.linear.x,command.angular.z}},
    {"safe_footprint_steps",safe_footprint_steps},{"trace",trace}}.dump()<<std::endl;
}

TEST_F(ReturnNavigation, ClosedWallCannotProduceReachableCandidateOrRealSmacSuccess) {
  auto job=scene();job.task.origin={-.75,0,0};job.current={1,0,0};
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*job.map);
  for(int y=0;y<100;++y)map->data[y*100+50]=100;
  job.map=map;
  ASSERT_TRUE(carcar_mapping_autonomy::free_footprint(*map,job.task.origin));
  const auto selection=ReturnSelector::Search::select(job);
  EXPECT_TRUE(selection.candidates.empty());
  RealNavigation navigation(job);
  const bool reachable=navigation.can_plan(job.current,job.task.origin);
  EXPECT_FALSE(reachable);
  // 同一侧都没有目标时不向RPP提供假路径，更不能产生成功终态。
  std::cout<<"RETURN_NAVIGATION_REPLAY "<<Json{{"hardware",false},{"synthetic_map",true},
    {"scene","closed_wall"},{"candidates",selection.candidates.size()},{"smac_reachable",reachable},
    {"smac_error",navigation.last_plan_error},{"follow_path_called",false},{"success",false}}.dump()<<std::endl;
}

TEST_F(ReturnNavigation, UnknownOriginRegionHasNoCandidateAndRealSmacRejects) {
  auto job=scene();
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>(*job.map);
  for(int y=25;y<75;++y)for(int x=25;x<75;++x)map->data[y*100+x]=-1;
  job.map=map;job.current={1.8,0,0};
  ASSERT_TRUE(carcar_mapping_autonomy::free_footprint(*map,job.current));
  const auto selection=ReturnSelector::Search::select(job);
  EXPECT_TRUE(selection.candidates.empty());
  RealNavigation navigation(job);
  const bool reachable=navigation.can_plan(job.current,job.task.origin);
  EXPECT_FALSE(reachable);
  std::cout<<"RETURN_NAVIGATION_REPLAY "<<Json{{"hardware",false},{"synthetic_map",true},
    {"scene","unknown_origin_region"},{"candidates",selection.candidates.size()},
    {"smac_reachable",reachable},{"smac_error",navigation.last_plan_error},
    {"follow_path_called",false},{"success",false}}.dump()<<std::endl;
}
}  // namespace
