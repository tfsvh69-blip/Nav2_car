// 固定目标录制输入经过真实RPP重算；不启动驱动，不向任何速度话题发布命令。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rcl/time.h>
#include <nav2_controller/plugins/position_goal_checker.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_controller/plugins/simple_progress_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_velocity_smoother/velocity_smoother.hpp>
#include "carcar_mapping_autonomy/subcell_smac_planner.hpp"

namespace {
using Json=nlohmann::json;
using Twist=geometry_msgs::msg::Twist;
namespace ca=carcar_mapping_autonomy;

std::string environment(const char * name,const std::string & fallback={}) {
  const auto * value=std::getenv(name);return value?value:fallback;
}
int direction(double value) {return value>.03?1:(value<-.03?-1:0);}
bool rotating(const Twist & value) {
  return std::abs(value.linear.x)<1e-6&&direction(value.angular.z)!=0;
}
Twist twist(const Json & value) {
  Twist result;result.linear.x=value.at("x");result.linear.y=value.at("y");
  result.angular.z=value.at("theta");return result;
}
geometry_msgs::msg::PoseStamped stamped(const Json & value,int64_t time) {
  geometry_msgs::msg::PoseStamped result;result.header.frame_id="map";
  result.header.stamp=rclcpp::Time(time);
  result.pose.position.x=value.at("x");result.pose.position.y=value.at("y");
  const double yaw=value.at("yaw");
  result.pose.orientation.z=std::sin(yaw/2);result.pose.orientation.w=std::cos(yaw/2);
  return result;
}
nav_msgs::msg::Path path(const Json & value) {
  nav_msgs::msg::Path result;result.header.frame_id=value.at("frame_id");
  result.header.stamp=rclcpp::Time(value.at("stamp_ns").get<int64_t>());
  for(const auto & entry:value.at("poses")) {
    geometry_msgs::msg::PoseStamped pose;pose.header=result.header;
    pose.pose.position.x=entry.at("x");pose.pose.position.y=entry.at("y");pose.pose.position.z=entry.at("z");
    pose.pose.orientation.x=entry.at("qx");pose.pose.orientation.y=entry.at("qy");
    pose.pose.orientation.z=entry.at("qz");pose.pose.orientation.w=entry.at("qw");
    result.poses.push_back(pose);
  }
  return result;
}

class RecordedRpp:public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
  Json lookahead(const geometry_msgs::msg::PoseStamped & pose,const Twist & velocity) {
    const auto transformed=transformGlobalPlan(pose);
    const auto distance=getLookAheadDistance(velocity);
    const auto carrot=getLookAheadPoint(distance,transformed);
    return {{"x",carrot.pose.position.x},{"y",carrot.pose.position.y},
      {"angle_rad",std::atan2(carrot.pose.position.y,carrot.pose.position.x)},
      {"lookahead_m",distance},{"transformed_poses",transformed.poses.size()}};
  }
};

class ManualSmoother:public nav2_velocity_smoother::VelocitySmoother {
public:
  explicit ManualSmoother(const rclcpp::NodeOptions & options):VelocitySmoother(options){}
  void start(const Twist & initial) {
    configure();if(get_current_state().label()!="inactive")throw std::runtime_error("平滑器配置失败");
    activate();if(get_current_state().label()!="active")throw std::runtime_error("平滑器激活失败");
    timer_->cancel();last_cmd_=initial;
  }
  Twist step(const Twist & request) {
    inputCommandCallback(std::make_shared<Twist>(request));smootherTimer();return last_cmd_;
  }
  void stop(){deactivate();cleanup();}
};

class ManualProgressChecker:public nav2_controller::SimpleProgressChecker {
public:
  void start(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) {
    initialize(node,"progress_checker");
    // 仅替换测试时钟；真实check及达到原5厘米后重置基线的逻辑保持。
    clock_=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    if(rcl_enable_ros_time_override(clock_->get_clock_handle())!=RCL_RET_OK)
      throw std::runtime_error("进展检查器测试时钟启用失败");
  }
  bool at(geometry_msgs::msg::PoseStamped & pose,double elapsed) {
    if(rcl_set_ros_time_override(clock_->get_clock_handle(),int64_t((elapsed+1.)*1e9))!=RCL_RET_OK)
      throw std::runtime_error("进展检查器测试时钟推进失败");
    return check(pose);
  }
  double radius()const{return radius_;}
  double allowance()const{return time_allowance_.seconds();}
};

Json parameter_override(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) {
  if(environment("CARCAR_POINT_PARAMETER").empty())return nullptr;
  const auto name=environment("CARCAR_POINT_PARAMETER");const auto text=environment("CARCAR_POINT_VALUE");
  const auto value=Json::parse(text);rclcpp::Parameter parameter;
  if(value.is_boolean())parameter=rclcpp::Parameter(name,value.get<bool>());
  else parameter=rclcpp::Parameter(name,value.get<double>());
  const auto result=node->set_parameter(parameter);
  if(!result.successful)throw std::runtime_error(result.reason);
  return {{"name",name},{"value",value}};
}

class FrozenRecordedPlanner {
public:
  explicit FrozenRecordedPlanner(const std::string & file) {
    std::ifstream input(file);if(!input.good())throw std::runtime_error("原图夹具缺失："+file);
    const auto fixture=Json::parse(input);const auto & source=fixture.at("map");
    raw.header.frame_id="map";raw.info.width=source.at("info").at("width");
    raw.info.height=source.at("info").at("height");raw.info.resolution=source.at("info").at("resolution");
    raw.info.origin.position.x=source.at("info").at("origin").at("position").at("x");
    raw.info.origin.position.y=source.at("info").at("origin").at("position").at("y");
    raw.info.origin.orientation.w=1;raw.data=source.at("data").get<std::vector<int8_t>>();
    global=std::make_shared<nav2_costmap_2d::Costmap2DROS>("point_navigation_frozen_global");
    for(const auto & result:global->set_parameters({{"plugins",std::vector<std::string>{}},
        {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
        {"rolling_window",false},{"track_unknown_space",true},
        {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
        {"footprint_padding",0.0}}))if(!result.successful)throw std::runtime_error(result.reason);
    global->configure();global->getCostmap()->resizeMap(raw.info.width,raw.info.height,raw.info.resolution,
      raw.info.origin.position.x,raw.info.origin.position.y);
    const auto costs=fixture.at("global_costmap").at("data").get<std::vector<unsigned char>>();
    if(costs.size()!=raw.data.size())throw std::runtime_error("原图与录制全局代价图尺寸不符");
    std::copy(costs.begin(),costs.end(),global->getCostmap()->getCharMap());
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server",
      rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,
        "--params-file",MAPPING_CONFIG}).automatically_declare_parameters_from_overrides(true));
    planner.configure(node,"ExplorationGrid",global->getTfBuffer(),global);
    io=std::make_shared<rclcpp::Node>("point_navigation_frozen_map_input");
    publisher=io->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
    executor.add_node(node->get_node_base_interface());executor.add_node(io);
    raw.header.stamp=io->now();publisher->publish(raw);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
    while(std::chrono::steady_clock::now()<deadline){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  }
  ~FrozenRecordedPlanner() {
    planner.cleanup();global->cleanup();executor.remove_node(node->get_node_base_interface());executor.remove_node(io);
  }
  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & current,const geometry_msgs::msg::PoseStamped & target) {
    return planner.createPlan(current,target);
  }
  nav_msgs::msg::OccupancyGrid raw;
private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr io;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
  ca::SubcellSmacPlanner planner;
};

struct Pattern {
  size_t samples=0,rotation_samples=0,reversals=0,short_forward_bursts=0;
  int last_rotation_sign=0;double forward_distance=0;
  void accept(const Twist & value,double dt) {
    ++samples;
    if(rotating(value)) {
      ++rotation_samples;
      const int sign=direction(value.angular.z);
      if(last_rotation_sign&&sign!=last_rotation_sign)++reversals;
      last_rotation_sign=sign;
      if(forward_distance>1e-5&&forward_distance<.08)++short_forward_bursts;
      forward_distance=0;
    }else if(value.linear.x>1e-6)forward_distance+=value.linear.x*dt;
  }
  Json state()const {
    return {{"samples",samples},{"rotation_samples",rotation_samples},
      {"in_place_direction_reversals",reversals},{"short_forward_bursts",short_forward_bursts}};
  }
};

class PointNavigation:public testing::Test {
protected:
  static void SetUpTestSuite() {
    if(environment("ROS_DOMAIN_ID")!="206"||environment("ROS_LOCALHOST_ONLY")!="1")
      throw std::runtime_error("只允许域206、本机DDS的离线RPP回放");
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite(){rclcpp::shutdown();}
};

#ifndef CARCAR_POINT_FIXTURE_ONLY
TEST_F(PointNavigation, RecordedV27InputsExplainTheObservedSevenDirectionReversals) {
  const std::string fixture_path=environment("CARCAR_POINT_FIXTURE",POINT_NAV_FIXTURE);
  std::ifstream input(fixture_path);ASSERT_TRUE(input.good())<<fixture_path;
  const auto fixture=Json::parse(input);ASSERT_GE(fixture.at("samples").size(),20U);
  const auto & samples=fixture.at("samples");const auto & plans=fixture.at("plans");
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("point_navigation_replay_map");
  for(const auto & result:local->set_parameters({{"plugins",std::vector<std::string>{}},
      {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
      {"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
      {"footprint_padding",0.0}}))ASSERT_TRUE(result.successful)<<result.reason;
  local->configure();ASSERT_EQ(local->get_current_state().label(),"inactive");
  // 保持正式3米局部图的裁剪/搜索范围；零成本只隔离转向判据，不验证历史碰撞或代价减速幅度。
  local->getCostmap()->resizeMap(60,60,.05,-1.5,-1.5);
  std::fill_n(local->getCostmap()->getCharMap(),3600,0);
  auto options=rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,
    "--params-file",MAPPING_CONFIG,"--params-file",V27_RPP_CONFIG,
    "-r","received_global_plan:=/point_navigation_replay/received_global_plan",
    "-r","lookahead_point:=/point_navigation_replay/lookahead_point",
    "-r","lookahead_collision_arc:=/point_navigation_replay/lookahead_collision_arc"})
    .automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",options);
  ASSERT_TRUE(node->set_parameter({"controller_frequency",10.0}).successful);
  double desired_speed=.18;
  if(!fixture.at("autonomy_states").empty())
    desired_speed=fixture.at("autonomy_states").front().at("state").at("speed").at("confirmed_mps");
  ASSERT_TRUE(node->set_parameter({"FollowPath.desired_linear_vel",desired_speed}).successful);
  // 每次实验最多覆盖一个参数；保持原配置文件不变，并将该变量写入结果。
  const auto override=parameter_override(node);
  nav2_controller::PositionGoalChecker checker;checker.initialize(node,"exploration_goal_checker",local);
  RecordedRpp controller;controller.configure(node,"FollowPath",local->getTfBuffer(),local);controller.activate();
  Pattern computed_pattern,recorded_pattern;size_t mode_matches=0,rotation_sign_matches=0;
  size_t compared_rotations=0,plan_changes=0;int previous_plan=-1;double previous_time=0;
  const bool fixed_plan=environment("CARCAR_POINT_FIXED_PLAN")=="1";
  Json trace=Json::array();
  for(const auto & sample:samples) {
    const auto pose=stamped(sample.at("map_pose"),sample.at("time_ns"));
    local->getCostmap()->updateOrigin(pose.pose.position.x-1.5,pose.pose.position.y-1.5);
    std::fill_n(local->getCostmap()->getCharMap(),3600,0);
    geometry_msgs::msg::TransformStamped transform;transform.header=pose.header;
    transform.child_frame_id="base_footprint";transform.transform.translation.x=pose.pose.position.x;
    transform.transform.translation.y=pose.pose.position.y;transform.transform.rotation=pose.pose.orientation;
    ASSERT_TRUE(local->getTfBuffer()->setTransform(transform,"recorded_point_navigation",true));
    const int plan_index=fixed_plan?samples.front().at("plan_index").get<int>():sample.at("plan_index").get<int>();ASSERT_GE(plan_index,0);
    ASSERT_LT(static_cast<size_t>(plan_index),plans.size());
    if(plan_index!=previous_plan){controller.setPlan(path(plans.at(plan_index)));previous_plan=plan_index;++plan_changes;}
    const auto velocity=twist(sample.at("velocity"));const auto expected=twist(sample.at("expected_raw"));
    const auto carrot=controller.lookahead(pose,velocity);
    const auto command=controller.computeVelocityCommands(pose,velocity,&checker).twist;
    const double now=sample.at("t_sec");const double dt=previous_time?now-previous_time:.1;previous_time=now;
    computed_pattern.accept(command,dt);recorded_pattern.accept(expected,dt);
    const bool same_mode=rotating(command)==rotating(expected);if(same_mode)++mode_matches;
    if(rotating(expected)) {
      ++compared_rotations;
      if(rotating(command)&&direction(command.angular.z)==direction(expected.angular.z))++rotation_sign_matches;
    }
    trace.push_back({{"t_sec",now},{"plan_index",plan_index},{"map_pose",sample.at("map_pose")},
      {"measured_velocity",sample.at("velocity")},{"carrot",carrot},
      {"computed",{{"x",command.linear.x},{"theta",command.angular.z}}},
      {"recorded_raw",sample.at("expected_raw")},{"same_turn_mode",same_mode}});
  }
  const Json summary={{"fixture",fixture_path},{"real_rpp",true},{"hardware",false},
    {"recorded_input_replay",true},{"closed_loop",false},{"zero_cost_local_map",true},{"pinned_v27_config",true},
    {"cost_and_collision_equivalence",false},{"single_parameter_override",override},
    {"fixed_recorded_plan",fixed_plan},
    {"desired_linear_vel",desired_speed},{"plan_changes",plan_changes},
    {"computed",computed_pattern.state()},{"recorded",recorded_pattern.state()},
    {"same_turn_mode_fraction",double(mode_matches)/samples.size()},
    {"recorded_rotation_sign_fraction",compared_rotations?double(rotation_sign_matches)/compared_rotations:1.0},
    {"alignment_notes",fixture.at("alignment_notes")},{"trace",trace}};
  const auto output_path=environment("CARCAR_POINT_TRACE","point_navigation_trace.json");
  std::ofstream(output_path)<<summary.dump(2)<<'\n';
  auto brief=summary;brief.erase("trace");brief.erase("alignment_notes");
  std::cout<<"POINT_NAVIGATION_REPLAY "<<brief.dump()<<std::endl;
  controller.deactivate();controller.cleanup();local->cleanup();
  // 固定历史位姿只锁定诊断来源；当前参数是否修复，必须由后面的闭环运动验收。
  if(override.is_null()&&!fixed_plan) {
    EXPECT_EQ(computed_pattern.reversals,7U);
    EXPECT_EQ(computed_pattern.short_forward_bursts,3U);
    EXPECT_EQ(mode_matches,samples.size());EXPECT_EQ(rotation_sign_matches,compared_rotations);
  }
}

TEST_F(PointNavigation, ClosedLoopRecordedPathReachesGoalWithoutRepeatedInPlaceReversals) {
  const auto fixture_path=environment("CARCAR_POINT_FIXTURE",POINT_NAV_FIXTURE);
  std::ifstream input(fixture_path);ASSERT_TRUE(input.good())<<fixture_path;const auto fixture=Json::parse(input);
  const auto & initial=fixture.at("samples").front();
  auto source_path=path(fixture.at("plans").at(initial.at("plan_index").get<size_t>()));
  ASSERT_GT(source_path.poses.size(),2U);const auto target=source_path.poses.back();
  const bool replan=environment("CARCAR_POINT_REPLAN")=="1";
  std::unique_ptr<FrozenRecordedPlanner> planner;
  if(replan)planner=std::make_unique<FrozenRecordedPlanner>(environment("CARCAR_POINT_MAP_FIXTURE",POINT_NAV_MAP_FIXTURE));
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("point_navigation_closed_loop_map");
  for(const auto & result:local->set_parameters({{"plugins",std::vector<std::string>{}},
      {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
      {"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
      {"footprint_padding",0.0}}))ASSERT_TRUE(result.successful)<<result.reason;
  local->configure();ASSERT_EQ(local->get_current_state().label(),"inactive");
  local->getCostmap()->resizeMap(60,60,.05,-1.5,-1.5);std::fill_n(local->getCostmap()->getCharMap(),3600,0);
  auto options=rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,
    "--params-file",MAPPING_CONFIG,"--params-file",RPP_CONFIG,
    "-r","received_global_plan:=/point_navigation_closed_loop/received_global_plan",
    "-r","lookahead_point:=/point_navigation_closed_loop/lookahead_point",
    "-r","lookahead_collision_arc:=/point_navigation_closed_loop/lookahead_collision_arc"})
    .automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",options);
  const double desired=fixture.at("autonomy_states").front().at("state").at("speed").at("confirmed_mps");
  ASSERT_TRUE(node->set_parameter({"FollowPath.desired_linear_vel",desired}).successful);
  const auto override=parameter_override(node);
  nav2_controller::PositionGoalChecker checker;checker.initialize(node,"exploration_goal_checker",local);
  RecordedRpp controller;controller.configure(node,"FollowPath",local->getTfBuffer(),local);controller.activate();
  controller.setPlan(source_path);
  auto smoother=std::make_shared<ManualSmoother>(rclcpp::NodeOptions().arguments({"--ros-args",
    "--params-file",NAV_BASE_CONFIG,"--params-file",MAPPING_CONFIG,
    "-r","cmd_vel:=/point_navigation_closed_loop/smoother_input",
    "-r","cmd_vel_smoothed:=/point_navigation_closed_loop/smoother_output"}));
  Twist actual=twist(initial.at("velocity")),requested;const Twist initial_velocity=actual;smoother->start(actual);
  auto current=initial.at("map_pose");const auto initial_pose=current;
  const size_t delay_steps=std::stoul(environment("CARCAR_POINT_FEEDBACK_DELAY_STEPS","0"));
  ASSERT_LE(delay_steps,10U)<<"反馈延迟须来自可核查数据，不允许任意扩大模型追求失败";
  std::deque<Twist> feedback(delay_steps+1,actual);
  const size_t actuation_delay=std::stoul(environment("CARCAR_POINT_ACTUATION_DELAY_STEPS","0"));
  ASSERT_LE(actuation_delay,5U)<<"执行延迟仅作0～0.25秒有界敏感性检查";
  std::deque<Twist> pending_commands(actuation_delay,actual);
  Pattern pattern;bool reached=false,trajectory_safe=true,kinematic_limits=true;size_t stopped_steps=0,planning_calls=0;
  size_t physical_reversals=0;int physical_last_sign=0;
  double travelled=0,low_translation_turn=0,total_turn=0;Json trace=Json::array();
  constexpr double dt=.05;size_t steps=0;
  for(;steps<1200;++steps) {
    const auto pose=stamped(current,initial.at("time_ns").get<int64_t>()+int64_t(steps)*50000000);
    local->getCostmap()->updateOrigin(pose.pose.position.x-1.5,pose.pose.position.y-1.5);
    std::fill_n(local->getCostmap()->getCharMap(),3600,0);
    geometry_msgs::msg::TransformStamped tf;tf.header=pose.header;tf.child_frame_id="base_footprint";
    tf.transform.translation.x=pose.pose.position.x;tf.transform.translation.y=pose.pose.position.y;
    tf.transform.rotation=pose.pose.orientation;ASSERT_TRUE(local->getTfBuffer()->setTransform(tf,"point_navigation_virtual_wheel",true));
    const Twist measured=feedback.front();
    if(checker.isGoalReached(pose.pose,target.pose,measured))reached=true;
    Json carrot=nullptr;
    if(steps%2==0) {
      if(reached)requested=Twist();
      else {
        if(planner&&steps%20==0){controller.setPlan(planner->plan(pose,target));++planning_calls;}
        carrot=controller.lookahead(pose,measured);requested=controller.computeVelocityCommands(pose,measured,&checker).twist;
      }
      pattern.accept(requested,.1);
    }
    const auto smoothed=smoother->step(requested);
    pending_commands.push_back(smoothed);actual=pending_commands.front();pending_commands.pop_front();
    if(!std::isfinite(actual.linear.x)||!std::isfinite(actual.angular.z)||actual.linear.x<0||
        actual.linear.x>.18+1e-9||std::abs(actual.angular.z)>.36+1e-9)kinematic_limits=false;
    // 平滑器后另以有界纯延迟表示执行响应；不把平滑器自身的加减速相位重复计入机械延迟。
    const auto previous=current;
    const double yaw=current.at("yaw"),distance=actual.linear.x*dt;
    current["x"]=current.at("x").get<double>()+distance*std::cos(yaw+actual.angular.z*dt/2);
    current["y"]=current.at("y").get<double>()+distance*std::sin(yaw+actual.angular.z*dt/2);
    current["yaw"]=std::remainder(yaw+actual.angular.z*dt,2*std::acos(-1.));travelled+=std::abs(distance);
    total_turn+=std::abs(actual.angular.z)*dt;
    if(std::abs(actual.linear.x)<.02&&direction(actual.angular.z)) {
      const int sign=direction(actual.angular.z);
      if(physical_last_sign&&sign!=physical_last_sign)++physical_reversals;
      physical_last_sign=sign;low_translation_turn+=std::abs(actual.angular.z)*dt;
    }
    if(planner&&!ca::free_connector(planner->raw,
        {previous.at("x"),previous.at("y"),previous.at("yaw")},
        {current.at("x"),current.at("y"),current.at("yaw")}))trajectory_safe=false;
    feedback.push_back(actual);feedback.pop_front();
    if(reached&&std::abs(actual.linear.x)+std::abs(actual.angular.z)<1e-8)++stopped_steps;else stopped_steps=0;
    trace.push_back({{"t_sec",steps*dt},{"pose",current},{"carrot",carrot},
      {"requested",{requested.linear.x,requested.angular.z}},{"smoothed",{smoothed.linear.x,smoothed.angular.z}},
      {"executed",{actual.linear.x,actual.angular.z}},
      {"measured",{measured.linear.x,measured.angular.z}},{"reached",reached}});
    if(stopped_steps>=10||!trajectory_safe||!kinematic_limits)break;
  }
  const double remaining=std::hypot(current.at("x").get<double>()-target.pose.position.x,
    current.at("y").get<double>()-target.pose.position.y);
  const Json summary={{"fixture",fixture_path},{"real_rpp",true},{"real_velocity_smoother",true},
    {"hardware",false},{"closed_loop",true},{"fixed_recorded_plan",!replan},{"zero_cost_local_map",true},
    {"real_smac",replan},{"frozen_recorded_global_costmap",replan},{"planning_calls",planning_calls},
    {"raw_map_clearance_m",replan?Json(.22):Json(nullptr)},{"trajectory_safe",trajectory_safe},
    {"kinematic_limits",kinematic_limits},
    {"recorded_input_replay",false},{"single_parameter_override",override},{"feedback_delay_steps",delay_steps},
    {"feedback_delay_seconds",delay_steps*dt},{"plant_model","理想指令运动积分，非实物动力学"},
    {"actuation_delay_steps",actuation_delay},{"actuation_delay_seconds",actuation_delay*dt},
    {"physical_low_translation_threshold_mps",.02},{"physical_direction_reversals",physical_reversals},
    {"physical_low_translation_turn_rad",low_translation_turn},{"physical_total_turn_rad",total_turn},
    {"initial_pose",initial_pose},{"initial_velocity",{initial_velocity.linear.x,initial_velocity.angular.z}},
    {"computed",pattern.state()},{"steps",steps+1},{"simulated_seconds",(steps+1)*dt},
    {"reached",reached},{"stopped_steps",stopped_steps},{"travelled_m",travelled},
    {"remaining_m",remaining},{"trace",trace}};
  const auto output_path=environment("CARCAR_POINT_TRACE","point_navigation_closed_loop_trace.json");
  std::ofstream(output_path)<<summary.dump(2)<<'\n';auto brief=summary;brief.erase("trace");
  std::cout<<"POINT_NAVIGATION_CLOSED_LOOP "<<brief.dump()<<std::endl;
  smoother->stop();controller.deactivate();controller.cleanup();local->cleanup();
  EXPECT_TRUE(reached)<<"固定录制路径未在60秒软件预算内到达";
  EXPECT_TRUE(trajectory_safe)<<"闭环轨迹越过录制原图0.22米净空";
  EXPECT_TRUE(kinematic_limits)<<"软件执行速度非有限值或超过原速度包络";
  EXPECT_GE(stopped_steps,10U);EXPECT_LE(remaining,.15);
  EXPECT_LE(physical_reversals,1U)<<"低于0.02m/s的实际软件运动仍反复换向";
  EXPECT_LE(pattern.reversals,1U)<<"闭环中出现重复原地反向；详见"<<output_path;
  EXPECT_LE(pattern.short_forward_bursts,1U)<<"闭环中出现多次不足8厘米的短进／原地转向交替";
}

TEST_F(PointNavigation, ShortBackwardReturnMeetsOriginalProgressAndYawRequirements) {
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("point_navigation_short_return_map");
  for(const auto & result:local->set_parameters({{"plugins",std::vector<std::string>{}},
      {"filters",std::vector<std::string>{}},{"global_frame","map"},{"robot_base_frame","base_footprint"},
      {"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},
      {"footprint_padding",0.0}}))ASSERT_TRUE(result.successful)<<result.reason;
  local->configure();local->getCostmap()->resizeMap(60,60,.05,-1.5,-1.5);
  std::fill_n(local->getCostmap()->getCharMap(),3600,0);
  auto options=rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,
    "--params-file",MAPPING_CONFIG,"--params-file",RPP_CONFIG,
    "-r","received_global_plan:=/point_navigation_short_return/received_global_plan",
    "-r","lookahead_point:=/point_navigation_short_return/lookahead_point",
    "-r","lookahead_collision_arc:=/point_navigation_short_return/lookahead_collision_arc"})
    .automatically_declare_parameters_from_overrides(true);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",options);
  const auto override=parameter_override(node);
  nav2_controller::SimpleGoalChecker checker;checker.initialize(node,"general_goal_checker",local);
  ManualProgressChecker progress;progress.start(node);
  ASSERT_NEAR(progress.radius(),.05,1e-12);ASSERT_NEAR(progress.allowance(),30.,1e-12);
  RecordedRpp controller;controller.configure(node,"FollowPath",local->getTfBuffer(),local);controller.activate();
  Json current={{"x",0.},{"y",0.},{"yaw",0.}};
  const int64_t start_time=1000000000;
  const auto target=stamped(Json{{"x",-.19},{"y",0.},{"yaw",0.}},start_time);
  nav_msgs::msg::Path source;source.header=target.header;
  for(const auto x:{0.,-.05,-.10,-.15,-.19}) {
    auto waypoint=stamped(Json{{"x",x},{"y",0.},{"yaw",x==-.19?0.:std::acos(-1.)}},start_time);
    source.poses.push_back(waypoint);
  }
  controller.setPlan(source);
  auto smoother=std::make_shared<ManualSmoother>(rclcpp::NodeOptions().arguments({"--ros-args",
    "--params-file",NAV_BASE_CONFIG,"--params-file",MAPPING_CONFIG,
    "-r","cmd_vel:=/point_navigation_short_return/smoother_input",
    "-r","cmd_vel_smoothed:=/point_navigation_short_return/smoother_output"}));
  Twist actual,requested;smoother->start(actual);
  const size_t delay=std::stoul(environment("CARCAR_POINT_ACTUATION_DELAY_STEPS","0"));
  ASSERT_LE(delay,5U);std::deque<Twist> commands(delay,actual);
  bool reached=false,progress_failed=false;size_t stopped_steps=0,steps=0;
  double first_five_cm=-1,progress_failure_at=-1,reached_at=-1,max_displacement=0;
  double travelled=0;Json trace=Json::array();constexpr double dt=.05;
  for(;steps<1600;++steps) {
    auto pose=stamped(current,start_time+int64_t(steps)*50000000);
    geometry_msgs::msg::TransformStamped tf;tf.header=pose.header;tf.child_frame_id="base_footprint";
    tf.transform.translation.x=pose.pose.position.x;tf.transform.translation.y=pose.pose.position.y;
    tf.transform.rotation=pose.pose.orientation;ASSERT_TRUE(local->getTfBuffer()->setTransform(tf,"short_return_virtual_wheel",true));
    if(steps%2==0) {
      if(checker.isGoalReached(pose.pose,target.pose,actual)) {
        if(!reached)reached_at=steps*dt;
        reached=true;requested=Twist();
      }else if(!progress.at(pose,steps*dt)) {
        progress_failed=true;progress_failure_at=steps*dt;requested=Twist();
      }else requested=controller.computeVelocityCommands(pose,actual,&checker).twist;
    }
    if(progress_failed)break;
    const auto smoothed=smoother->step(requested);commands.push_back(smoothed);actual=commands.front();commands.pop_front();
    const double yaw=current.at("yaw"),distance=actual.linear.x*dt;
    current["x"]=current.at("x").get<double>()+distance*std::cos(yaw+actual.angular.z*dt/2);
    current["y"]=current.at("y").get<double>()+distance*std::sin(yaw+actual.angular.z*dt/2);
    current["yaw"]=std::remainder(yaw+actual.angular.z*dt,2*std::acos(-1.));travelled+=std::abs(distance);
    const double displacement=std::hypot(current.at("x").get<double>(),current.at("y").get<double>());
    max_displacement=std::max(max_displacement,displacement);
    if(first_five_cm<0&&displacement>.05)first_five_cm=(steps+1)*dt;
    if(reached&&std::abs(actual.linear.x)+std::abs(actual.angular.z)<1e-8)++stopped_steps;else stopped_steps=0;
    trace.push_back({{"t_sec",steps*dt},{"pose",current},
      {"requested",{requested.linear.x,requested.angular.z}},{"smoothed",{smoothed.linear.x,smoothed.angular.z}},
      {"executed",{actual.linear.x,actual.angular.z}},{"reached",reached}});
    if(stopped_steps>=10)break;
  }
  const double distance=std::hypot(current.at("x").get<double>()+.19,current.at("y").get<double>());
  const double yaw_error=std::abs(std::remainder(current.at("yaw").get<double>(),2*std::acos(-1.)));
  const Json summary={{"hardware",false},{"closed_loop",true},{"real_rpp",true},{"real_velocity_smoother",true},
    {"real_simple_goal_checker",true},{"real_simple_progress_checker",true},{"manual_ros_progress_clock",true},
    {"progress_allowance_s",progress.allowance()},{"progress_required_m",progress.radius()},
    {"desired_linear_vel",node->get_parameter("FollowPath.desired_linear_vel").as_double()},
    {"rotate_to_heading_angular_vel",node->get_parameter("FollowPath.rotate_to_heading_angular_vel").as_double()},
    {"single_parameter_override",override},{"actuation_delay_seconds",delay*dt},
    {"start",{0,0,0}},{"target",{-.19,0,0}},{"final_pose",current},
    {"progress_failed",progress_failed},{"progress_failure_at_s",progress_failure_at},
    {"first_five_cm_at_s",first_five_cm},{"max_displacement_m",max_displacement},
    {"reached",reached},{"reached_at_s",reached_at},{"simulated_seconds",steps*dt},
    {"stopped_steps",stopped_steps},{"travelled_m",travelled},{"target_distance_m",distance},{"yaw_error_rad",yaw_error},
    {"trace",trace}};
  const auto output=environment("CARCAR_POINT_TRACE","point_navigation_short_return_trace.json");
  std::ofstream(output)<<summary.dump(2)<<'\n';auto brief=summary;brief.erase("trace");
  std::cout<<"POINT_NAVIGATION_SHORT_RETURN "<<brief.dump()<<std::endl;
  smoother->stop();controller.deactivate();controller.cleanup();local->cleanup();
  EXPECT_FALSE(progress_failed)<<"真实进展检查在"<<progress_failure_at<<"秒拒绝，首次5厘米在"<<first_five_cm<<"秒；详见"<<output;
  EXPECT_TRUE(reached);EXPECT_GE(stopped_steps,10U);EXPECT_LE(distance,.15);EXPECT_LE(yaw_error,.175);
}
#endif
}  // namespace
