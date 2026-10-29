#include <gtest/gtest.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nlohmann/json.hpp>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "carcar_mapping_autonomy/map_geometry.hpp"
extern char **environ;
using Json=nlohmann::json;
using Nav=nav2_msgs::action::NavigateToPose;
using Plan=nav2_msgs::action::ComputePathToPose;
class SupervisorIntegration:public testing::Test {
protected:
  rclcpp::Node::SharedPtr n;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> fixed;
  pid_t pid=-1;Json state=Json::object();bool feed_scan=true,ack_cancel=true,feed_pose=true,feed_map=true,loop_active=false;
  double pose_x=0;uint64_t loop_generation=0;
  bool freeze_nav_pose=false;builtin_interfaces::msg::Time nav_pose_stamp;
  bool succeed_return=false,stall_plan=false,unknown_start=false,automatic_on_boot=false;
  bool delay_initial_odometry=false;double odom_feed_after=0,odom_x=0;
  int grid_width=100;double grid_origin=-2.5;bool block_route=false,observation_wall=false,block_distant=false,narrow_door=false;
  size_t nav_requests=0,plan_requests=0;std::vector<double> nav_targets,proxy_distances;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Plan>> stalled_plan;
  bool nav2_active=true;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lifecycle;
  double last_velocity=0;size_t nonzero=0;std::string task_id;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan,filtered;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_pose,global_pose;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slam,ready,command;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_velocity,collision;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr candidate_rejections;
  std::vector<std::string> rejected_candidates;
  double previous_output_time=-1,max_output_gap=0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input,output;
  rclcpp_action::Server<Nav>::SharedPtr nav_server;
  rclcpp_action::Server<Plan>::SharedPtr plan_server;
  rclcpp_action::Client<Nav>::SharedPtr proxy;
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr proxy_handle;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> active;
  rclcpp::TimerBase::SharedPtr timer;
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{
    n=std::make_shared<rclcpp::Node>("mapping_fixture");tf=std::make_unique<tf2_ros::TransformBroadcaster>(*n);fixed=std::make_unique<tf2_ros::StaticTransformBroadcaster>(*n);
    lifecycle=n->create_service<std_srvs::srv::Trigger>("/lifecycle_manager_navigation/is_active",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){response->success=nav2_active;});
    geometry_msgs::msg::TransformStamped s;s.header.frame_id="base_footprint";s.child_frame_id="laser_frame";s.transform.rotation.w=1;s.transform.translation.z=.217;fixed->sendTransform(s);
    scan=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());filtered=n->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered",rclcpp::SensorDataQoS());
    odom=n->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
    map=n->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
    pose=n->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",rclcpp::QoS(1).reliable().transient_local());
    local_pose=n->create_publisher<geometry_msgs::msg::PolygonStamped>("/local_costmap/published_footprint",1);
    global_pose=n->create_publisher<geometry_msgs::msg::PolygonStamped>("/global_costmap/published_footprint",1);
    slam=n->create_publisher<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local());
    ready=n->create_publisher<std_msgs::msg::String>("/mapping/session/state",1);command=n->create_publisher<std_msgs::msg::String>("/mapping/autonomy/request",10);
    nav_velocity=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/auto_cmd_vel",1);collision=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/collision_output",1);
    status=n->create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),[this](const std_msgs::msg::String&m){state=Json::parse(m.data);task_id=state.value("task_id","");});
    candidate_rejections=n->create_subscription<std_msgs::msg::String>("/mapping/explore_candidate_rejection",10,[this](const std_msgs::msg::String&m){rejected_candidates.push_back(m.data);});
    input=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/collision_input",1,[this](const geometry_msgs::msg::Twist&m){collision->publish(m);});
    output=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,[this](const geometry_msgs::msg::Twist&m){
      const double t=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      if(previous_output_time>0)max_output_gap=std::max(max_output_gap,t-previous_output_time);
      previous_output_time=t;
      last_velocity=m.linear.x;if(std::abs(m.linear.x)>1e-5)++nonzero;});
    nav_server=rclcpp_action::create_server<Nav>(n,"/navigate_to_pose",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){active=h;++nav_requests;nav_targets.push_back(h->get_goal()->pose.pose.position.x);});
    plan_server=rclcpp_action::create_server<Plan>(n,"/compute_path_to_pose",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){++plan_requests;if(stall_plan){stalled_plan=h;return;}auto result=std::make_shared<Plan::Result>();result->path.header.frame_id="map";geometry_msgs::msg::PoseStamped start;start.pose.orientation.w=1;result->path.poses={start,h->get_goal()->goal};h->succeed(result);});
    proxy=rclcpp_action::create_client<Nav>(n,"/mapping/explore_navigate_to_pose");
    if(delay_initial_odometry)odom_feed_after=n->now().seconds()+1.5;
    timer=n->create_wall_timer(std::chrono::milliseconds(50),[this]{feed();});
    char executable[]=AUTONOMY_EXE;
    char ros_args[]="--ros-args",parameter[]="-p";
    char mode_manual[]="control_mode:=manual",mode_automatic[]="control_mode:=automatic";
    char* args[]={executable,ros_args,parameter,automatic_on_boot?mode_automatic:mode_manual,nullptr};
    ASSERT_EQ(posix_spawn(&pid,executable,nullptr,nullptr,args,environ),0);
    ASSERT_TRUE(until([this]{return state.value("healthy",false)&&state.value("still",false);},8))<<state.dump();
  }
  void TearDown()override{
    if(pid>0){kill(pid,SIGINT);for(int i=0;i<50;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
    timer.reset();lifecycle.reset();n.reset();
    // 清除该fixture拥有的DDS发布者后，再进入下一用例。
    active.reset();stalled_plan.reset();proxy_handle.reset();proxy.reset();nav_server.reset();plan_server.reset();status.reset();candidate_rejections.reset();input.reset();output.reset();
    scan.reset();filtered.reset();odom.reset();map.reset();pose.reset();local_pose.reset();global_pose.reset();slam.reset();ready.reset();command.reset();nav_velocity.reset();collision.reset();tf.reset();fixed.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  template<class F>bool until(F f,double timeout){const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(n);if(f())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return f();}
  void feed(){
    const auto now=n->now();
    geometry_msgs::msg::TransformStamped t;t.header.stamp=now;t.header.frame_id="map";t.child_frame_id="odom";t.transform.rotation.w=1;tf->sendTransform(t);t.header.frame_id="odom";t.child_frame_id="base_footprint";t.transform.translation.x=odom_x;tf->sendTransform(t);
    sensor_msgs::msg::LaserScan s;s.header.stamp=now;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_increment=2*M_PI/360;s.angle_max=M_PI;s.range_min=.05;s.range_max=12;s.ranges.assign(360,2.0);if(feed_scan){scan->publish(s);filtered->publish(s);}
    nav_msgs::msg::Odometry o;o.header.stamp=now;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.pose.pose.orientation.w=1;o.pose.pose.position.x=odom_x;if(now.seconds()>=odom_feed_after)odom->publish(o);
    nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.header.stamp=now;m.info.resolution=.05;m.info.width=grid_width;m.info.height=100;m.info.origin.position.x=grid_origin;m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;m.data.assign(grid_width*100,0);if(unknown_start)m.data[50*grid_width+53]=-1;if(block_route)m.data[50*grid_width+58]=100;
    if(block_distant)m.data[80*grid_width+80]=100;
    if(observation_wall)for(int y=0;y<100;++y){m.data[y*grid_width+63]=100;for(int x=64;x<grid_width;++x)m.data[y*grid_width+x]=-1;}
    // 激光可穿过0.35米门缝，0.22米圆包络及规划余量不能通过。
    if(narrow_door)for(int y=0;y<100;++y)if(y<47||y>53)m.data[y*grid_width+63]=100;
    if(feed_map)map->publish(m);
    geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.frame_id="map";p.header.stamp=now;p.pose.pose.orientation.w=1;p.pose.pose.position.x=pose_x;if(feed_pose)pose->publish(p);
    if(!freeze_nav_pose)nav_pose_stamp=now;
    geometry_msgs::msg::PolygonStamped fp;fp.header.stamp=nav_pose_stamp;fp.polygon.points.resize(4);fp.header.frame_id="odom";local_pose->publish(fp);fp.header.frame_id="map";global_pose->publish(fp);
    std_msgs::msg::String j;j.data=Json{{"sequence",1},{"accepted",true},{"loop_generation",loop_generation},{"loop_active",loop_active}}.dump();slam->publish(j);j.data="{\"ready\":true}";ready->publish(j);
    geometry_msgs::msg::Twist v;v.linear.x=.06;nav_velocity->publish(v);
    if(active&&active->is_canceling()&&ack_cancel){active->canceled(std::make_shared<Nav::Result>());active.reset();}
    else if(active&&succeed_return&&std::abs(active->get_goal()->pose.pose.position.x)<.01){active->succeed(std::make_shared<Nav::Result>());active.reset();}
  }
  void request(const std::string&a,const std::string&id){std_msgs::msg::String m;m.data=Json{{"task_id",task_id},{"request_id",id},{"action",a}}.dump();command->publish(m);}
  void start_goal(){request("start","start");
ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
ASSERT_TRUE(proxy->wait_for_action_server(std::chrono::seconds(2)));Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=.7;g.pose.pose.orientation.w=1;
rclcpp_action::Client<Nav>::SendGoalOptions opts;opts.feedback_callback=[this](auto,auto f){proxy_distances.push_back(f->distance_remaining);};auto future=proxy->async_send_goal(g,opts);
ASSERT_TRUE(until([&]{return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));proxy_handle=future.get();ASSERT_TRUE(proxy_handle);
ASSERT_TRUE(until([this]{return active&&nonzero>0&&state.contains("goal")&&!state["goal"].is_null();},4))<<state.dump();}
};
class SlowGraphIntegration:public SupervisorIntegration {
 void SetUp()override{setenv("LD_PRELOAD",SLOW_GRAPH_LIBRARY,1);SupervisorIntegration::SetUp();unsetenv("LD_PRELOAD");}
};
TEST_F(SlowGraphIntegration, SlowDiscoveryCannotStarveVelocityAndPause){
 start_goal();max_output_gap=0;until([]{return false;},3.0);
 EXPECT_LT(max_output_gap,.20)<<"同步图查询阻塞速度输出，足以触发0.3秒底盘超时";
 request("pause","slow-graph-pause");ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2));
 EXPECT_EQ(last_velocity,0);EXPECT_TRUE(state["navigation_terminal"]);EXPECT_TRUE(state["still"]);
}
class ExpiredGraphIntegration:public SupervisorIntegration {
 void SetUp()override{
  setenv("LD_PRELOAD",SLOW_GRAPH_LIBRARY,1);setenv("CARCAR_TEST_GRAPH_DELAY_MS","1300",1);
  SupervisorIntegration::SetUp();unsetenv("LD_PRELOAD");unsetenv("CARCAR_TEST_GRAPH_DELAY_MS");
 }
};
TEST_F(ExpiredGraphIntegration, StaleTopologyLocksAndLateQueryCannotResume){
 start_goal();max_output_gap=0;
 ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},4))<<state.dump();
 until([]{return false;},1.5);
 EXPECT_EQ(state["phase"],"FAULT");EXPECT_DOUBLE_EQ(last_velocity,0);
 EXPECT_TRUE(state["navigation_terminal"]);EXPECT_TRUE(state["still"]);
 EXPECT_LT(max_output_gap,.20)<<"查询超时期间停车输出也必须连续";
}
TEST_F(SupervisorIntegration, PauseWaitsForTerminalThenAllowsManualWithoutReset){start_goal();request("pause","pause");
ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
EXPECT_DOUBLE_EQ(last_velocity,0);
EXPECT_TRUE(state["navigation_terminal"]);const auto origin=state["origin"];request("manual","manual");
ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL";},2));request("start","resume");
ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));
EXPECT_EQ(state["origin"],origin);}
TEST_F(SupervisorIntegration, ProxyCancellationKeepsItsSourceInTerminalReason){
 start_goal();auto result=proxy->async_get_result(proxy_handle);proxy->async_cancel_goal(proxy_handle);
 ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},3));
 EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::CANCELED);
 ASSERT_TRUE(until([this]{return state.value("navigation_terminal",false)&&state["goal"].is_null();},2));
 EXPECT_NE(state.value("goal_change_reason","").find("探索Action客户端"),std::string::npos);
 EXPECT_TRUE(state.value("navigation_detail","").empty());EXPECT_EQ(state["phase"],"EXPLORING");
}
TEST_F(SupervisorIntegration, HarmlessGridGrowthAndOriginChangeKeepActiveGoal){
  start_goal();const auto original=active;ack_cancel=false;
  grid_width=101;
  EXPECT_FALSE(until([&]{return original->is_canceling();},.6));
  grid_origin=-2.55;grid_width=102;
  EXPECT_FALSE(until([&]{return original->is_canceling();},.6));
  block_distant=true;
  EXPECT_FALSE(until([&]{return original->is_canceling();},.6));
  EXPECT_EQ(active,original);EXPECT_EQ(nav_requests,1U);
  EXPECT_EQ(state["phase"],"EXPLORING");ack_cancel=true;
}
TEST_F(SupervisorIntegration, OccupiedRemainingRouteStopsWithoutChangingLogicalGoal){
  start_goal();const auto goal=state["goal"];block_route=true;
  ASSERT_TRUE(until([this]{return last_velocity==0;},1));
  ASSERT_TRUE(until([this]{return state.value("navigation_terminal",false)&&state["goal"].is_null();},3));
  EXPECT_EQ(nav_requests,1U);EXPECT_EQ(state["phase"],"EXPLORING");
  EXPECT_NE(state.value("navigation_detail","").find("净空"),std::string::npos);
  (void)goal;
}
TEST_F(SupervisorIntegration, UnknownStartAreaRejectsBeforeExplorationAndKeepsManualAvailable){
unknown_start=true;until([]{return false;},.3);
ASSERT_TRUE(state.value("healthy",false));
request("start","unknown-start");
ASSERT_TRUE(until([this]{return state.value("request_id","")=="unknown-start";},2));
EXPECT_EQ(state["request_result"],"rejected");
EXPECT_EQ(state["phase"],"MANUAL");
EXPECT_EQ(state["control_mode"],"manual");
EXPECT_FALSE(state.value("anchored",true));
EXPECT_FALSE(state.value("save_required",true));
EXPECT_EQ(state["elapsed_seconds"],0);
EXPECT_NE(state.value("error","").find("当前车位"),std::string::npos);
EXPECT_NE(state.value("error","").find("未知"),std::string::npos);
EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,0U);EXPECT_FALSE(active);
unknown_start=false;until([]{return false;},.3);
request("start","clear-start");
ASSERT_TRUE(until([this]{return state.value("request_id","")=="clear-start";},2));
EXPECT_EQ(state["request_result"],"accepted");EXPECT_EQ(state["phase"],"EXPLORING");
}
class AutomaticStartupUnknown:public SupervisorIntegration {
  void SetUp()override{unknown_start=true;automatic_on_boot=true;SupervisorIntegration::SetUp();}
};
TEST_F(AutomaticStartupUnknown, RefusalReturnsToManualWithoutAnchoringOrSaving){
ASSERT_TRUE(until([this]{return !state.value("auto_start_pending",true)&&state.value("request_result","")=="rejected";},2));
EXPECT_EQ(state["phase"],"MANUAL");EXPECT_EQ(state["control_mode"],"manual");
EXPECT_FALSE(state.value("anchored",true));EXPECT_FALSE(state.value("save_required",true));
EXPECT_FALSE(state.value("automatic_start_ready",true));
EXPECT_NE(state.value("automatic_start_reason","").find("当前车位"),std::string::npos);
EXPECT_EQ(state["elapsed_seconds"],0);EXPECT_EQ(nonzero,0U);EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_FALSE(active);
}
class DelayedStartupOdometry:public SupervisorIntegration{
  void SetUp()override{delay_initial_odometry=true;odom_x=1.0;SupervisorIntegration::SetUp();}
};
TEST_F(DelayedStartupOdometry, AbsoluteInitialPositionIsNotTravelAndRealTravelStillLocks){
until([]{return false;},3.5);
ASSERT_EQ(state["phase"],"MANUAL")<<state.dump();
EXPECT_TRUE(state.value("healthy",false));EXPECT_EQ(state["quality_fault"],"");
EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,0U);
odom_x+=.2;
until([]{return false;},.4);
EXPECT_EQ(state["phase"],"MANUAL")<<"静止等待不能预先耗尽移动后扫描接纳的等待窗口";
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},4))<<state.dump();
EXPECT_NE(state.value("quality_fault","").find("接纳"),std::string::npos);
EXPECT_DOUBLE_EQ(last_velocity,0);
}
TEST_F(SupervisorIntegration, StaleLaserCancelsAndLocksInsteadOfReturning){start_goal();feed_scan=false;
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3))<<state.dump();
EXPECT_DOUBLE_EQ(last_velocity,0);
EXPECT_TRUE(state["save_required"]);
EXPECT_NE(state["phase"],"RETURNING");}
TEST_F(SupervisorIntegration, CancelWithoutTerminalTimesOutAndNeverTransfers){start_goal();ack_cancel=false;request("manual","takeover");
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},7));
EXPECT_DOUBLE_EQ(last_velocity,0);
EXPECT_FALSE(state["navigation_terminal"]);}
TEST_F(SupervisorIntegration, DuplicateAndOldTaskRequestsDoNotRestartMotion){request("start","same-id");
ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));const auto generation=state["generation"];request("start","same-id");until([]{return false;},.3);
EXPECT_EQ(state["generation"],generation);std_msgs::msg::String m;m.data="{\"task_id\":\"old\",\"request_id\":\"stale\",\"action\":\"manual\"}";command->publish(m);
ASSERT_TRUE(until([this]{return state.value("request_result","")=="rejected";},2));
EXPECT_EQ(state["phase"],"EXPLORING");}
TEST_F(SupervisorIntegration, RejectedOldRequestCannotRelabelActiveGoal){
 start_goal();const auto goal=state["goal_id"];
 std_msgs::msg::String m;m.data="{\"task_id\":\"old\",\"request_id\":\"stale-request\",\"action\":\"manual\"}";command->publish(m);
 ASSERT_TRUE(until([this]{return state.value("request_id","")=="stale-request"&&state.value("request_result","")=="rejected";},2));
 EXPECT_EQ(state.value("active_request_id",""),"start");EXPECT_EQ(state.value("goal_request_id",""),"start");
 EXPECT_EQ(state["goal_id"],goal);EXPECT_EQ(state["phase"],"EXPLORING");EXPECT_EQ(nav_requests,1U);
}
TEST_F(SupervisorIntegration, ProgressObservabilityUsesFiniteNavigationFeedback){
 start_goal();auto feedback=std::make_shared<Nav::Feedback>();feedback->distance_remaining=0;active->publish_feedback(feedback);
 until([]{return false;},.2);EXPECT_EQ(state.value("navigation_remaining_m",-2.),-1.)<<"尚未建好路径的默认零值不能作为远目标的进展基线";
 feedback->distance_remaining=.17071068;active->publish_feedback(feedback);until([]{return false;},.2);
 EXPECT_EQ(state.value("navigation_remaining_m",-2.),-1.)<<"上个目标的正距离也不能作为新目标的进展基线";EXPECT_TRUE(proxy_distances.empty());
 feedback->distance_remaining=1.5;active->publish_feedback(feedback);
 ASSERT_TRUE(until([this]{return state.value("navigation_remaining_m",-1.)==1.5;},2));
 feedback->distance_remaining=1.1;active->publish_feedback(feedback);
 ASSERT_TRUE(until([this]{return std::abs(state.value("navigation_progress_m",-1.)-.4)<1e-5;},2));
 feedback->distance_remaining=std::numeric_limits<float>::quiet_NaN();active->publish_feedback(feedback);
 until([]{return false;},.25);EXPECT_NEAR(state.value("navigation_remaining_m",-1.),1.1,1e-5);
 feedback->distance_remaining=0;active->publish_feedback(feedback);until([]{return false;},.2);
 EXPECT_NEAR(state.value("navigation_remaining_m",-1.),1.1,1e-5)<<"车仍远离观测点时，默认零反馈也不能伪造进展";
 feedback->distance_remaining=.36;active->publish_feedback(feedback);until([]{return false;},.2);
 EXPECT_NEAR(state.value("navigation_remaining_m",-1.),1.1,1e-5);ASSERT_FALSE(proxy_distances.empty());EXPECT_NEAR(proxy_distances.back(),1.1,1e-5);
 odom_x=pose_x=.6;ASSERT_TRUE(until([this]{return state.value("wheel_position",std::vector<double>{0,0})[0]>.59;},2));
 until([]{return false;},.2);feedback->distance_remaining=0;active->publish_feedback(feedback);
 ASSERT_TRUE(until([this]{return state.value("navigation_remaining_m",-1.)==0;},2))<<"真实已接近目标时仍允许零距离反馈";
 EXPECT_NEAR(state.value("navigation_progress_m",-1.),1.5,1e-5);
 EXPECT_GT(state.value("goal_seconds",0.),0.);EXPECT_GE(state.value("goal_seconds",0.),state.value("navigation_attempt_seconds",0.));
}
TEST_F(SupervisorIntegration, StationaryPoseThrottleIsNotAQualityFault){request("start","start-stationary");
ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));feed_pose=false;until([]{return false;},3.5);
EXPECT_EQ(state["phase"],"EXPLORING");EXPECT_TRUE(state["healthy"]);}

TEST_F(SupervisorIntegration, NavigationFailureDoesNotPublishCandidateRejection){
 start_goal();auto result=proxy->async_get_result(proxy_handle);
 active->abort(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));
 EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);
 ASSERT_TRUE(until([this]{return state.contains("goal")&&state["goal"].is_null();},1));
 EXPECT_TRUE(rejected_candidates.empty())<<"已经规划／执行的失败不得触发候选快速重选";
}
class OccludedFrontier:public SupervisorIntegration {void SetUp()override{observation_wall=true;SupervisorIntegration::SetUp();}};
class DisconnectedFrontier:public SupervisorIntegration {void SetUp()override{narrow_door=true;SupervisorIntegration::SetUp();}};
TEST_F(DisconnectedFrontier, DisconnectedObservationIsRejectedBeforePlanner){
 request("start","disconnected-start");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
 ASSERT_TRUE(proxy->wait_for_action_server(std::chrono::seconds(2)));Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=1.5;g.pose.pose.orientation.w=1;
 auto sent=proxy->async_send_goal(g);ASSERT_TRUE(until([&]{return sent.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));
 ASSERT_TRUE(sent.get());auto result=proxy->async_get_result(sent.get());
 ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2))<<state.dump();
 EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);
 EXPECT_EQ(plan_requests,0U)<<"净空掩膜已经断路，不应继续消耗路径预检";
 ASSERT_TRUE(until([this]{return !rejected_candidates.empty();},.5));
 EXPECT_EQ(rejected_candidates.front(),rclcpp_action::to_string(sent.get()->get_goal_id()));
 EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
}
class LargeObservationMap:public SupervisorIntegration {void SetUp()override{grid_width=10000;SupervisorIntegration::SetUp();feed_map=false;}};
TEST_F(LargeObservationMap, PauseAndMapChangeCannotDispatchLateSelection){
 request("start","selection-start");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
 ASSERT_TRUE(proxy->wait_for_action_server(std::chrono::seconds(2)));Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=1.5;g.pose.pose.orientation.w=1;
 auto sent=proxy->async_send_goal(g);ASSERT_TRUE(until([&]{return sent.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));ASSERT_TRUE(sent.get());
 ASSERT_TRUE(until([this]{return state.value("goal_selecting",false);},2))<<state.dump();
 EXPECT_TRUE(state["goal"].is_null());request("pause","selection-pause");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},2))<<state.dump();
 auto result=proxy->async_get_result(sent.get());ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},1));
 EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);
 until([]{return false;},1.0);EXPECT_EQ(plan_requests,0U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
 EXPECT_TRUE(state["navigation_terminal"]);EXPECT_TRUE(state["still"]);EXPECT_EQ(state["phase"],"PAUSED");
 EXPECT_TRUE(rejected_candidates.empty())<<"暂停不应被标记为可快速换点的候选拒绝";
 // 同一任务恢复后改变远处栅格，让第二次筛选使用新快照，避免复用已完成缓存。
 const auto before=state["map_revision"].get<uint64_t>();block_distant=true;feed_map=true;
 ASSERT_TRUE(until([&]{return state.value("map_revision",uint64_t{0})>before;},1));feed_map=false;
 request("start","selection-map-resume");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
 sent=proxy->async_send_goal(g);ASSERT_TRUE(until([&]{return sent.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));ASSERT_TRUE(sent.get());
 ASSERT_TRUE(until([this]{return state.value("goal_selecting",false);},2))<<state.dump();
 const auto revision=state["map_revision"].get<uint64_t>();narrow_door=true;feed_map=true;
 ASSERT_TRUE(until([&]{return state.value("map_revision",uint64_t{0})>revision;},1));feed_map=false;
 result=proxy->async_get_result(sent.get());ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},4))<<state.dump();
 EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);EXPECT_EQ(plan_requests,0U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
}
TEST_F(OccludedFrontier, ObservationAcrossWallIsRejectedBeforeNavigation){
 request("start","wall-start");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
 ASSERT_TRUE(proxy->wait_for_action_server(std::chrono::seconds(2)));Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=.725;g.pose.pose.orientation.w=1;
 auto sent=proxy->async_send_goal(g);ASSERT_TRUE(until([&]{return sent.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));
 ASSERT_TRUE(sent.get());auto result=proxy->async_get_result(sent.get());
 ASSERT_TRUE(until([&]{return result.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2))<<state.dump();
 EXPECT_EQ(result.get().code,rclcpp_action::ResultCode::ABORTED);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
}
TEST_F(SupervisorIntegration, DelayedLoopEventExplainsCorrectionAndRevalidatesGoal){start_goal();pose_x=.8;
ASSERT_TRUE(until([this]{return state.value("pose_correction_pending",false);},1));
++loop_generation;
ASSERT_TRUE(until([this]{return state.value("loop_generation",uint64_t{0})==loop_generation&&!state.value("pose_correction_pending",true)&&nav_requests==2;},3));
EXPECT_EQ(state["phase"],"EXPLORING");EXPECT_EQ(state["quality_fault"],"");
ASSERT_EQ(nav_targets.size(),2U);EXPECT_DOUBLE_EQ(nav_targets[0],nav_targets[1]);}

TEST_F(SupervisorIntegration, UnexplainedCorrectionStopsAndLocks){start_goal();pose_x=.8;
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3))<<state.dump();
EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_TRUE(state["save_required"]);}
TEST_F(SupervisorIntegration, RepeatedNav2FootprintWithFrozenStampCannotKeepMoving){start_goal();freeze_nav_pose=true;
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3))<<state.dump();
EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_TRUE(state["save_required"]);}
TEST_F(SupervisorIntegration, AllUnreachableStaysPartialEvenAfterReturn){start_goal();
auto publisher=n->create_publisher<explore_lite_msgs::msg::ExploreStatus>("/explore/status",rclcpp::QoS(1).reliable().transient_local());
explore_lite_msgs::msg::ExploreStatus status;status.status="exploration_incomplete_unreachable";publisher->publish(status);succeed_return=true;
ASSERT_TRUE(until([this]{return state.value("phase","")=="SAVE_PENDING";},4))<<state.dump();
EXPECT_EQ(state["outcome"],"partial_unreachable");EXPECT_EQ(state["return_result"],"succeeded");EXPECT_TRUE(state["save_required"]);}
TEST_F(SupervisorIntegration, InactiveNav2CannotKeepDriving){start_goal();nav2_active=false;
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},3))<<state.dump();EXPECT_DOUBLE_EQ(last_velocity,0);}
TEST_F(SupervisorIntegration, StalledPlannerCannotReopenLatchedFault){
stall_plan=true;request("start","start-stalled-plan");
ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=.7;g.pose.pose.orientation.w=1;proxy->async_send_goal(g);
ASSERT_TRUE(until([this]{return state.value("phase","")=="FAULT";},15))<<state.dump();
const auto generation=state["generation"];until([]{return false;},.6);
EXPECT_EQ(state["phase"],"FAULT");EXPECT_EQ(state["generation"],generation);EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,0U);}
TEST_F(SupervisorIntegration, PauseDuringInternalRevalidationCannotReopenMotion){
 start_goal();const auto original=active;ack_cancel=false;++loop_generation;
 ASSERT_TRUE(until([&]{return original->is_canceling();},2));
 request("pause","pause-revalidation");ack_cancel=true;
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
 until([]{return false;},.5);EXPECT_EQ(nav_requests,1U);EXPECT_DOUBLE_EQ(last_velocity,0);
 EXPECT_EQ(state.value("navigation_detail",""),"");
}
TEST_F(SupervisorIntegration, LatePlanIsCheckedAgainstNewestMapBeforeDispatch){
 stall_plan=true;request("start","start-late-plan");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));
 Nav::Goal g;g.pose.header.frame_id="map";g.pose.pose.position.x=.7;g.pose.pose.orientation.w=1;proxy->async_send_goal(g);
 ASSERT_TRUE(until([this]{return bool(stalled_plan);},2));block_route=true;until([]{return false;},.15);
 auto result=std::make_shared<Plan::Result>();result->path.header.frame_id="map";geometry_msgs::msg::PoseStamped first;first.pose.orientation.w=1;result->path.poses={first,stalled_plan->get_goal()->goal};stalled_plan->succeed(result);
 ASSERT_TRUE(until([this]{return !state.value("navigation_detail","").empty();},2));
 EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(SupervisorIntegration, LoopHoldsUntilClosureEndsAndNewMapArrives){
 start_goal();loop_active=true;++loop_generation;feed_map=false;
 ASSERT_TRUE(until([this]{return state.value("loop_waiting_map",false)&&last_velocity==0;},2));
 until([]{return false;},.5);EXPECT_EQ(nav_requests,1U);EXPECT_DOUBLE_EQ(last_velocity,0);
 loop_active=false;++loop_generation;until([]{return false;},.5);
 EXPECT_EQ(nav_requests,1U);EXPECT_TRUE(state.value("loop_waiting_map",false));
 feed_map=true;ASSERT_TRUE(until([this]{return nav_requests==2;},3));
 EXPECT_DOUBLE_EQ(nav_targets[0],nav_targets[1]);
}
