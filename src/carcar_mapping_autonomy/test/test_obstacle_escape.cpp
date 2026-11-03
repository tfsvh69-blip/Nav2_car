// 从已验证短退候选迁移的隔离动作夹具；生产源码使用当前稳定性工作区。
#include <gtest/gtest.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nlohmann/json.hpp>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "carcar_mapping_autonomy/map_geometry.hpp"
#include "carcar_mapping_autonomy/map_fingerprint.hpp"
extern char **environ;
using Json=nlohmann::json;
using Nav=nav2_msgs::action::NavigateToPose;
using Plan=nav2_msgs::action::ComputePathToPose;
using Backup=nav2_msgs::action::BackUp;
class SupervisorIntegration:public testing::Test {
protected:
  rclcpp::Node::SharedPtr n;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> fixed;
  pid_t pid=-1;Json state=Json::object();bool feed_scan=true,ack_cancel=true,feed_pose=true,feed_map=true,loop_active=false;
  double sideways_speed=0;
  double pose_x=0,pose_y=0,odom_y=0;uint64_t loop_generation=0,slam_sequence=1,loop_rejections=0;
  bool freeze_nav_pose=false,session_ready=true,publish_session=true,storage_ready=true,near_obstacle=false;builtin_interfaces::msg::Time nav_pose_stamp;
  bool succeed_return=false,stall_plan=false,stall_after_failures=false,unknown_start=false,automatic_on_boot=false,continuous_mode=false;
  bool succeed_any_return=false,fake_return_success=false,origin_obstacle=false,fill_return_neighborhood=false;
  bool fast_selection_timeout=false;
  bool wait_only=false,fixed_observation=false;
  bool obstacle_escape_mode=false,missing_rear_observation=false,rear_blocked=false,table_side_obstacle=true;
  bool future_escape_observation=false;
  bool backup_moves=true,backup_late_success=false,backup_ack_cancel=true,feed_odometry=true;
  bool follow_explore_resume=false,fixture_explore_enabled=false,queued_explore_goal=false;
  size_t blocked_start_plans=0,backup_requests=0,backup_cancels=0,backward_outputs=0;
  double backup_start_x=0,backup_distance=0,backup_coast_speed=0;
  std::vector<Backup::Goal> backup_goals;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_diagnostic;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_resume;
  rclcpp_action::Server<Backup>::SharedPtr backup_server;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Backup>> backup_active;
  bool delay_initial_odometry=false;double odom_feed_after=0,odom_x=0;
  int grid_width=100;double grid_origin=-2.5;bool block_route=false,observation_wall=false,block_distant=false,narrow_door=false;
  size_t nav_requests=0,plan_requests=0;std::vector<double> nav_targets,proxy_distances;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Plan>> stalled_plan;
  bool nav2_active=true;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lifecycle;
  bool recovery_mode=false,online_snapshot_mode=false,closed_outer_walls=false,fail_checkpoints=false;size_t plan_failures=0;
  Json recovery_state={{"phase","CHECKPOINT_READY"},{"checkpoint_id","fixture-stable"},{"imu_ready",true},{"imu_stable",true},{"gate_write",false},{"requires_stop",false},{"generation",0}};
  std::vector<std::string> recovery_actions,nav_trees;
  std::vector<geometry_msgs::msg::PoseStamped> nav_poses;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_commands;
  uint64_t navigation_generation=0;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr navigation_generation_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mask_status_pub;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr global_raw_pub,local_raw_pub;
  std::vector<rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr>clear_services;
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
    if(recovery_mode||obstacle_escape_mode){
      const auto latched=rclcpp::QoS(1).reliable().transient_local();
      mask_pub=n->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapping/known_space_mask",latched);
      mask_status_pub=n->create_publisher<std_msgs::msg::String>("/mapping/known_space/state",latched);
      global_raw_pub=n->create_publisher<nav2_msgs::msg::Costmap>("/global_costmap/costmap_raw",latched);
      local_raw_pub=n->create_publisher<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",latched);
      navigation_generation_sub=n->create_subscription<std_msgs::msg::UInt64>("/mapping/navigation_generation",latched,[this](const std_msgs::msg::UInt64&m){navigation_generation=m.data;});
      for(const auto*name:{"/global_costmap/clear_entirely_global_costmap","/local_costmap/clear_entirely_local_costmap"})
        clear_services.push_back(n->create_service<nav2_msgs::srv::ClearEntireCostmap>(name,[](nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr,nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr){}));
    }
    if(recovery_mode){
      recovery_pub=n->create_publisher<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local());
      recovery_commands=n->create_subscription<std_msgs::msg::String>("/mapping/recovery/request",10,[this](const std_msgs::msg::String&m){
        const auto j=Json::parse(m.data);const auto action=j.value("action",std::string());recovery_actions.push_back(action);
        recovery_state["generation"]=j["generation"];recovery_state["request_id"]=j["request_id"];
        if(action=="freeze")recovery_state.update({{"phase","DISTURBED"},{"gate_write",true},{"requires_stop",true}});
        else if(action=="checkpoint"&&fail_checkpoints)recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true},{"reason","检查点几何校验失败"}});
        else if(action=="recover"||action=="recover_rollback")recovery_state["phase"]="WAITING_STABLE";
        else if(action=="commit_verified")recovery_state.update({{"phase","SUCCEEDED"},{"gate_write",false},{"requires_stop",false}});
        else if(action=="snapshot"){recovery_state["snapshot"]=j;recovery_state["snapshot"]["phase"]="PREPARING";recovery_state["snapshot"]["in_flight"]=true;}
      });
    }
    input=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/collision_input",1,[this](const geometry_msgs::msg::Twist&m){collision->publish(m);});
    output=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,[this](const geometry_msgs::msg::Twist&m){
      const double t=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      if(previous_output_time>0)max_output_gap=std::max(max_output_gap,t-previous_output_time);
      previous_output_time=t;
      last_velocity=m.linear.x;if(std::abs(m.linear.x)>1e-5)++nonzero;if(m.linear.x< -1e-5)++backward_outputs;});
    nav_server=rclcpp_action::create_server<Nav>(n,"/navigate_to_pose",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){active=h;++nav_requests;nav_targets.push_back(h->get_goal()->pose.pose.position.x);nav_poses.push_back(h->get_goal()->pose);nav_trees.push_back(h->get_goal()->behavior_tree);});
    planner_diagnostic=n->create_publisher<std_msgs::msg::String>("/mapping/planner_diagnostic",10);
    plan_server=rclcpp_action::create_server<Plan>(n,"/compute_path_to_pose",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){++plan_requests;if(stall_plan||(stall_after_failures&&plan_failures==0)){stalled_plan=h;return;}auto result=std::make_shared<Plan::Result>();if(blocked_start_plans){--blocked_start_plans;std_msgs::msg::String diagnostic;diagnostic.data=Json{{"planner_id",h->get_goal()->planner_id},{"code","START_BLOCKED"},{"reason","规划未执行：实际车位存在动态障碍"},{"goal_stamp_ns",rclcpp::Time(h->get_goal()->goal.header.stamp).nanoseconds()},{"target",{h->get_goal()->goal.pose.position.x,h->get_goal()->goal.pose.position.y}}}.dump();planner_diagnostic->publish(diagnostic);h->abort(result);return;}if(plan_failures){--plan_failures;h->abort(result);return;}result->path.header.frame_id="map";geometry_msgs::msg::PoseStamped start;start.pose.orientation.w=1;start.pose.position.x=pose_x;start.pose.position.y=pose_y;result->path.poses={start,h->get_goal()->goal};h->succeed(result);});
    if(obstacle_escape_mode)backup_server=rclcpp_action::create_server<Backup>(n,"/backup",
      [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](auto){++backup_cancels;return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto h){backup_active=h;backup_start_x=odom_x;backup_distance=0;++backup_requests;backup_goals.push_back(*h->get_goal());});
    proxy=rclcpp_action::create_client<Nav>(n,"/mapping/explore_navigate_to_pose");
    if(obstacle_escape_mode)explore_resume=n->create_subscription<std_msgs::msg::Bool>("/explore/resume",
      rclcpp::QoS(1).reliable().transient_local(),[this](const std_msgs::msg::Bool&m){
        if(m.data&&!fixture_explore_enabled&&follow_explore_resume)queued_explore_goal=true;
        fixture_explore_enabled=m.data;
      });
    if(delay_initial_odometry)odom_feed_after=n->now().seconds()+1.5;
    timer=n->create_wall_timer(std::chrono::milliseconds(50),[this]{feed();});
    std::string executable_path=std::getenv("CARCAR_RETURN_BASELINE_EXE")?std::getenv("CARCAR_RETURN_BASELINE_EXE"):AUTONOMY_EXE;
    char* executable=executable_path.data();
    char ros_args[]="--ros-args",parameter[]="-p";
    char mode_manual[]="control_mode:=manual",mode_automatic[]="control_mode:=automatic";
    char recovery_on[]="recovery_enabled:=true",recovery_off[]="recovery_enabled:=false";
    char continuous_on[]="continuous_exploration:=true",continuous_off[]="continuous_exploration:=false";
    char snapshot_on[]="online_snapshots:=true",snapshot_off[]="online_snapshots:=false";
    char selection_fast[]="selection_stall_seconds:=1.0",selection_normal[]="selection_stall_seconds:=15.0";
    std::vector<std::string> arguments={executable,ros_args,parameter,automatic_on_boot?mode_automatic:mode_manual,parameter,recovery_mode?recovery_on:recovery_off,parameter,continuous_mode?continuous_on:continuous_off,parameter,online_snapshot_mode?snapshot_on:snapshot_off,parameter,fast_selection_timeout?selection_fast:selection_normal};
    if(obstacle_escape_mode)for(const auto*value:{wait_only?"obstacle_escape_enabled:=false":"obstacle_escape_enabled:=true",
      wait_only?"obstacle_wait_enabled:=true":"obstacle_wait_enabled:=false","obstacle_escape_wait_seconds:=0.1",
      "obstacle_escape_max_seconds:=6.0","obstacle_escape_max_attempts:=2","obstacle_escape_max_distance_m:=0.06",
      "obstacle_escape_max_total_distance_m:=0.12","obstacle_escape_speed_mps:=0.03"}){
      arguments.emplace_back("-p");arguments.emplace_back(value);
    }
    if(fixed_observation){arguments.emplace_back("-p");arguments.emplace_back("fixed_observation_goals:=true");}
    std::vector<char*> args;for(auto&value:arguments)args.push_back(value.data());args.push_back(nullptr);
    ASSERT_EQ(posix_spawn(&pid,executable,nullptr,nullptr,args.data(),environ),0);
    ASSERT_TRUE(until([this]{return state.value("healthy",false)&&state.value("still",false);},8))<<state.dump();
  }
  void TearDown()override{
    if(pid>0){kill(pid,SIGINT);for(int i=0;i<50;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
    timer.reset();lifecycle.reset();recovery_commands.reset();recovery_pub.reset();navigation_generation_sub.reset();
    mask_pub.reset();mask_status_pub.reset();global_raw_pub.reset();local_raw_pub.reset();clear_services.clear();n.reset();
    // 清除该fixture拥有的DDS发布者后，再进入下一用例。
    active.reset();stalled_plan.reset();proxy_handle.reset();proxy.reset();nav_server.reset();plan_server.reset();status.reset();candidate_rejections.reset();input.reset();output.reset();
    backup_active.reset();backup_server.reset();planner_diagnostic.reset();explore_resume.reset();
    scan.reset();filtered.reset();odom.reset();map.reset();pose.reset();local_pose.reset();global_pose.reset();slam.reset();ready.reset();command.reset();nav_velocity.reset();collision.reset();tf.reset();fixed.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  template<class F>bool until(F f,double timeout){const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(n);if(f())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return f();}
  void feed(){
    // 只有新避障夹具根据后退动作推进有符号轮式位移，旧测试的运动输入保持原样。
    if(backup_active&&backup_active->is_active()&&!backup_active->is_canceling()&&backup_moves){
      odom_x-=std::min(.03,double(std::abs(backup_active->get_goal()->speed)))*.05;pose_x=odom_x;
      backup_distance=backup_start_x-odom_x;
    }else if(backup_cancels>0&&backup_coast_speed>0){
      // 软件夹具显式保留取消后的制动滑行，不把协议终态伪造成实体立即停稳。
      odom_x-=backup_coast_speed*.05;pose_x=odom_x;backup_distance=backup_start_x-odom_x;
    }
    const auto now=n->now();
    // 当前扫描与匹配TF一起前推80毫秒，保持健康门允许范围内的TF可解。
    const auto scan_time=now+rclcpp::Duration::from_seconds(future_escape_observation?.08:0.);
    geometry_msgs::msg::TransformStamped t;t.header.stamp=scan_time;t.header.frame_id="map";t.child_frame_id="odom";t.transform.rotation.w=1;tf->sendTransform(t);t.header.frame_id="odom";t.child_frame_id="base_footprint";t.transform.translation.x=odom_x;t.transform.translation.y=odom_y;tf->sendTransform(t);
    sensor_msgs::msg::LaserScan s;s.header.stamp=scan_time;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_increment=2*M_PI/360;s.angle_max=M_PI;s.range_min=.05;s.range_max=12;s.ranges.assign(360,2.0);if(near_obstacle){s.ranges[180]=.1;s.ranges[181]=.1;}
    if(missing_rear_observation)for(size_t i=0;i<s.ranges.size();++i)if(std::cos(s.angle_min+i*s.angle_increment)<0)s.ranges[i]=std::numeric_limits<float>::quiet_NaN();
    // 固定世界坐标中的右侧桌腿；后退后回波相对车位前移，不伪造随车移动的障碍。
    if(obstacle_escape_mode&&table_side_obstacle){
      const double x=-odom_x,y=-.24;
      const auto beam=size_t(std::lround((std::atan2(y,x)-s.angle_min)/s.angle_increment))%s.ranges.size();
      s.ranges[beam]=std::hypot(x,y);
    }
    if(rear_blocked){s.ranges[0]=.15;s.ranges[1]=.15;s.ranges[359]=.15;}
    if(feed_scan){scan->publish(s);filtered->publish(s);}
    nav_msgs::msg::Odometry o;o.header.stamp=now;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.twist.twist.linear.y=sideways_speed;o.pose.pose.orientation.w=1;o.pose.pose.position.x=odom_x;o.pose.pose.position.y=odom_y;
    if(backup_active&&backup_active->is_active()&&!backup_active->is_canceling()&&backup_moves)o.twist.twist.linear.x=-.03;
    else if(backup_cancels>0&&backup_coast_speed>0)o.twist.twist.linear.x=-backup_coast_speed;
    if(feed_odometry&&now.seconds()>=odom_feed_after)odom->publish(o);
    nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.header.stamp=now;m.info.resolution=.05;m.info.width=grid_width;m.info.height=100;m.info.origin.position.x=grid_origin;m.info.origin.position.y=obstacle_escape_mode?grid_origin:-2.5;m.info.origin.orientation.w=1;m.data.assign(grid_width*100,0);if(unknown_start)m.data[50*grid_width+53]=-1;if(block_route)m.data[50*grid_width+58]=100;
    if(origin_obstacle)m.data[50*grid_width+50]=100;
    if(fill_return_neighborhood)for(int y=0;y<100;++y)for(int x=0;x<grid_width;++x)
      if(std::hypot(grid_origin+(x+.5)*.05,-2.5+(y+.5)*.05)<.95)m.data[y*grid_width+x]=100;
    if(block_distant)m.data[80*grid_width+80]=100;
    if(observation_wall)for(int y=0;y<100;++y){m.data[y*grid_width+63]=100;for(int x=64;x<grid_width;++x)m.data[y*grid_width+x]=-1;}
    // 激光可穿过0.35米门缝，0.22米圆包络及规划余量不能通过。
    if(narrow_door)for(int y=0;y<100;++y)if(y<47||y>53)m.data[y*grid_width+63]=100;
    if(closed_outer_walls){for(unsigned x=0;x<m.info.width;++x){m.data[x]=100;m.data[(m.info.height-1)*m.info.width+x]=100;}
      for(unsigned y=0;y<m.info.height;++y){m.data[y*m.info.width]=100;m.data[y*m.info.width+m.info.width-1]=100;}}
    if(feed_map)map->publish(m);
    if(mask_pub){
      mask_pub->publish(m);std_msgs::msg::String status;status.data=Json{{"source_fingerprint",std::to_string(carcar_mapping_autonomy::map_content_fingerprint(m))},{"navigation_generation",navigation_generation}}.dump();mask_status_pub->publish(status);
      nav2_msgs::msg::Costmap raw;raw.header=m.header;raw.metadata.size_x=m.info.width;raw.metadata.size_y=m.info.height;
      raw.metadata.layer="master";raw.metadata.resolution=m.info.resolution;raw.metadata.origin=m.info.origin;raw.metadata.update_time=now;
      raw.data.assign(m.data.size(),0);global_raw_pub->publish(raw);raw.header.frame_id="odom";local_raw_pub->publish(raw);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.frame_id="map";p.header.stamp=now;p.pose.pose.orientation.w=1;p.pose.pose.position.x=pose_x;p.pose.pose.position.y=pose_y;if(feed_pose)pose->publish(p);
    if(!freeze_nav_pose)nav_pose_stamp=now;
    geometry_msgs::msg::PolygonStamped fp;fp.header.stamp=nav_pose_stamp;fp.polygon.points.resize(4);fp.header.frame_id="odom";local_pose->publish(fp);fp.header.frame_id="map";global_pose->publish(fp);
    std_msgs::msg::String j;j.data=Json{{"sequence",slam_sequence},{"accepted",true},{"accepted_stamp_ns",now.nanoseconds()},{"loop_generation",loop_generation},{"loop_active",loop_active},{"loop_guard",{{"rejections",loop_rejections},{"rolled_back",loop_rejections>0}}}}.dump();slam->publish(j);j.data=Json{{"ready",session_ready},{"storage",{{"start_allowed",storage_ready},{"reason",storage_ready?"":"磁盘余量不足2 GiB"}}}}.dump();if(publish_session)ready->publish(j);
    if(recovery_pub&&!task_id.empty()){recovery_state["task_id"]=task_id;j.data=recovery_state.dump();recovery_pub->publish(j);}
    geometry_msgs::msg::Twist v;v.linear.x=.06;
    if(backup_active&&backup_active->is_active())v.linear.x=backup_active->is_canceling()?0:-std::min(.03,double(std::abs(backup_active->get_goal()->speed)));
    nav_velocity->publish(v);
    if(backup_active&&backup_active->is_canceling()&&backup_ack_cancel){
      if(backup_late_success)backup_active->succeed(std::make_shared<Backup::Result>());
      else backup_active->canceled(std::make_shared<Backup::Result>());
      backup_active.reset();
    }else if(backup_active&&backup_active->is_active()){
      auto feedback=std::make_shared<Backup::Feedback>();feedback->distance_traveled=backup_distance;backup_active->publish_feedback(feedback);
      if(backup_moves&&backup_distance>=std::abs(backup_active->get_goal()->target.x)){
        backup_active->succeed(std::make_shared<Backup::Result>());backup_active.reset();
      }
    }
    if(active&&active->is_canceling()&&ack_cancel){active->canceled(std::make_shared<Nav::Result>());active.reset();}
    else if(active&&((succeed_return&&std::abs(active->get_goal()->pose.pose.position.x)<.01)||succeed_any_return)){
      if(!fake_return_success){pose_x=odom_x=active->get_goal()->pose.pose.position.x;pose_y=odom_y=active->get_goal()->pose.pose.position.y;}
      active->succeed(std::make_shared<Nav::Result>());active.reset();}
    // 模拟explore_lite在监督器重新开放选点后提交下一候选，不在避障期间自行重试。
    if(queued_explore_goal&&fixture_explore_enabled&&state.value("phase","")=="EXPLORING"&&
       !state.value("obstacle_escape",Json::object()).value("active",false)&&proxy->action_server_is_ready()){
      queued_explore_goal=false;Nav::Goal goal;goal.pose.header.frame_id="map";goal.pose.pose.position.x=.7;goal.pose.pose.orientation.w=1;
      rclcpp_action::Client<Nav>::SendGoalOptions options;
      options.goal_response_callback=[this](auto h){proxy_handle=h;};proxy->async_send_goal(goal,options);
    }
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
#ifndef CARCAR_SUPERVISOR_FIXTURE_ONLY
class ObstacleEscapeIntegration:public SupervisorIntegration {
protected:
  void SetUp()override{obstacle_escape_mode=true;grid_origin=-2.51;SupervisorIntegration::SetUp();}
  void blocked_exploration(size_t failures=2){
    blocked_start_plans=failures;follow_explore_resume=true;request("start","escape-start");
    ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3))<<state.dump();
    ASSERT_TRUE(until([this]{return plan_requests>0;},3))<<state.dump();
  }
  Json escape()const{return state.value("obstacle_escape",Json::object());}
};
TEST_F(SupervisorIntegration, ObstacleEscapeIsDisabledByDefault){
 EXPECT_FALSE(state.value("obstacle_escape",Json::object()).value("enabled",false));
 EXPECT_EQ(backup_requests,0U);
}
TEST_F(ObstacleEscapeIntegration, FreshObservationReplansWithoutBackingWhenBlockageHasGone){
 table_side_obstacle=false;
 blocked_exploration(1);
 ASSERT_TRUE(until([this]{return plan_requests>=2&&nav_requests==1;},5))<<state.dump();
 EXPECT_EQ(state.value("phase",""),"EXPLORING");EXPECT_EQ(backup_requests,0U);
 EXPECT_EQ(backward_outputs,0U);EXPECT_FALSE(escape().value("active",false));
}
TEST_F(ObstacleEscapeIntegration, StartBlockedBacksThroughCollisionGateThenReplansOriginalTask){
 blocked_exploration();const auto origin=state["origin"],id=state["task_id"];
 ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
 EXPECT_EQ(state.value("phase",""),"EXPLORING");EXPECT_TRUE(escape().value("active",false));
 ASSERT_EQ(backup_goals.size(),1U);EXPECT_LT(backup_goals[0].target.x,0);
 EXPECT_LE(std::abs(backup_goals[0].target.x),.060001);EXPECT_LE(backup_goals[0].speed,.030001);
 EXPECT_GE(last_velocity,-.030001);
 ASSERT_TRUE(until([this]{return nav_requests==1&&!escape().value("active",true);},6))<<state.dump();
 EXPECT_EQ(state["origin"],origin);EXPECT_EQ(state["task_id"],id);
 EXPECT_EQ(state.value("phase",""),"EXPLORING");EXPECT_GE(plan_requests,3U);
 EXPECT_GT(backup_distance,0);EXPECT_LE(backup_distance,.062);
}
TEST_F(ObstacleEscapeIntegration, MissingRearObservationNeverStartsBackupAndEventuallyPauses){
 missing_rear_observation=true;blocked_exploration();
 ASSERT_TRUE(until([this]{return escape().value("active",false);},3))<<state.dump();
 EXPECT_EQ(state.value("phase",""),"EXPLORING");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},8))<<state.dump();
 EXPECT_EQ(backup_requests,0U);EXPECT_EQ(backward_outputs,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
 EXPECT_FALSE(escape().value("active",true));EXPECT_FALSE(state.value("reason","").empty());
}
TEST_F(ObstacleEscapeIntegration, NewRearObstacleCancelsBackupAndClosesOutputImmediately){
 blocked_exploration();ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
 rear_blocked=true;
 ASSERT_TRUE(until([this]{return backup_cancels>0&&last_velocity==0;},1))<<state.dump();
 const auto backward_before=backward_outputs;
 until([]{return false;},.4);EXPECT_EQ(backward_outputs,backward_before);EXPECT_DOUBLE_EQ(last_velocity,0);
 EXPECT_EQ(nav_requests,0U);
}
TEST_F(ObstacleEscapeIntegration, ManualPauseCancelsAndLateBackupSuccessCannotResume){
 blocked_exploration();ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
 backup_late_success=true;request("pause","escape-pause");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state.dump();
 const auto backward_before=backward_outputs;until([]{return false;},.6);
 EXPECT_EQ(backup_cancels,1U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(backward_outputs,backward_before);
 EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(state.value("phase",""),"PAUSED");
 EXPECT_FALSE(escape().value("active",true));
}
TEST_F(ObstacleEscapeIntegration, EmergencyHaltCancelsAndLateBackupSuccessCannotResume){
 blocked_exploration();ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
 backup_late_success=true;request("halt","escape-halt");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3))<<state.dump();
 const auto backward_before=backward_outputs;until([]{return false;},.6);
 EXPECT_EQ(backup_cancels,1U);EXPECT_EQ(nav_requests,0U);EXPECT_EQ(backward_outputs,backward_before);
 EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(state.value("phase",""),"PAUSED");
}
TEST_F(ObstacleEscapeIntegration, LatestFutureScanCannotBeSkippedWhileBackupKeepsMoving){
  blocked_exploration();ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
  // 新扫描仍被输入健康门接受，但尚未进入非未来的扫掠审查集合；不能只沿用旧帧。
  future_escape_observation=true;
  EXPECT_TRUE(until([this]{return backup_cancels>0&&last_velocity==0;},.8))<<state.dump();
  const auto backward_before=backward_outputs;
  until([]{return false;},.4);
  EXPECT_EQ(backward_outputs,backward_before);EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_EQ(nav_requests,0U);
  EXPECT_TRUE(escape().value("active",false));
}
TEST_F(ObstacleEscapeIntegration, ManualTakeoverWaitsForStrictStillAfterBackupCancellation){
  blocked_exploration();ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
  backup_coast_speed=.01;request("manual","escape-manual-coasting");
  ASSERT_TRUE(until([this]{return backup_cancels>0&&escape().value("motion",Json::object()).value("result_received",false);},2))<<state.dump();
  const auto coasting_x=odom_x;
  until([]{return false;},.6);
  EXPECT_LT(odom_x,coasting_x-.003);EXPECT_EQ(state.value("phase",""),"STOPPING")<<state.dump();
  EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nav_requests,0U);
  // 0.01 m/s 低于旧停稳阈值，但仍在后退；只有真实零速连续确认后才交接。
  backup_coast_speed=0;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL";},3))<<state.dump();
  EXPECT_DOUBLE_EQ(last_velocity,0);
}
TEST_F(ObstacleEscapeIntegration, ReturnWaitsForStrictStillAfterBackupCancellation){
  blocked_exploration();ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
  backup_coast_speed=.01;request("return","escape-return-coasting");
  ASSERT_TRUE(until([this]{return backup_cancels>0&&escape().value("motion",Json::object()).value("result_received",false);},2))<<state.dump();
  const auto coasting_x=odom_x;
  until([]{return false;},.6);
  EXPECT_LT(odom_x,coasting_x-.003);EXPECT_EQ(state.value("phase",""),"STOPPING")<<state.dump();
  EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nav_requests,0U);
  backup_coast_speed=0;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RETURNING";},3))<<state.dump();
  EXPECT_EQ(state.value("return_result",""),"running");
}
TEST_F(ObstacleEscapeIntegration, ReturnUsesSameAutomaticEscapeAndDoesNotClaimEarlyArrival){
 request("start","escape-return-start");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
 blocked_start_plans=2;request("return","escape-return");
 ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
 EXPECT_EQ(state.value("phase",""),"RETURNING");EXPECT_EQ(state.value("return_result",""),"running");
 EXPECT_EQ(nav_requests,0U);
 ASSERT_TRUE(until([this]{return nav_requests==1&&!escape().value("active",true);},6))<<state.dump();
 EXPECT_EQ(state.value("phase",""),"RETURNING");EXPECT_EQ(state.value("return_result",""),"running");
}
TEST_F(ObstacleEscapeIntegration, MissingSignedProgressCancelsAndCannotLoopIndefinitely){
 backup_moves=false;blocked_start_plans=100;blocked_exploration(100);
 ASSERT_TRUE(until([this]{return backward_outputs>0;},5))<<state.dump();
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},9))<<state.dump();
 EXPECT_LE(backup_requests,2U);EXPECT_GE(backup_cancels,1U);EXPECT_DOUBLE_EQ(last_velocity,0);
 EXPECT_EQ(nav_requests,0U);EXPECT_FALSE(escape().value("active",true));
 const auto requests_before=backup_requests;until([]{return false;},.5);EXPECT_EQ(backup_requests,requests_before);
}
TEST_F(ObstacleEscapeIntegration, RepeatedBlockageKeepsEpisodeAttemptAndDistanceBudget){
 blocked_exploration(100);
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},12))<<state.dump();
 EXPECT_GE(backup_requests,1U);EXPECT_LE(backup_requests,2U);EXPECT_LE(-odom_x,.124);
 EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nav_requests,0U);
 const auto requests_before=backup_requests;until([]{return false;},.5);EXPECT_EQ(backup_requests,requests_before);
}
TEST_F(ObstacleEscapeIntegration, ShorterNav2FeedbackWithoutWheelTravelCannotResetEscapeBudget){
 blocked_exploration();ASSERT_TRUE(until([this]{return nav_requests==1&&active;},7))<<state.dump();
 ASSERT_EQ(backup_requests,1U);const double stopped_x=odom_x;
 auto feedback=std::make_shared<Nav::Feedback>();feedback->distance_remaining=2;
 active->publish_feedback(feedback);until([]{return false;},.1);
 feedback->distance_remaining=.6;active->publish_feedback(feedback);
 ASSERT_TRUE(until([this]{return state.value("navigation_remaining_m",-1.)<.7;},2))<<state.dump();
 EXPECT_DOUBLE_EQ(odom_x,stopped_x);
 EXPECT_EQ(escape().value("attempts",0),1)<<"只有Nav2剩余距离变短，不得清空同一受阻地点的脱困次数";
 active->succeed(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([this]{return state.value("navigation_terminal",false);},2));
 blocked_start_plans=100;queued_explore_goal=true;
 ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},10))<<state.dump();
 EXPECT_LE(backup_requests,2U);EXPECT_LE(-odom_x,.124);EXPECT_DOUBLE_EQ(last_velocity,0);
}

class ObstacleWaitIntegration:public ObstacleEscapeIntegration {
  void SetUp()override{wait_only=true;ObstacleEscapeIntegration::SetUp();}
};
TEST_F(ObstacleWaitIntegration, FreshScanReplansWithoutEnablingUnacceptedRetreat){
  blocked_exploration(2);
  ASSERT_TRUE(until([this]{return nav_requests==1&&!escape().value("active",true);},8))<<state.dump();
  EXPECT_EQ(backup_requests,0U);EXPECT_EQ(backward_outputs,0U);
  EXPECT_FALSE(escape().value("retreat_enabled",true));EXPECT_EQ(state.value("phase",""),"EXPLORING");
}
TEST_F(ObstacleWaitIntegration, PersistentObstacleExhaustsBoundedRetryAndAllowsOperatorPause){
  blocked_exploration(100);
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},10))<<state.dump();
  EXPECT_EQ(backup_requests,0U);EXPECT_EQ(backward_outputs,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_LE(escape().value("rechecks",0),6);const auto count=plan_requests;
  until([]{return false;},.4);EXPECT_EQ(plan_requests,count);
}
class FixedObservationIntegration:public SupervisorIntegration {
 void SetUp()override{fixed_observation=true;SupervisorIntegration::SetUp();}
};
TEST_F(FixedObservationIntegration, ValidKnownObservationKeepsExactPositionAndExistingNavigationProtocol){
 start_goal();ASSERT_EQ(nav_targets.size(),1U);EXPECT_DOUBLE_EQ(nav_targets[0],.7);
 EXPECT_DOUBLE_EQ(state["goal"][0].get<double>(),.7);EXPECT_EQ(state["phase"],"EXPLORING");
}
TEST_F(FixedObservationIntegration, KnownObservationStillRequiresContinuousSafeRoute){
 block_route=true;until([]{return false;},.2);request("start","fixed-blocked-start");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3));
 Nav::Goal goal;goal.pose.header.frame_id="map";goal.pose.pose.position.x=.7;goal.pose.pose.orientation.w=1;
 auto future=proxy->async_send_goal(goal);ASSERT_TRUE(until([&]{return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2));
 ASSERT_TRUE(future.get());ASSERT_TRUE(until([this]{return plan_requests>0;},3));until([]{return false;},.3);
 EXPECT_EQ(nav_requests,0U);EXPECT_EQ(nonzero,0U);
}
#endif
