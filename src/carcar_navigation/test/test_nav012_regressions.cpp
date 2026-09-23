// NAV-012：真实 ROS 回调与行为树回归；测试域由 CTest 隔离。
#include <chrono>
#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/srv/is_path_valid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <atomic>
#include <fstream>
#include <filesystem>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include "carcar_navigation/recovery_runtime.hpp"
#include "carcar_navigation/bounded_action.hpp"
#include "carcar_navigation/motion_gate.hpp"
#include "carcar_navigation/navigation_motion_gate.hpp"
#include "carcar_navigation/goal_status_policy.hpp"

using namespace std::chrono_literals;
extern char ** environ;

class Nav012Regression : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    client = std::make_shared<rclcpp::Node>("regression_bt_client");
    provider = std::make_shared<rclcpp::Node>("regression_provider");
    // 特意不把 client 加入执行器，模拟 Nav2 blackboard 的真实执行模型。
    exec.add_node(provider);
    thread = std::thread([this] {exec.spin();});
    factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_goal_updated_condition_bt_node.so");
    factory.registerFromPlugin(
      "/opt/ros/humble/lib/libnav2_globally_updated_goal_condition_bt_node.so");
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_is_path_valid_condition_bt_node.so");
    factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_path_expiring_timer_condition.so");
    path_valid_service = provider->create_service<nav2_msgs::srv::IsPathValid>(
      "/is_path_valid",
      [this](
        const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
        std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response) {
        response->is_valid = path_valid.load() && !request->path.poses.empty();
      });
    bb = BT::Blackboard::create();
    bb->set("node", client);
    bb->set("bt_loop_duration", 10ms);
    bb->set("server_timeout", 1000ms);
    bb->set("wait_for_service_timeout", 1000ms);
  }
  void TearDown() override
  {
    bb.reset();
    exec.cancel();
    if (thread.joinable()) {thread.join();}
  }
  void static_tf()
  {
    broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(provider);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_footprint";
    tf.transform.rotation.w = 1.0;
    broadcaster->sendTransform(tf);
  }
  BT::NodeConfiguration configuration() {BT::NodeConfiguration config;config.blackboard=bb;return config;}
  rclcpp::Node::SharedPtr client, provider;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::thread thread;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  std::atomic<bool> path_valid{true};
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr path_valid_service;
};

TEST_F(Nav012Regression, RearClearConsumesWithoutSpinningBlackboardNode)
{
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<RearClear costmap_topic='/regression/costmap' scan_topic='/regression/scan'/>"
    "</BehaviorTree></root>", bb);
  static_tf();
  auto cm_pub = provider->create_publisher<nav2_msgs::msg::Costmap>(
    "/regression/costmap", rclcpp::QoS(1).reliable().transient_local());
  auto scan_pub = provider->create_publisher<sensor_msgs::msg::LaserScan>(
    "/regression/scan", rclcpp::SensorDataQoS());
  auto footprint_pub = provider->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/local_costmap/published_footprint", rclcpp::QoS(1).reliable());
  auto deadline = std::chrono::steady_clock::now() + 2s;
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  while (std::chrono::steady_clock::now() < deadline && status != BT::NodeStatus::SUCCESS) {
    nav2_msgs::msg::Costmap cm;
    cm.header.frame_id = "map";
    cm.header.stamp = provider->now();
    cm.metadata.resolution = 0.05;
    cm.metadata.size_x = cm.metadata.size_y = 40;
    cm.metadata.origin.position.x = cm.metadata.origin.position.y = -1.0;
    cm.metadata.origin.orientation.w = 1.0;
    cm.data.assign(1600, 0);
    cm_pub->publish(cm);
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = provider->now();
    scan.header.frame_id = "base_footprint";
    scan.range_min = 0.05;
    scan.range_max = 10.0;
    scan.angle_min = -3.14;
    scan.angle_increment = 0.01;
    scan.ranges.assign(629, 5.0);
    scan_pub->publish(scan);
    geometry_msgs::msg::PolygonStamped fp;
    fp.header = scan.header;
    for (auto xy : {std::pair<float,float>{-.14F,-.13F}, {-.14F,.13F},
        {.14F,.13F}, {.14F,-.13F}}) {
      geometry_msgs::msg::Point32 p; p.x=xy.first; p.y=xy.second;
      fp.polygon.points.push_back(p);
    }
    footprint_pub->publish(fp);
    std::this_thread::sleep_for(30ms);
    status = tree.tickRoot();
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(Nav012Regression, ControlledSpinConsumesSuccessfulResult)
{
  using Spin = nav2_msgs::action::Spin;
  auto server = rclcpp_action::create_server<Spin>(provider, "/spin",
    [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
    [](auto handle) {handle->succeed(std::make_shared<Spin::Result>());});
  static_tf();
  auto cm_pub = provider->create_publisher<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap_raw", rclcpp::QoS(1).reliable().transient_local());
  auto scan_pub = provider->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS());
  auto footprint_pub = provider->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/local_costmap/published_footprint", rclcpp::QoS(1).reliable());
  std::mutex diagnostic_mutex;
  std::string spin_diagnostic;
  auto diagnostic_sub = provider->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "/controlled_spin/status", 10,
    [&](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(diagnostic_mutex);
      spin_diagnostic=msg->message;
      for (const auto & value : msg->values) {
        spin_diagnostic += " " + value.key + "=" + value.value;
      }
    });
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.y = 1;
  p.pose.orientation.w = 1;
  path.poses.push_back(p);
  bb->set("path", path);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<ControlledSpin path='{path}'/></BehaviorTree></root>", bb);
  // 先让传感器订阅真正收到首批数据；条件节点一旦首 tick 缺数据会按设计立即失败。
  for (int warmup=0;warmup<10;++warmup) {
    nav2_msgs::msg::Costmap cm;
    cm.header.frame_id="map";cm.header.stamp=provider->now();
    cm.metadata.resolution=.05;cm.metadata.size_x=cm.metadata.size_y=80;
    cm.metadata.origin.position.x=cm.metadata.origin.position.y=-2.;
    cm.metadata.origin.orientation.w=1.;cm.data.assign(6400,0);cm_pub->publish(cm);
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id="base_footprint";scan.header.stamp=provider->now();
    scan.range_min=.05;scan.range_max=10.;scan.angle_min=-3.14;scan.angle_increment=.01;
    scan.ranges.assign(629,5.);scan_pub->publish(scan);
    geometry_msgs::msg::PolygonStamped fp;fp.header=scan.header;
    for (auto xy : {std::pair<float,float>{-.14F,-.13F}, {-.14F,.13F},
        {.14F,.13F}, {.14F,-.13F}}) {
      geometry_msgs::msg::Point32 point;point.x=xy.first;point.y=xy.second;
      fp.polygon.points.push_back(point);
    }
    footprint_pub->publish(fp);std::this_thread::sleep_for(30ms);
  }
  auto deadline = std::chrono::steady_clock::now() + 2s;
  auto status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    nav2_msgs::msg::Costmap cm;
    cm.header.frame_id="map";cm.header.stamp=provider->now();
    cm.metadata.resolution=.05;cm.metadata.size_x=cm.metadata.size_y=80;
    cm.metadata.origin.position.x=cm.metadata.origin.position.y=-2.;
    cm.metadata.origin.orientation.w=1.;cm.data.assign(6400,0);cm_pub->publish(cm);
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id="base_footprint";scan.header.stamp=provider->now();
    scan.range_min=.05;scan.range_max=10.;scan.angle_min=-3.14;scan.angle_increment=.01;
    scan.ranges.assign(629,5.);scan_pub->publish(scan);
    geometry_msgs::msg::PolygonStamped fp;fp.header=scan.header;
    for (auto xy : {std::pair<float,float>{-.14F,-.13F}, {-.14F,.13F},
        {.14F,.13F}, {.14F,-.13F}}) {
      geometry_msgs::msg::Point32 point;point.x=xy.first;point.y=xy.second;
      fp.polygon.points.push_back(point);
    }
    footprint_pub->publish(fp);
    std::this_thread::sleep_for(30ms);
    status = tree.tickRoot();
  }
  std::string diagnostic;
  {std::lock_guard<std::mutex> lock(diagnostic_mutex);diagnostic=spin_diagnostic;}
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS) << diagnostic;
  tree.haltTree();
}

TEST(MotionGate, ClosedLeaseExpiryAndReopeningDropCachedCommands)
{
  carcar_navigation::MotionGate gate;
  geometry_msgs::msg::Twist v;v.linear.x=.1;
  gate.command(v,1);EXPECT_DOUBLE_EQ(gate.output(1).linear.x,0);
  gate.permit(1,1);EXPECT_DOUBLE_EQ(gate.output(1).linear.x,0);
  gate.command(v,1.01);EXPECT_DOUBLE_EQ(gate.output(1.02).linear.x,.1);
  EXPECT_DOUBLE_EQ(gate.output(1.21).linear.x,0);
  gate.permit(1,1.22);EXPECT_DOUBLE_EQ(gate.output(1.23).linear.x,0);
  gate.command(v,1.24);EXPECT_DOUBLE_EQ(gate.output(1.25).linear.x,.1);
  gate.permit(2,1.26);EXPECT_DOUBLE_EQ(gate.output(1.27).linear.x,0);
  gate.command(v,1.28);gate.permit(0,1.29);EXPECT_DOUBLE_EQ(gate.output(1.3).linear.x,0);
}
TEST(MotionGate, RejectsNonfiniteInput)
{
  carcar_navigation::MotionGate gate;gate.permit(1,1);
  geometry_msgs::msg::Twist v;v.angular.z=std::numeric_limits<double>::quiet_NaN();
  gate.command(v,1.01);EXPECT_DOUBLE_EQ(gate.output(1.02).angular.z,0);
}
TEST(GoalStatusPolicy, OldTerminalCannotOwnNewTaskRegardlessOfArrayOrder)
{
  carcar_navigation::GoalStatusPolicy policy;
  action_msgs::msg::GoalStatusArray array;
  action_msgs::msg::GoalStatus old,newer;
  old.goal_info.goal_id.uuid[0]=1;old.goal_info.stamp.sec=1;old.status=2;
  newer.goal_info.goal_id.uuid[0]=2;newer.goal_info.stamp.sec=2;newer.status=2;
  array.status_list={old};policy.observe(array);
  old.status=6;array.status_list={newer,old};
  EXPECT_EQ(policy.observe(array),carcar_navigation::goal_uuid_text(newer.goal_info.goal_id));
  EXPECT_FALSE(policy.owns_terminal(carcar_navigation::goal_uuid_text(old.goal_info.goal_id)));
  old.status=2;array.status_list={old}; // 旧的 EXECUTING 状态包晚到也不能倒退。
  EXPECT_EQ(policy.observe(array),carcar_navigation::goal_uuid_text(newer.goal_info.goal_id));
}
TEST(RecoveryFreshness, RosAgeTreatsTinyNegativeAsFresh)
{
  EXPECT_TRUE(carcar_navigation::ros_age_fresh(-0.001,0.5));
  EXPECT_TRUE(carcar_navigation::ros_age_fresh(0.10,0.5));
  EXPECT_FALSE(carcar_navigation::ros_age_fresh(0.60,0.5));
  EXPECT_FALSE(carcar_navigation::ros_age_fresh(-0.20,0.5));
  EXPECT_FALSE(carcar_navigation::ros_age_fresh(0.10,0.0));
}
TEST(HeadingAlign, FlipResetsBeforeOscillation)
{
  using carcar_navigation::HeadingAlignEvent;
  using carcar_navigation::classify_heading_align;
  EXPECT_EQ(classify_heading_align(false,0.0,0.8,0,0.35,3),HeadingAlignEvent::NewTarget);
  EXPECT_EQ(classify_heading_align(true,0.8,0.85,0,0.35,3),HeadingAlignEvent::Hold);
  EXPECT_EQ(classify_heading_align(true,0.8,-0.8,0,0.35,3),HeadingAlignEvent::NewTarget);
  EXPECT_EQ(classify_heading_align(true,0.8,-0.8,1,0.35,3),HeadingAlignEvent::NewTarget);
  EXPECT_EQ(classify_heading_align(true,0.8,-0.8,2,0.35,3),HeadingAlignEvent::Oscillating);
  EXPECT_TRUE(carcar_navigation::heading_target_flipped(0.8,-0.8,0.35));
  EXPECT_FALSE(carcar_navigation::heading_target_flipped(0.8,0.85,0.35));
}

TEST(ForwardProgress, AcceptsForwardArcButRejectsSidewaysReverseAndJump)
{
  using carcar_navigation::classify_forward_progress;
  const auto forward=classify_forward_progress(0.04,0.01,0.0);
  EXPECT_TRUE(forward.plausible);EXPECT_GT(forward.forward,0.03);
  const auto sideways=classify_forward_progress(0.0,0.04,0.0);
  EXPECT_TRUE(sideways.plausible);EXPECT_LT(std::abs(sideways.forward),0.03);
  const auto reverse=classify_forward_progress(-0.04,0.0,0.0);
  EXPECT_TRUE(reverse.plausible);EXPECT_LT(reverse.forward,0.0);
  const auto jump=classify_forward_progress(0.30,0.0,0.0);
  EXPECT_TRUE(jump.displaced);EXPECT_FALSE(jump.plausible);
}

TEST(AdaptiveBackup, SelectsLongestSafeSteppedDistanceWithStoppingClearance)
{
  std::vector<double> checked;
  const auto selected=carcar_navigation::select_backup_distance(
    .20,.40,[&](double swept) {
      checked.push_back(swept);
      return carcar_navigation::SafetyResult{swept<=.231,"TEST",swept<=.231?"safe":"blocked"};
    });
  ASSERT_TRUE(selected.ok);
  EXPECT_NEAR(selected.selected,.15,1e-9);
  EXPECT_NEAR(selected.swept_distance,.23,1e-9);
  ASSERT_FALSE(checked.empty());
  EXPECT_NEAR(checked.front(),.28,1e-9);
}

TEST(AdaptiveBackup, HonorsBudgetAndRejectsLessThanMinimum)
{
  auto clear=[](double) {return carcar_navigation::SafetyResult{true,"CLEAR","safe"};};
  const auto budgeted=carcar_navigation::select_backup_distance(.20,.12,clear);
  ASSERT_TRUE(budgeted.ok);
  EXPECT_NEAR(budgeted.selected,.10,1e-9);
  const auto exhausted=carcar_navigation::select_backup_distance(.20,.049,clear);
  EXPECT_FALSE(exhausted.ok);
  EXPECT_EQ(exhausted.result.code,"BUDGET_EXHAUSTED");
}

TEST(ControlledSpinRanking, UsesClearancePathAngleAndStableDirectionInOrder)
{
  using carcar_navigation::SpinCandidateEvaluation;
  using carcar_navigation::better_spin_candidate;
  SpinCandidateEvaluation current{true,-.52,.14,.05};
  EXPECT_TRUE(better_spin_candidate({true,.78,.15,.40},current,0));
  current={true,-.52,.15,.30};
  EXPECT_TRUE(better_spin_candidate({true,.78,.15,.20},current,0));
  current={true,-.78,.15,.20};
  EXPECT_TRUE(better_spin_candidate({true,.52,.15,.20},current,0));
  current={true,-.52,.15,.20};
  EXPECT_FALSE(better_spin_candidate({true,.52,.15,.20},current,-1));
  EXPECT_TRUE(better_spin_candidate({true,.52,.15,.20},current,1));
  EXPECT_TRUE(better_spin_candidate({true,.52,.15,.20},current,0));
}

TEST_F(Nav012Regression, ValidPathIsHeldUntilExpiryAndInvalidPathReplansImmediately)
{
  std::atomic<int> replans{0};
  factory.registerSimpleAction("CountReplan", [&replans](BT::TreeNode &) {
    ++replans;return BT::NodeStatus::SUCCESS;
  });
  nav_msgs::msg::Path path;path.header.frame_id="map";
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.pose.orientation.w=1;
  path.poses={pose,pose};bb->set("path",path);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><Fallback>"
    "<ReactiveSequence><Inverter><PathExpiringTimer seconds='0.15' path='{path}'/></Inverter>"
    "<IsPathValid path='{path}'/></ReactiveSequence><CountReplan/>"
    "</Fallback></BehaviorTree></root>",bb);
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(replans.load(),0);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(replans.load(),0);
  std::this_thread::sleep_for(120ms);
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(replans.load(),1);
  path_valid=false;
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(replans.load(),2);
}
TEST(HeadingReference, ChoosesForwardSegmentInsteadOfOldPathStart)
{
  geometry_msgs::msg::PoseStamped robot,goal;robot.header.frame_id="map";
  robot.pose.position.x=1;robot.pose.orientation.w=1;goal=robot;goal.pose.position.x=2;
  nav_msgs::msg::Path path;path.header.frame_id="map";
  for (double x : {0.,.5,1.,1.5,2.}) {auto p=robot;p.pose.position.x=x;path.poses.push_back(p);}
  auto ref=carcar_navigation::heading_reference(robot,path,goal);
  ASSERT_TRUE(ref.valid);EXPECT_NEAR(ref.error,0,1e-8);
  robot.pose.position.x=2;goal.pose.orientation.w=std::cos(.5);goal.pose.orientation.z=std::sin(.5);
  EXPECT_NEAR(carcar_navigation::heading_reference(robot,path,goal).error,1,1e-8);
}
TEST_F(Nav012Regression, BudgetCannotBeResetByGoalEpoch)
{
  auto runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  EXPECT_FALSE(runtime->reserve_backup(.21));
  ASSERT_TRUE(runtime->reserve_backup(.2));ASSERT_TRUE(runtime->reserve_backup(.2));
  ++runtime->epoch;
  EXPECT_FALSE(runtime->reserve_backup(.2));runtime->forward_progress(-.5);
  EXPECT_FALSE(runtime->reserve_backup(.2));runtime->forward_progress(.19);
  EXPECT_FALSE(runtime->reserve_backup(.2));runtime->forward_progress(.02);
  EXPECT_TRUE(runtime->reserve_backup(.2));
}
TEST_F(Nav012Regression, PartialBackupFailureChargesMeasuredTravelAndUnknownRetainsReservation)
{
  auto runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  ASSERT_TRUE(runtime->reserve_backup(.20));
  runtime->reconcile_backup(.20,.075,true);
  EXPECT_NEAR(runtime->backup_used,.075,1e-9);
  ASSERT_TRUE(runtime->reserve_backup(.20));
  runtime->reconcile_backup(.20,0.0,false);
  EXPECT_NEAR(runtime->backup_used,.275,1e-9);
}
TEST_F(Nav012Regression, BackupCooldownBlocksImmediateRetry)
{
  auto runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  runtime->backup_cooldown=12;
  EXPECT_FALSE(runtime->backup_cooling());
  runtime->begin_recovery();
  runtime->note_backup(true);
  EXPECT_TRUE(runtime->backup_cooling());
  runtime->note_backup(false);
  EXPECT_TRUE(runtime->backup_cooling());
  runtime->last_backup_at_=carcar_navigation::Steady::now()-
    std::chrono::duration_cast<carcar_navigation::Steady::duration>(std::chrono::duration<double>(13));
  EXPECT_FALSE(runtime->backup_cooling());
  runtime->backup_cooldown=0;
  runtime->note_backup(true);
  EXPECT_FALSE(runtime->backup_cooling());
}
TEST_F(Nav012Regression, ParkObserveAllowsOneBoundedFreshReplan)
{
  auto runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  runtime->max_observe_replans=1;
  runtime->observe_started=carcar_navigation::Steady::now();
  EXPECT_TRUE(runtime->request_observe_replan());
  EXPECT_EQ(runtime->observe_replans_used,1u);
  EXPECT_EQ(runtime->observe_started,carcar_navigation::TimePoint{});
  EXPECT_FALSE(runtime->request_observe_replan());

  // 只有确认净前进 0.20 m 脱离当前受阻区，才恢复观察重规划额度。
  runtime->forward_progress(.19);
  EXPECT_FALSE(runtime->request_observe_replan());
  runtime->forward_progress(.02);
  EXPECT_EQ(runtime->observe_replans_used,0u);
  EXPECT_TRUE(runtime->request_observe_replan());
}

struct CallbackLifetime {std::mutex mutex;bool alive{true};};
template<class Action>
struct FakeAction {
  std::shared_ptr<CallbackLifetime> lifetime=std::make_shared<CallbackLifetime>();
  using Handle=rclcpp_action::ServerGoalHandle<Action>;
  typename rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp::CallbackGroup::SharedPtr group;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> server_exec;
  std::unique_ptr<std::thread> server_thread;
  rclcpp::TimerBase::SharedPtr timer;
  std::vector<std::pair<std::shared_ptr<Handle>,carcar_navigation::TimePoint>> handles;
  std::atomic<int> accepted{0},canceled{0},finished{0};
  std::atomic<bool> reject{false},drop_result{false},drop_cancel{false};
  std::atomic<int> reject_delay_ms{0},cancel_delay_ms{0},abort_first{0},first_accept_delay_ms{0};
  double finish_after{.08};
  std::function<void(std::shared_ptr<Handle>,typename Action::Result::SharedPtr)> populate;
  ~FakeAction() {
    {
      std::lock_guard<std::mutex> lock(lifetime->mutex);lifetime->alive=false;timer->cancel();
    }
    if (server_exec) {server_exec->cancel();}
    if (server_thread && server_thread->joinable()) {server_thread->join();}
  }
  FakeAction(rclcpp::Node::SharedPtr node,const std::string & name,bool isolated=false) {
    rclcpp::CallbackGroup::SharedPtr cb_group;
    if (isolated) {
      group=node->create_callback_group(rclcpp::CallbackGroupType::Reentrant,false);
      server_exec=std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2);
      server_exec->add_callback_group(group,node->get_node_base_interface());
      server_thread=std::make_unique<std::thread>([this] {server_exec->spin();});
      cb_group=group;
    }
    server=rclcpp_action::create_server<Action>(node,name,
      [this,guard=lifetime](const rclcpp_action::GoalUUID &,std::shared_ptr<const typename Action::Goal>) {
        int extra_delay=0;
        {
          std::lock_guard<std::mutex> lock(guard->mutex);
          if (!guard->alive) {return rclcpp_action::GoalResponse::REJECT;}
          extra_delay=first_accept_delay_ms.exchange(0);
          if (reject_delay_ms && extra_delay==0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(reject_delay_ms.load()));
          }
          if (reject) {return rclcpp_action::GoalResponse::REJECT;}
        }
        if (extra_delay) {
          std::this_thread::sleep_for(std::chrono::milliseconds(extra_delay));
          std::lock_guard<std::mutex> lock(guard->mutex);
          if (!guard->alive) {return rclcpp_action::GoalResponse::REJECT;}
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this,guard=lifetime](std::shared_ptr<Handle>) {
        std::lock_guard<std::mutex> lock(guard->mutex);
        if (!guard->alive) {return rclcpp_action::CancelResponse::REJECT;}
        if (cancel_delay_ms) {std::this_thread::sleep_for(std::chrono::milliseconds(cancel_delay_ms.load()));}
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this,guard=lifetime](std::shared_ptr<Handle> handle) {
        std::lock_guard<std::mutex> lock(guard->mutex);if (!guard->alive) {return;}
        ++accepted;handles.emplace_back(handle,carcar_navigation::Steady::now());},
      rcl_action_server_get_default_options(),cb_group);
    timer=node->create_wall_timer(10ms,[this,guard=lifetime] {
      std::lock_guard<std::mutex> lock(guard->mutex);if (!guard->alive) {return;}
      for (auto & entry : handles) {
        auto handle=entry.first;if (!handle->is_active()) {continue;}
        auto result=std::make_shared<typename Action::Result>();
        if (populate) {populate(handle,result);}
        if (handle->is_canceling()) {
          if (!drop_cancel) {handle->canceled(result);++canceled;}
        } else if (!drop_result && carcar_navigation::seconds(entry.second)>finish_after) {
          if (finished++<abort_first) {handle->abort(result);} else {handle->succeed(result);}
        }
      }
    });
  }
};

struct SimInputs {
  std::shared_ptr<CallbackLifetime> lifetime=std::make_shared<CallbackLifetime>();
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr cm_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr fp_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub;
  rclcpp::Publisher<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr goal_pub;
  rclcpp::Publisher<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>::SharedPtr goals_pub;
  rclcpp::TimerBase::SharedPtr timer;
  std::atomic<uint8_t> uuid{1};
  std::atomic<bool> enabled{true},rear_obstacle{true};
  bool through;
  ~SimInputs() {std::lock_guard<std::mutex> lock(lifetime->mutex);lifetime->alive=false;timer->cancel();}
  SimInputs(rclcpp::Node::SharedPtr n,bool multiple=false):node(n),through(multiple) {
    cm_pub=node->create_publisher<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",rclcpp::QoS(1).transient_local());
    scan_pub=node->create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
    fp_pub=node->create_publisher<geometry_msgs::msg::PolygonStamped>("/local_costmap/published_footprint",rclcpp::QoS(1));
    odom_pub=node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",rclcpp::QoS(1));
    health_pub=node->create_publisher<std_msgs::msg::Bool>("/localization_monitor/ready",rclcpp::QoS(1).transient_local());
    goal_pub=node->create_publisher<nav2_msgs::action::NavigateToPose_FeedbackMessage>("/navigate_to_pose/_action/feedback",rclcpp::QoS(1));
    goals_pub=node->create_publisher<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>("/navigate_through_poses/_action/feedback",rclcpp::QoS(1));
    timer=node->create_wall_timer(40ms,[this,guard=lifetime] {
      std::lock_guard<std::mutex> lock(guard->mutex);if (guard->alive) {publish();}
    });
  }
  void publish() {
    if (!enabled) {return;}
    nav2_msgs::msg::Costmap cm;cm.header.frame_id="map";cm.header.stamp=node->now();
    cm.metadata.resolution=.05;cm.metadata.size_x=cm.metadata.size_y=60;
    cm.metadata.origin.position.x=cm.metadata.origin.position.y=-1.5;cm.metadata.origin.orientation.w=1;
    cm.data.assign(3600,0);if (rear_obstacle) {cm.data[30*60+25]=254;}cm_pub->publish(cm);
    sensor_msgs::msg::LaserScan scan;scan.header.frame_id="base_footprint";scan.header.stamp=node->now();
    scan.angle_min=-3.14;scan.angle_increment=.01;scan.range_min=.05;scan.range_max=10;scan.ranges.assign(629,5);
    scan_pub->publish(scan);
    geometry_msgs::msg::PolygonStamped fp;fp.header=scan.header;
    for (auto xy : {std::pair<float,float>{-.14F,-.13F},{-.14F,.13F},{.14F,.13F},{.14F,-.13F}}) {
      geometry_msgs::msg::Point32 p;p.x=xy.first;p.y=xy.second;fp.polygon.points.push_back(p);
    }
    fp_pub->publish(fp);
    nav_msgs::msg::Odometry odom;odom.header.stamp=node->now();odom.header.frame_id="odom";
    odom.pose.pose.orientation.w=1;odom_pub->publish(odom);
    std_msgs::msg::Bool health;health.data=true;health_pub->publish(health);
    if (through) {
      nav2_msgs::action::NavigateThroughPoses_FeedbackMessage msg;msg.goal_id.uuid[0]=uuid.load();goals_pub->publish(msg);
    } else {
      nav2_msgs::action::NavigateToPose_FeedbackMessage msg;msg.goal_id.uuid[0]=uuid.load();goal_pub->publish(msg);
    }
  }
};

TEST_F(Nav012Regression, FullSingleGoalTreeRecoversThroughRealNav2ControlNodes)
{
  using Plan=nav2_msgs::action::ComputePathToPose;
  FakeAction<Plan> planner(provider,"/compute_path_to_pose");
  planner.populate=[](auto handle,auto result) {
    result->path.header.frame_id="map";
    auto start=handle->get_goal()->goal;start.pose.position.x=start.pose.position.y=0;
    result->path.poses={start,handle->get_goal()->goal};
  };
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");follow.abort_first=2;
  FakeAction<nav2_msgs::action::Spin> spin(provider,"/spin");
  FakeAction<nav2_msgs::action::BackUp> backup(provider,"/backup");
  static_tf();SimInputs inputs(provider);
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_rate_controller_bt_node.so");
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=goal.pose.position.y=1;
  goal.pose.orientation.w=1;bb->set("goal",goal);
  auto tree=factory.createTreeFromFile(std::string(BEHAVIOR_TREES_DIR)+"/navigate_to_pose_experimental_recovery.xml",bb);
  auto end=carcar_navigation::after(12);auto status=BT::NodeStatus::RUNNING;
  while (status==BT::NodeStatus::RUNNING && carcar_navigation::Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(status,BT::NodeStatus::SUCCESS);
  EXPECT_GE(follow.accepted,3);EXPECT_EQ(spin.accepted,1);EXPECT_EQ(backup.accepted,0);
  tree.haltTree();exec.cancel();thread.join();
}
TEST_F(Nav012Regression, FullThroughPosesTreeReplacesGoalAfterCancelAndStillness)
{
  client->declare_parameter("transform_tolerance",.1);
  auto shared_runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  bb->set("tf_buffer",std::shared_ptr<tf2_ros::Buffer>(shared_runtime,&shared_runtime->tf));
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_remove_passed_goals_action_bt_node.so");
  using Plan=nav2_msgs::action::ComputePathThroughPoses;
  FakeAction<Plan> planner(provider,"/compute_path_through_poses");
  planner.populate=[](auto handle,auto result) {
    result->path.header.frame_id="map";
    auto start=handle->get_goal()->goals.back();start.pose.position.x=start.pose.position.y=0;
    result->path.poses={start,handle->get_goal()->goals.back()};
  };
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");follow.drop_result=true;
  FakeAction<nav2_msgs::action::Spin> spin(provider,"/spin");
  FakeAction<nav2_msgs::action::BackUp> backup(provider,"/backup");
  static_tf();SimInputs inputs(provider,true);
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_rate_controller_bt_node.so");
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=1;goal.pose.orientation.w=1;
  auto passed=goal;passed.pose.position.x=0;
  bb->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{passed,goal});
  auto tree=factory.createTreeFromFile(std::string(BEHAVIOR_TREES_DIR)+"/navigate_through_poses_experimental_recovery.xml",bb);
  auto end=carcar_navigation::after(5);
  while (follow.accepted<1 && carcar_navigation::Steady::now()<end) {tree.tickRoot();std::this_thread::sleep_for(10ms);}
  ASSERT_GE(follow.accepted,1);
  auto runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  EXPECT_EQ(bb->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals").size(),1u);
  EXPECT_EQ(runtime->epoch,2u); // 自动删去已到达前缀不触发目标抢占。
  runtime->backup_used=.3;
  // 相同坐标、仅朝向改变，再立即重复一份新 UUID；恢复额度不刷新。
  goal.pose.orientation.w=0;goal.pose.orientation.z=1;
  bb->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{goal});inputs.uuid=2;
  auto status=BT::NodeStatus::RUNNING;end=carcar_navigation::after(5);
  while (follow.canceled<1 && status==BT::NodeStatus::RUNNING && carcar_navigation::Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(follow.canceled,1);EXPECT_NEAR(runtime->backup_used,.3,1e-8);
  follow.drop_result=false;inputs.uuid=3;
  end=carcar_navigation::after(6);
  while (status==BT::NodeStatus::RUNNING && carcar_navigation::Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(status,BT::NodeStatus::SUCCESS);EXPECT_GE(follow.accepted,2);
  EXPECT_NEAR(runtime->backup_used,.3,1e-8);
  tree.haltTree();exec.cancel();thread.join();
}

TEST_F(Nav012Regression, ActionRejectionAndMissingResultHaveFiniteOutcomes)
{
  using Spin=nav2_msgs::action::Spin;
  FakeAction<Spin> server(provider,"/bounded_spin");
  server.reject=true;
  BT::NodeConfiguration cfg;cfg.blackboard=bb;
  auto session=std::make_shared<carcar_navigation::BoundedAction<Spin>>("test","/bounded_spin",cfg,.3,.3);
  Spin::Goal goal;goal.target_yaw=.5;
  session->send(goal,.3);
  auto end=carcar_navigation::after(1);
  while (!session->state().terminal && carcar_navigation::Steady::now()<end) {session->poll();std::this_thread::sleep_for(10ms);}
  EXPECT_TRUE(session->state().terminal);EXPECT_TRUE(session->state().rejected);
  server.reject=false;server.drop_result=true;
  session=std::make_shared<carcar_navigation::BoundedAction<Spin>>("test2","/bounded_spin",cfg,.3,.3);
  session->send(goal,.2);end=carcar_navigation::after(1);
  while (!session->state().terminal && carcar_navigation::Steady::now()<end) {session->poll();std::this_thread::sleep_for(10ms);}
  EXPECT_TRUE(session->state().terminal);EXPECT_TRUE(session->state().timed_out);
  EXPECT_EQ(server.canceled,1);
}
TEST_F(Nav012Regression, CancelBeforeGoalResponseCancelsOnlyOwnedLateHandle)
{
  using Spin=nav2_msgs::action::Spin;
  FakeAction<Spin> server(provider,"/late_spin");server.reject_delay_ms=150;server.drop_result=true;
  BT::NodeConfiguration cfg;cfg.blackboard=bb;
  auto session=std::make_shared<carcar_navigation::BoundedAction<Spin>>("test","/late_spin",cfg,.5,.5);
  Spin::Goal goal;goal.target_yaw=.5;
  session->send(goal,1);session->cancel();
  auto end=carcar_navigation::after(1);
  while (!session->state().terminal && carcar_navigation::Steady::now()<end) {session->poll();std::this_thread::sleep_for(10ms);}
  EXPECT_TRUE(session->state().terminal);EXPECT_TRUE(session->state().cancel_requested);
  EXPECT_EQ(server.canceled,1);
}
TEST_F(Nav012Regression, LateAcceptOfSupersededGoalIsCancelledWithoutPreemptingLive)
{
  using Follow=nav2_msgs::action::FollowPath;
  FakeAction<Follow> server(provider,"/late_superseded_follow",true);
  server.drop_result=true; server.finish_after=100; server.first_accept_delay_ms=400;
  BT::NodeConfiguration cfg;cfg.blackboard=bb;
  auto session=std::make_shared<carcar_navigation::BoundedAction<Follow>>("test","/late_superseded_follow",cfg,.1,.1);
  Follow::Goal goal;
  session->send(goal,0);
  std::this_thread::sleep_for(30ms);
  session->send(goal,0);
  auto end=carcar_navigation::after(.25);
  while (carcar_navigation::Steady::now()<end) {session->poll();std::this_thread::sleep_for(10ms);}
  EXPECT_EQ(server.accepted,1);
  end=carcar_navigation::after(.6);
  while (server.canceled<1 && carcar_navigation::Steady::now()<end) {
    session->poll();std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server.accepted,2);
  EXPECT_EQ(server.canceled,1);
  EXPECT_TRUE(session->state().accepted);
  EXPECT_FALSE(session->state().cancel_requested);
  EXPECT_FALSE(session->state().timed_out);
  EXPECT_NE(session->state().reason,"PREVIOUS_RESULT_MISSING");
  EXPECT_EQ(session->state().revision,2u);
}
TEST_F(Nav012Regression, CancelWithoutTerminalResultLatchesUnconfirmed)
{
  using Spin=nav2_msgs::action::Spin;
  FakeAction<Spin> server(provider,"/uncancelable_spin");server.drop_result=true;server.drop_cancel=true;
  BT::NodeConfiguration cfg;cfg.blackboard=bb;
  auto session=std::make_shared<carcar_navigation::BoundedAction<Spin>>("test","/uncancelable_spin",cfg,.3,.2);
  Spin::Goal goal;goal.target_yaw=.5;session->send(goal,1);
  std::this_thread::sleep_for(100ms);session->cancel();
  std::this_thread::sleep_for(300ms);session->poll();
  EXPECT_FALSE(session->state().terminal);EXPECT_TRUE(session->state().cancel_ack);
  EXPECT_EQ(session->state().reason,"CANCEL_UNCONFIRMED");
}

class KeepRunning : public BT::StatefulActionNode {
public:
  KeepRunning(const std::string & n,const BT::NodeConfiguration & c):BT::StatefulActionNode(n,c) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {}
};
TEST_F(Nav012Regression, RealGoalUpdatedIsResetByReactiveFallback)
{
  factory.registerNodeType<KeepRunning>("KeepRunning");
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.orientation.w=1;
  bb->set("goal",goal);bb->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{});
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<ReactiveFallback><GoalUpdated/><KeepRunning/></ReactiveFallback>"
    "</BehaviorTree></root>",bb);
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);
  goal.pose.position.x=2;bb->set("goal",goal);
  // 刻画安装版本的原始缺陷；完整树测试验证替代监督节点，而非修改这个断言掩盖根因。
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);
  tree.haltTree();
}

TEST_F(Nav012Regression, PathRevisionUpdatesDoNotResetAngularProgressWindow)
{
  factory.registerNodeType<KeepRunning>("KeepRunning");
  static_tf();
  SimInputs inputs(provider);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id="map";goal.pose.position.y=1.0;goal.pose.orientation.w=1.0;
  nav_msgs::msg::Path path;path.header.frame_id="map";
  geometry_msgs::msg::PoseStamped start=goal;start.pose.position.y=0.0;
  path.poses={start,goal};bb->set("goal",goal);bb->set("path",path);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<ProgressGuard path='{path}' goal='{goal}' angular_stagnation_timeout='0.25' "
    "angular_convergence_threshold='0.05' max_rotation_budget='0.80'>"
    "<KeepRunning/></ProgressGuard></BehaviorTree></root>",bb);
  std::this_thread::sleep_for(120ms);
  auto status=BT::NodeStatus::RUNNING;
  const auto began=carcar_navigation::Steady::now();
  const auto deadline=carcar_navigation::after(.65);
  unsigned revision=0;
  while (status==BT::NodeStatus::RUNNING && carcar_navigation::Steady::now()<deadline) {
    path.poses.back().pose.position.x=1e-4*++revision;
    bb->set("path",path);
    status=tree.tickRoot();
    std::this_thread::sleep_for(30ms);
  }
  EXPECT_EQ(status,BT::NodeStatus::FAILURE);
  EXPECT_GE(carcar_navigation::seconds(began),.20);
  EXPECT_GT(revision,3u);
  tree.haltTree();
}

TEST_F(Nav012Regression, SweptGeometryHandlesWorldFootprintAndInteriorUnknown)
{
  using namespace carcar_navigation;
  for (auto scenario : {std::array<double,3>{0,0,0},{4,3,1.57079632679},{-3,4,-.7}}) {
    const double x=scenario[0],y=scenario[1],yaw=scenario[2];
    tf2_ros::Buffer tf(client->get_clock());
    geometry_msgs::msg::TransformStamped body;body.header.frame_id="map";body.child_frame_id="base_footprint";
    body.transform.translation.x=x;body.transform.translation.y=y;
    body.transform.rotation.z=std::sin(yaw/2);body.transform.rotation.w=std::cos(yaw/2);
    ASSERT_TRUE(tf.setTransform(body,"regression",true));
    auto cm=std::make_shared<nav2_msgs::msg::Costmap>();cm->header.frame_id="map";cm->header.stamp=client->now();
    cm->metadata.resolution=.05;cm->metadata.size_x=cm->metadata.size_y=40;
    cm->metadata.origin.position.x=x-1;cm->metadata.origin.position.y=y-1;cm->metadata.origin.orientation.w=1;
    cm->data.assign(1600,0);
    auto fp=std::make_shared<geometry_msgs::msg::PolygonStamped>();fp->header=cm->header;
    for (auto p : {std::pair<double,double>{-.14,-.13},{-.14,.13},{.14,.13},{.14,-.13}}) {
      geometry_msgs::msg::Point32 point;
      point.x=x+std::cos(yaw)*p.first-std::sin(yaw)*p.second;
      point.y=y+std::sin(yaw)*p.first+std::cos(yaw)*p.second;fp->polygon.points.push_back(point);
    }
    auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();scan->header=cm->header;
    scan->header.frame_id="base_footprint";scan->angle_increment=.01;scan->range_min=.05;scan->range_max=10;
    scan->ranges.assign(629,5);
    SensorSnapshot d;d.costmap=cm;d.scan=scan;d.footprint=fp;
    d.costmap_received=d.scan_received=d.footprint_received=Steady::now();
    auto check=[&] {return swept_clear(d,tf,client->now(),"base_footprint",-.1,0);};
    auto body_cell=[&](double bx,double by) {
      const double wx=x+std::cos(yaw)*bx-std::sin(yaw)*by;
      const double wy=y+std::sin(yaw)*bx+std::cos(yaw)*by;
      const int mx=int(std::floor((wx-(x-1))/0.05));
      const int my=int(std::floor((wy-(y-1))/0.05));
      return my*40+mx;
    };
    EXPECT_TRUE(check().ok)<<check().detail;
    cm->data[body_cell(.12,0)]=254;EXPECT_TRUE(check().ok)<<check().detail;
    cm->data[body_cell(.12,0)]=0;
    cm->data[body_cell(.12,0)]=255;EXPECT_EQ(check().code,"COSTMAP_UNKNOWN");
    cm->data[body_cell(.12,0)]=0;
    cm->data[20*40+20]=255;EXPECT_EQ(check().code,"COSTMAP_UNKNOWN");
    cm->data[20*40+20]=254;EXPECT_EQ(check().code,"COSTMAP_OBSTACLE");
    cm->data[20*40+20]=0;
    cm->data[body_cell(-.24,0)]=255;EXPECT_EQ(check().code,"COSTMAP_UNKNOWN");
    cm->data[body_cell(-.24,0)]=254;EXPECT_EQ(check().code,"COSTMAP_OBSTACLE");
    cm->data[body_cell(-.24,0)]=0;
    cm->metadata.origin.position.x=x+1;EXPECT_EQ(check().code,"COSTMAP_OUT_OF_BOUNDS");
    cm->metadata.origin.position.x=x-1;
    d.scan_received=Steady::now()-1s;EXPECT_EQ(check().code,"DATA_EXPIRED");
    d.scan_received=Steady::now();scan->header.frame_id="unknown_laser";
    EXPECT_EQ(check().code,"TF_UNAVAILABLE");
  }
}

TEST_F(Nav012Regression, ScanExtrinsicAndTimestampAreAppliedBeforeRearCheck)
{
  tf2_ros::Buffer tf(client->get_clock());
  geometry_msgs::msg::TransformStamped laser;laser.header.frame_id="base_footprint";laser.child_frame_id="laser";
  laser.transform.translation.x=-.05;laser.transform.rotation.z=1;laser.transform.rotation.w=0;
  ASSERT_TRUE(tf.setTransform(laser,"regression",true));
  sensor_msgs::msg::LaserScan scan;scan.header.frame_id="laser";scan.header.stamp=client->now();
  scan.range_min=.05;scan.range_max=10;scan.angle_increment=.01;scan.ranges={.15};
  std::vector<geometry_msgs::msg::Point> points;
  ASSERT_TRUE(carcar_navigation::scan_points_in_base(scan,tf,"base_footprint",points));
  ASSERT_EQ(points.size(),1u);EXPECT_NEAR(points[0].x,-.20,1e-6);EXPECT_NEAR(points[0].y,0,1e-6);
  tf2_ros::Buffer dynamic(client->get_clock());laser.header.stamp=client->now();
  ASSERT_TRUE(dynamic.setTransform(laser,"regression",false));
  scan.header.stamp=rclcpp::Time(scan.header.stamp)-rclcpp::Duration(1s);
  points.clear();EXPECT_FALSE(carcar_navigation::scan_points_in_base(scan,dynamic,"base_footprint",points));
}

TEST_F(Nav012Regression, StopConfirmationRequiresNewFreshWheelSamples)
{
  auto runtime=carcar_navigation::RecoveryRuntime::get(configuration());
  static_tf();SimInputs inputs(provider);std::this_thread::sleep_for(1200ms);
  ASSERT_TRUE(runtime->still(1));runtime->restart_stillness();EXPECT_FALSE(runtime->still(1));
  std::this_thread::sleep_for(200ms);EXPECT_FALSE(runtime->still(1));
  std::this_thread::sleep_for(900ms);EXPECT_TRUE(runtime->still(1));
  inputs.enabled=false;std::this_thread::sleep_for(600ms);
  EXPECT_FALSE(runtime->still(1));EXPECT_FALSE(runtime->inputs_ready().ok);
  exec.cancel();thread.join();
}

TEST_F(Nav012Regression, RealMotionGatePublishesZeroOnExpiryAndWaitsForNewCommand)
{
  auto gate=std::make_shared<NavigationMotionGate>();exec.add_node(gate);
  auto measured=std::make_shared<std::atomic<double>>(99);
  auto sub=provider->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(1),
    [measured](geometry_msgs::msg::Twist::ConstSharedPtr msg) {measured->store(msg->linear.x);});
  auto permit=provider->create_publisher<std_msgs::msg::UInt64>("/navigation/motion_permit",rclcpp::QoS(1));
  auto input=provider->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed_raw",rclcpp::QoS(1));
  std::this_thread::sleep_for(200ms);EXPECT_DOUBLE_EQ(measured->load(),0);
  std_msgs::msg::UInt64 lease;lease.data=5;permit->publish(lease);std::this_thread::sleep_for(40ms);
  geometry_msgs::msg::Twist velocity;velocity.linear.x=.12;input->publish(velocity);
  std::this_thread::sleep_for(70ms);EXPECT_DOUBLE_EQ(measured->load(),.12);
  std::this_thread::sleep_for(200ms);EXPECT_DOUBLE_EQ(measured->load(),0);
  permit->publish(lease);std::this_thread::sleep_for(50ms);EXPECT_DOUBLE_EQ(measured->load(),0);
  input->publish(velocity);std::this_thread::sleep_for(50ms);EXPECT_DOUBLE_EQ(measured->load(),.12);
  gate->stop();std::this_thread::sleep_for(50ms);EXPECT_DOUBLE_EQ(measured->load(),0);
  exec.cancel();thread.join();exec.remove_node(gate);
}

TEST_F(Nav012Regression, RecoverySpinIsCanceledBeforeLatestGoalTakesOver)
{
  using namespace carcar_navigation;
  FakeAction<nav2_msgs::action::ComputePathToPose> planner(provider,"/compute_path_to_pose");
  planner.populate=[](auto handle,auto result) {
    result->path.header.frame_id="map";auto start=handle->get_goal()->goal;
    start.pose.position.x=start.pose.position.y=0;result->path.poses={start,handle->get_goal()->goal};
  };
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");follow.abort_first=2;
  FakeAction<nav2_msgs::action::Spin> spin(provider,"/spin");spin.drop_result=true;
  FakeAction<nav2_msgs::action::BackUp> backup(provider,"/backup");
  static_tf();SimInputs inputs(provider);
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_rate_controller_bt_node.so");
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
  goal.pose.position.x=goal.pose.position.y=1;goal.pose.orientation.w=1;bb->set("goal",goal);
  auto tree=factory.createTreeFromFile(std::string(BEHAVIOR_TREES_DIR)+"/navigate_to_pose_experimental_recovery.xml",bb);
  auto end=after(15);auto status=BT::NodeStatus::RUNNING;
  while (spin.accepted==0 && status==BT::NodeStatus::RUNNING && Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(spin.accepted,1);auto runtime=RecoveryRuntime::get(configuration());runtime->backup_used=.3;
  goal.pose.orientation.z=1;goal.pose.orientation.w=0;bb->set("goal",goal);inputs.uuid=2;
  for (int i=0;i<10;++i) {tree.tickRoot();std::this_thread::sleep_for(10ms);}
  inputs.uuid=3; // 相同位置和朝向再次提交；同一停止屏障内只保留最新任务。
  end=after(6);
  while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
    if (follow.accepted>2) {EXPECT_EQ(spin.canceled,1);}
  }
  EXPECT_EQ(status,BT::NodeStatus::SUCCESS);EXPECT_EQ(spin.canceled,1);
  EXPECT_EQ(follow.accepted,3);EXPECT_NEAR(runtime->backup_used,.3,1e-8);
  EXPECT_EQ(runtime->goal_uuid(false).substr(0,2),"03");
  tree.haltTree();exec.cancel();thread.join();
}

TEST_F(Nav012Regression, TerminalFailureDoesNotPoisonNextSingleGoalTask)
{
  using namespace carcar_navigation;
  static_tf();SimInputs inputs(provider);
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
  goal.pose.position.x=1;goal.pose.orientation.w=1;bb->set("goal",goal);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<RecoverySupervisor goal='{goal}'><AlwaysSuccess/><AlwaysFailure/>"
    "</RecoverySupervisor></BehaviorTree></root>",bb);
  auto run=[&] {
    auto status=BT::NodeStatus::RUNNING;auto end=after(4);
    while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {
      status=tree.tickRoot();std::this_thread::sleep_for(10ms);
    }
    return status;
  };
  ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
  tree.haltTree();
  auto runtime=RecoveryRuntime::get(configuration());
  runtime->fail("RECOVERY_EXHAUSTED");runtime->backup_used=.4;runtime->spin_used=true;
  runtime->observe_replans_used=1;runtime->recovery_active=true;
  inputs.uuid=2;
  ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(runtime->fault);EXPECT_TRUE(runtime->replan_required);
  EXPECT_DOUBLE_EQ(runtime->backup_used,0);EXPECT_FALSE(runtime->spin_used);
  EXPECT_EQ(runtime->observe_replans_used,0u);EXPECT_FALSE(runtime->recovery_active);
  EXPECT_EQ(runtime->goal_uuid(false).substr(0,2),"02");
  tree.haltTree();exec.cancel();thread.join();
}

TEST_F(Nav012Regression, TerminalFailureDoesNotPoisonNextThroughPosesTask)
{
  using namespace carcar_navigation;
  static_tf();SimInputs inputs(provider,true);
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
  goal.pose.position.x=1;goal.pose.orientation.w=1;
  bb->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{goal});
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<RecoverySupervisor goals='{goals}' through_poses='true'><AlwaysSuccess/><AlwaysFailure/>"
    "</RecoverySupervisor></BehaviorTree></root>",bb);
  auto run=[&] {
    auto status=BT::NodeStatus::RUNNING;auto end=after(4);
    while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {
      status=tree.tickRoot();std::this_thread::sleep_for(10ms);
    }
    return status;
  };
  ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
  tree.haltTree();
  auto runtime=RecoveryRuntime::get(configuration());
  runtime->fail("RECOVERY_TOTAL_TIMEOUT");runtime->backup_used=.4;
  inputs.uuid=2;
  ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(runtime->fault);EXPECT_DOUBLE_EQ(runtime->backup_used,0);
  EXPECT_EQ(runtime->goal_uuid(true).substr(0,2),"02");
  tree.haltTree();exec.cancel();thread.join();
}

TEST_F(Nav012Regression, CancelFailureClosesGateAndPreventsNewMotion)
{
  using namespace carcar_navigation;
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");
  follow.drop_result=true;follow.drop_cancel=true;
  static_tf();SimInputs inputs(provider);auto gate=std::make_shared<NavigationMotionGate>();exec.add_node(gate);
  auto latest=std::make_shared<std::atomic<double>>(0);
  auto sub=provider->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(1),
    [latest](geometry_msgs::msg::Twist::ConstSharedPtr msg) {latest->store(msg->linear.x);});
  auto publisher=provider->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed_raw",rclcpp::QoS(1));
  auto timer=provider->create_wall_timer(20ms,[publisher] {
    geometry_msgs::msg::Twist v;v.linear.x=.12;publisher->publish(v); // 模拟取消失败仍不断发速的服务器。
  });
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=1;goal.pose.orientation.w=1;
  nav_msgs::msg::Path path;path.header=goal.header;path.poses={goal};bb->set("goal",goal);bb->set("path",path);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><RecoverySupervisor goal='{goal}'>"
    "<SafeFollowPath path='{path}'/><AlwaysFailure/></RecoverySupervisor></BehaviorTree></root>",bb);
  auto end=after(4);
  while (latest->load()==0 && Steady::now()<end) {tree.tickRoot();std::this_thread::sleep_for(10ms);}
  ASSERT_GT(latest->load(),0);
  goal.pose.orientation.z=1;goal.pose.orientation.w=0;bb->set("goal",goal);inputs.uuid=2;
  auto status=BT::NodeStatus::RUNNING;auto start=Steady::now();end=after(2);
  while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
    if (seconds(start)>.1) {EXPECT_DOUBLE_EQ(latest->load(),0);}
  }
  EXPECT_EQ(status,BT::NodeStatus::FAILURE);EXPECT_EQ(follow.accepted,1);
  auto runtime=RecoveryRuntime::get(configuration());EXPECT_TRUE(runtime->fault);
  EXPECT_EQ(runtime->fault_reason,"CANCEL_UNCONFIRMED");
  tree.haltTree();timer->cancel();exec.cancel();thread.join();exec.remove_node(gate);
}

TEST_F(Nav012Regression, LateResultFromPreviousRevisionCannotCompleteNewAction)
{
  using Spin=nav2_msgs::action::Spin;
  FakeAction<Spin> server(provider,"/revised_spin");server.finish_after=.3;
  auto session=std::make_shared<carcar_navigation::BoundedAction<Spin>>("test","/revised_spin",configuration(),1,1);
  Spin::Goal goal;goal.target_yaw=.3;session->send(goal,2);
  std::this_thread::sleep_for(200ms);session->send(goal,2);
  std::this_thread::sleep_for(170ms);session->poll();
  EXPECT_EQ(server.finished,1);EXPECT_FALSE(session->state().terminal);EXPECT_FALSE(session->state().success);
  std::this_thread::sleep_for(220ms);session->poll();EXPECT_TRUE(session->state().terminal);
  EXPECT_TRUE(session->state().success);
}

TEST_F(Nav012Regression, LoggerProcessKeepsNewGoalStateWhenOldAbortArrives)
{
  struct Process {
    pid_t pid{-1};
    ~Process() {
      if (pid<=0) {return;}
      kill(pid,SIGINT);
      for (int i=0;i<100;++i) {
        if (waitpid(pid,nullptr,WNOHANG)==pid) {return;}
        std::this_thread::sleep_for(20ms);
      }
      kill(pid,SIGTERM);waitpid(pid,nullptr,0);
    }
  } process;
  const auto directory=std::filesystem::temp_directory_path()/
    ("nav012_logger_"+std::to_string(getpid()));
  std::filesystem::create_directories(directory);
  std::string executable=NAV_LOGGER_EXECUTABLE;
  std::string sessions="sessions_base_dir:="+directory.string();
  std::vector<std::string> arguments={executable,"--ros-args","-p",sessions,"-p","record_raw_bag:=false"};
  std::vector<char*> argv;for (auto & argument : arguments) {argv.push_back(argument.data());}argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions,STDOUT_FILENO,(directory/"console.log").c_str(),O_WRONLY|O_CREAT|O_TRUNC,0600);
  posix_spawn_file_actions_adddup2(&actions,STDOUT_FILENO,STDERR_FILENO);
  int result=posix_spawn(&process.pid,executable.c_str(),&actions,nullptr,argv.data(),environ);
  posix_spawn_file_actions_destroy(&actions);ASSERT_EQ(result,0);
  struct Observed {std::mutex mutex;std::map<std::string,std::string> values;};
  auto observed=std::make_shared<Observed>();
  auto status=provider->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/navigation/status",10,
    [observed](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(observed->mutex);
      for (const auto & status : msg->status) {for (const auto & kv : status.values) {observed->values[kv.key]=kv.value;}}
    });
  auto publisher=provider->create_publisher<action_msgs::msg::GoalStatusArray>(
    "/navigate_to_pose/_action/status",rclcpp::QoS(10).transient_local());
  action_msgs::msg::GoalStatus old,newer;old.goal_info.goal_id.uuid[0]=1;old.goal_info.stamp.sec=1;old.status=2;
  newer=old;newer.goal_info.goal_id.uuid[0]=2;newer.goal_info.stamp.sec=2;
  auto wait_uuid=[&](const std::string & expected,const action_msgs::msg::GoalStatusArray & message) {
    auto end=carcar_navigation::after(4);
    while (carcar_navigation::Steady::now()<end) {
      publisher->publish(message);std::this_thread::sleep_for(60ms);
      std::lock_guard<std::mutex> lock(observed->mutex);
      if (observed->values["full_goal_uuid"]==expected) {return true;}
    }
    return false;
  };
  action_msgs::msg::GoalStatusArray message;message.status_list={old};
  ASSERT_TRUE(wait_uuid(carcar_navigation::goal_uuid_text(old.goal_info.goal_id),message));
  message.status_list={newer,old};
  ASSERT_TRUE(wait_uuid(carcar_navigation::goal_uuid_text(newer.goal_info.goal_id),message));
  old.status=6;message.status_list={newer,old};
  for (int i=0;i<12;++i) {publisher->publish(message);std::this_thread::sleep_for(60ms);}
  std::lock_guard<std::mutex> lock(observed->mutex);
  EXPECT_EQ(observed->values["full_goal_uuid"],carcar_navigation::goal_uuid_text(newer.goal_info.goal_id));
  EXPECT_NE(observed->values["stage_code"],"TASK_FAILED");
  EXPECT_NE(observed->values["stop_category"],"NAVIGATION_FAILED");
  EXPECT_NE(observed->values["task_status"],"ABORTED");
}

TEST_F(Nav012Regression, FullTreeUsesBackupThenSpinDuringCooldown)
{
  using namespace carcar_navigation;
  FakeAction<nav2_msgs::action::ComputePathToPose> planner(provider,"/compute_path_to_pose");
  planner.populate=[](auto handle,auto result) {
    result->path.header.frame_id="map";auto start=handle->get_goal()->goal;
    start.pose.position.x=start.pose.position.y=0;result->path.poses={start,handle->get_goal()->goal};
  };
  // 首次跟随失败后倒车一段；冷却期内再次失败改走转向，不再立刻倒第二次。
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");follow.abort_first=3;
  FakeAction<nav2_msgs::action::Spin> spin(provider,"/spin");
  FakeAction<nav2_msgs::action::BackUp> backup(provider,"/backup");
  static_tf();SimInputs inputs(provider);inputs.rear_obstacle=false;
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
  factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_rate_controller_bt_node.so");
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
  goal.pose.position.x=goal.pose.position.y=1;goal.pose.orientation.w=1;bb->set("goal",goal);
  auto tree=factory.createTreeFromFile(std::string(BEHAVIOR_TREES_DIR)+"/navigate_to_pose_experimental_recovery.xml",bb);
  auto end=after(20);auto status=BT::NodeStatus::RUNNING;
  while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
    if (spin.accepted>0) {EXPECT_EQ(backup.finished,1);}
  }
  EXPECT_EQ(status,BT::NodeStatus::SUCCESS);EXPECT_EQ(backup.accepted,1);EXPECT_EQ(spin.accepted,1);
  EXPECT_EQ(follow.accepted,4);EXPECT_NEAR(RecoveryRuntime::get(configuration())->backup_used,.2,1e-8);
  tree.haltTree();exec.cancel();thread.join();
}

TEST_F(Nav012Regression, MissingServerAndDelayedCancelAreBounded)
{
  using namespace carcar_navigation;
  static_tf();geometry_msgs::msg::PoseStamped point;point.header.frame_id="map";
  point.pose.position.y=1;point.pose.orientation.w=1;nav_msgs::msg::Path path;path.header=point.header;path.poses={point};
  bb->set("path",path);
  auto tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<ControlledSpin path='{path}'/></BehaviorTree></root>",bb);
  std::this_thread::sleep_for(200ms);auto start=Steady::now();auto end=after(2);
  auto status=BT::NodeStatus::RUNNING;
  while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {status=tree.tickRoot();std::this_thread::sleep_for(10ms);}
  EXPECT_EQ(status,BT::NodeStatus::FAILURE);EXPECT_LT(seconds(start),1.5);tree.haltTree();
  FakeAction<nav2_msgs::action::Spin> server(provider,"/delayed_cancel");server.drop_result=true;server.cancel_delay_ms=300;
  auto session=std::make_shared<BoundedAction<nav2_msgs::action::Spin>>("bounded","/delayed_cancel",configuration(),1,.1);
  nav2_msgs::action::Spin::Goal goal;goal.target_yaw=.5;session->send(goal,2);
  end=after(1);while (!session->state().accepted && Steady::now()<end) {std::this_thread::sleep_for(10ms);}
  ASSERT_TRUE(session->state().accepted);session->cancel();std::this_thread::sleep_for(180ms);session->poll();
  EXPECT_EQ(session->state().reason,"CANCEL_UNCONFIRMED");EXPECT_FALSE(session->state().cancel_ack);
  std::this_thread::sleep_for(250ms);session->poll();EXPECT_TRUE(session->state().terminal);
}

TEST_F(Nav012Regression, SensorLossClosesMotionAndMissingOdometryPreventsHandover)
{
  using namespace carcar_navigation;
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");follow.drop_result=true;
  static_tf();SimInputs inputs(provider);
  auto lease=std::make_shared<std::atomic<uint64_t>>(0);
  auto subscription=provider->create_subscription<std_msgs::msg::UInt64>("/navigation/motion_permit",rclcpp::QoS(1),
    [lease](std_msgs::msg::UInt64::ConstSharedPtr msg) {lease->store(msg->data);});
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=1;goal.pose.orientation.w=1;
  nav_msgs::msg::Path path;path.header=goal.header;path.poses={goal};bb->set("goal",goal);bb->set("path",path);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><RecoverySupervisor goal='{goal}'>"
    "<SafeFollowPath path='{path}'/><AlwaysFailure/></RecoverySupervisor></BehaviorTree></root>",bb);
  auto end=after(4);while (lease->load()==0 && Steady::now()<end) {tree.tickRoot();std::this_thread::sleep_for(10ms);}
  ASSERT_NE(lease->load(),0u);inputs.enabled=false;
  auto start=Steady::now();auto status=BT::NodeStatus::RUNNING;end=after(4);
  while (status==BT::NodeStatus::RUNNING && Steady::now()<end) {
    status=tree.tickRoot();std::this_thread::sleep_for(10ms);
    if (seconds(start)>.7) {EXPECT_EQ(lease->load(),0u);}
  }
  EXPECT_EQ(status,BT::NodeStatus::FAILURE);EXPECT_EQ(follow.accepted,1);EXPECT_EQ(follow.canceled,1);
  EXPECT_EQ(RecoveryRuntime::get(configuration())->fault_reason,"STOP_NOT_CONFIRMED");
  tree.haltTree();exec.cancel();thread.join();
}

TEST_F(Nav012Regression, FollowPathStampOnlyUpdateDoesNotOrphanLiveGoal)
{
  using namespace carcar_navigation;
  FakeAction<nav2_msgs::action::FollowPath> follow(provider,"/follow_path");
  follow.drop_result=true; follow.finish_after=100;
  static_tf();SimInputs inputs(provider);
  auto lease=std::make_shared<std::atomic<uint64_t>>(0);
  auto subscription=provider->create_subscription<std_msgs::msg::UInt64>("/navigation/motion_permit",rclcpp::QoS(1),
    [lease](std_msgs::msg::UInt64::ConstSharedPtr msg) {lease->store(msg->data);});
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
  goal.pose.position.x=1;goal.pose.orientation.w=1;
  nav_msgs::msg::Path path;path.header=goal.header;path.header.stamp=provider->now();
  path.poses={goal};bb->set("goal",goal);bb->set("path",path);
  auto tree=factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><RecoverySupervisor goal='{goal}'>"
    "<SafeFollowPath path='{path}'/><AlwaysFailure/></RecoverySupervisor></BehaviorTree></root>",bb);
  auto end=after(4);
  while ((follow.accepted<1 || lease->load()==0) && Steady::now()<end) {
    tree.tickRoot();std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(follow.accepted,1);ASSERT_NE(lease->load(),0u);
  auto runtime=RecoveryRuntime::get(configuration());
  end=after(2.3);
  while (Steady::now()<end) {
    tree.tickRoot();std::this_thread::sleep_for(10ms);
    ASSERT_TRUE(runtime->motion);
    EXPECT_NE(runtime->motion->state().reason,"PREVIOUS_RESULT_MISSING");
    EXPECT_FALSE(runtime->motion->state().timed_out);
    EXPECT_FALSE(runtime->motion->state().cancel_requested);
    EXPECT_EQ(follow.canceled,0);
  }
  EXPECT_TRUE(runtime->motion->state().accepted);
  path.header.stamp=provider->now();bb->set("path",path);
  for (int i=0;i<20;++i) {tree.tickRoot();std::this_thread::sleep_for(10ms);}
  EXPECT_EQ(follow.accepted,1);EXPECT_EQ(follow.canceled,0);
  ASSERT_TRUE(runtime->motion);
  EXPECT_NE(runtime->motion->state().reason,"PREVIOUS_RESULT_MISSING");
  EXPECT_TRUE(runtime->motion->state().accepted);
  EXPECT_FALSE(runtime->motion->state().timed_out);
  EXPECT_NE(lease->load(),0u);
  goal.pose.position.x=1.4;path.poses={goal};path.header.stamp=provider->now();bb->set("path",path);
  end=after(2);
  while (follow.accepted<2 && Steady::now()<end) {tree.tickRoot();std::this_thread::sleep_for(10ms);}
  EXPECT_EQ(follow.accepted,2);
  ASSERT_TRUE(runtime->motion);
  EXPECT_NE(runtime->motion->state().reason,"PREVIOUS_RESULT_MISSING");
  EXPECT_TRUE(runtime->motion->state().accepted);
  EXPECT_FALSE(runtime->motion->state().cancel_requested);
  EXPECT_NE(lease->load(),0u);
  tree.haltTree();exec.cancel();thread.join();
}
