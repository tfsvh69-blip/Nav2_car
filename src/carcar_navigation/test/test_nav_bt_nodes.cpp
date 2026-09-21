// NAV-009: 行为树节点与 XML 回归测试
// 验证：
// 1. 动态加载 libcarcar_nav_bt_nodes.so 插件并验证所有自定义节点注册与端口定义；
// 2. 四棵行为树 XML 实际实例化，检查节点拓扑与端口无错；
// 3. WaitForLocalizationStatus 五态测试（健康正常、不健康、延迟到达、断流超时、等待取消）；
// 4. Groot 显示桥接拓扑兼容性（四棵树均能被 Mirror 工厂无异常加载）。

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"

using namespace std::chrono_literals;

#ifndef BEHAVIOR_TREES_DIR
#define BEHAVIOR_TREES_DIR "/home/jetson/luhao/my_nav_carcar/src/carcar_navigation/behavior_trees"
#endif

#ifndef CARCAR_NAV_BT_PLUGIN_PATH
#define CARCAR_NAV_BT_PLUGIN_PATH "/home/jetson/luhao/my_nav_carcar/install/carcar_navigation/lib/libcarcar_nav_bt_nodes.so"
#endif

namespace {
// 通用 Mock 节点，供测试中实例化完整 Nav2 行为树使用
BT::PortsList mock_ports() {
  return {
    BT::InputPort<std::string>("goal"),
    BT::InputPort<std::string>("goals"),
    BT::InputPort<std::string>("path"),
    BT::InputPort<std::string>("planner_id"),
    BT::InputPort<std::string>("controller_id"),
    BT::InputPort<std::string>("goal_checker_id"),
    BT::InputPort<std::string>("through_poses"),
    BT::InputPort<std::string>("input_goals"),BT::InputPort<std::string>("output_goals"),
    BT::InputPort<std::string>("radius"),BT::InputPort<std::string>("robot_base_frame"),
    BT::InputPort<std::string>("costmap_topic"),
    BT::InputPort<std::string>("scan_topic"),
    BT::InputPort<std::string>("nomotion_service"),
    BT::InputPort<std::string>("global_service"),
    BT::InputPort<std::string>("hz"),
    BT::InputPort<std::string>("seconds"),
    BT::InputPort<std::string>("server_timeout"),
    BT::InputPort<std::string>("timeout"),
    BT::InputPort<std::string>("wait_duration"),
    BT::InputPort<std::string>("spin_dist"),
    BT::InputPort<std::string>("time_allowance"),
    BT::InputPort<std::string>("backup_dist"),
    BT::InputPort<std::string>("backup_speed"),
    BT::InputPort<std::string>("backup_distance"),
    BT::OutputPort<std::string>("selected_distance"),
    BT::InputPort<std::string>("max_data_age"),
    BT::InputPort<std::string>("local_timeout"),
    BT::InputPort<std::string>("global_timeout"),
    BT::InputPort<std::string>("number_of_retries"),
    BT::InputPort<std::string>("linear_stagnation_timeout"),
    BT::InputPort<std::string>("linear_displacement_threshold"),
    BT::InputPort<std::string>("angular_stagnation_timeout"),
    BT::InputPort<std::string>("angular_convergence_threshold"),
    BT::InputPort<std::string>("max_rotation_budget"),
    BT::InputPort<std::string>("heading_flip_threshold"),
    BT::InputPort<std::string>("max_heading_flips"),
    BT::InputPort<std::string>("max_spin_angle"),
    BT::InputPort<std::string>("observe_duration")
  };
}

class MockAction : public BT::SyncActionNode {
public:
  MockAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return mock_ports(); }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

class MockCondition : public BT::ConditionNode {
public:
  MockCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() { return mock_ports(); }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

class MockControl : public BT::ControlNode {
public:
  MockControl(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}
  static BT::PortsList providedPorts() { return mock_ports(); }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  void halt() override { haltChildren(); }
};

class MockDecorator : public BT::DecoratorNode {
public:
  MockDecorator(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}
  static BT::PortsList providedPorts() { return mock_ports(); }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

void register_nav2_standard_mocks(BT::BehaviorTreeFactory & factory) {
  for (const auto & tag : {
      "ComputePathToPose", "ComputePathThroughPoses", "FollowPath", "Wait", "Spin", "BackUp", "RemovePassedGoals"}) {
    factory.registerNodeType<MockAction>(tag);
  }
  for (const auto & tag : {"GoalUpdated", "GlobalUpdatedGoal", "IsPathValid", "PathExpiringTimer"}) {
    factory.registerNodeType<MockCondition>(tag);
  }
  for (const auto & tag : {"PipelineSequence", "RecoveryNode", "RoundRobin"}) {
    factory.registerNodeType<MockControl>(tag);
  }
  factory.registerNodeType<MockDecorator>("RateController");
}

}  // namespace

class NavBtNodesTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    test_node_ = std::make_shared<rclcpp::Node>("test_nav_bt_node_" + std::to_string(std::rand() % 100000));
  }

  rclcpp::Node::SharedPtr test_node_;
};

// ── 测试 1：动态插件注册与行为树实例化 ──────────────────────────────────────
TEST_F(NavBtNodesTest, TestPluginAndAllTreesLoading) {
  BT::BehaviorTreeFactory factory;

  // 1. 动态加载 carcar_nav_bt_nodes 插件
  std::string plugin_path = CARCAR_NAV_BT_PLUGIN_PATH;
  EXPECT_NO_THROW({
    factory.registerFromPlugin(plugin_path);
  });

  // 注册 Nav2 内置标准节点 Mock
  register_nav2_standard_mocks(factory);

  // 2. 检查自定义节点类型是否均已注册
  const auto & registered = factory.manifests();
  EXPECT_TRUE(registered.find("WaitForLocalizationStatus") != registered.end());
  EXPECT_TRUE(registered.find("LocalizationHealthy") != registered.end());
  EXPECT_TRUE(registered.find("RecoverLocalization") != registered.end());
  EXPECT_TRUE(registered.find("RearClear") != registered.end());
  EXPECT_TRUE(registered.find("ReplanNotRequired") != registered.end());

  // 3. 逐个实例化四棵行为树
  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  const std::vector<std::string> tree_files = {
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_to_pose_no_recovery.xml",
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_through_poses_no_recovery.xml",
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_to_pose_experimental_recovery.xml",
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_through_poses_experimental_recovery.xml"
  };

  for (const auto & file_path : tree_files) {
    std::ifstream f(file_path);
    ASSERT_TRUE(f.good()) << "行为树文件不存在: " << file_path;
    EXPECT_NO_THROW({
      auto tree = factory.createTreeFromFile(file_path, bb);
      EXPECT_FALSE(tree.nodes.empty()) << "解析出空行为树: " << file_path;
    }) << "行为树 XML 实例化失败: " << file_path;
  }
}

// ── 测试 2：WaitForLocalizationStatus 健康正常信号（返回 SUCCESS） ───────────
TEST_F(NavBtNodesTest, TestWaitForLocalizationHealthy) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);

  auto pub = test_node_->create_publisher<std_msgs::msg::Bool>(
      "/localization_monitor/ready", rclcpp::QoS(1).reliable().transient_local());

  std_msgs::msg::Bool msg;
  msg.data = true;
  pub->publish(msg);

  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["timeout"] = "2.0";

  auto node = factory.instantiateTreeNode("wait_healthy_node", "WaitForLocalizationStatus", config);
  ASSERT_NE(node, nullptr);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 25; ++i) {
    status = node->executeTick();
    if (status == BT::NodeStatus::SUCCESS) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ── 测试 3：WaitForLocalizationStatus 不健康信号（立即返回 FAILURE） ─────────
TEST_F(NavBtNodesTest, TestWaitForLocalizationUnhealthyImmediateFailure) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);

  auto pub = test_node_->create_publisher<std_msgs::msg::Bool>(
      "/localization_monitor/ready", rclcpp::QoS(1).reliable().transient_local());

  std_msgs::msg::Bool msg;
  msg.data = false;
  pub->publish(msg);

  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["timeout"] = "2.0";

  auto node = factory.instantiateTreeNode("wait_unhealthy_node", "WaitForLocalizationStatus", config);
  ASSERT_NE(node, nullptr);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 25; ++i) {
    status = node->executeTick();
    if (status == BT::NodeStatus::FAILURE) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ── 测试 4：WaitForLocalizationStatus 延迟信号（初始 RUNNING，到达后 SUCCESS）
TEST_F(NavBtNodesTest, TestWaitForLocalizationDelayedMessage) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);

  auto pub = test_node_->create_publisher<std_msgs::msg::Bool>(
      "/localization_monitor/ready", rclcpp::QoS(1).reliable().transient_local());

  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["timeout"] = "2.0";

  auto node = factory.instantiateTreeNode("wait_delayed_node", "WaitForLocalizationStatus", config);
  ASSERT_NE(node, nullptr);

  // 1. 无消息时执行，预期返回 RUNNING
  BT::NodeStatus status = node->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // 2. 延迟发布健康信号
  std_msgs::msg::Bool msg;
  msg.data = true;
  pub->publish(msg);

  std::this_thread::sleep_for(150ms);

  // 3. 再次 tick，预期收到后转为 SUCCESS
  status = node->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ── 测试 5：WaitForLocalizationStatus 超时无消息（返回 FAILURE） ─────────────
TEST_F(NavBtNodesTest, TestWaitForLocalizationTimeoutFailure) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);

  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["timeout"] = "0.2";  // 0.2 秒短超时加速测试

  auto node = factory.instantiateTreeNode("wait_timeout_node", "WaitForLocalizationStatus", config);
  ASSERT_NE(node, nullptr);

  BT::NodeStatus status = node->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // 等待超过 0.2 秒
  std::this_thread::sleep_for(250ms);

  status = node->executeTick();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ── 测试 6：WaitForLocalizationStatus 等待取消（halt 干净重置） ───────────────
TEST_F(NavBtNodesTest, TestWaitForLocalizationHalt) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);

  std::string xml =
    "<root main_tree_to_execute=\"Main\">"
    "  <BehaviorTree ID=\"Main\">"
    "    <WaitForLocalizationStatus name=\"wait_halt\" timeout=\"2.0\"/>"
    "  </BehaviorTree>"
    "</root>";

  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  auto tree = factory.createTreeFromText(xml, bb);
  BT::NodeStatus status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // 外部中断取消：tree.haltTree() 触发 onHalted() 并复位为 IDLE
  EXPECT_NO_THROW({
    tree.haltTree();
  });
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}

// ── 测试 7：Groot 显示桥接工厂四棵树兼容性 ──────────────────────────────────
TEST_F(NavBtNodesTest, TestGrootDisplayBridgeCompatibility) {
  BT::BehaviorTreeFactory factory;

  // 严格模拟 bt_monitor_node.cpp 中的注册逻辑
  for (const auto & tag : {
      "ComputePathToPose", "ComputePathThroughPoses", "FollowPath", "Wait", "Spin", "BackUp",
      "RecoverLocalization", "WaitForLocalizationStatus", "ProtectedBackUp", "ControlledSpin", "ParkAndObserve",
      "SafeFollowPath", "SafeBackUp", "SafeComputePathToPose", "SafeComputePathThroughPoses", "RemovePassedGoals"}) {
    factory.registerNodeType<MockAction>(tag);
  }
  for (const auto & tag : {"GoalUpdated", "GlobalUpdatedGoal", "IsPathValid", "PathExpiringTimer",
      "LocalizationHealthy", "RearClear", "RecoveryInputsReady", "ReplanNotRequired"}) {
    factory.registerNodeType<MockCondition>(tag);
  }
  for (const auto & tag : {"PipelineSequence", "RecoveryNode", "RoundRobin", "RecoverySupervisor"}) {
    factory.registerNodeType<MockControl>(tag);
  }
  for (const auto & tag : {"RateController", "ProgressGuard"}) {
    factory.registerNodeType<MockDecorator>(tag);
  }

  const std::vector<std::string> tree_files = {
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_to_pose_no_recovery.xml",
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_through_poses_no_recovery.xml",
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_to_pose_experimental_recovery.xml",
    std::string(BEHAVIOR_TREES_DIR) + "/navigate_through_poses_experimental_recovery.xml"
  };

  for (const auto & file_path : tree_files) {
    EXPECT_NO_THROW({
      auto tree = factory.createTreeFromFile(file_path);
      EXPECT_FALSE(tree.nodes.empty());
    }) << "Groot 桥接显示树载入失败: " << file_path;
  }
}

// ── 测试 8：RearClear 结构化拒绝原因与放行判定 ────────────────────────────────
TEST_F(NavBtNodesTest, TestRearClearStructuredRejections) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(CARCAR_NAV_BT_PLUGIN_PATH);

  auto bb = BT::Blackboard::create();
  bb->set("node", test_node_);

  const std::string costmap_topic = "/test_rc_costmap_" + std::to_string(std::rand() % 100000);
  const std::string scan_topic = "/test_rc_scan_" + std::to_string(std::rand() % 100000);

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["costmap_topic"] = costmap_topic;
  config.input_ports["scan_topic"] = scan_topic;
  config.input_ports["max_data_age"] = "0.5";
  config.input_ports["backup_distance"] = "0.10";

  auto node = factory.instantiateTreeNode("test_rear_clear", "RearClear", config);
  ASSERT_NE(node, nullptr);

  // 1. 无数据时 tick -> 预期返回 FAILURE (缺少数据)
  EXPECT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);

  // 建立发布者
  auto costmap_pub = test_node_->create_publisher<nav2_msgs::msg::Costmap>(
    costmap_topic, rclcpp::QoS(1).reliable().transient_local());
  auto scan_pub = test_node_->create_publisher<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS());
  auto footprint_pub = test_node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/local_costmap/published_footprint", rclcpp::QoS(1));
  geometry_msgs::msg::PolygonStamped footprint;
  footprint.header.frame_id="base_footprint";
  for (auto xy : {std::pair<float,float>{-.14F,-.13F},{-.14F,.13F},{.14F,.13F},{.14F,-.13F}}) {
    geometry_msgs::msg::Point32 p;p.x=xy.first;p.y=xy.second;footprint.polygon.points.push_back(p);
  }
  auto publish_footprint = [&] {
    footprint.header.stamp=test_node_->now();footprint_pub->publish(footprint);
  };

  // 广播 map -> base_footprint 静态 TF
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = test_node_->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_footprint";
  tf_msg.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(tf_msg);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(test_node_);

  // 2. 发布过期数据 (时间戳为 10 秒前) -> 预期 FAILURE (数据过期)
  {
    publish_footprint();
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id="base_footprint";scan.range_min=.05;scan.range_max=10;
    scan.angle_increment=.01;scan.ranges.assign(629,5);
    scan.header.stamp = test_node_->now() - rclcpp::Duration::from_seconds(10.0);
    scan_pub->publish(scan);

    nav2_msgs::msg::Costmap cm;
    cm.header.stamp = test_node_->now() - rclcpp::Duration::from_seconds(10.0);
    cm.header.frame_id = "map";
    cm.metadata.resolution = 0.05F;
    cm.metadata.size_x = 40;
    cm.metadata.size_y = 40;
    cm.metadata.origin.position.x = -1.0;
    cm.metadata.origin.position.y = -1.0;
    cm.data.assign(40 * 40, 0);
    costmap_pub->publish(cm);

    executor->spin_some();
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
  }

  // 3. 发布新鲜数据，但回退路径存在障碍物 (254) -> 预期 FAILURE (障碍占用)
  {
    publish_footprint();
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id="base_footprint";scan.range_min=.05;scan.range_max=10;
    scan.angle_increment=.01;scan.ranges.assign(629,5);
    scan.header.stamp = test_node_->now();
    scan_pub->publish(scan);

    nav2_msgs::msg::Costmap cm;
    cm.header.stamp = test_node_->now();
    cm.header.frame_id = "map";
    cm.metadata.resolution = 0.05F;
    cm.metadata.size_x = 40;
    cm.metadata.size_y = 40;
    cm.metadata.origin.position.x = -1.0;
    cm.metadata.origin.position.y = -1.0;
    cm.data.assign(40 * 40, 0);
    // 车体后缘约 x=-0.14。格子 15 覆盖 [-0.25,-0.20]，在当前包络外、0.10/0.20 m 倒车扫掠内。
    cm.data[20 * 40 + 15] = 254;
    costmap_pub->publish(cm);

    executor->spin_some();
    std::this_thread::sleep_for(50ms);
    EXPECT_EQ(node->executeTick(), BT::NodeStatus::FAILURE);
  }

  // 4. 发布新鲜数据，回退路径全空闲 (0) -> 预期 SUCCESS (通过)
  {
    publish_footprint();
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id="base_footprint";scan.range_min=.05;scan.range_max=10;
    scan.angle_increment=.01;scan.ranges.assign(629,5);
    scan.header.stamp = test_node_->now();
    scan_pub->publish(scan);

    nav2_msgs::msg::Costmap cm;
    cm.header.stamp = test_node_->now();
    cm.header.frame_id = "map";
    cm.metadata.resolution = 0.05F;
    cm.metadata.size_x = 40;
    cm.metadata.size_y = 40;
    cm.metadata.origin.position.x = -1.0;
    cm.metadata.origin.position.y = -1.0;
    cm.data.assign(40 * 40, 0);  // 全空闲
    costmap_pub->publish(cm);

    executor->spin_some();
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(node->executeTick(), BT::NodeStatus::SUCCESS);
  }
}
