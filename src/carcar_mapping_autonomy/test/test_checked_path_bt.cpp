#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <thread>

// 使用真正的路径审查插件和DDS输入；只隔离硬件与规划器。
class CheckedPathTree:public testing::Test {
protected:
 rclcpp::Node::SharedPtr node;
 std::shared_ptr<tf2_ros::Buffer> tf;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
 BT::BehaviorTreeFactory factory;BT::Tree tree;BT::Blackboard::Ptr board;
 nav_msgs::msg::OccupancyGrid map;nav_msgs::msg::Path path;
 int plans=0,follows=0,remaining_failures=0;
 static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 void SetUp()override{
   node=std::make_shared<rclcpp::Node>("checked_path_fixture");tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
   const auto qos=rclcpp::QoS(1).reliable().transient_local();
   map_pub=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",qos);
   state_pub=node->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",qos);
   map.header.frame_id="map";map.info.resolution=.05;map.info.width=map.info.height=100;
   map.info.origin.position.x=map.info.origin.position.y=-2.5;map.info.origin.orientation.w=1;map.data.assign(10000,0);
   path.header.frame_id="map";geometry_msgs::msg::PoseStamped p;p.pose.orientation.w=1;path.poses.push_back(p);p.pose.position.x=1;path.poses.push_back(p);
   board=BT::Blackboard::create();board->set("node",node);board->set("tf_buffer",tf);board->set("path",path);
   factory.registerFromPlugin(CHECKED_PATH_PLUGIN);
   tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><CheckedMappingPath path='{path}' checked_path='{checked}'/></BehaviorTree></root>",board);
 }
 void continuous_tree(int failures){
   remaining_failures=failures;path.poses.back().pose.position.x=0;board->set("path",path);
   board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});
   for(const auto*plugin:{"nav2_pipeline_sequence_bt_node","nav2_recovery_node_bt_node"})
     factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
   factory.registerSimpleAction("RemovePassedGoals",[](BT::TreeNode&){return BT::NodeStatus::SUCCESS;},
     {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals"),BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals"),
      BT::InputPort<double>("radius"),BT::InputPort<std::string>("global_frame"),BT::InputPort<std::string>("robot_base_frame")});
   factory.registerSimpleAction("ComputePathThroughPoses",[this](BT::TreeNode&node){
     ++plans;if(remaining_failures-->0)return BT::NodeStatus::FAILURE;
     node.setOutput("path",path);return BT::NodeStatus::SUCCESS;
   },{BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),BT::OutputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("planner_id")});
   factory.registerSimpleAction("FollowPath",[this](BT::TreeNode&){++follows;return BT::NodeStatus::SUCCESS;},
     {BT::InputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("controller_id"),BT::InputPort<std::string>("goal_checker_id")});
   tree=factory.createTreeFromFile(CONTINUOUS_BT_XML,board);
 }
 BT::NodeStatus run(const std::string&phase="EXPLORING",bool stop_on_terminal=false){
   BT::NodeStatus result=BT::NodeStatus::IDLE;
   for(int i=0;i<20;++i){
     geometry_msgs::msg::TransformStamped t;t.header.stamp=node->now();t.header.frame_id="map";t.child_frame_id="base_footprint";t.transform.rotation.w=1;tf->setTransform(t,"fixture");
     map.header.stamp=node->now();map_pub->publish(map);
     std_msgs::msg::String s;s.data="{\"phase\":\""+phase+"\",\"origin\":[0,0,0],\"radius_m\":20}";state_pub->publish(s);
     std::this_thread::sleep_for(std::chrono::milliseconds(25));result=tree.tickRoot();
     if(stop_on_terminal&&result!=BT::NodeStatus::RUNNING)break;
   }
   return result;
 }
};
TEST_F(CheckedPathTree, PublishesOnlyTheActuallyValidatedPath){
 ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 EXPECT_EQ(board->get<nav_msgs::msg::Path>("checked"),path);
 // 实际路径改变，而旧预检路径仍安全；必须拒绝改变后的路径。
 path.poses.back().pose.position.x=4;board->set("path",path);
 EXPECT_EQ(run(),BT::NodeStatus::FAILURE);
 EXPECT_NE(board->get<nav_msgs::msg::Path>("checked"),path);
}
TEST_F(CheckedPathTree, NewObstacleAndPausedTaskRejectExecution){
 ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 map.data[50*100+60]=100;EXPECT_EQ(run(),BT::NodeStatus::FAILURE);
 map.data[50*100+60]=0;ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 EXPECT_EQ(run("PAUSED"),BT::NodeStatus::FAILURE);
}

TEST_F(CheckedPathTree, OldPathSuccessCannotCompleteNewEndpoint){
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});
 factory.registerSimpleAction("FinishedController",[](BT::TreeNode&){return BT::NodeStatus::SUCCESS;});
 tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><ExplorationFollow path='{path}'><FinishedController/></ExplorationFollow></BehaviorTree></root>",board);
 EXPECT_EQ(run(),BT::NodeStatus::RUNNING)<<"旧路径成功且车仍在原地时不能宣布到达一米外的新末点";
 geometry_msgs::msg::TransformStamped t;t.header.stamp=node->now();t.header.frame_id="map";t.child_frame_id="base_footprint";t.transform.translation.x=1;t.transform.rotation.w=1;tf->setTransform(t,"fixture");
 EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);
 auto goal=path.poses.back();goal.pose.position.x=2;board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{goal});
 EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING)<<"新末点尚无通过审查的路径，不能重用旧路径成功结果";
}

TEST_F(CheckedPathTree, GoalUpdateImmediatelyReplansWithoutWaitingForRatePeriod){
 int plans=0;board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});
 factory.registerSimpleAction("PlanCounter",[&](BT::TreeNode&){++plans;return BT::NodeStatus::SUCCESS;});
 tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><ExplorationReplan><PlanCounter/></ExplorationReplan></BehaviorTree></root>",board);
 EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);
 EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);EXPECT_EQ(plans,1);
 auto goal=path.poses.back();goal.pose.position.x=2;board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{goal});
 EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,2);
}
TEST_F(CheckedPathTree, ProductionTreeRetriesATransientPlannerFailureBeforeFollowing){
 continuous_tree(1);const auto began=std::chrono::steady_clock::now();
 EXPECT_EQ(run("EXPLORING",true),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,2);EXPECT_EQ(follows,1);
 EXPECT_GE(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.4);
}
TEST_F(CheckedPathTree, ProductionTreeDoesNotFollowAfterTwoPlannerFailures){
 continuous_tree(20);EXPECT_EQ(run("EXPLORING",true),BT::NodeStatus::FAILURE);EXPECT_EQ(plans,2);EXPECT_EQ(follows,0);
}
TEST_F(CheckedPathTree, ProductionTreeCancellationDuringRetrySubmitsNothingElse){
 continuous_tree(1);ASSERT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);ASSERT_EQ(plans,1);
 tree.haltTree();std::this_thread::sleep_for(std::chrono::milliseconds(450));EXPECT_EQ(plans,1);EXPECT_EQ(follows,0);
}
