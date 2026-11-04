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
 int plans=0,follows=0,remaining_failures=0;uint64_t changing_revision=0;
 std::string state_context=",\"task_id\":\"bt-task\",\"navigation_generation\":1,\"map_revision\":1,\"handoff\":{\"distance_m\":0.25}";
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
     if(changing_revision)state_context=",\"task_id\":\"bt-task\",\"navigation_generation\":1,\"map_revision\":"+std::to_string(changing_revision++)+",\"handoff\":{\"distance_m\":0.25}";
     std_msgs::msg::String s;s.data="{\"phase\":\""+phase+"\",\"origin\":[0,0,0],\"radius_m\":20"+state_context+"}";state_pub->publish(s);
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

TEST_F(CheckedPathTree, OldBlackboardPathCannotSkipNewGoalsBeforeTheyAreChecked){
 auto intermediate=path.poses.front();intermediate.pose.position.x=.20;
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{intermediate,path.poses.back()});
 // 旧路径恰好经过新首点，但新目标尚未获得本轮实际路径审查资格。
 tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><MappingPassedGoals input_goals='{goals}' output_goals='{goals}' path='{path}'/></BehaviorTree></root>",board);
 EXPECT_EQ(run(),BT::NodeStatus::SUCCESS);
 const auto remaining=board->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
 ASSERT_EQ(remaining.size(),2U)<<"新目标不能凭上一条路径先删掉首点，再去规划剩余点";
 EXPECT_EQ(remaining.front(),intermediate);
}

TEST_F(CheckedPathTree, FreshCheckedPathHandsOffIntermediateAndKeepsLastGoal){
 // 车仍在原点，中间点剩余弧长1.20米，确实使用本轮放宽的0.80米以上范围。
 path.poses.back().pose.position.x=1.80;board->set("path",path);
 auto intermediate=path.poses.front();intermediate.pose.position.x=1.20;
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{intermediate,path.poses.back()});
 state_context=",\"task_id\":\"bt-task\",\"navigation_generation\":1,\"map_revision\":1,\"handoff\":{\"distance_m\":1.50}";
 ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><MappingPassedGoals input_goals='{goals}' output_goals='{goals}' path='{path}'/></BehaviorTree></root>",board);
 ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 const auto remaining=board->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
 ASSERT_EQ(remaining.size(),1U);EXPECT_EQ(remaining.front(),path.poses.back());
}

TEST_F(CheckedPathTree, GoalPathTaskGenerationRevisionAndExpiredProofCannotSkipGoals){
 const auto context=state_context;const auto original_path=path;
 for(const std::string change:{"goals","goal_stamp","path","task","generation","revision","missing_context","expired"}){
   SCOPED_TRACE(change);path=original_path;state_context=context;board->set("path",path);
   auto intermediate=path.poses.front();intermediate.pose.position.x=.20;
   const std::vector<geometry_msgs::msg::PoseStamped>goals={intermediate,path.poses.back()};board->set("goals",goals);
   tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><CheckedMappingPath path='{path}' checked_path='{checked}'/></BehaviorTree></root>",board);
   ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
   if(change=="goals"){auto updated=goals;updated.front().pose.position.x=.21;board->set("goals",updated);}
   if(change=="goal_stamp"){auto updated=goals;updated.front().header.stamp=node->now();board->set("goals",updated);}
   if(change=="path"){auto updated=path;updated.header.stamp=node->now();board->set("path",updated);}
   if(change=="task")state_context=",\"task_id\":\"different-task\",\"navigation_generation\":1,\"map_revision\":1";
   if(change=="generation")state_context=",\"task_id\":\"bt-task\",\"navigation_generation\":2,\"map_revision\":1";
   if(change=="revision")state_context=",\"task_id\":\"bt-task\",\"navigation_generation\":1,\"map_revision\":2";
   if(change=="missing_context")state_context="";
   if(change=="expired")std::this_thread::sleep_for(std::chrono::milliseconds(1550));
   tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><MappingPassedGoals input_goals='{goals}' output_goals='{goals}' path='{path}'/></BehaviorTree></root>",board);
   ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
   EXPECT_EQ(board->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals").size(),2U)
     <<"缺少本次新鲜完整审查资格时，应先规划而非删除中间点";
 }
}

TEST_F(CheckedPathTree, ProductionTreeHandsOffFreshCheckedPathWhileMapRevisionKeepsChanging){
 continuous_tree(0);path.poses.back().pose.position.x=1;board->set("path",path);
 auto intermediate=path.poses.front();intermediate.pose.position.x=.20;
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{intermediate,path.poses.back()});
 changing_revision=2;
 EXPECT_EQ(run(),BT::NodeStatus::RUNNING)<<"末点尚未到达，不能因通过中间点结束任务";
 EXPECT_GT(plans,0);EXPECT_GT(changing_revision,10U);
 const auto remaining=board->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
 ASSERT_EQ(remaining.size(),1U)<<"新版本先规划及审查，再在本轮使用新凭据过点，不能一直等待地图静止";
 EXPECT_EQ(remaining.front(),path.poses.back());
}

TEST_F(CheckedPathTree, ProductionReplanFailureKeepsFreshSafeCommittedRoute){
 continuous_tree(0);path.poses.back().pose.position.x=1;board->set("path",path);
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});
 ASSERT_EQ(run(),BT::NodeStatus::RUNNING);ASSERT_GT(follows,0);
 remaining_failures=20;const auto before=follows;
 std::this_thread::sleep_for(std::chrono::milliseconds(650));
 EXPECT_EQ(run("EXPLORING",true),BT::NodeStatus::RUNNING)
   <<"重规划失败不能直接终止仍通过最新地图审查的同一条执行路线";
 EXPECT_GT(follows,before);
}

TEST_F(CheckedPathTree, RetainedRouteRejectsNewObstacleNewGoalsAndChangedContext){
 const auto context=state_context;
 for(const std::string change:{"obstacle","goals","goal_stamp","task","generation","missing_context","paused"}){
   SCOPED_TRACE(change);state_context=context;map.data.assign(10000,0);
   board->set("path",path);board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});
   tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><CheckedMappingPath path='{path}' checked_path='{checked}'/></BehaviorTree></root>",board);
   ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
   if(change=="obstacle")map.data[50*100+60]=100;
   if(change=="goals"||change=="goal_stamp"){auto goal=path.poses.back();if(change=="goals")goal.pose.position.y=.1;else goal.header.stamp=node->now();board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{goal});}
   if(change=="task")state_context=",\"task_id\":\"other\",\"navigation_generation\":1,\"map_revision\":2";
   if(change=="generation")state_context=",\"task_id\":\"bt-task\",\"navigation_generation\":2,\"map_revision\":2";
   if(change=="missing_context")state_context="";
   tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><CheckedMappingPath path='{path}' checked_path='{checked}' reuse_checked_path='true'/></BehaviorTree></root>",board);
   EXPECT_EQ(run(change=="paused"?"PAUSED":"EXPLORING"),BT::NodeStatus::FAILURE);
 }
}
TEST_F(CheckedPathTree, RetainedRouteRechecksNewMapAndDoesNotRenewItsDeadline){
 auto mid=path.poses.back();mid.pose.position.x=.5;
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{mid,path.poses.back()});
 ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 // 正常过点允许移除前缀；新地图版本仍须重新检查，不要求地图静止。
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});changing_revision=2;
 tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><CheckedMappingPath path='{path}' checked_path='{checked}' reuse_checked_path='true'/></BehaviorTree></root>",board);
 ASSERT_EQ(run(),BT::NodeStatus::SUCCESS);
 std::this_thread::sleep_for(std::chrono::milliseconds(1600));
 EXPECT_EQ(run(),BT::NodeStatus::FAILURE)<<"反复复核不能无限续租旧路线";
}
TEST_F(CheckedPathTree, HaltClearsRetainedRouteBeforeAnotherRequest){
 continuous_tree(0);path.poses.back().pose.position.x=1;board->set("path",path);
 board->set("goals",std::vector<geometry_msgs::msg::PoseStamped>{path.poses.back()});
 ASSERT_EQ(run(),BT::NodeStatus::RUNNING);tree.haltTree();
 tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><CheckedMappingPath path='{path}' checked_path='{checked}' reuse_checked_path='true'/></BehaviorTree></root>",board);
 EXPECT_EQ(run(),BT::NodeStatus::FAILURE);
}
