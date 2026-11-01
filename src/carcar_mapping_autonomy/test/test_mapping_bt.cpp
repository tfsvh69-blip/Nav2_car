#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <chrono>
#include <thread>

// 使用本机官方PipelineSequence/RateController/RecoveryNode及真实XML，
// 仅替换网络Action端点，验证一次失败后重试/持续失败/取消的控制流与实际延迟。
class MappingTree:public testing::Test {
protected:
 BT::BehaviorTreeFactory factory;
 BT::Tree tree;
 int failures=0,plans=0,follows=0;
 virtual const char* xml()const{return MAPPING_BT_XML;}
 virtual std::string planner()const{return "GridBased";}
 virtual std::string checker()const{return "general_goal_checker";}
 void SetUp()override{
  for(const auto*plugin:{"nav2_pipeline_sequence_bt_node","nav2_rate_controller_bt_node","nav2_recovery_node_bt_node"})
   factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  factory.registerSimpleAction("ComputePathToPose",[this](BT::TreeNode&node){
   const auto goal=node.getInput<std::string>("goal");EXPECT_TRUE(goal);if(goal){EXPECT_EQ(goal.value(),"fixed-goal");}
   const auto id=node.getInput<std::string>("planner_id");EXPECT_TRUE(id);if(id){EXPECT_EQ(id.value(),planner());}
   ++plans;if(failures-->0)return BT::NodeStatus::FAILURE;
   node.setOutput("path",std::string("checked-path"));return BT::NodeStatus::SUCCESS;
  },{BT::InputPort<std::string>("goal"),BT::OutputPort<std::string>("path"),BT::InputPort<std::string>("planner_id")});
  factory.registerSimpleAction("FollowPath",[this](BT::TreeNode&node){
   const auto id=node.getInput<std::string>("goal_checker_id");EXPECT_TRUE(id);if(id){EXPECT_EQ(id.value(),checker());}
   ++follows;return BT::NodeStatus::SUCCESS;
  },{BT::InputPort<std::string>("path"),BT::InputPort<std::string>("controller_id"),BT::InputPort<std::string>("goal_checker_id")});
  auto blackboard=BT::Blackboard::create();blackboard->set("goal",std::string("fixed-goal"));
  tree=factory.createTreeFromFile(xml(),blackboard);
 }
 BT::NodeStatus finish(){
  const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  auto status=tree.tickRoot();
  while(status==BT::NodeStatus::RUNNING&&std::chrono::steady_clock::now()<end){std::this_thread::sleep_for(std::chrono::milliseconds(5));status=tree.tickRoot();}
  return status;
 }
};
class ExplorationTree:public MappingTree {
 const char* xml()const override{return EXPLORATION_BT_XML;}
 std::string planner()const override{return "ExplorationGrid";}
 std::string checker()const override{return "exploration_goal_checker";}
};
TEST_F(ExplorationTree, UsesExplorationPlannerAndPositionChecker){
 EXPECT_EQ(finish(),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);
}
TEST_F(MappingTree, SuccessfulPlanDoesNotWaitOrRetry){
 auto began=std::chrono::steady_clock::now();EXPECT_EQ(finish(),BT::NodeStatus::SUCCESS);
 EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.20);
}
TEST_F(MappingTree, TransientFailureRetriesSameGoalAfterCostmapPeriod){
 failures=1;auto began=std::chrono::steady_clock::now();EXPECT_EQ(finish(),BT::NodeStatus::SUCCESS);
 EXPECT_EQ(plans,2);EXPECT_EQ(follows,1);EXPECT_GE(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.35);
}
TEST_F(MappingTree, PersistentFailureStopsAfterOneRetry){
 failures=20;EXPECT_EQ(finish(),BT::NodeStatus::FAILURE);EXPECT_EQ(plans,2);EXPECT_EQ(follows,0);
}
TEST_F(MappingTree, HaltDuringDelayDoesNotSubmitAnotherPlan){
 failures=1;ASSERT_EQ(tree.tickRoot(),BT::NodeStatus::RUNNING);ASSERT_EQ(plans,1);
 tree.haltTree();std::this_thread::sleep_for(std::chrono::milliseconds(450));
 EXPECT_EQ(plans,1);EXPECT_EQ(follows,0);EXPECT_EQ(tree.rootNode()->status(),BT::NodeStatus::IDLE);
}
