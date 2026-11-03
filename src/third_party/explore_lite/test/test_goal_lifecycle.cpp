#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
extern char **environ;
using Nav=nav2_msgs::action::NavigateToPose;
class GoalLifecycle:public testing::Test {
protected:
 rclcpp::Node::SharedPtr node;
 std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr maps;
 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rejections;
 rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr generation;
 rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status;
 rclcpp_action::Server<Nav>::SharedPtr server;
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> active;
 rclcpp::TimerBase::SharedPtr timer;
 pid_t pid=-1;size_t goals=0;std::vector<geometry_msgs::msg::Point> targets;
 bool publish_map=true,narrow=false,progress=false,fully_known=false,closed_walls=false,dual_frontier=false;int frontier_x=70,border=0;double timeout=30,frequency=10,robot_yaw=0,orientation_scale=0;
 bool publish_tf=true,all_occupied=false;int small_frontier_cells=0,frontier_start_y=0;double robot_x=0,robot_y=0;
 bool ranking=false,cancel_allowed=true,advance_epoch_on_reject=false;uint64_t rank_epoch=0;size_t cancel_requests=0;
 size_t reject_goal_responses=0,rejected_goal_responses=0;
 nlohmann::json last_rank=nlohmann::json::object();size_t rank_requests=0;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rank_results,autonomy_state;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rank_requests_sub;
 std::string last_status;
 static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 template<class F>bool until(F f,double seconds){auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
  do{rclcpp::spin_some(node);if(f())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}while(std::chrono::steady_clock::now()<end);return f();}
 void SetUp()override{
  node=std::make_shared<rclcpp::Node>("explore_fixture");tf=std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  maps=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
  resume=node->create_publisher<std_msgs::msg::Bool>("/explore/resume",rclcpp::QoS(1).reliable().transient_local());
  rejections=node->create_publisher<std_msgs::msg::String>("/test_candidate_rejection",10);
  generation=node->create_publisher<std_msgs::msg::UInt64>("/test_navigation_generation",rclcpp::QoS(1).reliable().transient_local());
  status=node->create_subscription<explore_lite_msgs::msg::ExploreStatus>("/explore/status",rclcpp::QoS(10).reliable().transient_local(),[this](const explore_lite_msgs::msg::ExploreStatus&m){last_status=m.status;});
  server=rclcpp_action::create_server<Nav>(node,"/test_explore_nav",[this](const auto&,auto){
    if(reject_goal_responses){
      --reject_goal_responses;++rejected_goal_responses;
      if(advance_epoch_on_reject){
        advance_epoch_on_reject=false;++rank_epoch;std_msgs::msg::UInt64 next;next.data=rank_epoch;generation->publish(next);
        // 接口层模拟：旧目标应答到达前，新坐标世代已交付独立explore进程。
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
   },[this](auto){++cancel_requests;return cancel_allowed?rclcpp_action::CancelResponse::ACCEPT:rclcpp_action::CancelResponse::REJECT;},[this](auto h){active=h;++goals;targets.push_back(h->get_goal()->pose.pose.position);});
  if(ranking){
    rank_results=node->create_publisher<std_msgs::msg::String>("/mapping/exploration/rank_result",10);
    autonomy_state=node->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
    rank_requests_sub=node->create_subscription<std_msgs::msg::String>("/mapping/exploration/rank_request",10,[this](const std_msgs::msg::String&m){last_rank=nlohmann::json::parse(m.data);++rank_requests;});
  }
  timer=node->create_wall_timer(std::chrono::milliseconds(50),[this]{
   if(ranking){std_msgs::msg::String state;state.data=nlohmann::json{{"task_id","rank-fixture"},{"generation",1},{"navigation_generation",rank_epoch}}.dump();autonomy_state->publish(state);}
   geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";t.header.stamp=node->now();t.transform.rotation.z=std::sin(robot_yaw/2);t.transform.rotation.w=std::cos(robot_yaw/2);t.transform.translation.x=robot_x;t.transform.translation.y=robot_y;if(publish_tf)tf->sendTransform(t);
   if(publish_map){nav_msgs::msg::OccupancyGrid m;m.header=t.header;m.info.width=m.info.height=100;m.info.resolution=.05;
    m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;m.data.assign(10000,0);
    if(!fully_known)for(int y=0;y<100;++y)for(int x=frontier_x;x<100;++x)m.data[y*100+x]=y<frontier_start_y?100:narrow?(y>=49&&y<=53?-1:100):-1;
    if(dual_frontier)for(int y=0;y<100;++y)for(int x=0;x<30;++x)m.data[y*100+x]=-1;
    if(closed_walls)for(int i=0;i<100;++i){m.data[i]=100;m.data[9900+i]=100;m.data[i*100]=100;m.data[i*100+99]=100;}
    for(int i=0;i<small_frontier_cells;++i)m.data[(50+i)*100+frontier_x]=-1;
    if(all_occupied)std::fill(m.data.begin(),m.data.end(),100);
    maps->publish(m);}
   if(progress&&active&&!active->is_canceling()){
    auto f=std::make_shared<Nav::Feedback>();f->distance_remaining=1.0+(frontier_x%2)*.01;active->publish_feedback(f);}
  });
  char executable[]=EXPLORE_EXE,ros[]="--ros-args",p[]="-p",map[]="costmap_topic:=/map",base[]="robot_base_frame:=base_footprint",action[]="navigation_action:=/test_explore_nav",rejection[]="candidate_rejection_topic:=/test_candidate_rejection",min[]="min_frontier_size:=0.15";
  std::string rate="planner_frequency:="+std::to_string(frequency);
  std::string seconds="progress_timeout:="+std::to_string(timeout),padding="unknown_border_cells:="+std::to_string(border);
  std::string orientation="orientation_scale:="+std::to_string(orientation_scale);
  char epoch[]="navigation_generation_topic:=/test_navigation_generation";
  std::string ranked=std::string("path_aware_ranking:=")+(ranking?"true":"false");
  char*args[]={executable,ros,p,map,p,base,p,action,p,rate.data(),p,min,p,seconds.data(),p,padding.data(),p,rejection,p,orientation.data(),p,epoch,p,ranked.data(),nullptr};
  ASSERT_EQ(posix_spawn(&pid,executable,nullptr,nullptr,args,environ),0);
  ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},5));
 }
 void enable(bool value){std_msgs::msg::Bool m;m.data=value;resume->publish(m);}
 void TearDown()override{
  if(pid>0){kill(pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
  rank_results.reset();autonomy_state.reset();rank_requests_sub.reset();
  timer.reset();server.reset();active.reset();maps.reset();resume.reset();rejections.reset();generation.reset();status.reset();tf.reset();node.reset();
 }
};
TEST_F(GoalLifecycle, SuccessWaitsForFreshMapAndResumeKeepsRegionMemory){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));publish_map=false;until([]{return false;},.2);
 active->succeed(std::make_shared<Nav::Result>());active.reset();until([]{return false;},.4);EXPECT_EQ(goals,1U);
 enable(false);until([]{return false;},.2);publish_map=true;enable(true);
 ASSERT_TRUE(until([this]{return goals==2;},3));
 EXPECT_GT(std::hypot(targets[1].x-targets[0].x,targets[1].y-targets[0].y),.39);
}
TEST_F(GoalLifecycle, TwoFailuresFallbackInsideSameConnectedFrontier){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));
 active->abort(std::make_shared<Nav::Result>());active.reset();ASSERT_TRUE(until([this]{return goals==2;},3));
 active->abort(std::make_shared<Nav::Result>());active.reset();ASSERT_TRUE(until([this]{return goals==3;},3));
 EXPECT_GT(std::hypot(targets[2].x-targets[0].x,targets[2].y-targets[0].y),.39);
 EXPECT_NE(last_status,"exploration_complete");
}
class DirectionalExploration:public GoalLifecycle {void SetUp()override{
 dual_frontier=true;robot_yaw=std::acos(-1.0);orientation_scale=.8;GoalLifecycle::SetUp();
}};
TEST_F(DirectionalExploration, ComparableFrontierContinuesCurrentHeadingInsteadOfTurningBack){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));
 EXPECT_LT(targets.front().x,-.5)<<"同等前沿不应无故从朝西切换到朝东";
 const auto&q=active->get_goal()->pose.pose.orientation;
 EXPECT_LT(std::cos(2*std::atan2(q.z,q.w)),-.9)<<"探索目标不得固定为地图+X朝向";
}
TEST_F(DirectionalExploration, TurningToEastChangesComparableFrontierSelection){
 robot_yaw=0;until([]{return false;},.15);enable(true);
 ASSERT_TRUE(until([this]{return goals==1;},3));EXPECT_GT(targets.front().x,.5);
}
TEST_F(DirectionalExploration, BackwardFrontierRemainsSelectableWhenItIsTheOnlyCandidate){
 dual_frontier=false;until([]{return false;},.15);enable(true);
 ASSERT_TRUE(until([this]{return goals==1;},3));EXPECT_GT(targets.front().x,.5);
}
TEST_F(GoalLifecycle, CoordinateGenerationDropsOldVisitedRegions){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));const auto first=targets.front();
 publish_map=false;until([]{return false;},.1);
 active->succeed(std::make_shared<Nav::Result>());active.reset();until([]{return false;},.15);enable(false);
 ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},1));
 std_msgs::msg::UInt64 next;next.data=1;generation->publish(next);until([]{return false;},.15);publish_map=true;enable(true);
 ASSERT_TRUE(until([this]{return goals==2;},3));
 EXPECT_LT(std::hypot(first.x-targets.back().x,first.y-targets.back().y),.01);
}
class ShortProgress:public GoalLifecycle {void SetUp()override{timeout=1.;GoalLifecycle::SetUp();}};
TEST_F(ShortProgress, FrontierJitterCannotResetRealGoalProgress){
 progress=true;enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 const auto end=node->now().seconds()+2.;bool canceled=false;
 while(node->now().seconds()<end&&!canceled){frontier_x=frontier_x==70?71:70;until([]{return false;},.12);canceled=original->is_canceling();}
 EXPECT_TRUE(canceled);EXPECT_EQ(goals,1U);
 if(canceled){original->canceled(std::make_shared<Nav::Result>());active.reset();}
}
TEST_F(GoalLifecycle, AllTemporarilySuppressedIsNotComplete){
 narrow=true;until([]{return false;},.1);enable(true);
 ASSERT_TRUE(until([this]{return goals==1;},3));active->abort(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([this]{return goals==2;},3));active->abort(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([this]{return last_status=="exploration_waiting_candidates";},2));EXPECT_EQ(goals,2U);
}

class BorderFrontier:public GoalLifecycle {void SetUp()override{border=1;fully_known=true;GoalLifecycle::SetUp();}};
TEST_F(BorderFrontier, CroppedFreeEdgeDoesNotMeanWholeEnvironmentComplete){enable(true);
 ASSERT_TRUE(until([this]{return goals==1;},3));EXPECT_NE(last_status,"exploration_complete");
 EXPECT_GT(std::max(std::abs(targets[0].x),std::abs(targets[0].y)),2.49);}
TEST_F(BorderFrontier, ObservedClosedWallsStillAllowCompletion){closed_walls=true;until([]{return false;},.15);enable(true);
 ASSERT_TRUE(until([this]{return last_status=="exploration_complete";},3));EXPECT_EQ(goals,0U);}

class SlowCandidatePlanning:public GoalLifecycle {void SetUp()override{frequency=.2;GoalLifecycle::SetUp();}
protected:
 void reject(const std::string&id){std_msgs::msg::String m;m.data=id;rejections->publish(m);}
};
TEST_F(SlowCandidatePlanning, ExplicitRejectionEitherOrderSelectsAfterTerminalWithoutPeriodicWait){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));
 reject(rclcpp_action::to_string(active->get_goal_id()));until([]{return false;},.1);
 EXPECT_EQ(goals,1U);active->abort(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([this]{return goals==2;},.7))<<"已确认未预检的候选仍等待原周期";
 const auto id=rclcpp_action::to_string(active->get_goal_id());active->abort(std::make_shared<Nav::Result>());active.reset();
 until([]{return false;},.1);reject(id);
 ASSERT_TRUE(until([this]{return goals==3;},.7))<<"先终态后拒绝事件的顺序未处理";
}
TEST_F(SlowCandidatePlanning, OrdinaryFailureAndUnrelatedNoticeKeepPeriodicWait){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));
 active->abort(std::make_shared<Nav::Result>());active.reset();reject(std::string(32,'0'));
 until([]{return false;},.4);EXPECT_EQ(goals,1U);
}
TEST_F(SlowCandidatePlanning, PauseCancelsPendingCandidateRetry){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));
 reject(rclcpp_action::to_string(active->get_goal_id()));until([]{return false;},.1);
 active->abort(std::make_shared<Nav::Result>());active.reset();enable(false);
 ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},1));
 until([]{return false;},.3);EXPECT_EQ(goals,1U);
}

class RankedGoalLifecycle:public GoalLifecycle {
 void SetUp()override{ranking=true;dual_frontier=true;GoalLifecycle::SetUp();}
protected:
 void respond(const nlohmann::json&request,size_t index){
   auto response=request;response["candidates"]=nlohmann::json::array({{{"frontier",request.at("candidates").at(index)},{"path_m",1.5},{"gain_m2",2.}}});
   std_msgs::msg::String message;message.data=response.dump();rank_results->publish(message);
 }
};
TEST_F(RankedGoalLifecycle, UsesRankedFrontierAndKeepsItDespiteMapGrowth){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));ASSERT_GE(last_rank["candidates"].size(),2U);EXPECT_EQ(goals,0U);
 const auto chosen=last_rank["candidates"][1];respond(last_rank,1);ASSERT_TRUE(until([this]{return goals==1;},2));
 EXPECT_DOUBLE_EQ(targets[0].x,chosen[0].get<double>());EXPECT_DOUBLE_EQ(targets[0].y,chosen[1].get<double>());
 const auto requests_before=rank_requests;frontier_x=75;until([]{return false;},.4);
 EXPECT_EQ(goals,1U);EXPECT_EQ(rank_requests,requests_before);
}
TEST_F(RankedGoalLifecycle, PauseInvalidatesLateRankingAndResumeRequiresNewRequest){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));const auto old=last_rank;enable(false);
 ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},1));respond(old,0);until([]{return false;},.2);EXPECT_EQ(goals,0U);
 enable(true);ASSERT_TRUE(until([this,&old]{return last_rank.value("request_id",std::string())!=old.value("request_id",std::string());},2));
 respond(old,0);until([]{return false;},.2);EXPECT_EQ(goals,0U);respond(last_rank,0);ASSERT_TRUE(until([this]{return goals==1;},2));
}
TEST_F(RankedGoalLifecycle, CoordinateGenerationRejectsOldRankingBeforeDispatch){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));const auto old=last_rank;
 ++rank_epoch;std_msgs::msg::UInt64 next;next.data=rank_epoch;generation->publish(next);
 until([]{return false;},.25);respond(old,0);until([]{return false;},.2);EXPECT_EQ(goals,0U);
 ASSERT_TRUE(until([this]{return last_rank.value("navigation_generation",uint64_t{})==rank_epoch;},2));
 respond(last_rank,0);ASSERT_TRUE(until([this]{return goals==1;},2));
}
TEST_F(RankedGoalLifecycle, UnreachableRepresentativeDoesNotHideOtherPointsOnSameFrontier){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));const auto first=last_rank;
 for(size_t attempt=0;attempt<2;++attempt){
  const auto count=rank_requests;auto reply=last_rank;auto rejected=nlohmann::json::array();
  for(const auto&point:reply["candidates"])rejected.push_back({{"frontier",point},{"reason","该代表点不可达"}});
  reply["rejected"]=rejected;reply["candidates"]=nlohmann::json::array();reply["reason"]="检查同一前沿其他观测点";
  std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
  ASSERT_TRUE(until([this,count]{return rank_requests>count;},2));
 }
 EXPECT_EQ(goals,0U);EXPECT_NE(last_status,"exploration_complete");
 EXPECT_NE(last_rank["candidates"],first["candidates"]);
 respond(last_rank,0);ASSERT_TRUE(until([this]{return goals==1;},2));
}

TEST_F(RankedGoalLifecycle, UsesKnownObservationPrefixInsteadOfDrivingToFarFrontier){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));auto reply=last_rank;
 reply["candidates"]=nlohmann::json::array({{{"frontier",last_rank["candidates"][0]},{"observation",{.55,0.}}, {"path_m",.55},{"gain_m2",.4}}});
 std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
 ASSERT_TRUE(until([this]{return goals==1;},2));EXPECT_DOUBLE_EQ(targets[0].x,.55);EXPECT_DOUBLE_EQ(targets[0].y,0);
}

TEST_F(RankedGoalLifecycle, ObservedSourceCancelsFarObservationAndWaitsForTerminal){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));auto reply=last_rank;
 reply["candidates"]=nlohmann::json::array({{{"frontier",last_rank["candidates"][0]},
   {"observation",{1.5,0.}},{"path_m",1.5},{"gain_m2",.4}}});
 std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
 ASSERT_TRUE(until([this]{return goals==1;},2));auto original=active;const auto requests_before=rank_requests;
 fully_known=true;dual_frontier=false;closed_walls=true;
 ASSERT_TRUE(until([&]{return original->is_canceling();},2));
 until([]{return false;},.25);EXPECT_EQ(goals,1U);EXPECT_EQ(rank_requests,requests_before);
 EXPECT_NE(last_status,"exploration_complete")<<"取消未到终态时不能报告探索完成";
 original->canceled(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([this]{return last_status=="exploration_complete";},2));
 EXPECT_EQ(goals,1U);
}
TEST_F(RankedGoalLifecycle, OneObservedMapDoesNotCancelAndRestoredFrontierResetsEvidence){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));respond(last_rank,0);
 ASSERT_TRUE(until([this]{return goals==1;},2));auto original=active;
 fully_known=true;dual_frontier=false;until([]{return false;},.15);publish_map=false;
 until([]{return false;},.65);EXPECT_FALSE(original->is_canceling());
 fully_known=false;dual_frontier=true;publish_map=true;until([]{return false;},.2);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(goals,1U);
}
TEST_F(RankedGoalLifecycle, CoordinateChangeClearsOldFrontierObservationEvidence){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));respond(last_rank,0);
 ASSERT_TRUE(until([this]{return goals==1;},2));auto original=active;
 ++rank_epoch;std_msgs::msg::UInt64 next;next.data=rank_epoch;generation->publish(next);
 until([]{return false;},.15);fully_known=true;dual_frontier=false;
 until([]{return false;},.8);EXPECT_FALSE(original->is_canceling());EXPECT_EQ(goals,1U);
}
TEST_F(RankedGoalLifecycle, PauseDuringObservedCancellationPreventsLateResultRedispatch){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));respond(last_rank,0);
 ASSERT_TRUE(until([this]{return goals==1;},2));auto original=active;
 fully_known=true;dual_frontier=false;
 ASSERT_TRUE(until([&]{return original->is_canceling();},2));
 enable(false);ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},1));
 const auto before=rank_requests;original->canceled(std::make_shared<Nav::Result>());active.reset();
 fully_known=false;dual_frontier=true;until([]{return false;},.5);
 EXPECT_EQ(goals,1U);EXPECT_EQ(rank_requests,before);EXPECT_EQ(last_status,"exploration_paused");
}

TEST_F(RankedGoalLifecycle, RejectedObservedCancelDoesNotRetryOrRedispatch){
 cancel_allowed=false;enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));respond(last_rank,0);
 ASSERT_TRUE(until([this]{return goals==1;},2));auto original=active;const auto before=rank_requests;
 fully_known=true;dual_frontier=false;
 ASSERT_TRUE(until([this]{return cancel_requests==1;},2));until([]{return false;},.6);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(cancel_requests,1U);
 EXPECT_EQ(goals,1U);EXPECT_EQ(rank_requests,before);
}

TEST_F(RankedGoalLifecycle, FailedFrontierRepresentativeDoesNotBlacklistNearbySafeObservation){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));
 const auto source=last_rank["candidates"][0];
 for(int i=0;i<2;++i){
  const auto before=rank_requests;auto reply=last_rank;
  reply["rejected"]=nlohmann::json::array({{{"frontier",source},{"reason","源点不能直接作为观测点"}}});
  reply["candidates"]=nlohmann::json::array();
  std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
  ASSERT_TRUE(until([this,before]{return rank_requests>before;},2));
 }
 EXPECT_TRUE(last_rank.at("blocked_observations").empty());
 auto reply=last_rank;const double sx=source[0].get<double>();
 const double ox=sx-std::copysign(.30,sx),oy=source[1].get<double>();
 // 排序器已验证此观测点。原前沿失败区域不能在客户端再次把它否决。
 reply["candidates"]=nlohmann::json::array({{{"frontier",last_rank["candidates"][0]},
   {"observation",{ox,oy}},{"path_m",1.},{"gain_m2",1.}}});
 std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
 ASSERT_TRUE(until([this]{return goals==1;},2));EXPECT_DOUBLE_EQ(targets.back().x,ox);
}
TEST_F(RankedGoalLifecycle, ActualObservationFailureExportsCooldownAndFiltersLateReply){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));
 auto send=[&](bool fallback){
  auto reply=last_rank;
  reply["candidates"]=nlohmann::json::array({{{"frontier",last_rank["candidates"][0]},
    {"observation",{.5,0.}},{"path_m",.5},{"gain_m2",1.}}});
  if(fallback)reply["candidates"].push_back({{"frontier",last_rank["candidates"][0]},
    {"observation",{.1,.6}},{"path_m",.8},{"gain_m2",.8}});
  std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
 };
 for(size_t attempt=1;attempt<=2;++attempt){
  const auto before=rank_requests;send(false);
  ASSERT_TRUE(until([this,attempt]{return goals==attempt;},2));
  active->abort(std::make_shared<Nav::Result>());active.reset();
  ASSERT_TRUE(until([this,before]{return rank_requests>before;},2));
 }
 ASSERT_EQ(last_rank.at("blocked_observations").size(),1U);
 const auto blocked=last_rank["blocked_observations"][0];
 EXPECT_DOUBLE_EQ(blocked.at("center").at(0).get<double>(),.5);
 EXPECT_EQ(blocked.at("kind").get<std::string>(),"failed_observation");
 send(true);ASSERT_TRUE(until([this]{return goals==3;},2));
 EXPECT_DOUBLE_EQ(targets.back().x,.1);EXPECT_DOUBLE_EQ(targets.back().y,.6);
 // 坐标世代切换必须连同真实运动目标禁区一起清除，旧异步应答仍由既有协议隔离。
 enable(false);ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},1));
 if(active){ASSERT_TRUE(until([this]{return active->is_canceling();},1));active->canceled(std::make_shared<Nav::Result>());}
 active.reset();
 ++rank_epoch;std_msgs::msg::UInt64 next;next.data=rank_epoch;generation->publish(next);
 until([]{return false;},.15);enable(true);
 ASSERT_TRUE(until([this]{return last_rank.value("navigation_generation",uint64_t{})==rank_epoch;},2));
 EXPECT_TRUE(last_rank.at("blocked_observations").empty());
}

TEST_F(RankedGoalLifecycle, OldCoordinateRejectionCannotRepopulateClearedObservationMemory){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));
 auto respond_observation=[&]{
  auto reply=last_rank;reply["candidates"]=nlohmann::json::array({{{"frontier",last_rank["candidates"][0]},
    {"observation",{.5,0.}},{"path_m",.5},{"gain_m2",1.}}});
  std_msgs::msg::String message;message.data=reply.dump();rank_results->publish(message);
 };
 const auto before=rank_requests;reject_goal_responses=2;advance_epoch_on_reject=true;respond_observation();
 ASSERT_TRUE(until([this,before]{return rejected_goal_responses==1&&rank_requests>before&&
   last_rank.value("navigation_generation",uint64_t{})==rank_epoch;},3));
 const auto second=rank_requests;respond_observation();
 ASSERT_TRUE(until([this,second]{return rejected_goal_responses==2&&rank_requests>second;},3));
 // 新世代只发生一次真实拒绝，尚未达到2次冷却阈值；旧世代应答不能凑成第二次。
 EXPECT_TRUE(last_rank.at("blocked_observations").empty());
 respond_observation();ASSERT_TRUE(until([this]{return goals==1;},2));
 EXPECT_DOUBLE_EQ(targets.back().x,.5);
}

TEST_F(GoalLifecycle, ActiveGoalWithNoLegalFrontiersWaitsForTerminalAndFreshMap){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 fully_known=true;closed_walls=true;ASSERT_TRUE(until([&]{return original->is_canceling();},2));
 EXPECT_NE(last_status,"exploration_complete");publish_map=false;until([]{return false;},.15);
 original->canceled(std::make_shared<Nav::Result>());active.reset();
 until([]{return false;},.35);EXPECT_NE(last_status,"exploration_complete");EXPECT_EQ(goals,1U);
 publish_map=true;ASSERT_TRUE(until([this]{return last_status=="exploration_complete";},2));
 EXPECT_EQ(goals,1U);
}
TEST_F(GoalLifecycle, ActiveGoalKeepsRemainingLegalFrontier){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 frontier_x=74;until([]{return false;},.9);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(cancel_requests,0U);EXPECT_NE(last_status,"exploration_complete");
}
TEST_F(GoalLifecycle, SubthresholdFragmentsUseTheExistingCompletionSize){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 fully_known=true;closed_walls=true;small_frontier_cells=2;
 ASSERT_TRUE(until([&]{return original->is_canceling();},2));
 original->canceled(std::make_shared<Nav::Result>());active.reset();
 ASSERT_TRUE(until([this]{return last_status=="exploration_complete";},2));
}
TEST_F(GoalLifecycle, CoordinateChangeInvalidatesGlobalEmptyMapConfirmation){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 fully_known=true;closed_walls=true;until([]{return false;},.2);
 std_msgs::msg::UInt64 next;next.data=1;generation->publish(next);
 until([]{return false;},.9);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(cancel_requests,0U);EXPECT_NE(last_status,"exploration_complete");
}
TEST_F(GoalLifecycle, MissingTfCannotConfirmAnEmptyMap){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 publish_tf=false;until([]{return false;},.35);fully_known=true;closed_walls=true;
 until([]{return false;},1.2);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(cancel_requests,0U);EXPECT_NE(last_status,"exploration_complete");
 publish_tf=true;ASSERT_TRUE(until([&]{return original->is_canceling();},3));
}
TEST_F(GoalLifecycle, OutOfBoundsOrNonfreeRobotPoseCannotConfirmCompletion){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 fully_known=true;closed_walls=true;robot_x=20;
 until([]{return false;},.9);EXPECT_FALSE(original->is_canceling());
 robot_x=0;all_occupied=true;until([]{return false;},.9);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(cancel_requests,0U);EXPECT_NE(last_status,"exploration_complete");
 all_occupied=false;ASSERT_TRUE(until([&]{return original->is_canceling();},2));
}
TEST_F(GoalLifecycle, InvalidTfAfterCancellationCannotDeclareIdleCompletion){
 enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 fully_known=true;closed_walls=true;ASSERT_TRUE(until([&]{return original->is_canceling();},2));
 publish_tf=false;until([]{return false;},.35);
 original->canceled(std::make_shared<Nav::Result>());active.reset();until([]{return false;},.9);
 EXPECT_NE(last_status,"exploration_complete");EXPECT_EQ(goals,1U);
 publish_tf=true;ASSERT_TRUE(until([this]{return last_status=="exploration_complete";},3));
}
TEST_F(GoalLifecycle, RejectedGlobalEmptyCancellationDoesNotRetryOrRedispatch){
 cancel_allowed=false;enable(true);ASSERT_TRUE(until([this]{return goals==1;},3));auto original=active;
 fully_known=true;closed_walls=true;ASSERT_TRUE(until([this]{return cancel_requests==1;},2));
 until([]{return false;},.7);EXPECT_EQ(cancel_requests,1U);EXPECT_EQ(goals,1U);
 EXPECT_FALSE(original->is_canceling());EXPECT_NE(last_status,"exploration_complete");
}
TEST_F(RankedGoalLifecycle, SourceClusterMayShiftCentroidWhileNearbyPointsRemain){
 enable(true);ASSERT_TRUE(until([this]{return rank_requests>0;},3));
 size_t chosen=0;while(chosen<last_rank["candidates"].size()&&last_rank["candidates"][chosen][0].get<double>()<=0)++chosen;
 ASSERT_LT(chosen,last_rank["candidates"].size());respond(last_rank,chosen);
 ASSERT_TRUE(until([this]{return goals==1;},2));auto original=active;
 frontier_start_y=45;until([]{return false;},.9);
 EXPECT_FALSE(original->is_canceling());EXPECT_EQ(cancel_requests,0U);EXPECT_EQ(goals,1U);
}
