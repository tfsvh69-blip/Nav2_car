#include <gtest/gtest.h>
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
 rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status;
 rclcpp_action::Server<Nav>::SharedPtr server;
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> active;
 rclcpp::TimerBase::SharedPtr timer;
 pid_t pid=-1;size_t goals=0;std::vector<geometry_msgs::msg::Point> targets;
 bool publish_map=true,narrow=false,progress=false,fully_known=false,closed_walls=false;int frontier_x=70,border=0;double timeout=30,frequency=10;
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
  status=node->create_subscription<explore_lite_msgs::msg::ExploreStatus>("/explore/status",rclcpp::QoS(10).reliable().transient_local(),[this](const explore_lite_msgs::msg::ExploreStatus&m){last_status=m.status;});
  server=rclcpp_action::create_server<Nav>(node,"/test_explore_nav",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){active=h;++goals;targets.push_back(h->get_goal()->pose.pose.position);});
  timer=node->create_wall_timer(std::chrono::milliseconds(50),[this]{
   geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";t.header.stamp=node->now();t.transform.rotation.w=1;tf->sendTransform(t);
   if(publish_map){nav_msgs::msg::OccupancyGrid m;m.header=t.header;m.info.width=m.info.height=100;m.info.resolution=.05;
    m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;m.data.assign(10000,0);
    if(!fully_known)for(int y=0;y<100;++y)for(int x=frontier_x;x<100;++x)m.data[y*100+x]=narrow?(y>=49&&y<=53?-1:100):-1;
    if(closed_walls)for(int i=0;i<100;++i){m.data[i]=100;m.data[9900+i]=100;m.data[i*100]=100;m.data[i*100+99]=100;}
    maps->publish(m);}
   if(progress&&active&&!active->is_canceling()){
    auto f=std::make_shared<Nav::Feedback>();f->distance_remaining=1.0+(frontier_x%2)*.01;active->publish_feedback(f);}
  });
  char executable[]=EXPLORE_EXE,ros[]="--ros-args",p[]="-p",map[]="costmap_topic:=/map",base[]="robot_base_frame:=base_footprint",action[]="navigation_action:=/test_explore_nav",rejection[]="candidate_rejection_topic:=/test_candidate_rejection",min[]="min_frontier_size:=0.15";
  std::string rate="planner_frequency:="+std::to_string(frequency);
  std::string seconds="progress_timeout:="+std::to_string(timeout),padding="unknown_border_cells:="+std::to_string(border);
  char*args[]={executable,ros,p,map,p,base,p,action,p,rate.data(),p,min,p,seconds.data(),p,padding.data(),p,rejection,nullptr};
  ASSERT_EQ(posix_spawn(&pid,executable,nullptr,nullptr,args,environ),0);
  ASSERT_TRUE(until([this]{return last_status=="exploration_paused";},5));
 }
 void enable(bool value){std_msgs::msg::Bool m;m.data=value;resume->publish(m);}
 void TearDown()override{
  if(pid>0){kill(pid,SIGINT);for(int i=0;i<100;++i){if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(20));}if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
  timer.reset();server.reset();active.reset();maps.reset();resume.reset();rejections.reset();status.reset();tf.reset();node.reset();
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
