// 复用真实监督器夹具，明确检查多点Action，而非仅检查候选排序。
#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
using Through=nav2_msgs::action::NavigateThroughPoses;
using ThroughPlan=nav2_msgs::action::ComputePathThroughPoses;
class ContinuousSupervisorIntegration:public SupervisorIntegration {
protected:
 rclcpp_action::Server<Through>::SharedPtr through;
 rclcpp_action::Server<ThroughPlan>::SharedPtr through_plan;
 std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Through>>> active_routes;
 std::vector<std::vector<geometry_msgs::msg::PoseStamped>>routes;
 bool stall_route_plan=false;
 std::shared_ptr<rclcpp_action::ServerGoalHandle<ThroughPlan>>pending_route_plan;
 void SetUp()override{
   continuous_mode=true;SupervisorIntegration::SetUp();
   through=rclcpp_action::create_server<Through>(n,"/navigate_through_poses",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
     [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){active_routes.push_back(h);routes.push_back(h->get_goal()->poses);});
   through_plan=rclcpp_action::create_server<ThroughPlan>(n,"/compute_path_through_poses",[](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
     [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto h){
       if(stall_route_plan){pending_route_plan=h;return;}
       auto result=std::make_shared<ThroughPlan::Result>();result->path.header.frame_id="map";
       geometry_msgs::msg::PoseStamped start;start.pose.orientation.w=1;result->path.poses.push_back(start);
       for(const auto&p:h->get_goal()->goals)result->path.poses.push_back(p);
       h->succeed(result);
     });
 }
 void TearDown()override{through.reset();through_plan.reset();active_routes.clear();pending_route_plan.reset();SupervisorIntegration::TearDown();}
 void cancel_routes(){for(auto&h:active_routes)if(h->is_canceling())h->canceled(std::make_shared<Through::Result>());}
};
TEST_F(ContinuousSupervisorIntegration, DispatchesOneMultiPointRouteAndPauseWaitsForItsTerminal){
 request("start","continuous-start");
 ASSERT_TRUE(until([this]{return !routes.empty();},4))<<state.dump();
 ASSERT_GE(routes.front().size(),2U);EXPECT_EQ(nav_requests,0U);
 ASSERT_TRUE(until([this]{return last_velocity>0;},1));
 request("pause","continuous-pause");
 ASSERT_TRUE(until([this]{return state.value("phase","")=="STOPPING"&&last_velocity==0;},1));
 EXPECT_FALSE(state.value("navigation_terminal",true));
 ASSERT_TRUE(until([this]{cancel_routes();return state.value("phase","")=="PAUSED";},2));
 EXPECT_TRUE(state.value("navigation_terminal",false));EXPECT_EQ(last_velocity,0);
}

TEST_F(ContinuousSupervisorIntegration, PauseCancelsPendingRoutePlanningBeforeBecomingPaused){
 stall_route_plan=true;request("start","pending-start");
 ASSERT_TRUE(until([this]{return bool(pending_route_plan);},3));EXPECT_TRUE(routes.empty());
 request("pause","pending-pause");ASSERT_TRUE(until([this]{return state.value("phase","")=="STOPPING";},1));
 EXPECT_FALSE(state.value("navigation_terminal",true));EXPECT_EQ(last_velocity,0);
 ASSERT_TRUE(until([this]{if(pending_route_plan&&pending_route_plan->is_canceling()){
   pending_route_plan->canceled(std::make_shared<ThroughPlan::Result>());pending_route_plan.reset();}
   return state.value("phase","")=="PAUSED";},2));
 EXPECT_TRUE(routes.empty());EXPECT_EQ(last_velocity,0);
}
TEST_F(ContinuousSupervisorIntegration, SensorLossStopsContinuousRouteAndLateResultCannotRestart){
 request("start","sensor-start");ASSERT_TRUE(until([this]{return !routes.empty()&&last_velocity>0;},3));
 feed_scan=false;
 ASSERT_TRUE(until([this]{return last_velocity==0&&state.value("phase","")!="EXPLORING";},2));
 ASSERT_TRUE(until([this]{cancel_routes();return state.value("navigation_terminal",false);},2));
 const auto count=routes.size();feed_scan=true;until([]{return false;},.6);
 EXPECT_EQ(routes.size(),count);EXPECT_EQ(last_velocity,0);
}
TEST_F(ContinuousSupervisorIntegration, ApproachingViaPointLimitsControllerAndPrefetchesWithoutSingleGoal){
 double limit=-1;auto limits=n->create_subscription<nav2_msgs::msg::SpeedLimit>("/speed_limit",10,[&](const nav2_msgs::msg::SpeedLimit&m){limit=m.speed_limit;});
 request("start","via-start");ASSERT_TRUE(until([this]{return !routes.empty();},3));
 const auto point=routes.front().front().pose.position;
 // 位姿和轮式同时沿短步推进，避免把瞬移当成导航连续性证据。
 const double length=std::hypot(point.x,point.y);
 for(double d=.1;d<length-.35;d+=.1){pose_x=odom_x=point.x*d/length;pose_y=odom_y=point.y*d/length;until([]{return false;},.06);}
 // 当前衔接距离为0.25米，预减速在0.35米内；旧夹具停在范围外永远不会触发。
 const double approach=std::max(0.,length-.30);
 pose_x=odom_x=point.x*approach/length;pose_y=odom_y=point.y*approach/length;
 ASSERT_TRUE(until([&]{return limit==60.;},1));
 pose_x=odom_x=point.x;pose_y=odom_y=point.y;
 ASSERT_TRUE(until([this]{return state.value("continuous_exploration",Json::object()).value("passed_points",0)>0;},1));
 ASSERT_TRUE(until([this]{return routes.size()>1;},3))<<state.dump();
 EXPECT_EQ(nav_requests,0U);EXPECT_EQ(state.value("phase",""),"EXPLORING");
 EXPECT_LE(routes.back().size(),2U);
 // 新路线接受后旧Action迟到失败，不得清空新路线或重新发单点。
 const auto accepted_id=state.value("continuous_exploration",Json::object()).value("route_id",uint64_t{0});
 active_routes.front()->abort(std::make_shared<Through::Result>());
 until([]{return false;},.2);
 EXPECT_EQ(state.value("phase",""),"EXPLORING");EXPECT_GT(last_velocity,0);
 EXPECT_GE(state.value("continuous_exploration",Json::object()).value("route_id",uint64_t{0}),accepted_id);
 request("pause","via-pause");ASSERT_TRUE(until([this]{cancel_routes();return state.value("phase","")=="PAUSED";},2));
}

TEST_F(ContinuousSupervisorIntegration, ThreeClosedMapsFinishExplorationWithoutDrivingTheObsoleteRemainder){
 request("start","covered-start");ASSERT_TRUE(until([this]{return !routes.empty()&&last_velocity>0;},3));
 // 目标仍在执行，随后连续地图显示四周已封闭；不能再把这段无收益路线走到底。
 closed_outer_walls=true;
 ASSERT_TRUE(until([this]{return active_routes.back()->is_canceling()&&last_velocity==0;},2))<<state.dump();
 EXPECT_EQ(state.value("phase",""),"EXPLORING");
 EXPECT_FALSE(state.at("continuous_exploration").value("complete",true));
 cancel_routes();
 ASSERT_TRUE(until([this]{return state.value("phase","")=="STOPPING"||state.value("phase","")=="RETURNING";},4))<<state.dump();
 EXPECT_EQ(state.value("stop_target",""),"FINISH");EXPECT_EQ(last_velocity,0);
 EXPECT_TRUE(state.at("continuous_exploration").value("terminal",false));
 ASSERT_TRUE(until([this]{cancel_routes();return state.value("continuous_exploration",Json::object()).value("terminal",false);},2));
 EXPECT_TRUE(state.value("continuous_exploration",Json::object()).value("complete",false));
}

TEST_F(ContinuousSupervisorIntegration, PrematureSuccessNeverRecordsUnvisitedEndpoint){
 request("start","premature-start");ASSERT_TRUE(until([this]{return !routes.empty();},3));
 const auto original=routes.front();ASSERT_GE(original.size(),2U);
 active_routes.front()->succeed(std::make_shared<Through::Result>());
 ASSERT_TRUE(until([this]{return routes.size()>1;},3))<<state.dump();
 const auto continuous=state.at("continuous_exploration");
 EXPECT_TRUE(continuous.at("visit_observations").empty());EXPECT_EQ(continuous.value("passed_points",0),0);
 EXPECT_EQ(routes.back().back().pose.position,original.back().pose.position);
 EXPECT_EQ(state.value("phase",""),"EXPLORING");
 request("pause","premature-pause");ASSERT_TRUE(until([this]{cancel_routes();return state.value("phase","")=="PAUSED";},2));
}

// 地图与掩膜乱序只临时封锁速度，不能把连续路线当作新的任务取消。
class LiveContinuousSupervisorIntegration:public ContinuousSupervisorIntegration {
protected:
 void SetUp()override{live_mode=recovery_mode=true;ContinuousSupervisorIntegration::SetUp();}
};
TEST_F(LiveContinuousSupervisorIntegration, MapReorderingHoldsOutputAndPreservesRoute){
 request("start","live-continuous-start");
 ASSERT_TRUE(until([this]{return !routes.empty()&&last_velocity>0;},4))<<state.dump();
 const auto route=active_routes.back();const auto count=routes.size();
 live_state=live_mask=false;++grid_width;grid_origin-=.05;
 ASSERT_TRUE(until([this]{return !state["walls"].value("navigation_ready",true)&&last_velocity==0;},1));
 until([]{return false;},.45);
 EXPECT_FALSE(route->is_canceling())<<"短暂地图同步不应取消连续路线";
 live_state=live_mask=true;
 ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false)&&last_velocity>0;},2))<<state.dump();
 EXPECT_EQ(routes.size(),count);
 request("pause","live-continuous-pause");
 ASSERT_TRUE(until([this]{cancel_routes();return state.value("phase","")=="PAUSED";},3));
 EXPECT_EQ(last_velocity,0);
}
