// 真实Action回调与可控业务时钟重放：失败等待不能消耗下一条路线的行驶进展预算。
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include "carcar_mapping_autonomy/continuous_exploration.hpp"
namespace ca=carcar_mapping_autonomy;
using Plan=nav2_msgs::action::ComputePathThroughPoses;
using Nav=nav2_msgs::action::NavigateThroughPoses;
TEST(ContinuousProgress, FailedPlanCooldownDoesNotConsumeNewMotionBudget){
 ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"","216");
 ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY")?std::getenv("ROS_LOCALHOST_ONLY"):"","1");
 int argc=0;rclcpp::init(argc,nullptr);
 auto node=std::make_shared<rclcpp::Node>("progress_fixture");
 unsigned plans=0;std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> active;
 auto planner=rclcpp_action::create_server<Plan>(node,"/compute_path_through_poses",
  [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[&](auto h){
   auto result=std::make_shared<Plan::Result>();
   if(++plans==1){h->abort(result);return;}
   result->path.header.frame_id="map";
   geometry_msgs::msg::PoseStamped start;start.pose.position.x=-.5440107188;start.pose.position.y=-.3653434538;start.pose.orientation.w=1;
   result->path.poses.push_back(start);
   // 绕过保存地图中的内角，保持与生产路径相同的净空审查。
   auto corner=start;corner.pose.position.x=-.0761056922;result->path.poses.push_back(corner);
   for(const auto&p:h->get_goal()->goals)result->path.poses.push_back(p);
   h->succeed(result);
  });
 auto navigation=rclcpp_action::create_server<Nav>(node,"/navigate_through_poses",
  [](const auto&,auto){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[&](auto h){active=h;});
 {
  std::ifstream file(PROGRESS_FIXTURE);auto j=ca::Json::parse(file);
  auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();map->header.frame_id="map";
  map->info.width=j["width"];map->info.height=j["height"];map->info.resolution=j["resolution"];
  map->info.origin.position.x=j["origin"][0];map->info.origin.position.y=j["origin"][1];map->info.origin.orientation.w=1;
  map->data=j["data"].get<std::vector<int8_t>>();
  ca::ContinuousExploration route(*node,"");ca::ContinuousExploration::Snapshot s;
  s.route.map=map;s.route.current={-.5440107188,-.3653434538,-.56182962};s.route.revision=30;
  s.route.task.anchored=true;s.route.task.phase="EXPLORING";s.route.task.limits.radius=20;
  s.enabled=s.healthy=s.still=true;
  auto pump=[&](double now,auto condition){
   s.route.now=now;
   for(int i=0;i<400;++i){route.tick(s);rclcpp::spin_some(node);if(condition())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
   return false;
  };
  EXPECT_TRUE(pump(0,[&]{return plans==1&&route.status().value("reason",std::string()).find("规划失败")!=std::string::npos;}));
  EXPECT_TRUE(pump(1,[&]{return route.status().value("reason",std::string()).find("到访抑制")!=std::string::npos;}));
  EXPECT_FALSE(active);EXPECT_EQ(plans,1U);
  EXPECT_TRUE(pump(61,[&]{return route.motion();}));
  route.tick(s);rclcpp::spin_some(node);
  EXPECT_TRUE(route.motion())<<"60秒失败等待后，新路线不得立即被30秒无进展检查取消："<<route.status();
  // 活动路线确实没有进展时仍须取消，不能通过禁用保护使上面的断言通过。
  EXPECT_TRUE(pump(92,[&]{return active&&active->is_canceling();}));
  EXPECT_FALSE(route.motion());if(active&&active->is_canceling())active->canceled(std::make_shared<Nav::Result>());
  rclcpp::spin_some(node);
 }
 navigation.reset();planner.reset();node.reset();rclcpp::shutdown();
}
