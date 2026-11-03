/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <nlohmann/json.hpp>
#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/goal_memory.h>
#include <explore/direction_preference.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();
  void dispatchGoal(const geometry_msgs::msg::Pose&pose,const geometry_msgs::msg::Point&target_position);
  bool rankPending();
  void requestRanking(const std::vector<geometry_msgs::msg::Point>&candidates);
  // 只跟踪本次排序的原始前沿，不能把安全观测点当作待观测未知边界。
  bool searchValidFrontiers(geometry_msgs::msg::Pose&pose,std::vector<frontier_exploration::Frontier>&frontiers);
  bool activeFrontierObserved();
  bool ranked_frontier_valid_=false,frontier_observed_cancel_=false;
  bool frontier_observation_disabled_=false,global_frontiers_observed_=false;
  geometry_msgs::msg::Point ranked_frontier_;
  uint64_t frontier_map_sequence_=0;
  unsigned frontier_absent_updates_=0;
  double frontier_absent_since_=0;

  bool path_aware_ranking_=false,rank_pending_=false;
  std::string rank_request_id_;uint64_t rank_sequence_=0;
  double rank_started_=0;
  nlohmann::json autonomy_=nlohmann::json::object(),rank_reply_=nlohmann::json::object(),rank_visits_=nlohmann::json::array();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rank_requests_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rank_results_,autonomy_state_;

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  bool frontierOnBlacklist(const geometry_msgs::msg::Point& frontier);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  /**
    * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
    */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  rclcpp::TimerBase::SharedPtr candidate_retry_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr candidate_rejections_sub_;
  std::deque<std::string> candidate_rejections_;
  std::string last_failed_goal_;
  void retryRejectedCandidate(const std::string&goal_id);
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  GoalMemory goal_memory_,rank_frontier_memory_;
  DirectionPreference direction_;
  geometry_msgs::msg::Point dispatch_start_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr navigation_generation_sub_;
  uint64_t navigation_generation_=0;
  uint64_t dispatch_navigation_generation_=0;
  bool timeout_cancel_=false,waiting_success_map_=false,feedback_seen_=false;
  uint64_t success_map_sequence_=0;
  double progress_epsilon_=.10;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  bool resuming_ = false;
  bool goal_active_{false};
  bool stopped_{true};
  uint64_t dispatch_generation_{0};
  rclcpp_action::GoalUUID active_goal_id_;
};
}  // namespace explore

#endif
