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

#include <explore/explore.h>

#include <thread>

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  progress_epsilon_=declare_parameter<double>("progress_epsilon",.10);
  goal_memory_.radius=declare_parameter<double>("region_radius",.40);
  goal_memory_.cooldown=declare_parameter<double>("region_cooldown",60.0);
  if(!(std::isfinite(planner_frequency_)&&planner_frequency_>0&&planner_frequency_<=100&&
       std::isfinite(progress_timeout_)&&progress_timeout_>0&&std::isfinite(progress_epsilon_)&&progress_epsilon_>0&&
       std::isfinite(goal_memory_.radius)&&goal_memory_.radius>0&&std::isfinite(goal_memory_.cooldown)&&goal_memory_.cooldown>0))throw std::invalid_argument("探索参数必须为有限正数且频率不超过100Hz");
  // 显式接口参数避免Humble中action内部服务/话题重映射差异绕过监督器。
  const auto navigation_action = this->declare_parameter<std::string>("navigation_action", ACTION_NAME);
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, navigation_action);
  // 仅建图监督器显式声明“未提交路径预检”的UUID才允许加速换候选；默认不订阅。
  const auto rejection_topic=declare_parameter<std::string>("candidate_rejection_topic","");
  if(!rejection_topic.empty())candidate_rejections_sub_=create_subscription<std_msgs::msg::String>(
    rejection_topic,rclcpp::QoS(10),[this](const std_msgs::msg::String&m){
      // Humble to_string逐字节输出十六进制且不补零，合法UUID文本不一定长32。
      if(m.data.empty()||m.data.size()>32||std::find(candidate_rejections_.begin(),candidate_rejections_.end(),m.data)!=candidate_rejections_.end())return;
      candidate_rejections_.push_back(m.data);while(candidate_rejections_.size()>64)candidate_rejections_.pop_front();
      retryRejectedCandidate(m.data);
    });

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  if (!move_base_client_->wait_for_action_server()) return;
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // carcar: cold standby; supervisor explicitly resumes after SLAM/ownership checks.
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
  status_pub_->publish(status_msg);
  exploring_timer_->cancel();
}

Explore::~Explore()
{
  if (rclcpp::ok() && exploring_timer_) stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.middle)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (stopped_) return;
  if(goal_active_){
    // 只跟踪已发送目标的真实导航反馈；候选前沿变化不得刷新进展计时。
    if(!timeout_cancel_&&(this->now()-last_progress_).seconds()>progress_timeout_){
      timeout_cancel_=true;
      RCLCPP_WARN(logger_,"探索目标 goal_id=%s event=no_progress elapsed=%.3f；等待取消终态后再选点",
        rclcpp_action::to_string(active_goal_id_).c_str(),(this->now()-last_progress_).seconds());
      if(navigation_goal_handle_)move_base_client_->async_cancel_goal(navigation_goal_handle_);
    }
    return;
  }
  if(waiting_success_map_){
    if(costmap_client_.mapSequence()<=success_map_sequence_)return;
    waiting_success_map_=false;
  }
  const auto pose=costmap_client_.getRobotPose();
  auto frontiers=search_.searchFrom(pose.position);
  if(visualize_)visualizeFrontiers(frontiers);
  if(frontiers.empty()){
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="exploration_complete";status_pub_->publish(status);stop(true);return;
  }
  geometry_msgs::msg::Point target_position;bool found=false;
  for(const auto&f:frontiers){
    double nearest=std::numeric_limits<double>::infinity();
    auto consider=[&](const geometry_msgs::msg::Point&p){
      const auto d=std::hypot(p.x-pose.position.x,p.y-pose.position.y);
      if(d>.25&&d<nearest&&!goalOnBlacklist(p)){nearest=d;target_position=p;found=true;}
    };
    consider(f.middle);
    for(const auto&p:f.points)consider(p);
    if(found)break;
  }
  if(!found){
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="exploration_waiting_candidates";status_pub_->publish(status);
    RCLCPP_INFO_THROTTLE(logger_,*get_clock(),5000,"前沿仍存在，但观测候选正在暂避或过近；保持未完成，等待地图及冷却");return;
  }
  last_failed_goal_.clear();
  prev_goal_=target_position;last_progress_=this->now();feedback_seen_=false;timeout_cancel_=false;
  auto goal=nav2_msgs::action::NavigateToPose::Goal();goal.pose.pose.position=target_position;
  goal.pose.pose.orientation.w=1.;goal.pose.header.frame_id=costmap_client_.getGlobalFrameID();goal.pose.header.stamp=this->now();
  goal_active_=true;
  const auto dispatch=++dispatch_generation_;
  RCLCPP_INFO(logger_,"探索选点 dispatch=%lu frontier=[%.3f,%.3f]；固定目标执行至终态",dispatch,target_position.x,target_position.y);
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this, dispatch](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (dispatch != dispatch_generation_ || stopped_) {
          if (goal_handle) move_base_client_->async_cancel_goal(goal_handle);
          return;
        }
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
          goal_memory_.record(prev_goal_,false,this->now().seconds());
        } else {
          navigation_goal_handle_ = goal_handle;
          active_goal_id_ = goal_handle->get_goal_id();
          if(timeout_cancel_)move_base_client_->async_cancel_goal(goal_handle);
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.feedback_callback=[this,dispatch](NavigationGoalHandle::SharedPtr handle,const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>f){
    if(dispatch!=dispatch_generation_||stopped_||!goal_active_||timeout_cancel_||handle->get_goal_id()!=active_goal_id_)return;
    const double d=f->distance_remaining;
    if(!std::isfinite(d)||d<=.15)return;
    if(!feedback_seen_){feedback_seen_=true;prev_distance_=d;}
    else if(prev_distance_-d>=progress_epsilon_){prev_distance_=d;last_progress_=this->now();}
  };
  send_goal_options.result_callback =
      [this, dispatch,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        if (dispatch != dispatch_generation_ || stopped_) return;
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  return goal_memory_.blocked(goal,this->now().seconds());
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_=false;navigation_goal_handle_.reset();
  const bool success=result.code==rclcpp_action::ResultCode::SUCCEEDED;
  const bool failure=result.code==rclcpp_action::ResultCode::ABORTED||timeout_cancel_;
  if(success||failure)goal_memory_.record(frontier_goal,success,this->now().seconds());
  RCLCPP_INFO(logger_,"探索目标 goal_id=%s result=%s reason=%s；保留区域记忆",rclcpp_action::to_string(result.goal_id).c_str(),
    success?"succeeded":(failure?"failed":"canceled"),timeout_cancel_?"配置窗口内无有效路径进展":"导航终态");
  timeout_cancel_=false;
  if(success){waiting_success_map_=true;success_map_sequence_=costmap_client_.mapSequence();}
  last_failed_goal_=result.code==rclcpp_action::ResultCode::ABORTED?rclcpp_action::to_string(result.goal_id):"";
  retryRejectedCandidate(last_failed_goal_);
  // 等下次timer；成功还必须等终态之后的新地图，禁止递归重选。

}

void Explore::retryRejectedCandidate(const std::string&goal_id){
  if(stopped_||goal_active_||goal_id.empty()||goal_id!=last_failed_goal_||
     std::find(candidate_rejections_.begin(),candidate_rejections_.end(),goal_id)==candidate_rejections_.end())return;
  const auto dispatch=dispatch_generation_;
  if(candidate_retry_timer_)candidate_retry_timer_->cancel();
  candidate_retry_timer_=create_wall_timer(std::chrono::milliseconds(100),[this,dispatch]{
    if(dispatch!=dispatch_generation_)return;
    candidate_retry_timer_->cancel();
    if(!stopped_&&!goal_active_)makePlan();
  });
  RCLCPP_INFO(logger_,"探索候选 goal_id=%s 未提交路径预检且失败终态已确认；100毫秒后继续选点，区域记忆保持",goal_id.c_str());
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  if (!exploring_timer_ || !rclcpp::ok()) return;
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;
  stopped_ = true;
  ++dispatch_generation_;
  last_failed_goal_.clear();
  if(candidate_retry_timer_)candidate_retry_timer_->cancel();
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  if (navigation_goal_handle_) {
    try {move_base_client_->async_cancel_goal(navigation_goal_handle_);}
    catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
      RCLCPP_DEBUG(logger_, "Owned goal already terminal; no other goal canceled");
    }
  }
  navigation_goal_handle_.reset();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  if (!stopped_) return;
  stopped_ = false;
  // 暂停/恢复不清区域记忆；新任务通过重启本节点建立新会话。
  last_progress_ = this->now();
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

#ifndef EXPLORE_NO_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}

#endif
