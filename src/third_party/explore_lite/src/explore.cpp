#include <mutex>
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
#include <array>

#include <thread>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  path_aware_ranking_=declare_parameter("path_aware_ranking",false);
  if(path_aware_ranking_){
    rank_requests_=create_publisher<std_msgs::msg::String>("/mapping/exploration/rank_request",10);
    autonomy_state_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{autonomy_=nlohmann::json::parse(m.data);}catch(...){}});
    rank_results_=create_subscription<std_msgs::msg::String>("/mapping/exploration/rank_result",10,
      [this](const std_msgs::msg::String&m){try{const auto result=nlohmann::json::parse(m.data);
        if(rank_pending_&&!stopped_&&!goal_active_&&result.value("request_id",std::string())==rank_request_id_&&
           result.value("task_id",std::string())==autonomy_.value("task_id",std::string())&&
           result.value("generation",uint64_t{})==autonomy_.value("generation",uint64_t{})&&
           result.value("navigation_generation",uint64_t{})==navigation_generation_)rank_reply_=result;
      }catch(...){}});
  }
  progress_timeout_ = timeout;
  progress_epsilon_=declare_parameter<double>("progress_epsilon",.10);
  goal_memory_.radius=declare_parameter<double>("region_radius",.40);
  goal_memory_.cooldown=declare_parameter<double>("region_cooldown",60.0);
  rank_frontier_memory_.radius=goal_memory_.radius;
  rank_frontier_memory_.cooldown=goal_memory_.cooldown;

  if(!(std::isfinite(planner_frequency_)&&planner_frequency_>0&&planner_frequency_<=100&&
       std::isfinite(progress_timeout_)&&progress_timeout_>0&&std::isfinite(progress_epsilon_)&&progress_epsilon_>0&&
       std::isfinite(goal_memory_.radius)&&goal_memory_.radius>0&&std::isfinite(goal_memory_.cooldown)&&goal_memory_.cooldown>0&&
       std::isfinite(orientation_scale_)&&orientation_scale_>=0))throw std::invalid_argument("探索参数必须为有限正数且频率不超过100Hz，方向权重须为非负有限数");
  const auto generation_topic=declare_parameter<std::string>("navigation_generation_topic","");
  if(!generation_topic.empty())navigation_generation_sub_=create_subscription<std_msgs::msg::UInt64>(
    generation_topic,rclcpp::QoS(1).reliable().transient_local(),[this](const std_msgs::msg::UInt64&m){
      if(m.data==navigation_generation_)return;
      navigation_generation_=m.data;goal_memory_.clear();rank_frontier_memory_.clear();direction_.clear();waiting_success_map_=false;
      rank_pending_=false;rank_request_id_.clear();rank_reply_=nlohmann::json::object();rank_visits_=nlohmann::json::array();
      ranked_frontier_valid_=false;frontier_observed_cancel_=false;frontier_absent_updates_=0;frontier_observation_disabled_=false;
      RCLCPP_INFO(logger_,"地图坐标增代 generation=%lu；清理旧坐标中的区域和方向记忆",navigation_generation_);
    });
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
  if(path_aware_ranking_&&rankPending())return;
  if(goal_active_){
    // 有效目标继续保持；仅在原前沿邻域已被连续新图看清后取消过时的观测行程。
    // 沿用已有 action 取消和终态屏障，绝不在取消尚未完成时重发目标。
    if(navigation_goal_handle_&&!timeout_cancel_&&!frontier_observed_cancel_&&
       dispatch_navigation_generation_==navigation_generation_&&activeFrontierObserved()){
      frontier_observed_cancel_=true;
      RCLCPP_INFO(logger_,"探索目标 goal_id=%s event=frontier_observed reason=%s；有效前沿已由连续新地图确认消失，取消剩余行程并等待终态",
        rclcpp_action::to_string(active_goal_id_).c_str(),global_frontiers_observed_?"global_frontiers_empty":"source_frontier_observed");
      const auto dispatch=dispatch_generation_;
      try{
        move_base_client_->async_cancel_goal(navigation_goal_handle_,[this,dispatch](auto response){
          if(stopped_||!goal_active_||dispatch!=dispatch_generation_||!frontier_observed_cancel_)return;
          if(!response||response->return_code!=response->ERROR_NONE||response->goals_canceling.empty()){
            frontier_observed_cancel_=false;ranked_frontier_valid_=false;frontier_observation_disabled_=true;
            RCLCPP_WARN(logger_,"原前沿已观测但导航取消未获接受；保留原进展超时及当前目标，不重发目标");
          }
        });
      }catch(const std::exception&e){
        frontier_observed_cancel_=false;ranked_frontier_valid_=false;frontier_observation_disabled_=true;
        RCLCPP_WARN(logger_,"原前沿观测取消异常：%s；保留原进展超时，不重发目标",e.what());
      }
    }
    // 取消失去响应时仍保留原进展期限；不能用观测完成状态关闭原有无进展保护。
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
  geometry_msgs::msg::Pose pose;std::vector<frontier_exploration::Frontier> frontiers;
  if(!searchValidFrontiers(pose,frontiers))return;
  const double yaw=tf2::getYaw(pose.orientation);
  if(frontiers.empty()){
    if(visualize_)visualizeFrontiers(frontiers);
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="exploration_complete";status_pub_->publish(status);stop(true);return;
  }
  std::vector<std::pair<double,geometry_msgs::msg::Point>> rank_candidates;
  geometry_msgs::msg::Point target_position;bool found=false;
  double best=std::numeric_limits<double>::infinity();
  for(auto&f:frontiers){
    double nearest=std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Point candidate;
    auto consider=[&](const geometry_msgs::msg::Point&p){
      const auto d=std::hypot(p.x-pose.position.x,p.y-pose.position.y);
      const auto cost=d+direction_.penalty(pose.position,yaw,p,orientation_scale_);
      if(d>.25&&cost<nearest&&!frontierOnBlacklist(p)){nearest=cost;candidate=p;}
    };
    consider(f.middle);
    for(const auto&p:f.points)consider(p);
    if(!std::isfinite(nearest))continue;
    f.cost+=direction_.penalty(pose.position,yaw,candidate,orientation_scale_);
    if(path_aware_ranking_){
      // 同一连通前沿也可能绕过多个墙角；按方位保留分散代表，不能只试最近一个点。
      std::array<double,8>sector_distance;sector_distance.fill(std::numeric_limits<double>::infinity());
      std::array<geometry_msgs::msg::Point,8>sector_points;
      for(const auto&p:f.points){
        const double d=std::hypot(p.x-pose.position.x,p.y-pose.position.y);
        if(d<=.25||frontierOnBlacklist(p))continue;
        const double angle=std::atan2(p.y-pose.position.y,p.x-pose.position.x);
        const size_t sector=std::min<size_t>(7,size_t((angle+M_PI)/(M_PI/4)));
        if(d<sector_distance[sector]){sector_distance[sector]=d;sector_points[sector]=p;}
      }
      for(size_t i=0;i<8;++i)if(std::isfinite(sector_distance[i]))rank_candidates.emplace_back(
        f.cost+sector_distance[i]+direction_.penalty(pose.position,yaw,sector_points[i],orientation_scale_),sector_points[i]);
    }else rank_candidates.emplace_back(f.cost,candidate);
    if(f.cost<best){best=f.cost;target_position=candidate;found=true;}
  }
  std::stable_sort(frontiers.begin(),frontiers.end(),[](const auto&a,const auto&b){return a.cost<b.cost;});
  if(visualize_)visualizeFrontiers(frontiers);
  if(!found){
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="exploration_waiting_candidates";status_pub_->publish(status);
    RCLCPP_INFO_THROTTLE(logger_,*get_clock(),5000,"前沿仍存在，但观测候选正在暂避或过近；保持未完成，等待地图及冷却");return;
  }
  if(path_aware_ranking_){
    std::stable_sort(rank_candidates.begin(),rank_candidates.end(),[](const auto&a,const auto&b){return a.first<b.first;});
    std::vector<geometry_msgs::msg::Point> candidates;
    for(const auto&entry:rank_candidates){
      if(candidates.size()==8)break;
      if(std::none_of(candidates.begin(),candidates.end(),[&](const auto&p){return std::hypot(p.x-entry.second.x,p.y-entry.second.y)<.40;}))candidates.push_back(entry.second);
    }
    requestRanking(candidates);return;
  }
  dispatchGoal(pose,target_position);
}

void Explore::dispatchGoal(const geometry_msgs::msg::Pose&pose,const geometry_msgs::msg::Point&target_position){
  ranked_frontier_valid_=false;frontier_observed_cancel_=false;frontier_absent_updates_=0;frontier_observation_disabled_=false;
  frontier_map_sequence_=costmap_client_.mapSequence();
  last_failed_goal_.clear();
  prev_goal_=target_position;dispatch_start_=pose.position;dispatch_navigation_generation_=navigation_generation_;
  last_progress_=this->now();feedback_seen_=false;timeout_cancel_=false;
  auto goal=nav2_msgs::action::NavigateToPose::Goal();goal.pose.pose.position=target_position;
  const double approach=std::atan2(target_position.y-pose.position.y,target_position.x-pose.position.x);
  goal.pose.pose.orientation.z=std::sin(approach/2);goal.pose.pose.orientation.w=std::cos(approach/2);
  goal.pose.header.frame_id=costmap_client_.getGlobalFrameID();goal.pose.header.stamp=this->now();
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
          if(dispatch_navigation_generation_==navigation_generation_)goal_memory_.record(prev_goal_,false,this->now().seconds());
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

void Explore::requestRanking(const std::vector<geometry_msgs::msg::Point>&candidates){
  if(candidates.empty()||autonomy_.value("task_id",std::string()).empty())return;
  rank_request_id_=autonomy_.value("task_id",std::string())+"-rank-"+std::to_string(++rank_sequence_);
  rank_pending_=true;rank_started_=this->now().seconds();rank_reply_=nlohmann::json::object();
  auto points=nlohmann::json::array();for(const auto&p:candidates)points.push_back({p.x,p.y});
  auto blocked=nlohmann::json::array();
  for(const auto&region:goal_memory_.blocked_regions(this->now().seconds()))
    blocked.push_back({{"center",{region.point.x,region.point.y}},{"radius",region.radius},
      {"until",region.until},{"kind",region.visited?"visited_observation":"failed_observation"}});

  std_msgs::msg::String request;request.data=nlohmann::json{{"request_id",rank_request_id_},
    {"task_id",autonomy_.value("task_id",std::string())},{"generation",autonomy_.value("generation",uint64_t{})},
    {"navigation_generation",navigation_generation_},{"candidates",points},{"visits",rank_visits_},{"blocked_observations",blocked}}.dump();rank_requests_->publish(request);
}
bool Explore::rankPending(){
  if(!rank_pending_)return false;
  if(!rank_reply_.empty()){
    rank_pending_=false;const auto result=std::move(rank_reply_);rank_reply_=nlohmann::json::object();
    if(result.value("task_id",std::string())!=autonomy_.value("task_id",std::string())||
       result.value("generation",uint64_t{})!=autonomy_.value("generation",uint64_t{})||
       result.value("navigation_generation",uint64_t{})!=navigation_generation_)return true;
    if(result.contains("rejected")&&result["rejected"].is_array())for(const auto&candidate:result["rejected"]){
      try{geometry_msgs::msg::Point rejected;rejected.x=candidate.at("frontier").at(0);rejected.y=candidate.at("frontier").at(1);
        if(std::isfinite(rejected.x)&&std::isfinite(rejected.y))rank_frontier_memory_.record(rejected,false,this->now().seconds());
      }catch(const std::exception&){}
    }
    const auto pose=costmap_client_.getRobotPose();
    if(result.contains("candidates")&&result["candidates"].is_array())for(const auto&candidate:result["candidates"]){
      try{geometry_msgs::msg::Point target;const auto point=candidate.value("observation",candidate.at("frontier"));target.x=point.at(0);target.y=point.at(1);
        if(!std::isfinite(target.x)||!std::isfinite(target.y)||goalOnBlacklist(target))continue;
        RCLCPP_INFO(logger_,"探索路径排序 request_id=%s 路程=%.3fm 新增可见未知=%.3fm²；沿用固定目标和取消协议",
          rank_request_id_.c_str(),candidate.value("path_m",0.),candidate.value("gain_m2",0.));
        const auto source=candidate.at("frontier");
        geometry_msgs::msg::Point frontier;frontier.x=source.at(0);frontier.y=source.at(1);
        if(!std::isfinite(frontier.x)||!std::isfinite(frontier.y))continue;
        dispatchGoal(pose,target);
        ranked_frontier_=frontier;ranked_frontier_valid_=true;
        frontier_map_sequence_=costmap_client_.mapSequence();frontier_absent_updates_=0;
        return true;
      }catch(const std::exception&){}
    }
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="exploration_waiting_candidates";status_pub_->publish(status);
    RCLCPP_INFO_THROTTLE(logger_,*get_clock(),5000,"探索排序未得到可执行目标：%s",result.value("reason",std::string()).c_str());return true;
  }
  if(this->now().seconds()-rank_started_>6){
    rank_pending_=false;rank_request_id_.clear();
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="exploration_waiting_candidates";status_pub_->publish(status);
    RCLCPP_WARN(logger_,"探索只读排序超过6秒，丢弃迟到结果；未下发运动目标");
  }
  return true;
}

bool Explore::searchValidFrontiers(geometry_msgs::msg::Pose&pose,std::vector<frontier_exploration::Frontier>&frontiers){
  bool valid=false;pose=costmap_client_.getRobotPose(&valid);
  if(!valid||!std::isfinite(pose.position.x)||!std::isfinite(pose.position.y))return false;
  auto*map=costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
  unsigned mx=0,my=0;
  if(!map->getSizeInCellsX()||!map->getSizeInCellsY()||!map->getCharMap()||
     !std::isfinite(map->getResolution())||map->getResolution()<=0||
     !std::isfinite(map->getOriginX())||!std::isfinite(map->getOriginY())||
     !map->worldToMap(pose.position.x,pose.position.y,mx,my)||
     map->getCost(mx,my)!=nav2_costmap_2d::FREE_SPACE)return false;
  // 与原完成准则共用同一搜索与 min_frontier_size；冷却区域不参与完成判断。
  frontiers=search_.searchFrom(pose.position);return true;
}
bool Explore::activeFrontierObserved(){
  if(frontier_observation_disabled_)return false;
  const auto sequence=costmap_client_.mapSequence();
  if(sequence==frontier_map_sequence_)return false;
  frontier_map_sequence_=sequence;
  geometry_msgs::msg::Pose pose;std::vector<frontier_exploration::Frontier> frontiers;
  if(!searchValidFrontiers(pose,frontiers)){frontier_absent_updates_=0;return false;}
  const bool global_empty=frontiers.empty();
  if(!global_empty){
    if(!path_aware_ranking_||!ranked_frontier_valid_){frontier_absent_updates_=0;return false;}
    auto*map=costmap_client_.getCostmap();unsigned cx=0,cy=0;
    if(!map->worldToMap(ranked_frontier_.x,ranked_frontier_.y,cx,cy)){
      frontier_absent_updates_=0;return false;
    }
    auto nearby=[this](const geometry_msgs::msg::Point&p){
      return std::hypot(p.x-ranked_frontier_.x,p.y-ranked_frontier_.y)<=.40;};
    // 检查整个合法簇的点（含首格），不是可能随部分观测而偏移的centroid。
    for(const auto&frontier:frontiers)if(nearby(frontier.initial)||
        std::any_of(frontier.points.begin(),frontier.points.end(),nearby)){
      frontier_absent_updates_=0;return false;
    }
  }
  // 三次新地图且跨越至少0.5秒；完成仍需取消终态和后续新图的有效搜索。
  if(frontier_absent_updates_++==0){
    frontier_absent_since_=this->now().seconds();global_frontiers_observed_=global_empty;
  }else global_frontiers_observed_=global_frontiers_observed_&&global_empty;
  return frontier_absent_updates_>=3&&this->now().seconds()-frontier_absent_since_>=.5;
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
bool Explore::frontierOnBlacklist(const geometry_msgs::msg::Point&frontier){
  // 排序模式的前沿代表点失败只暂避源点，不能连带否决附近已验证安全的观测位置。
  return (path_aware_ranking_?rank_frontier_memory_:goal_memory_).blocked(frontier,this->now().seconds());
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
  const bool observed_cancel=frontier_observed_cancel_;
  frontier_observed_cancel_=false;ranked_frontier_valid_=false;
  const bool success=result.code==rclcpp_action::ResultCode::SUCCEEDED;
  const bool failure=result.code==rclcpp_action::ResultCode::ABORTED||timeout_cancel_;
  const bool same_coordinates=dispatch_navigation_generation_==navigation_generation_;
  if((success||failure)&&same_coordinates)goal_memory_.record(frontier_goal,success,this->now().seconds());
  RCLCPP_INFO(logger_,"探索目标 goal_id=%s result=%s reason=%s；保留区域记忆",rclcpp_action::to_string(result.goal_id).c_str(),
    success?"succeeded":(failure?"failed":"canceled"),timeout_cancel_?"配置窗口内无有效路径进展":"导航终态");
  timeout_cancel_=false;
  if(success&&same_coordinates&&path_aware_ranking_){
    if(rank_visits_.size()>=64)rank_visits_.erase(rank_visits_.begin());
    rank_visits_.push_back({{"point",{frontier_goal.x,frontier_goal.y}},{"at",this->now().seconds()}});
  }
  if(success&&same_coordinates){direction_.succeeded(dispatch_start_,costmap_client_.getRobotPose().position);
    waiting_success_map_=true;success_map_sequence_=costmap_client_.mapSequence();}
  if(observed_cancel&&!success&&!failure&&same_coordinates){
    // 取消成功不是到达原目标，不写“成功到点”记忆；只等待终态之后的新图再选点。
    waiting_success_map_=true;success_map_sequence_=costmap_client_.mapSequence();
  }
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
  rank_pending_=false;rank_request_id_.clear();rank_reply_=nlohmann::json::object();
  ranked_frontier_valid_=false;frontier_observed_cancel_=false;frontier_absent_updates_=0;frontier_observation_disabled_=false;
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
