// Copyright (c) 2021 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <angles/angles.h>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>
#include <nav2_controller/plugins/position_goal_checker.hpp>
#include "carcar_mapping_autonomy/parameter_revision_fence.hpp"
namespace carcar_mapping_autonomy {
// 复用Humble 1.1.20的起步对准、碰撞检查与末端朝向；补齐短路径采样及控制位姿时间一致性。
// RPP自己的rotate_to_heading须关闭，避免路径跟踪时反复进入另一层对准。
class MappingRotationShim:public nav2_rotation_shim_controller::RotationShimController {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer>tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>costmap)override{
    RotationShimController::configure(parent,name,tf,costmap);sampling_limit_=forward_sampling_distance_;
    tuning_fence_.configure(parent,name);
    timing_=parent.lock()->create_publisher<std_msgs::msg::String>("/mapping/controller_timing",rclcpp::QoS(1).best_effort());
    histogram_.fill(0);samples_=overruns_=exceptions_=0;maximum_ms_=total_ms_=0;
  }
  void activate()override{RotationShimController::activate();tuning_fence_.activate();timing_->on_activate();}
  void deactivate()override{tuning_fence_.deactivate();timing_->on_deactivate();RotationShimController::deactivate();}
  void cleanup()override{timing_.reset();RotationShimController::cleanup();}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped&pose,
      const geometry_msgs::msg::Twist&velocity,nav2_core::GoalChecker*checker)override{
    const auto began=std::chrono::steady_clock::now();
    try{
    const auto result=compute_at_pose_time(pose,velocity,checker);
    record_time(began,false);return result;
    }catch(...){record_time(began,true);throw;}
  }
private:
  // Nav2 1.1.20 RotationShimController::computeVelocityCommands 的局部适配：
  // https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp
  // 原实现 Copyright (c) 2021 Samsung Research, Apache-2.0；旋转速度/加速度与碰撞预测继续直接复用父类。
  // 上游非虚采样函数将stamp设为now，会等待尚未到达的实时TF。统一使用Nav2已取得的本周期车体位姿时间。
  geometry_msgs::msg::PoseStamped at_pose_time(geometry_msgs::msg::PoseStamped point,
      const geometry_msgs::msg::PoseStamped&pose)const{
    point.header.frame_id=current_path_.header.frame_id;point.header.stamp=pose.header.stamp;
    if(point.header.frame_id==pose.header.frame_id)return point;
    geometry_msgs::msg::PoseStamped transformed;
    try{tf_->transform(point,transformed,pose.header.frame_id,tf2::durationFromSec(0.0));}
    catch(const tf2::TransformException&e){
      throw nav2_core::PlannerException(std::string("当前控制位姿时间的TF不可用，停止本周期控制：")+e.what());
    }
    return transformed;
  }
  geometry_msgs::msg::TwistStamped compute_at_pose_time(const geometry_msgs::msg::PoseStamped&pose,
      const geometry_msgs::msg::Twist&velocity,nav2_core::GoalChecker*checker){
    const rclcpp::Time stamp(pose.header.stamp,clock_->get_clock_type());
    const double age=(clock_->now()-stamp).seconds();
    if(stamp.nanoseconds()==0||age<0||age>costmap_ros_->getTransformTolerance())
      throw nav2_core::PlannerException("控制位姿时间无效或过期，保留原costmap变换新鲜度门槛");
    auto*costmap=costmap_ros_->getCostmap();
    {
      // 与上游路径转向分支保持costmap、参数互斥量的获取顺序；原碰撞检查仍在锁内执行。
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>map_lock(*(costmap->getMutex()));
      std::lock_guard<std::mutex>parameters_lock(mutex_);
      if(path_updated_&&!current_path_.poses.empty()){
        if(current_path_.poses.size()==1&&current_path_.header.frame_id==pose.header.frame_id)
          current_path_.poses.insert(current_path_.poses.begin(),pose);
        const auto&start=current_path_.poses.front().pose.position;double extent=0;
        for(const auto&point:current_path_.poses)extent=std::max(extent,
          std::hypot(point.pose.position.x-start.x,point.pose.position.y-start.y));
        forward_sampling_distance_=std::min(sampling_limit_,std::max(.001,extent*.9));
      }
      // 探索只验收位置；普通返程仍保留末端朝向。避免成功判定前一周期多发末端转向。
      if(rotate_to_goal_heading_&&!current_path_.poses.empty()&&
         dynamic_cast<nav2_controller::PositionGoalChecker*>(checker)==nullptr){
        const auto goal=at_pose_time(getSampledPathGoal(),pose);
        geometry_msgs::msg::Pose pose_tolerance;geometry_msgs::msg::Twist velocity_tolerance;
        checker->getTolerances(pose_tolerance,velocity_tolerance);
        position_goal_checker_->setXYGoalTolerance(pose_tolerance.position.x);
        if(position_goal_checker_->isGoalReached(pose.pose,goal.pose,velocity)){
          const double heading=angles::shortest_angular_distance(
            tf2::getYaw(pose.pose.orientation),tf2::getYaw(goal.pose.orientation));
          try{
            auto result=computeRotateToHeadingCommand(heading,pose,velocity);
            last_angular_vel_=result.twist.angular.z;return result;
          }catch(const std::runtime_error&e){
            RCLCPP_DEBUG(logger_,"末端旋转检查拒绝，交由主控制器原检查处理：%s",e.what());
          }
        }
      }
      if(path_updated_){
        // 采样失败保留上游回退行为；TF失败必须传播，不能把缺少坐标依据误当成已对准。
        geometry_msgs::msg::PoseStamped sample;bool sampled=false;
        try{sample=getSampledPathPt();sampled=true;}catch(const std::runtime_error&){path_updated_=false;}
        if(sampled){
          const auto point=at_pose_time(sample,pose);
          const double direction=std::atan2(point.pose.position.y-pose.pose.position.y,
            point.pose.position.x-pose.pose.position.x);
          const double heading=angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation),direction);
          const double threshold=in_rotation_?angular_disengage_threshold_:angular_dist_threshold_;
          if(std::abs(heading)>threshold){
            in_rotation_=true;
            try{
              auto result=computeRotateToHeadingCommand(heading,pose,velocity);
              last_angular_vel_=result.twist.angular.z;return result;
            }catch(const std::runtime_error&){path_updated_=false;}
          }else path_updated_=false;
        }
      }
    }
    in_rotation_=false;
    auto result=primary_controller_->computeVelocityCommands(pose,velocity,checker);
    last_angular_vel_=result.twist.angular.z;return result;
  }
  void record_time(std::chrono::steady_clock::time_point began,bool failed)noexcept{
    try{
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
      ++samples_;exceptions_+=failed;overruns_+=ms>100.;maximum_ms_=std::max(maximum_ms_,ms);total_ms_+=ms;
      ++histogram_[std::min<size_t>(800,std::ceil(ms*4))];
      if(samples_%10!=1&&ms<=100)return;
      const uint64_t threshold=std::ceil(samples_*.99);uint64_t count=0;size_t bucket=0;
      for(;bucket<800;++bucket){count+=histogram_[bucket];if(count>=threshold)break;}
      // 固定内存累计直方图，不在控制线程保存无限样本；四分之一毫秒分桶给出P99上界。
      std_msgs::msg::String message;message.data=nlohmann::json{{"controller","MappingRotationShim+RPP"},
        {"samples",samples_},{"p99_upper_ms",bucket==800?maximum_ms_:bucket*.25},{"quantile_bin_ms",.25},
        {"max_ms",maximum_ms_},{"mean_ms",total_ms_/samples_},{"over_100ms",overruns_},{"exceptions",exceptions_},
        {"scope","computeVelocityCommands"}}.dump();
      if(timing_&&timing_->is_activated())timing_->publish(message);
    }catch(...){/* 诊断输出失败不能改变控制器原有返回或异常语义。 */}
  }
  double sampling_limit_=.30;
  ParameterRevisionFence tuning_fence_;
  std::array<uint64_t,801>histogram_{};uint64_t samples_=0,overruns_=0,exceptions_=0;
  double maximum_ms_=0,total_ms_=0;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr timing_;
};
}
