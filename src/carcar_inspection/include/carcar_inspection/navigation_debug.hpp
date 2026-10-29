// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <opencv2/imgcodecs.hpp>
#include <nlohmann/json.hpp>

namespace carcar_inspection {
// 只读显示缓存。代价栅格保留原分辨率；路径仅为显示限到约400个点。
// PNG在独立线程生成，订阅回调只保留最新一帧，避免占住巡检控制循环。
// odom数据使用最新map<-odom变换，与RViz Use Timestamp=false一致。
class NavigationDebug {
  using Json=nlohmann::json;
  struct Layer {
    bool received=false,accepted=false; double at=0,stamp=0,accepted_at=0;uint64_t sequence=0,latest_serial=0;
    std::string frame;nav_msgs::msg::MapMetaData info;
    Json points=Json::array();std::vector<uint8_t> png;
  };
  struct Draft {
    bool occupied=false;size_t index=0;uint64_t serial=0;double at=0,stamp=0;
    std::string frame;nav_msgs::msg::MapMetaData info;std::vector<int8_t> data;
  };
 public:
  NavigationDebug() : worker_([this]{encode_loop();}) {}
  NavigationDebug(const NavigationDebug &)=delete;
  NavigationDebug & operator=(const NavigationDebug &)=delete;
  ~NavigationDebug() {
    {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}
    cv_.notify_all();
    if(worker_.joinable())worker_.join();
  }
  void grid(size_t index,const nav_msgs::msg::OccupancyGrid & value,double at) {
    if(index>=2||!value.info.width||!value.info.height||!std::isfinite(value.info.resolution)||value.info.resolution<=0||
      static_cast<uint64_t>(value.info.width)*value.info.height>4000000||
      value.data.size()!=static_cast<size_t>(value.info.width)*value.info.height)return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto & layer=grids_[index];
      if(layer.accepted&&at-layer.accepted_at<.5)return;
    }
    Draft draft;
    draft.occupied=true;draft.index=index;draft.at=at;draft.stamp=stamp(value.header.stamp);
    draft.frame=value.header.frame_id;draft.info=value.info;draft.data=value.data;
    {
      std::lock_guard<std::mutex> lock(mutex_);auto & layer=grids_[index];
      if(layer.accepted&&at-layer.accepted_at<.5)return;
      layer.accepted=true;layer.accepted_at=at;draft.serial=++layer.latest_serial;
      pending_[index]=std::move(draft);has_pending_=true;
    }
    cv_.notify_one();
  }
  // 测试等待这一层最新一帧完成编码。网页读取不调用它。
  bool wait_for(size_t index,std::chrono::milliseconds budget)const {
    const auto deadline=std::chrono::steady_clock::now()+budget;
    while(std::chrono::steady_clock::now()<deadline){
      {std::lock_guard<std::mutex> lock(mutex_);
        if(index<2&&grids_[index].received&&grids_[index].latest_serial>0&&grids_[index].sequence==grids_[index].latest_serial)return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }
  void path(size_t index,const nav_msgs::msg::Path & value,double at) {
    if(index>=2)return;
    std::lock_guard<std::mutex> lock(mutex_);auto & layer=paths_[index];
    layer.received=true;layer.at=at;layer.stamp=stamp(value.header.stamp);++layer.sequence;
    layer.frame=value.header.frame_id;layer.points=Json::array();
    const size_t stride=std::max<size_t>(1,(value.poses.size()+399)/400);
    for(size_t i=0;i<value.poses.size();i+=stride){const auto & pose=value.poses[i];
      if((!pose.header.frame_id.empty()&&pose.header.frame_id!=layer.frame)||
        !std::isfinite(pose.pose.position.x)||!std::isfinite(pose.pose.position.y)){layer.points=Json::array();layer.frame.clear();return;}
      layer.points.push_back({pose.pose.position.x,pose.pose.position.y});
    }
    if(!value.poses.empty()){const auto & p=value.poses.back().pose.position;
      if(std::isfinite(p.x)&&std::isfinite(p.y))layer.points.push_back({p.x,p.y});}
  }
  std::pair<std::vector<uint8_t>,Json> image(size_t index,tf2_ros::Buffer & buffer,double at,double ros_time)const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(index>=2)return {};
    return {grids_[index].png,grid_info(grids_[index],buffer,at,ros_time)};
  }
  Json snapshot(tf2_ros::Buffer & buffer,double at,double ros_time)const {
    std::lock_guard<std::mutex> lock(mutex_);Json result;
    const std::array<std::string,2> grid_names{"global_costmap","local_costmap"},path_names{"global_path","local_path"};
    for(size_t i=0;i<2;++i){
      auto info=grid_info(grids_[i],buffer,at,ros_time);
      result[grid_names[i]]=std::move(info);
      info=describe(paths_[i],at,ros_time);auto transform=lookup(buffer,paths_[i].frame,info);info["points"]=Json::array();
      if(info.value("available",false))for(const auto & point:paths_[i].points)
        info["points"].push_back(convert(point[0],point[1],transform));
      result[path_names[i]]=std::move(info);
    }
    return result;
  }
 private:
  void encode_loop() {
    while(true){
      Draft job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[this]{return stop_||has_pending_;});
        if(stop_)return;
        const size_t index=pending_[0].occupied?0:1;
        job=std::move(pending_[index]);pending_[index].occupied=false;
        has_pending_=pending_[0].occupied||pending_[1].occupied;
      }
      std::vector<uint8_t> encoded;
      if(!encode_png(job,encoded))continue;
      std::lock_guard<std::mutex> lock(mutex_);
      if(stop_)return;
      auto & layer=grids_[job.index];
      if(job.serial!=layer.latest_serial)continue;
      layer.received=true;layer.at=job.at;layer.stamp=job.stamp;layer.sequence=job.serial;
      layer.frame=std::move(job.frame);layer.info=job.info;layer.png=std::move(encoded);
    }
  }
  static bool encode_png(const Draft & job,std::vector<uint8_t> & encoded) {
    if(!job.info.width||!job.info.height||job.data.size()!=static_cast<size_t>(job.info.width)*job.info.height)return false;
    cv::Mat pixels(job.info.height,job.info.width,CV_8UC4,cv::Scalar(0,0,0,0));
    for(uint32_t y=0;y<job.info.height;++y)for(uint32_t x=0;x<job.info.width;++x){
      const auto cost=job.data[y*job.info.width+x];if(cost<=0)continue;
      auto & pixel=pixels.at<cv::Vec4b>(job.info.height-1-y,x);
      pixel=cost>=99?cv::Vec4b(55,45,225,210):job.index==0?
        cv::Vec4b(210,60,135,static_cast<uint8_t>(45+cost)):
        cv::Vec4b(25,145,245,static_cast<uint8_t>(45+cost));
    }
    return cv::imencode(".png",pixels,encoded,{cv::IMWRITE_PNG_COMPRESSION,1});
  }
  static Json grid_info(const Layer & layer,tf2_ros::Buffer & buffer,double at,double ros_time){
    auto info=describe(layer,at,ros_time);const auto transform=lookup(buffer,layer.frame,info);
    if(info.value("available",false)){
      const auto & grid=layer.info;const auto origin=convert(grid.origin.position.x,grid.origin.position.y,transform);
      info["width"]=grid.width;info["height"]=grid.height;info["resolution"]=grid.resolution;
      info["origin"]={{"x",origin[0]},{"y",origin[1]},{"yaw",heading(grid.origin.orientation)+transform[2]}};
    }
    return info;
  }
  static double stamp(const builtin_interfaces::msg::Time & value){return value.sec+value.nanosec*1e-9;}
  static double heading(const geometry_msgs::msg::Quaternion & q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
  static Json describe(const Layer & layer,double at,double ros_time){return {{"available",layer.received},{"source_frame",layer.frame},
    {"frame","map"},{"transform","latest"},{"sequence",layer.sequence},{"age_seconds",layer.received?at-layer.at:0},
    {"source_age_seconds",layer.received?ros_time-layer.stamp:0},{"error",layer.received?"":"尚未收到数据"}};}
  static std::array<double,3> lookup(tf2_ros::Buffer & buffer,const std::string & frame,Json & info){
    if(!info.value("available",false))return {};
    if(frame=="map")return {};
    try{if(frame.empty())throw std::runtime_error("消息缺少坐标系");
      const auto t=buffer.lookupTransform("map",frame,tf2::TimePointZero);
      return {t.transform.translation.x,t.transform.translation.y,heading(t.transform.rotation)};
    }catch(const std::exception & error){info["available"]=false;info["error"]=std::string("坐标转换未就绪：")+error.what();return {};}
  }
  static std::array<double,2> convert(double x,double y,const std::array<double,3> & t){
    return {t[0]+std::cos(t[2])*x-std::sin(t[2])*y,t[1]+std::sin(t[2])*x+std::cos(t[2])*y};}
  mutable std::mutex mutex_;std::condition_variable cv_;std::array<Layer,2>grids_,paths_;
  std::array<Draft,2> pending_{};bool has_pending_=false,stop_=false;std::thread worker_;
};
}  // namespace carcar_inspection
