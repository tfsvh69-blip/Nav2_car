#pragma once
#include <atomic>
#include <thread>
#include <carcar_interfaces/srv/inspection_runtime.hpp>
#include <carcar_interfaces/msg/inspection_runtime_state.hpp>
#include "carcar_mapping_console/task_process.hpp"
#include "carcar_mapping_console/publication.hpp"
namespace carcar_mapping_console {
// 唯一网页进程与按需硬件进程分开监督，所有阻塞退出都在工作线程完成。
class InspectionRuntime {
public:
  using State = carcar_interfaces::msg::InspectionRuntimeState;
  using Request = carcar_interfaces::srv::InspectionRuntime::Request;
  using Response = carcar_interfaces::srv::InspectionRuntime::Response;
  InspectionRuntime(std::filesystem::path root, std::filesystem::path logs, std::string data, unsigned port, bool supervise=true)
  : root_(std::move(root)), logs_(std::move(logs)), data_(std::move(data)), port_(port) {
    state_.phase="CONFIG_ONLY";
    if(supervise)worker_=std::thread([this]{loop();});
  }
  ~InspectionRuntime(){running_=false;if(worker_.joinable())worker_.join();}
  bool occupied() const {std::lock_guard<std::mutex> lock(mutex_);return occupied_;}
  State state() const {std::lock_guard<std::mutex> lock(mutex_);return state_;}
  void heartbeat(const State & value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(value.generation==state_.generation)heartbeat_=steady();
  }
  void request(const Request & input, Response & output, bool mapping_free) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if(input.request_id.empty()||input.request_id.size()>100)throw std::runtime_error("请求编号无效");
      if(input.action=="prepare") {
        if(input.request_id==state_.request_id){output.accepted=true;}
        else {
          if(occupied_||!mapping_free)throw std::runtime_error("建图或巡检仍占用车辆，请先结束车辆工作");
          if(input.map_id.empty()||input.map_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
            throw std::runtime_error("地图编号无效");
          auto yaml=root_/input.map_id/"map.yaml";
          if(!std::filesystem::is_regular_file(yaml))yaml=root_.parent_path()/(input.map_id+".yaml");
          if(!std::filesystem::is_regular_file(yaml)||!saved_map_navigation_allowed(yaml))
            throw std::runtime_error("地图尚未完整保存或存在质量异常，未启动车辆；请先查看地图状态");
          if(std::filesystem::space(logs_.parent_path()).available<20ULL*1024*1024*1024)throw std::runtime_error("磁盘不足20GiB预留空间，未启动车辆");
          map_=yaml;state_.generation=nonce();state_.request_id=input.request_id;state_.phase="STARTING";
          state_.reason="准备请求已接受，等待硬件与本次定位；没有下发目标";
          occupied_=true;start_=true;heartbeat_=steady();output.accepted=true;
        }
      } else if(input.action=="end") {
        if(input.generation!=state_.generation)throw std::runtime_error("车辆运行代次已变化，未结束其他任务");
        if(occupied_){stop_=true;state_.phase="STOPPING";state_.reason="等待巡检硬件进程完全退出";}
        output.accepted=true;
      } else throw std::runtime_error("车辆操作无效");
    }catch(const std::exception&e){output.accepted=false;output.reason=e.what();}
    output.generation=state_.generation;output.phase=state_.phase;
    if(output.reason.empty())output.reason=state_.reason;
  }
private:
  void loop() {
    double retry=0;
    while(running_) {
      try {
        if(!console_.alive()&&steady()>=retry) {
          // 控制台丢失时先清理本组硬件，不让重启后的配置进程继承运动。
          robot_.stop();console_.stop();
          {std::lock_guard<std::mutex> lock(mutex_);if(occupied_){occupied_=false;start_=stop_=false;state_.phase="FAULT";state_.reason="巡检控制台退出，硬件已关闭；旧任务不会恢复";}}
          console_.start({"ros2","launch","carcar_inspection","console.launch.xml","port:="+std::to_string(port_),
            "data_root:="+data_,"map_root:="+root_.parent_path().string(),"managed_runtime:=true","require_depth:=false","cmd_vel_topic:=/inspection/collision_input"},logs_/("inspection_console_"+nonce()+".log"));
          retry=steady()+5;
        }
        bool launch=false,stop=false;std::filesystem::path map;
        {std::lock_guard<std::mutex> lock(mutex_);launch=start_;start_=false;map=map_;
          if(occupied_&&!launch&&state_.phase=="ACTIVE"&&(!robot_.alive()||steady()-heartbeat_>2)){
            stop_=true;state_.phase="FAULT";state_.reason="硬件进程退出或巡检控制心跳超时，关闭硬件并保留记录";
          }
          stop=stop_;stop_=false;
        }
        if(launch&&!stop){
          robot_.start({"ros2","launch","carcar_inspection","inspection_lidar.launch.xml","real_hardware:=true",
            "start_console:=false","map:="+map.string()},logs_/("inspection_vehicle_"+nonce()+".log"));
          std::lock_guard<std::mutex> lock(mutex_);state_.phase="ACTIVE";heartbeat_=steady();
        }
        if(stop){robot_.stop();std::lock_guard<std::mutex> lock(mutex_);occupied_=false;
          if(state_.phase!="FAULT"){state_.phase="CONFIG_ONLY";state_.reason="车辆硬件已退出，配置和照片继续可用";}}
      }catch(const std::exception&e){robot_.stop();std::lock_guard<std::mutex> lock(mutex_);occupied_=false;start_=stop_=false;state_.phase="FAULT";state_.reason=e.what();retry=steady()+5;}
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    robot_.stop();console_.stop();
  }
  std::filesystem::path root_,logs_,map_;std::string data_;unsigned port_;
  mutable std::mutex mutex_;State state_;bool occupied_=false,start_=false,stop_=false;double heartbeat_=0;
  TaskProcess console_,robot_;std::atomic<bool> running_{true};std::thread worker_;
};
}
