#pragma once
#include <functional>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "carcar_mapping_autonomy/snapshot_process.hpp"
namespace carcar_mapping_autonomy {
struct CapturedSnapshot{GraphSnapshot archives;SnapshotJson metadata;};
class OnlineSnapshot {
public:
  using Capture=std::function<std::optional<CapturedSnapshot>(GraphSnapshot&,const SnapshotJson&)>;
  using Current=std::function<bool(const SnapshotJson&)>;
  OnlineSnapshot(rclcpp::Node&node,std::filesystem::path root,std::string worker,Capture capture,Current current):
    node_(node),root_(std::move(root)),worker_(std::move(worker)),capture_(std::move(capture)),current_(std::move(current)){
    publisher_=node_.create_publisher<std_msgs::msg::String>("/mapping/slam/snapshot/state",rclcpp::QoS(1).reliable().transient_local());
    autonomy_=node_.create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{autonomy_state_=SnapshotJson::parse(m.data);autonomy_at_=time();}catch(...){autonomy_state_=SnapshotJson::object();}});
    requests_=node_.create_subscription<std_msgs::msg::String>("/mapping/slam/snapshot/request",10,
      [this](const std_msgs::msg::String&m){request(m.data);});
    timer_=node_.create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  }
  ~OnlineSnapshot(){timer_.reset();process_.cancel("SLAM适配器退出，快照未提交");}
private:
  static double time(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  bool busy()const{return preparing_.valid()||copying_.valid()||prepared_.has_value()||process_.active();}
  bool valid()const{
    const auto phase=autonomy_state_.value("phase","");
    return time()-autonomy_at_<1&&autonomy_state_.value("healthy",false)&&snapshot_same_generation(request_,autonomy_state_)&&
      (phase=="MANUAL"||phase=="PAUSED"||phase=="EXPLORING"||phase=="RETURNING");
  }
  void fail(const std::string&reason){cancelled_=true;prepared_.reset();process_.cancel(reason);state_["phase"]="FAILED";state_["reason"]=reason;}
  void request(const std::string&text){
    try{
      if(busy())return; // 单个事务在途时忽略其他请求，坏请求不能取消已捕获副本。
      const auto incoming=SnapshotJson::parse(text);validate_snapshot_request(incoming);
      if(incoming.value("request_id","")==request_.value("request_id",""))return;
      request_=incoming;state_=request_;state_["phase"]="PREPARING";state_["reason"]="锁外准备捕获缓冲区";began_=time();cancelled_=false;worker_started_=false;
      if(!valid())throw std::runtime_error("快照请求与当前任务／坐标代次或健康状态不一致");
      preparing_=std::async(std::launch::async,[]{return prepare_graph_snapshot();});
    }catch(const std::exception&e){fail(e.what());}
  }
  void tick(){
    const bool was_busy=busy();
    try{
      if(busy()&&!cancelled_&&!valid())fail("任务取消、数据失效或坐标代次变化，未提升在途快照");
      if(busy()&&!cancelled_&&time()-began_>30)fail("快照事务超过30秒，旧可信快照保留");
      if(preparing_.valid()&&preparing_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        auto value=preparing_.get();if(!cancelled_)prepared_=std::move(value);
      }
      if(prepared_&&!cancelled_){
        state_["phase"]="CAPTURING";auto captured=capture_(*prepared_,request_);
        if(captured){
          prepared_.reset();metadata_=std::move(captured->metadata);
          state_["capture_ms"]=metadata_.at("capture_ms");state_["capture_steady_seconds"]=metadata_.at("capture_steady_seconds");
          if(metadata_.at("capture_ms").get<double>()>200)throw std::runtime_error("快照捕获超过200毫秒单次上限，未提交");
          state_["phase"]="COPYING";state_["reason"]="捕获锁已释放，向独立工作进程移交只读副本";
          copying_=std::async(std::launch::async,[archives=std::move(captured->archives)]()mutable{return snapshot_process_inputs(std::move(archives));});
        }
      }
      if(copying_.valid()&&copying_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
        auto inputs=copying_.get();if(!cancelled_){process_.start(worker_,root_,metadata_,std::move(inputs));worker_started_=true;}
      }
      process_.poll();
      if(worker_started_&&!cancelled_){
        const auto progress=process_.state();
        state_["phase"]=progress.value("phase","WRITING");
        if(state_["phase"]=="READY"&&process_.active())state_["phase"]="VALIDATING";
        if(!process_.active()){
          worker_started_=false;
          if(progress.value("phase","")=="READY"){
            if(!valid()||!current_(metadata_))throw std::runtime_error("保存期间生产图坐标代次变化，拒绝提升旧快照");
            state_["phase"]="READY";state_["path"]=progress.at("path");state_["manifest"]=progress.at("manifest");state_["reason"]="一致快照已完成后台落盘与校验";
          }else fail(progress.value("reason",std::string("快照后台处理失败，旧可信快照保留")));
        }
      }
    }catch(const std::exception&e){fail(e.what());}
    state_["in_flight"]=busy();
    // 终态耗时冻结；空闲心跳不能把已经结束的保存耗时不断增大。
    if(was_busy||busy()||!state_.contains("elapsed_seconds"))state_["elapsed_seconds"]=began_>0?time()-began_:0;
    std_msgs::msg::String message;message.data=state_.dump();publisher_->publish(message);
  }
  rclcpp::Node&node_;std::filesystem::path root_;std::string worker_;Capture capture_;Current current_;
  SnapshotJson request_=SnapshotJson::object(),state_={{"phase","IDLE"}},autonomy_state_=SnapshotJson::object(),metadata_;
  double began_=0,autonomy_at_=-1e9;bool cancelled_=false,worker_started_=false;
  std::future<GraphSnapshot>preparing_;std::optional<GraphSnapshot>prepared_;std::future<SnapshotInputs>copying_;SnapshotProcess process_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requests_,autonomy_;rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace carcar_mapping_autonomy
