#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include "carcar_mapping_autonomy/task.hpp"

namespace carcar_mapping_autonomy {
// 复用Humble BackUp动作；本模块只监督身份、轮式进展及真实终态，不发布任何速度。
// 来源：https://github.com/ros-navigation/navigation2/tree/humble/nav2_behaviors
class BackupMotion {
  using Action = nav2_msgs::action::BackUp;
  using Client = rclcpp_action::Client<Action>;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  struct Attempt {
    uint64_t generation=0;
    std::string request_id,phase="GOAL_PENDING",reason,result,goal_id;
    Pose origin;
    double distance=0,speed=0,allowance=0,started=0,last_progress_at=0;
    double progress=0,best_progress=0,lateral=0,yaw=0,cancel_at=0,verify_at=0;
    double feedback_distance=0;
    bool goal_pending=true,rejected=false,result_received=false,cancel_requested=false;
    bool cancel_pending=false,cancel_sent=false,issue=false,fault=false;
    // 协议结果到达后，允许外层继续喂入轮式反馈，记录取消／制动滑行；成功核验通过后不再追踪。
    bool track_after_result=false;
    Handle::SharedPtr goal;
  };
  struct Shared {
    std::mutex mutex;
    Client::SharedPtr client;
    std::shared_ptr<Attempt> current;
    uint64_t sequence=0;
    double now=0;
    bool owner_alive=true;
    std::string last_rejection;
  };
public:
  static constexpr double kMaxDistanceM=.25,kMaxSpeedMps=.03;
  static constexpr double kMaxLateralM=.01,kMaxYawRad=.05;
  explicit BackupMotion(rclcpp::Node &node):shared_(std::make_shared<Shared>()) {
    shared_->client=rclcpp_action::create_client<Action>(&node,"/backup");
  }
  BackupMotion(const BackupMotion&)=delete;
  BackupMotion&operator=(const BackupMotion&)=delete;
  ~BackupMotion() {
    std::shared_ptr<Attempt> attempt;
    {std::lock_guard<std::mutex> lock(shared_->mutex);shared_->owner_alive=false;
      attempt=shared_->current;if(attempt&&!done(*attempt))request_stop(*attempt,shared_->now,"后退监督器退出，取消动作");}
    if(attempt)send_cancel(shared_,attempt);
  }
  bool available()const{return shared_->client->action_server_is_ready();}
  bool start(double distance_m,double speed_mps,double time_allowance_s,const Pose&start_wheel,
             double steady_now,const std::string&request_id) {
    std::shared_ptr<Attempt> attempt;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      if(shared_->current&&(!done(*shared_->current)||shared_->current->track_after_result)){shared_->last_rejection="上一后退动作尚无真实终态或仍在核对停车";return false;}
      if(!std::isfinite(distance_m)||distance_m<=0||distance_m>kMaxDistanceM||
         !std::isfinite(speed_mps)||speed_mps<=0||speed_mps>kMaxSpeedMps||
         !std::isfinite(time_allowance_s)||time_allowance_s<=0||time_allowance_s>30||
         !valid_pose(start_wheel)||!std::isfinite(steady_now)||request_id.empty()) {
        shared_->last_rejection="后退距离、速度、时间预算、轮式位置或请求编号无效，未提交动作";return false;
      }
      if(!available()){shared_->last_rejection="/backup动作服务未就绪，未提交动作";return false;}
      attempt=std::make_shared<Attempt>();attempt->generation=++shared_->sequence;
      attempt->distance=distance_m;attempt->speed=speed_mps;attempt->allowance=time_allowance_s;
      attempt->origin=start_wheel;attempt->request_id=request_id;
      attempt->started=attempt->last_progress_at=steady_now;
      attempt->reason="已请求有界短退，等待/backup真实接受";
      shared_->now=steady_now;shared_->current=attempt;shared_->last_rejection.clear();
    }
    const auto shared=shared_;
    typename Client::SendGoalOptions options;
    options.goal_response_callback=[shared,attempt](Handle::SharedPtr goal){
      bool cancel=false;
      {
        std::lock_guard<std::mutex> lock(shared->mutex);
        attempt->goal_pending=false;attempt->goal=goal;
        if(!goal){attempt->rejected=true;attempt->issue=true;
          if(!attempt->fault)attempt->phase="FAILED";
          if(attempt->reason.empty()||!attempt->cancel_requested)attempt->reason="/backup拒绝后退请求，没有接受运动目标";
          return;
        }
        attempt->goal_id=rclcpp_action::to_string(goal->get_goal_id());
        if(shared->current!=attempt||!shared->owner_alive)
          request_stop(*attempt,shared->now,"后退接受应答已过期，取消该目标");
        cancel=attempt->cancel_requested;
        if(!cancel){attempt->phase="MOVING";attempt->reason="/backup已接受短退，持续核对轮式方向与安全条件";
          attempt->last_progress_at=shared->now;}
      }
      if(cancel)send_cancel(shared,attempt);
    };
    options.feedback_callback=[shared,attempt](Handle::SharedPtr goal,const std::shared_ptr<const Action::Feedback>feedback){
      std::lock_guard<std::mutex> lock(shared->mutex);
      if(shared->current!=attempt||attempt->cancel_requested||!attempt->goal||
         goal->get_goal_id()!=attempt->goal->get_goal_id()||!std::isfinite(feedback->distance_traveled))return;
      // 上游反馈只作诊断；不能以TF反馈距离代替有符号轮式进展和停稳确认。
      attempt->feedback_distance=feedback->distance_traveled;
    };
    options.result_callback=[shared,attempt](const Handle::WrappedResult&result){
      std::lock_guard<std::mutex> lock(shared->mutex);
      if(!attempt->goal||result.goal_id!=attempt->goal->get_goal_id())return;
      attempt->result_received=true;
      // 任何真实终态都继续记录制动滑行，直到严格停稳；成功回调也不能提前封存距离。
      attempt->track_after_result=true;
      // Humble目标句柄保留result_callback；结果到达后释放自持句柄，避免回调与状态形成引用环。
      attempt->goal.reset();
      switch(result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:attempt->result="SUCCEEDED";break;
        case rclcpp_action::ResultCode::CANCELED:attempt->result="CANCELED";break;
        case rclcpp_action::ResultCode::ABORTED:attempt->result="ABORTED";break;
        default:attempt->result="UNKNOWN";break;
      }
      if(shared->current!=attempt||!shared->owner_alive||attempt->cancel_requested||attempt->issue){
        attempt->issue=true;
        if(!attempt->fault)attempt->phase="FAILED";
        return;
      }
      if(result.code!=rclcpp_action::ResultCode::SUCCEEDED){attempt->issue=true;attempt->phase="FAILED";
        attempt->reason="/backup未成功完成，等待外层停稳确认";return;}
      attempt->verify_at=shared->now;attempt->phase="VERIFYING";
      attempt->reason="/backup报告完成，正在核对真实轮式后退距离与停稳";
    };
    Action::Goal goal;
    goal.target.x=-distance_m;goal.speed=static_cast<float>(speed_mps);
    goal.time_allowance=rclcpp::Duration::from_seconds(time_allowance_s);
    try{shared_->client->async_send_goal(goal,options);}
    catch(const std::exception&e){
      // 提交异常不等于服务端未收到；保留pending，禁止假造动作终态后再提交另一轮。
      std::lock_guard<std::mutex> lock(shared_->mutex);attempt->fault=attempt->issue=true;
      attempt->phase="FAULT";attempt->reason=std::string("/backup提交异常，接纳状态未知，保持锁停：")+e.what();
      request_stop(*attempt,shared_->now,attempt->reason);return false;
    }
    return true;
  }
  void cancel(const std::string&reason) {
    std::shared_ptr<Attempt> attempt;
    {std::lock_guard<std::mutex> lock(shared_->mutex);attempt=shared_->current;
      if(!attempt||done(*attempt))return;
      request_stop(*attempt,shared_->now,reason.empty()?"已取消自动短退":reason);}
    send_cancel(shared_,attempt);
  }
  void tick(double steady_now,const Pose&current_wheel,bool odom_fresh,bool still) {
    std::shared_ptr<Attempt> attempt;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);attempt=shared_->current;
      if(!attempt)return;
      // 真实取消／失败结果后仍接收制动阶段轮式反馈；动作协议已经结束，不能重新发动作。
      const bool post_result_track=done(*attempt)&&attempt->track_after_result;
      if(done(*attempt)&&!post_result_track)return;
      if(!std::isfinite(steady_now)||steady_now<shared_->now){
        attempt->fault=true;request_stop(*attempt,shared_->now,"后退单调时间无效或倒退，保持锁停");
      }else{
        shared_->now=steady_now;
        if(!odom_fresh||!valid_pose(current_wheel))request_stop(*attempt,steady_now,"后退轮式反馈过期或无效，立即取消并停车");
        else{
          const auto dx=current_wheel.x-attempt->origin.x,dy=current_wheel.y-attempt->origin.y;
          const auto c=std::cos(attempt->origin.yaw),s=std::sin(attempt->origin.yaw);
          attempt->progress=-(c*dx+s*dy);attempt->lateral=-s*dx+c*dy;
          attempt->yaw=std::remainder(current_wheel.yaw-attempt->origin.yaw,2*M_PI);
          if(attempt->progress<-.005)request_stop(*attempt,steady_now,"轮式反馈显示前进而非后退，立即取消");
          else if(std::abs(attempt->lateral)>kMaxLateralM)request_stop(*attempt,steady_now,"后退横向偏移超过1厘米，立即取消");
          else if(std::abs(attempt->yaw)>kMaxYawRad)request_stop(*attempt,steady_now,"后退朝向变化超过0.05弧度，立即取消");
          else if(attempt->progress>attempt->distance+.015)request_stop(*attempt,steady_now,"后退超过目标距离1.5厘米，立即取消");
          if(attempt->progress>attempt->best_progress+.001){attempt->best_progress=attempt->progress;attempt->last_progress_at=steady_now;}
          if(post_result_track&&still)attempt->track_after_result=false;
        }
        if(!attempt->cancel_requested&&!post_result_track){
          if(attempt->goal_pending&&steady_now-attempt->started>2)
            request_stop(*attempt,steady_now,"/backup接受应答超过2秒，取消意图保持至真实应答");
          else if(!attempt->result_received&&steady_now-attempt->started>attempt->allowance)
            request_stop(*attempt,steady_now,"自动短退时间预算耗尽，取消并停车");
          else if(attempt->phase=="MOVING"&&steady_now-attempt->last_progress_at>2)
            request_stop(*attempt,steady_now,"短退连续2秒没有有效后退进展，取消并停车");
          else if(attempt->phase=="VERIFYING"){
            const double tolerance=std::min(.005,attempt->distance*.1);
            if(odom_fresh&&still&&attempt->progress>=attempt->distance-tolerance){
              attempt->track_after_result=false;
              attempt->phase="SUCCEEDED";attempt->reason="短退动作、轮式距离与停稳核对均通过";
            }else if(steady_now-attempt->verify_at>2){
              attempt->issue=true;attempt->phase="FAILED";
              attempt->reason="/backup报告成功但轮式距离或停稳核对未通过，保持停车";
            }
          }
        }
      }
      if(attempt->cancel_requested&&!protocol_done(*attempt)&&shared_->now-attempt->cancel_at>5){
        attempt->fault=attempt->issue=true;attempt->phase="FAULT";
        attempt->reason="后退取消超过5秒仍缺少真实终态，保持锁停；不能视为动作已结束";
      }
    }
    send_cancel(shared_,attempt);
  }
  bool active()const{std::lock_guard<std::mutex> lock(shared_->mutex);return shared_->current&&!done(*shared_->current);}
  bool terminal()const{std::lock_guard<std::mutex> lock(shared_->mutex);return !shared_->current||done(*shared_->current);}
  // 协议终态与实体停稳分开报告；制动追踪结束前不能交接或覆盖本轮动作。
  bool settled()const{std::lock_guard<std::mutex> lock(shared_->mutex);return !shared_->current||
    (done(*shared_->current)&&!shared_->current->track_after_result);}
  bool succeeded()const{std::lock_guard<std::mutex> lock(shared_->mutex);return shared_->current&&shared_->current->phase=="SUCCEEDED"&&done(*shared_->current);}
  bool failed()const{std::lock_guard<std::mutex> lock(shared_->mutex);return shared_->current&&shared_->current->issue;}
  double distance_progress()const{std::lock_guard<std::mutex> lock(shared_->mutex);return shared_->current?shared_->current->progress:0;}
  Json status()const{
    std::lock_guard<std::mutex> lock(shared_->mutex);const auto attempt=shared_->current;
    if(!attempt)return {{"phase","IDLE"},{"active",false},{"terminal",true},{"settled",true},{"last_rejection",shared_->last_rejection}};
    return {{"phase",attempt->phase},{"reason",attempt->reason},{"request_id",attempt->request_id},{"generation",attempt->generation},
      {"distance_m",attempt->distance},{"speed_mps",attempt->speed},{"time_allowance_s",attempt->allowance},
      {"progress_m",attempt->progress},{"lateral_m",attempt->lateral},{"yaw_delta_rad",attempt->yaw},
      {"feedback_distance_m",attempt->feedback_distance},{"goal_pending",attempt->goal_pending},
      {"cancel_requested",attempt->cancel_requested},{"cancel_pending",attempt->cancel_pending},
      {"result_received",attempt->result_received},{"result",attempt->result},{"fault",attempt->fault},
      {"failed",attempt->issue},{"active",!done(*attempt)},{"terminal",done(*attempt)},
      {"settled",done(*attempt)&&!attempt->track_after_result},
      {"goal_id",attempt->goal_id},
      {"last_rejection",shared_->last_rejection}};
  }
private:
  static bool valid_pose(const Pose&p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);}
  static bool protocol_done(const Attempt&a){return !a.goal_pending&&!a.cancel_pending&&(a.rejected||a.result_received);}
  static bool done(const Attempt&a){return protocol_done(a)&&a.phase!="VERIFYING";}
  static void request_stop(Attempt&a,double now,const std::string&reason){
    if(!a.cancel_requested){a.cancel_requested=true;a.cancel_at=now;a.reason=reason;}
    a.issue=true;if(a.fault)a.phase="FAULT";else a.phase="CANCELING";
    // 结果已经到达时无需再向已终结的动作发取消；外层仍必须核对停稳。
    if(a.result_received||a.rejected){if(!a.fault)a.phase="FAILED";}
  }
  static void send_cancel(const std::shared_ptr<Shared>&shared,const std::shared_ptr<Attempt>&attempt){
    Handle::SharedPtr goal;
    {
      std::lock_guard<std::mutex> lock(shared->mutex);
      if(!attempt->cancel_requested||!attempt->goal||attempt->result_received||attempt->cancel_sent)return;
      attempt->cancel_sent=attempt->cancel_pending=true;goal=attempt->goal;
    }
    try{
      shared->client->async_cancel_goal(goal,[shared,attempt](typename Client::CancelResponse::SharedPtr response){
        std::lock_guard<std::mutex> lock(shared->mutex);attempt->cancel_pending=false;
        if(!response||response->return_code!=0){
          // 取消应答只说明请求处理结果；真正结束必须等result_callback。
          if(!attempt->fault)attempt->reason+="；/backup取消未获接受，仍等待真实终态";
        }
      });
    }catch(const std::exception&e){
      std::lock_guard<std::mutex> lock(shared->mutex);
      // 发送异常后不能证明取消请求不存在；保留pending并交由5秒故障门保持锁停。
      attempt->fault=attempt->issue=true;attempt->phase="FAULT";
      attempt->reason=std::string("/backup取消发送异常，终态未知，保持锁停：")+e.what();
    }
  }
  std::shared_ptr<Shared> shared_;
};
}  // namespace carcar_mapping_autonomy
