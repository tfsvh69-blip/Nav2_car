// 复用 Nav2 1.1.20 BtActionNode 客户端/回调组；基类 tick/halt 的 ROS 时钟等待不满足本项目取消边界。
#pragma once
#include <map>
#include <sstream>
#include <iomanip>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include "carcar_navigation/recovery_runtime.hpp"

namespace carcar_navigation {
template<class ActionT>
class BoundedAction final : public nav2_behavior_tree::BtActionNode<ActionT>, public ManagedSession
{
  using Base=nav2_behavior_tree::BtActionNode<ActionT>;
  using Client=rclcpp_action::Client<ActionT>;
  using Handle=rclcpp_action::ClientGoalHandle<ActionT>;
  struct Record {
    typename Handle::SharedPtr handle;
    bool terminal{false}, cancel_sent{false}, cancel_ack{false};
    bool response_done{false},cancel_response_done{false};
    TimePoint submitted{Steady::now()};
    TimePoint superseded_at{};
  };
  struct Shared {
    mutable std::mutex mutex;
    std::map<uint64_t,Record> records;
    uint64_t revision{0};
    ActionState status;
    TimePoint sent{}, cancel_at{};
    double result_timeout{0};
    typename ActionT::Result::SharedPtr result;
  };
public:
  BoundedAction(const std::string & name,const std::string & action,
    const BT::NodeConfiguration & config,double response_timeout,double cancel_timeout)
  : Base(name,action,client_config(config)), shared_(std::make_shared<Shared>()),
    response_timeout_(response_timeout),cancel_timeout_(cancel_timeout)
  {
    // 回调只捕获共享状态，不捕获 BT 节点 this。
    auto executor=std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>(
      &this->callback_group_executor_,[](auto*) {});
    thread_=std::make_unique<nav2_util::NodeThread>(executor);
  }
  ~BoundedAction() override {
    cancel();
    thread_.reset();
  }
  void send(const typename ActionT::Goal & goal,double result_timeout)
  {
    auto s=shared_;
    uint64_t revision;
    {
      std::lock_guard<std::mutex> lock(s->mutex);
      if (s->status.cancel_requested) {return;}
      for (auto it=s->records.begin();it!=s->records.end();) {
        const auto & r=it->second;
        if (r.terminal && r.response_done && (!r.cancel_sent || r.cancel_response_done)) {
          it=s->records.erase(it);
        } else {++it;}
      }
      if (s->revision>0) {
        auto prev=s->records.find(s->revision);
        if (prev!=s->records.end() && prev->second.superseded_at==TimePoint{}) {
          prev->second.superseded_at=Steady::now();
        }
      }
      const bool keep_accepted=s->status.accepted && !s->status.rejected && !s->status.timed_out;
      revision=++s->revision;
      s->records.emplace(revision,Record{});
      s->sent=Steady::now(); s->result_timeout=result_timeout;
      if (!keep_accepted) {s->status.accepted=false;}
      s->status.terminal=false; s->status.success=false;
      s->status.rejected=false;
    }
    typename Client::SendGoalOptions options;
    std::weak_ptr<Client> weak_client=this->action_client_;
    std::weak_ptr<Shared> weak_state=s;
    options.goal_response_callback=[weak_state,weak_client,revision](typename Handle::SharedPtr handle) {
      auto s=weak_state.lock();if (!s) {return;}
      std::lock_guard<std::mutex> lock(s->mutex);
      auto it=s->records.find(revision); if (it==s->records.end()) {return;}
      auto & rec=it->second;
      rec.response_done=true;
      rec.handle=handle;
      if (!handle) {
        rec.terminal=true;
        if (revision==s->revision) {
          s->status.accepted=false; s->status.rejected=true; s->status.reason="GOAL_REJECTED";
        }
      } else {
        if (revision==s->revision) {
          s->status.accepted=true;
          std::ostringstream out;
          for (auto b : handle->get_goal_id()) {out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);}
          s->status.uuid=out.str();
        } else {
          cancel_record(s,weak_client,revision);
        }
        // halt 先于目标响应时，这里接住晚到 handle 并只取消本次 UUID。
        if (s->status.cancel_requested) {cancel_record(s,weak_client,revision);}
      }
      refresh_terminal(*s);
    };
    options.result_callback=[weak_state,revision](const typename Handle::WrappedResult & result) {
      auto s=weak_state.lock();if (!s) {return;}
      std::lock_guard<std::mutex> lock(s->mutex);
      auto it=s->records.find(revision); if (it==s->records.end()) {return;}
      it->second.terminal=true;
      if (revision==s->revision) {
        s->result=result.result;
        s->status.success=result.code==rclcpp_action::ResultCode::SUCCEEDED;
        s->status.reason=s->status.success?"SUCCEEDED":
          (result.code==rclcpp_action::ResultCode::CANCELED?"CANCELED":"ABORTED");
      }
      refresh_terminal(*s);
    };
    try {this->action_client_->async_send_goal(goal,options);}
    catch (const std::exception & e) {
      std::lock_guard<std::mutex> lock(s->mutex);
      // 发送异常无法证明服务器没收到，保持未确认并进入取消路径。
      s->status.reason=std::string("SEND_ERROR: ")+e.what();
      s->status.timed_out=true;
    }
  }
  void poll() override
  {
    bool request_cancel=false;
    {
      std::lock_guard<std::mutex> lock(shared_->mutex);
      auto & s=*shared_;
      if (s.status.terminal || s.revision==0) {return;}
      const double elapsed=seconds(s.sent);
      bool current_responded=false;
      auto live=s.records.find(s.revision);
      if (live!=s.records.end()) {current_responded=live->second.response_done;}
      // 被替换的旧目标只从 superseded_at 起算孤儿时限；不得因此取消当前代次。
      for (auto & entry : s.records) {
        if (entry.first>=s.revision || entry.second.terminal) {continue;}
        const TimePoint origin=entry.second.superseded_at!=TimePoint{}?entry.second.superseded_at:s.sent;
        if (seconds(origin)>response_timeout_+cancel_timeout_) {
          cancel_record(shared_,this->action_client_,entry.first);
          entry.second.terminal=true;
        }
      }
      refresh_terminal(s);
      if (s.status.terminal) {return;}
      if (!s.status.cancel_requested &&
        ((!current_responded && elapsed>response_timeout_) ||
        (s.result_timeout>0 && elapsed>s.result_timeout) || s.status.timed_out)) {
        s.status.reason=(current_responded || s.status.accepted) && s.result_timeout>0 && elapsed>s.result_timeout?
          "RESULT_TIMEOUT":"GOAL_RESPONSE_TIMEOUT";
        s.status.timed_out=true;
        if (!current_responded) {s.status.accepted=false;}
        request_cancel=true;
      }
      if (s.status.cancel_requested && seconds(s.cancel_at)>cancel_timeout_) {
        s.status.timed_out=true; s.status.reason="CANCEL_UNCONFIRMED";
      }
    }
    if (request_cancel) {cancel();}
  }
  void cancel() override
  {
    auto s=shared_;
    std::lock_guard<std::mutex> lock(s->mutex);
    if (s->status.terminal || s->revision==0) {return;}
    if (!s->status.cancel_requested) {s->status.cancel_requested=true; s->cancel_at=Steady::now();}
    for (auto & rec : s->records) {cancel_record(s,this->action_client_,rec.first);}
  }
  ActionState state() const override
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    auto result=shared_->status;
    result.elapsed=shared_->sent==TimePoint{}?0:seconds(shared_->sent);
    result.revision=shared_->revision;
    result.response_remaining=std::max(0.0,response_timeout_-result.elapsed);
    result.result_remaining=shared_->result_timeout>0?std::max(0.0,shared_->result_timeout-result.elapsed):-1;
    result.cancel_remaining=shared_->status.cancel_requested?
      std::max(0.0,cancel_timeout_-seconds(shared_->cancel_at)):cancel_timeout_;
    return result;
  }
  typename ActionT::Result::SharedPtr result() const {
    std::lock_guard<std::mutex> lock(shared_->mutex); return shared_->result;
  }
  BT::NodeStatus tick() override {
    poll(); auto s=state();
    return s.terminal?(s.success?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE):BT::NodeStatus::RUNNING;
  }
  void halt() override {cancel(); this->setStatus(BT::NodeStatus::IDLE);}
private:
  static BT::NodeConfiguration client_config(const BT::NodeConfiguration & source) {
    // Action 会被运行上下文保留以处理晚到取消；不能反过来持有整个树的 blackboard。
    BT::NodeConfiguration result;
    result.blackboard=BT::Blackboard::create();
    result.blackboard->set("node",source.blackboard->get<rclcpp::Node::SharedPtr>("node"));
    for (auto key : {"bt_loop_duration","server_timeout","wait_for_service_timeout"}) {
      std::chrono::milliseconds value(1000);source.blackboard->get(key,value);
      // 发现阶段由叶节点跨 tick 完成；就绪检查之后服务器消失也不能阻塞 BT 构造。
      if (std::string(key)=="wait_for_service_timeout") {value=std::chrono::milliseconds(0);}
      result.blackboard->set(key,value);
    }
    return result;
  }
  static void refresh_terminal(Shared & s) {
    s.status.terminal=!s.records.empty(); s.status.cancel_ack=true;
    for (const auto & pair : s.records) {
      s.status.terminal=s.status.terminal && pair.second.terminal;
      s.status.cancel_ack=s.status.cancel_ack && (pair.second.terminal || pair.second.cancel_ack);
    }
  }
  // 调用方持有共享状态锁；响应回调仅更新共享状态。
  static void cancel_record(std::shared_ptr<Shared> s,std::weak_ptr<Client> weak,uint64_t revision) {
    auto it=s->records.find(revision); if (it==s->records.end()) {return;}
    auto & rec=it->second;
    if (!rec.handle || rec.cancel_sent) {return;}
    auto client=weak.lock(); if (!client) {return;}
    rec.cancel_sent=true;
    try {
      std::weak_ptr<Shared> weak_state=s;
      client->async_cancel_goal(rec.handle,[weak_state,revision](auto response) {
        auto s=weak_state.lock();if (!s) {return;}
        std::lock_guard<std::mutex> lock(s->mutex);
        auto rec_it=s->records.find(revision); if (rec_it==s->records.end()) {return;}
        auto & record=rec_it->second;
        record.cancel_response_done=true;
        for (const auto & info : response->goals_canceling) {
          if (record.handle && info.goal_id.uuid==record.handle->get_goal_id()) {record.cancel_ack=true;}
        }
        refresh_terminal(*s);
      });
    } catch (const std::exception &) {s->status.reason="CANCEL_SEND_ERROR";}
  }
  std::shared_ptr<Shared> shared_;
  double response_timeout_,cancel_timeout_;
  std::unique_ptr<nav2_util::NodeThread> thread_;
};
}  // namespace carcar_navigation
