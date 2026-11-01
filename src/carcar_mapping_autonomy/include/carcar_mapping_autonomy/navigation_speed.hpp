// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

namespace carcar_mapping_autonomy {
// RPP 的期望速度与最终执行上限分别维护。参数服务失败不能撤销已收紧的执行上限。
// tick 只检查已经完成的 future，不能在监督器的 50ms 控制循环等待远程服务。
class MappingNavigationSpeed {
public:
  using Set=rcl_interfaces::srv::SetParametersAtomically;
  using Get=rcl_interfaces::srv::GetParameters;
  explicit MappingNavigationSpeed(rclcpp::Node & node):node_(node) {
    set_=node_.create_client<Set>("/controller_server/set_parameters_atomically");
    get_=node_.create_client<Get>("/controller_server/get_parameters");
  }
  static bool valid(double value) {return std::isfinite(value)&&value>=.05&&value<=.18;}
  bool pending() const {return phase_=="SETTING"||phase_=="READING";}
  double limit() const {return cap_;}
  bool reject(const std::string & request_id,const std::string & reason) {
    // 任务层拒绝不能覆盖另一笔仍在确认的调速事务，也不能修改已知执行上限。
    if(pending())return false;
    request_id_=request_id;phase_="FAILED";error_=reason;
    RCLCPP_WARN(node_.get_logger(),"NAV-018 调速未下发 request_id=%s cap_mps=%.3f reason=%s",
      request_id_.c_str(),cap_,error_.c_str());
    return false;
  }
  bool select(double value,double now,const std::string & request_id) {
    if(pending())return false;
    if(!valid(value))return reject(request_id,"网页调速请求被车端拒绝：速度须在0.05～0.18m/s之间；速度参数未修改。");
    requested_=value;request_id_=request_id;cap_=std::min(cap_,value);error_.clear();
    if(!set_->service_is_ready()||!get_->service_is_ready()) {
      fail("controller_server参数接口未就绪");return false;
    }
    begin(now);return pending();
  }
  void tick(double now) {
    // 每次启动恢复默认请求，避免沿用上一任务留在控制器中的高速参数。
    if(phase_=="WAITING"&&set_->service_is_ready()&&get_->service_is_ready())begin(now);
    if(!pending())return;
    if(now>deadline_) {
      if(phase_=="SETTING")set_->remove_pending_request(pending_id_);else get_->remove_pending_request(pending_id_);
      fail("controller_server参数写入／读回总等待超过1.5秒");return;
    }
    try {
      if(phase_=="SETTING"&&set_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        const auto response=set_future_.get();
        if(!response->result.successful){fail("控制器拒绝参数："+response->result.reason);return;}
        auto request=std::make_shared<Get::Request>();request->names={"FollowPath.desired_linear_vel"};
        auto result=get_->async_send_request(request);pending_id_=result.request_id;
        get_future_=result.future.share();phase_="READING";
      }
      if(phase_=="READING"&&get_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        const auto response=get_future_.get();
        if(response->values.size()!=1||response->values[0].type!=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE||
          !std::isfinite(response->values[0].double_value)||std::abs(response->values[0].double_value-requested_)>1e-9) {
          fail("控制器读回值与请求不一致");return;
        }
        confirmed_=requested_;confirmed_valid_=true;cap_=requested_;phase_="READY";error_.clear();
        RCLCPP_INFO(node_.get_logger(),"NAV-018 调速确认 request_id=%s speed_mps=%.3f cap_mps=%.3f；控制器读回一致，未下发导航目标",
          request_id_.c_str(),confirmed_,cap_);
      }
    }catch(const std::exception & e){fail(std::string("参数通信异常：")+e.what());}
  }
  nlohmann::json state() const {
    return {{"requested_mps",requested_},{"confirmed_mps",confirmed_valid_?nlohmann::json(confirmed_):nlohmann::json(nullptr)},
      {"cap_mps",cap_},{"pending",pending()},{"phase",phase_},{"error",error_},{"request_id",request_id_}};
  }
private:
  void begin(double now) {
    auto request=std::make_shared<Set::Request>();
    request->parameters={rclcpp::Parameter("FollowPath.desired_linear_vel",requested_).to_parameter_msg()};
    try {
      auto result=set_->async_send_request(request);pending_id_=result.request_id;
      set_future_=result.future.share();deadline_=now+1.5;phase_="SETTING";error_.clear();
    }catch(const std::exception & e){fail(std::string("参数请求未提交：")+e.what());}
  }
  void fail(const std::string & reason) {
    phase_="FAILED";error_="车端调速未确认："+reason+"；执行上限保留在"+
      std::to_string(cap_).substr(0,4)+"m/s，请检查导航参数服务后重试。";
    RCLCPP_WARN(node_.get_logger(),"NAV-018 调速失败 request_id=%s requested_mps=%.3f cap_mps=%.3f reason=%s",
      request_id_.c_str(),requested_,cap_,error_.c_str());
  }
  rclcpp::Node & node_;
  double requested_=.10,confirmed_=.10,cap_=.10,deadline_=0;
  bool confirmed_valid_=false;
  std::string phase_="WAITING",error_,request_id_;
  int64_t pending_id_=0;
  rclcpp::Client<Set>::SharedPtr set_;
  rclcpp::Client<Get>::SharedPtr get_;
  std::shared_future<Set::Response::SharedPtr> set_future_;
  std::shared_future<Get::Response::SharedPtr> get_future_;
};
}  // namespace carcar_mapping_autonomy
