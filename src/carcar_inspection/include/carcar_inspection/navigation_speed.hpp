// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "carcar_inspection/domain.hpp"
#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

namespace carcar_inspection {
// 一次切档原子更新控制器，再读回确认；控制循环从不等待远程服务。
class NavigationSpeed {
public:
  using Set=rcl_interfaces::srv::SetParametersAtomically;
  using Get=rcl_interfaces::srv::GetParameters;
  explicit NavigationSpeed(rclcpp::Node & node,bool enabled)
  : node_(node),enabled_(enabled) {
    if(enabled_) {
      set_=node_.create_client<Set>("/controller_server/set_parameters_atomically");
      get_=node_.create_client<Get>("/controller_server/get_parameters");
    }
  }
  static bool valid(double speed) {
    for(const double value:{.10,.18,.25,.30})if(std::isfinite(speed)&&std::abs(speed-value)<1e-9)return true;
    return false;
  }
  bool ready() const {return !enabled_||phase_=="READY";}
  bool pending() const {return phase_=="SETTING"||phase_=="READING";}
  double limit() const {return ready()?applied_:0;}
  void select(double speed,double now) {
    require(enabled_,"当前入口未启用导航速度调档，没有修改档位或下发目标");
    require(valid(speed),"只支持0.10、0.18、0.25、0.30m/s四个速度档，没有修改档位或下发目标");
    require(!pending(),"上一次速度切档尚未确认，没有重复修改或下发目标");
    require(set_->service_is_ready()&&get_->service_is_ready(),"车端健康检查拒绝切档：controller_server参数接口未就绪，没有修改档位或下发目标；请查看导航服务");
    requested_=speed;begin(now);
  }
  void tick(double now) {
    if(!enabled_)return;
    if(phase_=="WAITING"&&set_->service_is_ready()&&get_->service_is_ready())begin(now);
    if(!pending())return;
    if(phase_=="SETTING"&&set_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      const auto result=set_future_.get();
      if(!result->result.successful){fail("控制器拒绝参数："+result->result.reason);return;}
      auto request=std::make_shared<Get::Request>();
      request->names={"FollowPath.max_vel_x","FollowPath.max_speed_xy"};
      auto response=get_->async_send_request(request);
      request_id_=response.request_id;get_future_=response.future.share();phase_="READING";deadline_=now+1.5;
    }
    if(phase_=="READING"&&get_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      const auto result=get_future_.get();
      bool matches=result->values.size()==2;
      for(const auto & value:result->values)matches=matches&&value.type==rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE&&std::abs(value.double_value-requested_)<1e-9;
      if(!matches){fail("控制器参数读回与请求档位不一致");return;}
      applied_=requested_;phase_="READY";
      message_="速度档位已确认："+std::to_string(applied_).substr(0,4)+"m/s；只修改上限，没有下发目标或启动行驶";
      RCLCPP_INFO(node_.get_logger(),"APP-001 切档确认 target=/api/navigation/speed speed_mps=%.2f result=READY；控制器读回一致，没有下发目标",applied_);
    }
    if(pending()&&now>deadline_) {
      if(phase_=="SETTING")set_->remove_pending_request(request_id_);
      else get_->remove_pending_request(request_id_);
      fail("controller_server参数接口等待超过1.5秒");
    }
  }
  json state() const {
    return {{"enabled",enabled_},{"phase",enabled_?phase_:"DISABLED"},{"pending",pending()},
      {"ready",ready()},{"requested_mps",requested_},{"applied_mps",phase_=="READY"?json(applied_):json(nullptr)},
      {"message",enabled_?message_:"当前入口未启用导航速度调档"},
      {"options",json::array({{{"mps",.10},{"label","1档 · 0.10 m/s"}},{{"mps",.18},{"label","2档 · 0.18 m/s"}},
        {{"mps",.25},{"label","3档 · 0.25 m/s（实验）"}},{{"mps",.30},{"label","4档 · 0.30 m/s（实验）"}}})}};
  }
private:
  void begin(double now) {
    auto request=std::make_shared<Set::Request>();
    // Humble DWB按参数顺序处理；重申原min_speed_theta=0以更新max_speed_xy的平方缓存。
    for(const auto & parameter:std::vector<rclcpp::Parameter>{{"FollowPath.max_vel_x",requested_},
      {"FollowPath.max_speed_xy",requested_},{"FollowPath.min_speed_theta",0.0}})request->parameters.push_back(parameter.to_parameter_msg());
    auto response=set_->async_send_request(request);
    request_id_=response.request_id;set_future_=response.future.share();phase_="SETTING";deadline_=now+1.5;
    message_="正在切换速度档位，等待控制器确认；没有下发目标";
  }
  void fail(const std::string & reason) {
    phase_="FAILED";message_="车端健康检查拒绝切档："+reason+"；档位未确认，目标未下发，导航保持禁止。请查看controller_server参数接口并重试切档";
    RCLCPP_ERROR(node_.get_logger(),"APP-001 切档失败 target=/api/navigation/speed speed_mps=%.2f reason=%s",requested_,message_.c_str());
  }
  rclcpp::Node & node_;bool enabled_;
  double requested_=.18,applied_=0,deadline_=0;int64_t request_id_=0;
  std::string phase_="WAITING",message_="等待controller_server参数接口，默认请求0.18m/s；没有下发目标";
  rclcpp::Client<Set>::SharedPtr set_;rclcpp::Client<Get>::SharedPtr get_;
  std::shared_future<Set::Response::SharedPtr> set_future_;
  std::shared_future<Get::Response::SharedPtr> get_future_;
};
}  // namespace carcar_inspection
