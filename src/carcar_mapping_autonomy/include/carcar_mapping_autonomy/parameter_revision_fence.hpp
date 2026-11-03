// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace carcar_mapping_autonomy {
// 本回调必须在被保护插件activate之后注册，使拒绝发生在上游会改变内部状态的回调之前。
class ParameterRevisionFence {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&parent,const std::string&prefix){
    node_=parent;names_={prefix+".inflation_cost_scaling_factor",prefix+".cost_scaling_dist"};revision_name_=prefix+".tuning_revision";
    // 插件参数名须带点，避免ControllerServer把其误当服务器参数而等待整段导航持有的锁。
    const auto n=node_.lock();if(!n->has_parameter(revision_name_))n->declare_parameter<int64_t>(revision_name_,0);
    revision_=n->get_parameter(revision_name_).as_int();
  }
  void activate(){handle_=node_.lock()->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>&params){
    rcl_interfaces::msg::SetParametersResult out;out.successful=true;bool controlled=false;int64_t next=-1;
    for(const auto&p:params){
      if(p.get_name()==revision_name_){
        if(p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER){out.successful=false;break;}next=p.as_int();
      }
      if(std::find(names_.begin(),names_.end(),p.get_name())!=names_.end()){
        controlled=true;if(p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE||!std::isfinite(p.as_double())||p.as_double()<=0){out.successful=false;break;}
      }
    }
    if((controlled&&next<0)||(next>=0&&next<=revision_))out.successful=false;
    if(!out.successful)out.reason="调参代次过期、缺失或参数无效，未更改控制器内部值";
    else if(next>=0)revision_=next;
    return out;
  });}
  void deactivate(){handle_.reset();}
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;std::vector<std::string>names_;std::string revision_name_;int64_t revision_=0;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr handle_;
};
}
