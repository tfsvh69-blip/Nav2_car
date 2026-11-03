// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include "carcar_mapping_autonomy/revision_inflation_layer.hpp"
#include <cmath>
#include <nlohmann/json.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace carcar_mapping_autonomy {
void RevisionInflationLayer::onInitialize(){
  InflationLayer::onInitialize();const auto node=node_.lock();node_name_=node->get_node_base_interface()->get_fully_qualified_name();
  declareParameter("tuning_revision",rclcpp::ParameterValue(int64_t(0)));
  revision_=node->get_parameter(name_+".tuning_revision").as_int();
  publisher_=node->create_publisher<std_msgs::msg::String>("/mapping/tuning/inflation_ack",rclcpp::QoS(20).reliable().transient_local());publisher_->on_activate();
  revision_handler_=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>&params){
    std::lock_guard<mutex_t>lock(*getMutex());rcl_interfaces::msg::SetParametersResult result;result.successful=true;
    bool changed=false;int64_t next=-1;
    for(const auto&p:params){
      if(p.get_name()==name_+".tuning_revision"){
        if(p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER){result.successful=false;break;}next=p.as_int();
      }
      if(p.get_name()==name_+".inflation_radius"||p.get_name()==name_+".cost_scaling_factor"){
        changed=true;if(p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE||!std::isfinite(p.as_double())||p.as_double()<=0){result.successful=false;break;}
      }
    }
    if((changed&&next<0)||(next>=0&&next<=revision_))result.successful=false;
    if(!result.successful)result.reason="膨胀参数代次缺失、过期或数值无效，未改变膨胀层";
    else if(next>=0){revision_=next;need_reinflation_=true;}
    return result;
  });
}
void RevisionInflationLayer::updateBounds(double x,double y,double yaw,double*min_x,double*min_y,double*max_x,double*max_y){
  // 上一周期未进入costs时不发布完成；使用本周期真实边界计算重新取证。
  std::lock_guard<mutex_t>lock(*getMutex());
  cycle_revision_=revision_;cycle_radius_=inflation_radius_;cycle_factor_=cost_scaling_factor_;
  cycle_full_=need_reinflation_;cycle_started_=true;
  try{InflationLayer::updateBounds(x,y,yaw,min_x,min_y,max_x,max_y);}
  catch(...){cycle_started_=false;throw;}
}
void RevisionInflationLayer::updateCosts(nav2_costmap_2d::Costmap2D&master,int min_i,int min_j,int max_i,int max_j){
  std::lock_guard<mutex_t>lock(*getMutex());
  try{
    InflationLayer::updateCosts(master,min_i,min_j,max_i,max_j);
    if(cycle_started_&&cycle_full_&&enabled_&&cycle_revision_>=reported_revision_&&cycle_revision_==revision_&&
       cycle_radius_==inflation_radius_&&cycle_factor_==cost_scaling_factor_&&
       min_i==0&&min_j==0&&max_i>=int(master.getSizeInCellsX())&&max_j>=int(master.getSizeInCellsY())){
      std_msgs::msg::String m;m.data=nlohmann::json{{"node",node_name_},{"layer",name_},{"revision",cycle_revision_},
        {"radius",cycle_radius_},{"factor",cycle_factor_},{"completed",true},{"full_rebuild",true}}.dump();
      publisher_->publish(m);reported_revision_=cycle_revision_;
    }
    cycle_started_=false;
  }catch(...){cycle_started_=false;throw;}
}
}
PLUGINLIB_EXPORT_CLASS(carcar_mapping_autonomy::RevisionInflationLayer,nav2_costmap_2d::Layer)
