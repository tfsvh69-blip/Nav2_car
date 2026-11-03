// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>

namespace carcar_mapping_autonomy {
struct TuningValues {
  double global_radius=.5,global_factor=3.,local_radius=.3,local_factor=5.;
  nlohmann::json json()const{return {{"global_inflation_radius",global_radius},{"global_cost_scaling_factor",global_factor},
    {"local_inflation_radius",local_radius},{"local_cost_scaling_factor",local_factor}};}
  static TuningValues parse(const nlohmann::json&j){
    if(!j.is_object()||j.size()!=4)throw std::runtime_error("代价配置必须且只能包含四个参数");
    TuningValues v;std::array<double*,4>dest{{&v.global_radius,&v.global_factor,&v.local_radius,&v.local_factor}};
    const std::array<std::string,4>keys{{"global_inflation_radius","global_cost_scaling_factor","local_inflation_radius","local_cost_scaling_factor"}};
    for(size_t i=0;i<keys.size();++i){if(!j.contains(keys[i])||!j.at(keys[i]).is_number())throw std::runtime_error(keys[i]+"必须为有限数值");
      *dest[i]=j.at(keys[i]).get<double>();if(!std::isfinite(*dest[i]))throw std::runtime_error(keys[i]+"必须为有限数值");}
    if(v.global_radius<.22||v.global_radius>1.5||v.local_radius<.22||v.local_radius>1.)
      throw std::runtime_error("全局半径须在0.22～1.50米、局部半径须在0.22～1.00米");
    if(v.global_factor<.5||v.global_factor>15||v.local_factor<.5||v.local_factor>15)
      throw std::runtime_error("代价衰减系数须在0.5～15之间");
    return v;
  }
};
inline bool tuning_equal(double a,double b){return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-9;}
inline bool tuning_equal(const TuningValues&a,const TuningValues&b){return tuning_equal(a.global_radius,b.global_radius)&&
  tuning_equal(a.global_factor,b.global_factor)&&tuning_equal(a.local_radius,b.local_radius)&&tuning_equal(a.local_factor,b.local_factor);}
// 较大的反算系数只证明同一非零软代价格的估距更保守，不声称整条执行链无瞬态差异。
inline double tuning_guard_factor(double old_factor,double next_factor){return std::max(old_factor,next_factor);}
inline double tuning_cost_distance(double nominal,double local_radius){return std::min(nominal,local_radius);}
inline bool tuning_ack_matches(const nlohmann::json&a,const std::string&node,int64_t revision,double radius,double factor){
  try{return a.is_object()&&a.value("completed",false)&&a.at("node")==node&&a.at("layer")=="inflation_layer"&&
    a.at("revision").get<int64_t>()==revision&&tuning_equal(a.at("radius").get<double>(),radius)&&
    tuning_equal(a.at("factor").get<double>(),factor)&&a.value("full_rebuild",false);}
  catch(const std::exception&){return false;}
}
}
