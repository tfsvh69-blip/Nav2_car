// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include <array>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/string.hpp>
#include "carcar_mapping_autonomy/tuning_transaction.hpp"

namespace carcar_mapping_autonomy {
class MappingTuningNode:public rclcpp::Node {
public:
  using Json=nlohmann::json;using Get=rcl_interfaces::srv::GetParameters;using Set=rcl_interfaces::srv::SetParametersAtomically;using Life=lifecycle_msgs::srv::GetState;
  explicit MappingTuningNode(const rclcpp::NodeOptions&options=rclcpp::NodeOptions()):Node("mapping_tuning",options){
    task_id_=declare_parameter("task_id",std::string());
    service_timeout_=declare_parameter("service_timeout_seconds",3.0);
    if(!std::isfinite(service_timeout_)||service_timeout_<.2||service_timeout_>5)throw std::runtime_error("参数服务期限须在0.2～5秒");
    const char*home=std::getenv("HOME");const std::string default_file=home?std::string(home)+"/.local/state/carcar_mapping_console/tuning-defaults.json":std::string();
    defaults_file_=declare_parameter("defaults_file",default_file);
    try{
      if(defaults_file_.empty()||!std::filesystem::path(defaults_file_).is_absolute())throw std::runtime_error("默认配置路径必须为绝对路径");
      if(std::filesystem::exists(defaults_file_)){
        if(!std::filesystem::is_regular_file(defaults_file_)||std::filesystem::file_size(defaults_file_)>65536)
          throw std::runtime_error("默认配置不是有效的小型普通文件");
        std::ifstream stream(defaults_file_);Json value;stream>>value;
        defaults_=TuningValues::parse(value.contains("values")?value.at("values"):value);have_defaults_=true;
      }
    }catch(const std::exception&e){defaults_error_=std::string("默认配置无效，未应用：")+e.what();}
    for(size_t i=0;i<nodes_.size();++i){set_[i]=create_client<Set>(nodes_[i]+"/set_parameters_atomically");get_[i]=create_client<Get>(nodes_[i]+"/get_parameters");life_[i]=create_client<Life>(nodes_[i]+"/get_state");}
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    state_pub_=create_publisher<std_msgs::msg::String>("/mapping/tuning/state",qos);
    requests_=create_subscription<std_msgs::msg::String>("/mapping/tuning/request",rclcpp::QoS(10),[this](const std_msgs::msg::String&m){request(m.data);});
    acks_=create_subscription<std_msgs::msg::String>("/mapping/tuning/inflation_ack",rclcpp::QoS(20).reliable().transient_local(),[this](const std_msgs::msg::String&m){
      try{auto a=Json::parse(m.data);const auto node=a.at("node").get<std::string>();for(size_t i=0;i<2;++i)if(node==nodes_[i]){ack_[i]=std::move(a);ack_at_[i]=seconds();}}
      catch(const std::exception&){}
    });
    autonomy_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",qos,[this](const std_msgs::msg::String&m){observe(m.data);});
    startup_deadline_=seconds()+60;timer_=create_wall_timer(std::chrono::milliseconds(25),[this]{tick();});
    if(!defaults_error_.empty()){phase_="FAILED";error_=defaults_error_;requires_stop_=true;}publish();
  }
  Json state()const{
    Json rejected=Json::array();for(const auto&r:rejected_)rejected.push_back(r);
    return {{"task_id",task_id_},{"generation",observed_generation_},{"request_generation",request_generation_},{"request_id",request_id_},
      {"request_result",request_result_},{"phase",phase_},{"ready",ready_&&(phase_=="READY"||phase_=="APPLYING")&&!requires_stop_},
      {"pending",phase_=="APPLYING"||phase_=="WAITING_STOP"||phase_=="ROLLING_BACK"},
      {"requested",requested_.json()},{"confirmed",confirmed_valid_?confirmed_.json():Json(nullptr)},{"values",confirmed_valid_?confirmed_.json():Json(nullptr)},
      {"baseline",baseline_.json()},{"error",error_},{"requires_stop",requires_stop_},{"rolled_back",rolled_back_},{"rollback_ready",rolled_back_&&ready_},
      {"defaults_loaded",have_defaults_},{"rejected_requests",rejected},{"transaction_revision",revision_},
      {"expected_revisions",expected_revision_},{"inflation_ack",ack_},
      {"stage",steps_.empty()?std::string():steps_.front().label}};
  }
  ~MappingTuningNode()override{remove_pending();}
private:
  enum class Kind{LIFECYCLE,READ,WRITE,ACK};
  struct Step{Kind kind;size_t node=0;std::vector<rclcpp::Parameter>parameters;int64_t revision=0;double radius=0,factor=0;std::string label;};
  struct Actual{TuningValues values;double rpp_factor=5.,rpp_distance=.3;std::array<int64_t,3>revision{{0,0,0}};};
  static double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  static bool identifier(const std::string&id){return !id.empty()&&id.size()<=128&&std::all_of(id.begin(),id.end(),[](unsigned char c){return std::isalnum(c)||c=='_'||c=='-';});}
  int64_t revision(){const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    revision_=std::max<int64_t>(revision_+1,stamp);return revision_;}
  void publish(){std_msgs::msg::String m;m.data=state().dump();state_pub_->publish(m);last_publish_=seconds();}
  bool stopped()const{return seconds()-autonomy_at_<=.6&&autonomy_at_>stop_requested_at_&&autonomy_hold_ack_&&
    autonomy_safe_phase_&&autonomy_still_&&autonomy_terminal_&&autonomy_zero_;}
  void observe(const std::string&text){
    try{const auto j=Json::parse(text);const auto task=j.at("task_id").get<std::string>();if(task.empty())return;
      if(task_id_.empty())task_id_=task;
      if(task!=task_id_){if(phase_=="APPLYING"||phase_=="ROLLING_BACK"||phase_=="WAITING_STOP"){
          remove_pending();steps_.clear();phase_="FAILED";ready_=false;requires_stop_=true;error_="任务身份变化，已废弃旧事务；未向新任务写入回退参数";}return;}
      const auto generation=j.at("navigation_generation").get<uint64_t>();
      const auto phase=j.value("phase",std::string());const bool safe_phase=phase=="PAUSED"||phase=="MANUAL"||phase=="FAULT";
      const auto tuning=j.value("tuning",Json::object());const bool hold_ack=tuning.value("request_id",std::string())==request_id_&&tuning.value("requires_stop",false);
      const bool is_still=j.value("still",false),is_terminal=j.value("navigation_terminal",false);
      bool zero=j.contains("output")&&j.at("output").is_array()&&j.at("output").size()==2;
      if(zero)for(const auto&v:j.at("output"))if(!v.is_number()||v.get<double>()!=0)zero=false;
      // 完整解析后才续新鲜度；畸形状态不能沿用旧停稳标记重新计时。
      autonomy_at_=seconds();autonomy_safe_phase_=safe_phase;autonomy_hold_ack_=hold_ack;
      autonomy_still_=is_still;autonomy_terminal_=is_terminal;autonomy_zero_=zero;
      observed_generation_=generation;
      if(phase_=="APPLYING"&&!startup_transaction_&&generation!=request_generation_)fail("任务代次变化，停止旧调参事务并等待停车后回退");
    }catch(const std::exception&){}
  }
  void reject(const std::string&id,const std::string&why){rejected_.push_back({{"request_id",id},{"request_result","rejected"},{"reason",why}});
    while(rejected_.size()>16)rejected_.pop_front();
    RCLCPP_WARN(get_logger(),"网页调参未提交 request_id=%s reason=%s",id.c_str(),why.c_str());
    publish();}
  void request(const std::string&text){
    std::string id;
    try{
      if(text.size()>16384)throw std::runtime_error("调参请求过大");
      const auto j=Json::parse(text);id=j.at("request_id").get<std::string>();
      if(!identifier(id))throw std::runtime_error("request_id格式无效");
      if(std::find(seen_.begin(),seen_.end(),id)!=seen_.end()){publish();return;}
      if(j.at("task_id").get<std::string>()!=task_id_||task_id_.empty())throw std::runtime_error("调参任务号不匹配");
      if(!j.at("generation").is_number_integer()||(!j.at("generation").is_number_unsigned()&&j.at("generation").get<int64_t>()<0))
        throw std::runtime_error("调参generation必须为非负整数");
      const auto generation=j.at("generation").get<uint64_t>();
      if(seconds()-autonomy_at_>.6||generation!=observed_generation_)throw std::runtime_error("任务状态或代次陈旧，未提交参数");
      if(phase_!="READY"||!ready_)throw std::runtime_error("已有事务或参数尚未就绪，未并行提交");
      const auto action=j.at("action").get<std::string>();if(action!="apply"&&action!="restore")throw std::runtime_error("仅支持apply或restore");
      auto target=baseline_;
      if(action=="apply"){
        const auto&values=j.at("values");
        if(!values.is_object()||values.empty()||values.size()>4)throw std::runtime_error("调参values须包含1～4个代价参数");
        if(!confirmed_valid_)throw std::runtime_error("尚无已确认参数快照，未提交调参");
        // 网页可只提交一个字段；其余值始终取车端最近成功核验的快照，防止旧页面缓存覆盖先前修改。
        auto merged=confirmed_.json();
        for(const auto&item:values.items()){
          if(!merged.contains(item.key()))throw std::runtime_error("未知代价参数："+item.key());
          merged[item.key()]=item.value();
        }
        target=TuningValues::parse(merged);
      }
      before_=confirmed_;before_rpp_distance_=confirmed_rpp_distance_;requested_=target;request_id_=id;request_generation_=generation;
      seen_.push_back(id);while(seen_.size()>64)seen_.pop_front();
      requires_stop_=false;rolled_back_=false;error_.clear();request_result_="accepted";startup_transaction_=false;
      phase_="APPLYING";transaction_deadline_=seconds()+25;mode_="READ_CURRENT";read_all();publish();
      RCLCPP_INFO(get_logger(),"在线调参受理 request_id=%s action=%s generation=%lu values=%s；正常应用不暂停导航",id.c_str(),action.c_str(),generation,target.json().dump().c_str());
    }catch(const std::exception&e){reject(id,e.what());}
  }
  void read_all(){remove_pending();steps_.clear();actual_=Actual{};
    for(size_t i=0;i<3;++i){Step s;s.kind=Kind::LIFECYCLE;s.node=i;s.label="确认生命周期ACTIVE："+nodes_[i];steps_.push_back(std::move(s));}
    for(size_t i=0;i<3;++i){Step s;s.kind=Kind::READ;s.node=i;s.label="读取真实参数："+nodes_[i];steps_.push_back(std::move(s));}step_started_=false;
  }
  Step write(size_t i,std::vector<rclcpp::Parameter>params,const std::string&label){Step s;s.kind=Kind::WRITE;s.node=i;s.parameters=std::move(params);s.label=label;return s;}
  void write_map(size_t i,double radius,double factor){
    const auto rev=revision();steps_.push_back(write(i,{rclcpp::Parameter("inflation_layer.inflation_radius",radius),
      rclcpp::Parameter("inflation_layer.cost_scaling_factor",factor),rclcpp::Parameter("inflation_layer.tuning_revision",rev)},"提交膨胀参数："+nodes_[i]));
    Step s;s.kind=Kind::ACK;s.node=i;s.revision=rev;s.radius=radius;s.factor=factor;s.label="等待完整膨胀周期代次回执："+nodes_[i];steps_.push_back(s);
    expected_revision_[i]=rev;
  }
  void build_writes(const TuningValues&target,double target_distance){
    mode_="WRITING";target_=target;target_rpp_distance_=target_distance;steps_.clear();
    const double guard=tuning_guard_factor(actual_.rpp_factor,target.local_factor);
    steps_.push_back(write(2,{rclcpp::Parameter("FollowPath.inflation_cost_scaling_factor",guard),
      rclcpp::Parameter("FollowPath.cost_scaling_dist",std::max(actual_.rpp_distance,target_distance)),
      rclcpp::Parameter("FollowPath.tuning_revision",revision())},"先采用保守反算系数"));
    write_map(0,target.global_radius,target.global_factor);write_map(1,target.local_radius,target.local_factor);
    const auto rev=revision();expected_revision_[2]=rev;
    steps_.push_back(write(2,{rclcpp::Parameter("FollowPath.inflation_cost_scaling_factor",target.local_factor),
      rclcpp::Parameter("FollowPath.cost_scaling_dist",target_distance),rclcpp::Parameter("FollowPath.tuning_revision",rev)},"地图代次确认后提交匹配的RPP参数"));
    for(size_t i=0;i<3;++i){Step s;s.kind=Kind::READ;s.node=i;s.label="核验最终真实参数："+nodes_[i];steps_.push_back(s);}step_started_=false;
  }
  static double numeric(const rcl_interfaces::msg::ParameterValue&v){
    if(v.type!=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE||!std::isfinite(v.double_value)||v.double_value<=0)
      throw std::runtime_error("节点参数未配置或不是有限正数");
    return v.double_value;}
  void accept_read(size_t i,const Get::Response::SharedPtr&response){
    if(!response||response->values.size()!=3)throw std::runtime_error("参数读回数量不符");
    const auto&v=response->values;
    if(v[2].type!=rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER||v[2].integer_value<0)throw std::runtime_error("节点缺少调参代次保护，未执行在线调参");
    actual_.revision[i]=v[2].integer_value;revision_=std::max(revision_,actual_.revision[i]);
    if(i==0){actual_.values.global_radius=numeric(v[0]);actual_.values.global_factor=numeric(v[1]);}
    else if(i==1){actual_.values.local_radius=numeric(v[0]);actual_.values.local_factor=numeric(v[1]);}
    else{actual_.rpp_factor=numeric(v[0]);actual_.rpp_distance=numeric(v[1]);}
  }
  void remove_pending(){if(pending_id_<0)return;
    if(pending_lifecycle_)life_[pending_node_]->remove_pending_request(pending_id_);
    else if(pending_write_)set_[pending_node_]->remove_pending_request(pending_id_);
    else get_[pending_node_]->remove_pending_request(pending_id_);
    pending_id_=-1;
  }
  void fail(const std::string&why){
    remove_pending();steps_.clear();step_started_=false;ready_=false;requires_stop_=true;request_result_="failed";
    if(phase_=="ROLLING_BACK"){phase_="FAILED";error_+="；回退未确认："+why;}
    else if(!confirmed_valid_){phase_="FAILED";error_=why;}
    else{phase_="WAITING_STOP";error_=why;stop_requested_at_=seconds();stop_deadline_=stop_requested_at_+8;}
    RCLCPP_WARN(get_logger(),"在线调参失败 request_id=%s phase=%s reason=%s；已请求监督器停车，回退必须先确认停稳",request_id_.c_str(),phase_.c_str(),why.c_str());
    publish();
  }
  void finished(){
    if(mode_=="READ_INITIAL"){
      baseline_=TuningValues::parse(actual_.values.json());confirmed_=baseline_;confirmed_rpp_distance_=actual_.rpp_distance;
      nominal_distance_=actual_.rpp_distance;confirmed_valid_=true;before_=baseline_;before_rpp_distance_=actual_.rpp_distance;
      requested_=have_defaults_?defaults_:baseline_;request_id_="startup-"+std::to_string(revision());request_result_="accepted";
      startup_transaction_=true;phase_="APPLYING";transaction_deadline_=seconds()+25;
      build_writes(requested_,tuning_cost_distance(nominal_distance_,requested_.local_radius));
    }else if(mode_=="READ_CURRENT"){
      // 即使网页请求有旧缓存，也以本次实际节点值决定过渡方向；回退目标仍是最后已验证配置。
      build_writes(requested_,tuning_cost_distance(nominal_distance_,requested_.local_radius));
    }else if(mode_=="READ_ROLLBACK"){
      build_writes(before_,before_rpp_distance_);
    }else{
      if(!tuning_equal(actual_.values,target_)||!tuning_equal(actual_.rpp_factor,target_.local_factor)||!tuning_equal(actual_.rpp_distance,target_rpp_distance_)||actual_.revision!=expected_revision_)
        throw std::runtime_error("最终参数或代次与事务目标不一致");
      for(size_t i=0;i<2;++i)if(!tuning_ack_matches(ack_[i],nodes_[i],expected_revision_[i],i==0?target_.global_radius:target_.local_radius,i==0?target_.global_factor:target_.local_factor))
        throw std::runtime_error("实际膨胀周期回执未匹配最终参数");
      confirmed_=target_;confirmed_rpp_distance_=target_rpp_distance_;confirmed_valid_=true;
      if(phase_=="ROLLING_BACK"){
        rolled_back_=true;
        // 原请求仍失败；仅在新的停稳回退事务完整核验后，判断当前配置是否已实际满足启动要求。
        const bool startup_satisfied=tuning_equal(actual_.values,requested_)&&
          tuning_equal(actual_.rpp_factor,requested_.local_factor)&&
          tuning_equal(actual_.rpp_distance,tuning_cost_distance(nominal_distance_,requested_.local_radius));
        if(startup_transaction_&&!startup_satisfied){phase_="FAILED";ready_=false;requires_stop_=true;error_+="；已回退，但启动要求仍未满足，保持未就绪";}
        else{
          if(startup_transaction_)baseline_=confirmed_;
          phase_="READY";ready_=true;requires_stop_=false;
          error_+=startup_transaction_?"；原请求失败，启动要求已经由独立停稳回退核验实际满足":"；已在停稳后回退并核验原配置";
        }
      }else{
        // 本轮恢复基线是启动默认成功落地的配置；冻结发行中的原始值另由发布资产保留。
        if(startup_transaction_)baseline_=confirmed_;
        phase_="READY";ready_=true;requires_stop_=false;error_.clear();request_result_="completed";}
      RCLCPP_INFO(get_logger(),"在线调参终态 request_id=%s result=%s phase=%s rolled_back=%d values=%s；真实参数与膨胀周期代次已核验",
        request_id_.c_str(),request_result_.c_str(),phase_.c_str(),rolled_back_,confirmed_.json().dump().c_str());
      publish();
    }
  }
  void tick(){
    const double now=seconds();
    if(phase_=="INITIALIZING"&&steps_.empty()){
      if(now>startup_deadline_){fail("导航参数服务或膨胀代次适配器60秒内未就绪");return;}
      bool available=true;for(size_t i=0;i<3;++i)available=available&&get_[i]->service_is_ready()&&set_[i]->service_is_ready()&&life_[i]->service_is_ready();
      if(available&&now>=retry_at_){mode_="READ_INITIAL";read_all();}
    }
    if(phase_=="WAITING_STOP"){
      if(stopped()){phase_="ROLLING_BACK";mode_="READ_ROLLBACK";transaction_deadline_=now+25;read_all();publish();}
      else if(now>stop_deadline_){phase_="FAILED";error_+="；8秒内未确认导航终态、轮式停稳和零输出，未执行回退";publish();}
    }
    if((phase_=="APPLYING"||phase_=="ROLLING_BACK")&&now>transaction_deadline_){fail("调参事务总期限超过25秒");return;}
    if(phase_=="ROLLING_BACK"&&!stopped()){fail("回退期间失去新鲜停稳证据，保持停车且不提交后续参数");return;}
    if(!steps_.empty()&&(phase_=="INITIALIZING"||phase_=="APPLYING"||phase_=="ROLLING_BACK")){
      try{
        auto&s=steps_.front();if(!step_started_){step_deadline_=now+service_timeout_;step_started_=true;
          pending_lifecycle_=s.kind==Kind::LIFECYCLE;
          if(s.kind==Kind::WRITE){if(!set_[s.node]->service_is_ready())throw std::runtime_error("参数写服务未就绪");
            auto request=std::make_shared<Set::Request>();for(const auto&p:s.parameters)request->parameters.push_back(p.to_parameter_msg());
            auto result=set_[s.node]->async_send_request(request);set_future_=result.future.share();pending_id_=result.request_id;pending_write_=true;pending_node_=s.node;
          }else if(s.kind==Kind::LIFECYCLE){if(!life_[s.node]->service_is_ready())throw std::runtime_error("生命周期服务未就绪");
            auto result=life_[s.node]->async_send_request(std::make_shared<Life::Request>());life_future_=result.future.share();pending_id_=result.request_id;pending_node_=s.node;
          }else if(s.kind==Kind::READ){if(!get_[s.node]->service_is_ready())throw std::runtime_error("参数读取服务未就绪");
            auto request=std::make_shared<Get::Request>();request->names=s.node==2?std::vector<std::string>{"FollowPath.inflation_cost_scaling_factor","FollowPath.cost_scaling_dist","FollowPath.tuning_revision"}:
              std::vector<std::string>{"inflation_layer.inflation_radius","inflation_layer.cost_scaling_factor","inflation_layer.tuning_revision"};
            auto result=get_[s.node]->async_send_request(request);get_future_=result.future.share();pending_id_=result.request_id;pending_write_=false;pending_node_=s.node;
          }
        }
        if(now>step_deadline_)throw std::runtime_error(s.label+"超时；迟到应答不得把事务改成成功");
        bool complete=false;
        if(s.kind==Kind::LIFECYCLE&&life_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){const auto r=life_future_.get();pending_id_=-1;
          if(!r||r->current_state.id!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)throw std::runtime_error(s.label+"尚未满足");
          complete=true;}
        if(s.kind==Kind::READ&&get_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){accept_read(s.node,get_future_.get());pending_id_=-1;complete=true;}
        else if(s.kind==Kind::WRITE&&set_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){const auto r=set_future_.get();pending_id_=-1;
          if(!r||!r->result.successful)throw std::runtime_error(s.label+"被节点拒绝："+(r?r->result.reason:"空应答"));
          complete=true;
        }else if(s.kind==Kind::ACK&&tuning_ack_matches(ack_[s.node],nodes_[s.node],s.revision,s.radius,s.factor)&&now-ack_at_[s.node]<=service_timeout_)complete=true;
        if(complete){steps_.pop_front();step_started_=false;if(steps_.empty())finished();}
      }catch(const std::exception&e){
        if(phase_=="INITIALIZING"&&now<startup_deadline_){error_=std::string("等待导航初始化：")+e.what();remove_pending();steps_.clear();step_started_=false;retry_at_=now+.2;}
        else fail(e.what());
      }
    }
    if(now-last_publish_>=.1)publish();
  }
  const std::array<std::string,3>nodes_{{"/global_costmap/global_costmap","/local_costmap/local_costmap","/controller_server"}};
  std::array<rclcpp::Client<Get>::SharedPtr,3>get_;std::array<rclcpp::Client<Set>::SharedPtr,3>set_;
  std::array<rclcpp::Client<Life>::SharedPtr,3>life_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requests_,acks_,autonomy_;rclcpp::TimerBase::SharedPtr timer_;
  std::shared_future<Get::Response::SharedPtr>get_future_;std::shared_future<Set::Response::SharedPtr>set_future_;
  std::shared_future<Life::Response::SharedPtr>life_future_;
  std::deque<Step>steps_;std::array<Json,2>ack_;std::array<double,2>ack_at_{{-1e9,-1e9}};std::array<int64_t,3>expected_revision_{{0,0,0}};
  std::deque<Json>rejected_;std::deque<std::string>seen_;Actual actual_;TuningValues baseline_,confirmed_,requested_,target_,defaults_,before_;
  std::string task_id_,request_id_,phase_="INITIALIZING",mode_,error_,defaults_file_,defaults_error_,request_result_;
  uint64_t observed_generation_=0,request_generation_=0;int64_t revision_=0,pending_id_=-1;size_t pending_node_=0;
  bool pending_write_=false,pending_lifecycle_=false,step_started_=false,ready_=false,confirmed_valid_=false,requires_stop_=false,rolled_back_=false,have_defaults_=false,startup_transaction_=false;
  bool autonomy_still_=false,autonomy_terminal_=false,autonomy_zero_=false,autonomy_hold_ack_=false,autonomy_safe_phase_=false;
  double autonomy_at_=-1e9,confirmed_rpp_distance_=.3,before_rpp_distance_=.3,nominal_distance_=.3,target_rpp_distance_=.3;
  double service_timeout_=3,startup_deadline_=0,transaction_deadline_=0,stop_deadline_=0,stop_requested_at_=0,step_deadline_=0,retry_at_=0,last_publish_=0;
};
}
#ifndef CARCAR_TUNING_NO_MAIN
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<carcar_mapping_autonomy::MappingTuningNode>());rclcpp::shutdown();return 0;}
#endif
