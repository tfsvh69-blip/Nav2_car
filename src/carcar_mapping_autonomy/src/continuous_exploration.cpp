#include "carcar_mapping_autonomy/continuous_exploration.hpp"
#include "carcar_mapping_autonomy/route_validation.hpp"
#include <future>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
namespace ca=carcar_mapping_autonomy;
using Navigate=nav2_msgs::action::NavigateThroughPoses;
using PlanRoute=nav2_msgs::action::ComputePathThroughPoses;
using NavHandle=rclcpp_action::ClientGoalHandle<Navigate>;
using PlanHandle=rclcpp_action::ClientGoalHandle<PlanRoute>;
namespace {
double distance(const ca::Pose&a,const ca::Pose&b){return std::hypot(a.x-b.x,a.y-b.y);}
bool same(const std::vector<ca::ExplorationPoint>&a,const std::vector<ca::ExplorationPoint>&b){
  if(a.size()!=b.size())return false;
  for(size_t i=0;i<a.size();++i)if(distance(a[i].target,b[i].target)>.01)return false;
  return true;
}
}
class ca::ContinuousExploration::Impl {
  struct NavRecord {uint64_t epoch=0,id=0;NavHandle::SharedPtr handle;bool done=false,cancel_sent=false;};
  struct PlanRecord {
    uint64_t epoch=0,id=0;PlanHandle::SharedPtr handle;bool done=false;
    std::vector<ExplorationPoint> route,prefix;bool cancel_sent=false;std::shared_ptr<const nav_msgs::msg::Path> path;
  };
public:
  Impl(rclcpp::Node & node,std::string tree):node_(node),tree_(std::move(tree)){
    limits_=node_.create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit",rclcpp::QoS(1));
    nav_=rclcpp_action::create_client<Navigate>(&node_,"/navigate_through_poses");
    planner_=rclcpp_action::create_client<PlanRoute>(&node_,"/compute_path_through_poses");
    paths_=node_.create_subscription<nav_msgs::msg::Path>("/plan",rclcpp::QoS(1),[this](nav_msgs::msg::Path::ConstSharedPtr path){
      if(stopping_||route_.empty()||path->header.frame_id!="map"||path->poses.empty()||
        rclcpp::Time(path->header.stamp)<dispatch_stamp_)return;
      const auto&p=path->poses.back().pose.position;
      if(std::hypot(p.x-route_.back().target.x,p.y-route_.back().target.y)<.15)active_path_=std::move(path);
    });
  }
  ~Impl(){try{stop();}catch(const std::exception&){} } // 进程退出时ROS上下文可能已经关闭。
  bool terminal()const{return (!plan_||plan_->done)&&std::all_of(navs_.begin(),navs_.end(),[](const auto&r){return r->done;});}
  bool motion()const{return !stopping_&&enabled_&&fault_.empty()&&std::any_of(navs_.begin(),navs_.end(),[](const auto&r){return r->handle&&!r->done;});}
  void event(const std::string&type,const std::string&reason){
    note_=reason;RCLCPP_INFO(node_.get_logger(),"连续探索 task_id=%s request_id=%s route_id=%lu event=%s generation=%lu map_revision=%lu targets=%zu reason=%s",
      snapshot_.task_id.c_str(),snapshot_.request_id.c_str(),serial_,type.c_str(),epoch_,snapshot_.route.revision,route_.size(),reason.c_str());
  }
  void stop(bool confirm_empty=false){
    confirm_empty_=confirm_empty;
    if(!stopping_){stopping_=true;++epoch_;stop_at_=snapshot_.route.now;event("cancel","取消连续路线及在途预取；迟到结果不得重新行驶");}
    enabled_=false;
    if(plan_&&!plan_->done&&plan_->handle&&!plan_->cancel_sent){plan_->cancel_sent=true;planner_->async_cancel_goal(plan_->handle);}
    for(const auto&r:navs_)if(!r->done&&r->handle&&!r->cancel_sent){r->cancel_sent=true;nav_->async_cancel_goal(r->handle);}
  }
  void visit(const ExplorationPoint & point,bool reached){
    visits_.push_back({point.target,point.gain,snapshot_.route.now+60,reached,point.predicted_cells,snapshot_.route.revision});
    if(visits_.size()>128)visits_.erase(visits_.begin());
  }
  void restart_after_stop(){
    route_.clear();active_path_.reset();plan_.reset();navs_.clear();stopping_=false;enabled_=true;
    validation_pending_=false;nav_pending_=false;complete_=false;
    if(!confirm_empty_)empty_count_=0;
    confirm_empty_=false;
    last_selection_=-1e9;last_progress_=snapshot_.route.now;progress_pose_=snapshot_.route.current;
    event("ready","取消终态与轮式停稳已确认，按原任务预算继续探索");
  }
  void validate_plan(){
    if(!plan_||!plan_->path||plan_->route.empty())return;
    RouteValidator::Job job;job.operation=plan_->id;job.attempt=plan_->epoch;
    job.revision=snapshot_.route.revision;job.map=snapshot_.route.map;job.path=plan_->path;
    job.task=snapshot_.route.task;job.current=snapshot_.route.current;job.target=plan_->route.back().target;job.remaining=true;
    validator_.submit(std::move(job));validation_pending_=true;
  }
  void plan_route(std::vector<ExplorationPoint> route){
    if(stopping_||!enabled_||route.empty()||(plan_&&!plan_->done)||validation_pending_||nav_pending_)return;
    if(!planner_->action_server_is_ready()||!nav_->action_server_is_ready()){note_="多点规划／导航接口未就绪；未下发连续路线";return;}
    auto record=std::make_shared<PlanRecord>();record->id=++serial_;record->epoch=epoch_;record->route=std::move(route);
    record->prefix=route_;plan_=record;plan_at_=snapshot_.route.now;
    PlanRoute::Goal goal;goal.use_start=false;goal.planner_id="ExplorationGrid";
    for(const auto & point:record->route){geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=node_.now();
      pose.pose.position.x=point.target.x;pose.pose.position.y=point.target.y;pose.pose.orientation.w=1;goal.goals.push_back(pose);}
    rclcpp_action::Client<PlanRoute>::SendGoalOptions options;
    options.goal_response_callback=[this,record](PlanHandle::SharedPtr handle){
      record->handle=handle;if(!handle){record->done=true;note_="连续路线规划请求被拒绝，原有效路线保持";}
      else if(stopping_||record->epoch!=epoch_)planner_->async_cancel_goal(handle);
    };
    options.result_callback=[this,record](const PlanHandle::WrappedResult&result){
      record->done=true;if(stopping_||record->epoch!=epoch_||plan_!=record)return;
      if(result.code!=rclcpp_action::ResultCode::SUCCEEDED||!result.result||result.result->path.poses.empty()){
        if(!record->route.empty())visit(record->route.back(),false);
        plan_.reset();event("plan_failed","Nav2连续路径规划失败，未下发新路线；保留已有有效路线");return;}
      record->path=std::make_shared<nav_msgs::msg::Path>(result.result->path);validate_plan();
    };
    planner_->async_send_goal(goal,options);event("prefetch_plan","预取路线交给Nav2真实规划，尚未替换活动路线");
  }
  void dispatch(){
    if(!plan_||stopping_||!enabled_||!snapshot_.healthy||plan_->epoch!=epoch_)return;
    if(!same(plan_->prefix,route_)){plan_.reset();validation_pending_=false;last_selection_=-1e9;return;}
    Navigate::Goal goal;goal.behavior_tree=tree_;
    for(const auto& point:plan_->route){geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=node_.now();
      pose.pose.position.x=point.target.x;pose.pose.position.y=point.target.y;pose.pose.orientation.w=1;goal.poses.push_back(pose);}
    const auto route=plan_->route;const auto path=plan_->path;
    auto record=std::make_shared<NavRecord>();record->id=plan_->id;record->epoch=epoch_;navs_.push_back(record);nav_pending_=true;nav_at_=snapshot_.route.now;
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback=[this,record,route,path](NavHandle::SharedPtr handle){
      nav_pending_=false;record->handle=handle;
      if(!handle){record->done=true;if(terminal()){route_.clear();active_path_.reset();last_selection_=-1e9;}event("route_rejected","导航器拒绝预取路线，未将请求受理误报为执行成功");return;}
      if(stopping_||record->epoch!=epoch_){nav_->async_cancel_goal(handle);return;}
      route_=route;active_path_=path;dispatch_stamp_=node_.now();active_id_=record->id;
      event("route_accepted","导航器接受已审查多点路线；通过原生抢占续接，不先取消旧路线");
    };
    options.result_callback=[this,record](const NavHandle::WrappedResult&result){
      record->done=true;if(stopping_||record->epoch!=epoch_||record->id!=active_id_||nav_pending_)return;
      if(result.code==rclcpp_action::ResultCode::SUCCEEDED&&!route_.empty()&&distance(snapshot_.route.current,route_.back().target)>.20){
        event("premature_success","导航器返回成功但当前末点尚未到达；不登记到访，重新预取实际剩余路线");
        active_path_.reset();last_selection_=-1e9;
        auto remaining=route_;route_.clear();plan_route(std::move(remaining));return;
      }
      if(result.code==rclcpp_action::ResultCode::SUCCEEDED){for(const auto&p:route_)visit(p,true);event("route_succeeded","连续路线末点到达；中间点不作为独立导航终态");}
      else{if(!route_.empty())visit(route_.front(),false);event("route_failed","当前连续路线执行终止，保留失败记录，重新检查安全候选");}
      route_.clear();active_path_.reset();last_selection_=-1e9;
    };
    nav_->async_send_goal(goal,options);plan_.reset();validation_pending_=false;
  }
  void tick(Snapshot snapshot){
    snapshot_=std::move(snapshot);const double now=snapshot_.route.now;
    nav2_msgs::msg::SpeedLimit limit;limit.header.stamp=node_.now();limit.percentage=true;
    limit.speed_limit=snapshot_.enabled&&!stopping_&&route_.size()>1&&distance(route_.front().target,snapshot_.route.current)<.60?60.:0.;
    limits_->publish(limit);
    const bool generation_changed=snapshot_.route.navigation_generation!=navigation_generation_;
    if(generation_changed){stop();visits_.clear();navigation_generation_=snapshot_.route.navigation_generation;}
    if(!snapshot_.enabled||!snapshot_.healthy){stop();}
    if(stopping_&&!terminal()&&now-stop_at_>5)fault_="连续路线取消或规划终态确认超过5秒，保持停车";
    if(!snapshot_.enabled||!snapshot_.healthy||!fault_.empty())return;
    if(stopping_){if(!terminal()||!snapshot_.still)return;restart_after_stop();task_generation_=snapshot_.route.generation;}
    if(task_generation_!=snapshot_.route.generation){stop();return;}
    if(motion()&&distance(progress_pose_,snapshot_.route.current)>=.10){progress_pose_=snapshot_.route.current;last_progress_=now;}
    if(motion()&&now-last_progress_>30){if(!route_.empty())visit(route_.front(),false);stop();event("no_progress","30秒无真实车位进展，取消后停稳再选点");return;}
    while(route_.size()>1&&distance(route_.front().target,snapshot_.route.current)<=.25){
      visit(route_.front(),true);route_.erase(route_.begin());++passed_;last_progress_=now;
      event("passed","通过中间观测点，保持同一连续导航任务");
    }
    if(plan_&&now-plan_at_>8){fault_="连续路线预检超过8秒，保持停车并等待取消终态";stop();return;}
    if(nav_pending_&&now-nav_at_>5){fault_="连续导航请求接受超过5秒，保持停车等待终态";stop();return;}
    if(auto checked=validator_.take()){
      validation_pending_=false;
      if(plan_&&checked->job.operation==plan_->id&&checked->job.attempt==epoch_&&!stopping_){
        if(checked->job.revision!=snapshot_.route.revision)validate_plan();
        else if(checked->safe)dispatch();
        else{if(!plan_->route.empty())visit(plan_->route.back(),false);plan_.reset();event("validation_rejected","最新地图路线审查拒绝："+checked->reason);}
      }
    }
    if(worker_.valid()&&worker_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
      auto result=worker_.get();compute_ms_=result.computation_ms;
      if(result.input.epoch==epoch_&&result.input.generation==task_generation_&&result.input.navigation_generation==navigation_generation_&&
        same(result.input.prefix,route_)){
        candidates_=std::move(result.candidates);observations_=std::move(result.observations);newly_known_m2_=result.newly_known_m2;frontiers_=result.frontiers;note_=result.reason;
        if(!result.active_safe&&result.input.revision==snapshot_.route.revision&&result.input.active_path==active_path_){stop();event("route_invalidated",result.reason);return;}
        if(result.input.revision==snapshot_.route.revision&&result.input.active_path==active_path_)validated_revision_=result.input.revision;
        // 地图已经连续确认没有前沿时，剩余承诺点已无扩图收益；交由监督器取消并返程。
        // 不能等旧路线的末点到达才判断，否则会在已建好的区域多走一整段。
        if(result.frontiers==0&&result.input.revision==snapshot_.route.revision){
          if(last_empty_sequence_!=snapshot_.map_sequence){last_empty_sequence_=snapshot_.map_sequence;++empty_count_;}
          complete_=empty_count_>=3;
          if(!complete_&&!route_.empty()){
            stop(true);event("checking_complete","新图已无前沿，取消无收益剩余路线并停稳，等待后续地图确认完成");return;
          }
        }else{empty_count_=0;complete_=false;}
        if(!same(result.route,route_)&&!result.route.empty())plan_route(std::move(result.route));
        else if(result.route.empty()&&!route_.empty()&&result.frontiers>0&&result.input.revision==snapshot_.route.revision){
          stop();event("no_useful_continuation","原承诺点已无新增收益且没有可达续接点，停稳等待新图；未宣布全场完成");return;
        }
      }
    }
    if(worker_.valid()&&now-worker_at_>8){fault_="连续探索计算超过8秒，未继续下发路线";stop();return;}
    // 预取在末端减速区之前启动；正常地图扩展只触发重验，不取消已有路线。
    const bool needs_extension=route_.size()<2||(!route_.empty()&&distance(route_.back().target,snapshot_.route.current)<1.2);
    const bool check_map=active_path_&&validated_revision_!=snapshot_.route.revision;
    if(!worker_.valid()&&now-last_selection_>=.75&&(needs_extension||check_map||empty_count_>0)&&(!plan_||plan_->done)&&!nav_pending_&&!validation_pending_){
      auto input=snapshot_.route;input.prefix=route_;input.visits=visits_;input.active_path=active_path_;input.epoch=epoch_;
      worker_at_=last_selection_=now;
      worker_=std::async(std::launch::async,[input=std::move(input)]()mutable{
        try{return exploration_route(input);}catch(const std::exception&e){ExplorationRouteResult r;r.input=std::move(input);r.active_safe=false;r.reason=std::string("探索计算异常：")+e.what();return r;}});
    }
    navs_.erase(std::remove_if(navs_.begin(),navs_.end(),[](const auto&r){return r->done;}),navs_.end());
  }
  Json status()const{
    Json goals=Json::array();for(const auto&p:route_)goals.push_back({{"target",{p.target.x,p.target.y}},{"frontier",{p.frontier.x,p.frontier.y}},{"predicted_unknown_m2",p.gain}});
    return {{"enabled",enabled_&&!stopping_},{"motion",motion()},{"terminal",terminal()},{"complete",complete_},{"fault",fault_},{"reason",note_},
      {"route_id",active_id_},{"route",goals},{"candidates",candidates_},{"raw_frontiers",frontiers_},{"passed_points",passed_},
      {"prefetch_pending",worker_.valid()||validation_pending_||bool(plan_)||nav_pending_},{"computation_ms",compute_ms_},
      {"visit_observations",observations_},{"observed_new_known_m2",newly_known_m2_},
      {"navigation_generation",navigation_generation_},{"validated_revision",validated_revision_},{"checkpoint_policy","连续导航实验；在线快照未验收，仍保留原维护停车保护"}};
  }
  rclcpp::Node&node_;std::string tree_;Snapshot snapshot_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_;rclcpp_action::Client<PlanRoute>::SharedPtr planner_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr paths_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr limits_;
  std::future<ExplorationRouteResult> worker_;RouteValidator validator_;
  std::vector<std::shared_ptr<NavRecord>>navs_;std::shared_ptr<PlanRecord>plan_;
  std::vector<ExplorationPoint>route_;std::vector<ExplorationVisit>visits_;
  std::shared_ptr<const nav_msgs::msg::Path>active_path_;Json candidates_=Json::array(),observations_=Json::array();double newly_known_m2_=0;
  bool stopping_=true,enabled_=false,validation_pending_=false,nav_pending_=false,complete_=false,confirm_empty_=false;
  uint64_t epoch_=0,serial_=0,active_id_=0,navigation_generation_=0,task_generation_=0,validated_revision_=0,last_empty_sequence_=0;
  size_t frontiers_=0,passed_=0,empty_count_=0;
  double stop_at_=0,last_selection_=-1e9,worker_at_=0,plan_at_=0,nav_at_=0,last_progress_=0,compute_ms_=0;
  Pose progress_pose_;rclcpp::Time dispatch_stamp_{0,0,RCL_ROS_TIME};std::string fault_,note_;
};
ca::ContinuousExploration::ContinuousExploration(rclcpp::Node&node,const std::string&tree):impl_(std::make_unique<Impl>(node,tree)){}
ca::ContinuousExploration::~ContinuousExploration()=default;
void ca::ContinuousExploration::tick(Snapshot snapshot){impl_->tick(std::move(snapshot));}
void ca::ContinuousExploration::stop(){impl_->stop();}
bool ca::ContinuousExploration::terminal()const{return impl_->terminal();}
bool ca::ContinuousExploration::motion()const{return impl_->motion();}
bool ca::ContinuousExploration::complete()const{return impl_->complete_;}
std::string ca::ContinuousExploration::fault()const{return impl_->fault_;}
ca::Json ca::ContinuousExploration::status()const{return impl_->status();}
