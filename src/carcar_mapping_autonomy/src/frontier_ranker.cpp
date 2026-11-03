#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include "carcar_mapping_autonomy/observation_selector.hpp"
#include "carcar_mapping_autonomy/frontier_gain.hpp"

namespace ca=carcar_mapping_autonomy;
using Plan=nav2_msgs::action::ComputePathToPose;
using Handle=rclcpp_action::ClientGoalHandle<Plan>;
class FrontierRanker:public rclcpp::Node {
  struct Job {
    ca::Json request,results=ca::Json::array(),rejected=ca::Json::array();ca::Task task;ca::Pose start,target,frontier;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    uint64_t token=0,revision=0;size_t index=0;
    double began=0,plan_at=0;bool selecting=false,pending=false;
    Handle::SharedPtr handle;
  };
  static double seconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
public:
  FrontierRanker():Node("mapping_frontier_ranker"),tf_(get_clock()),listener_(tf_){
    prefer_clearance_=declare_parameter("prefer_observation_clearance",true);
    heading_window_m_=declare_parameter("ranking_heading_window_m",.30);
    turn_cost_m_per_rad_=declare_parameter("ranking_turn_cost_m_per_rad",.10/.22);
    if(!std::isfinite(heading_window_m_)||heading_window_m_<.10||heading_window_m_>1.||
       !std::isfinite(turn_cost_m_per_rad_)||turn_cost_m_per_rad_<0)
      throw std::invalid_argument("探索排名转向代价和前视尺度必须为有效有限值");

    client_=rclcpp_action::create_client<Plan>(this,"/compute_path_to_pose");
    progress_=create_publisher<std_msgs::msg::String>("/mapping/exploration/rank_state",rclcpp::QoS(1).reliable().transient_local());
    response_=create_publisher<std_msgs::msg::String>("/mapping/exploration/rank_result",10);
    status_=create_subscription<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{state_=ca::Json::parse(m.data);state_at_=seconds();}catch(...){}});
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){if(ca::grid_valid(*m)&&m->data.size()<=2000000&&std::abs(ca::grid_yaw(*m))<1e-6){
        if(!map_||!ca::same_grid(*map_,*m))++revision_;
        map_=std::move(m);map_at_=seconds();}});
    requests_=create_subscription<std_msgs::msg::String>("/mapping/exploration/rank_request",10,
      [this](const std_msgs::msg::String&m){try{accept(ca::Json::parse(m.data));}catch(const std::exception&e){RCLCPP_WARN(get_logger(),"前沿排序请求无效：%s",e.what());}});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
  }
private:
  bool healthy(const ca::Json&request)const{
    return seconds()-state_at_<1&&seconds()-map_at_<10&&map_&&state_.value("phase",std::string())=="EXPLORING"&&
      state_.value("healthy",false)&&!state_.value("loop_waiting_map",false)&&
      request.value("task_id",std::string())==state_.value("task_id",std::string())&&
      request.value("generation",uint64_t{})==state_.value("generation",uint64_t{})&&
      request.value("navigation_generation",uint64_t{})==state_.value("navigation_generation",uint64_t{});
  }
  std::optional<ca::Pose> current(){try{
    const auto t=tf_.lookupTransform("map","base_footprint",tf2::TimePointZero);
    const double age=(now()-rclcpp::Time(t.header.stamp)).seconds();if(age<-.1||age>.5)return {};
    return ca::Pose{t.transform.translation.x,t.transform.translation.y,2*std::atan2(t.transform.rotation.z,t.transform.rotation.w)};
  }catch(const tf2::TransformException&){return {};}}
  void progress(const std::shared_ptr<Job>&job,bool active,const std::string&reason){
    std_msgs::msg::String message;message.data=ca::Json{{"task_id",job->request.value("task_id",std::string())},
      {"request_id",job->request.value("request_id",std::string())},{"generation",job->request.value("generation",uint64_t{})},
      {"navigation_generation",job->request.value("navigation_generation",uint64_t{})},{"active",active},{"reason",reason},
      {"checked",job->index},{"accepted",job->results.size()},{"elapsed_seconds",seconds()-job->began}}.dump();progress_->publish(message);
  }
  void reply(const std::shared_ptr<Job>&job,const std::string&reason){
    progress(job,false,reason);
    auto results=job->results;
    std::stable_sort(results.begin(),results.end(),[](const ca::Json&a,const ca::Json&b){return a["score"].get<double>()<b["score"].get<double>();});
    // 有收益相近的本地目标时保持当前区域；跨区收益明显更好才提前回访。
    if(!results.empty()&&results.front()["path_m"].get<double>()>1.8){
      for(size_t i=1;i<results.size();++i)if(results[i]["path_m"].get<double>()<=1.8&&
          results.front()["score"].get<double>()>=results[i]["score"].get<double>()*.70){
        auto local=results[i];results.erase(results.begin()+i);results.insert(results.begin(),local);break;
      }
    }
    ca::Json value={{"request_id",job->request.value("request_id",std::string())},
      {"task_id",job->request.value("task_id",std::string())},{"generation",job->request.value("generation",uint64_t{})},
      {"navigation_generation",job->request.value("navigation_generation",uint64_t{})},
      {"map_revision",job->revision},{"candidates",results},{"rejected",job->rejected},{"reason",reason},{"elapsed_seconds",seconds()-job->began}};
    std_msgs::msg::String message;message.data=value.dump();response_->publish(message);
    if(job->handle)client_->async_cancel_goal(job->handle);
    if(job_==job)job_.reset();
  }
  void accept(const ca::Json&request){
    if(!request.is_object()||!request.contains("candidates")||!request["candidates"].is_array()||
       request["candidates"].empty()||request["candidates"].size()>8||request.value("request_id",std::string()).empty())return;
    if(job_)reply(job_,"排序请求已被更新，旧结果仅供诊断");
    auto job=std::make_shared<Job>();job->request=request;job->token=++sequence_;job->began=seconds();job_=job;
    const auto pose=current();
    if(!healthy(request)||!pose||!client_->action_server_is_ready()){
      reply(job,"排序等待健康定位、新鲜地图与规划器；未下发运动目标");return;
    }
    job->map=map_;job->revision=revision_;job->start=*pose;job->task.anchored=state_.value("anchored",false);
    const auto origin=state_.at("origin");job->task.origin={origin.at(0),origin.at(1),origin.at(2)};
    job->task.limits.radius=state_.value("radius_m",20.);
    next(job);
  }
  void next(const std::shared_ptr<Job>&job){
    if(job_!=job)return;
    progress(job,true,"正在核对可达观测点的真实规划路程；尚未提交运动目标");
    job->handle.reset();job->pending=false;
    if(job->index>=job->request["candidates"].size()){reply(job,job->results.empty()?"本批代表点没有可执行观测位置，局部暂避后检查同一前沿的其他位置":"可达观测点已按真实规划路程、可见未知收益及转向排序");return;}
    const auto candidate=job->request["candidates"][job->index++];
    if(!candidate.is_array()||candidate.size()!=2){next(job);return;}
    job->frontier={candidate[0].get<double>(),candidate[1].get<double>(),0};
    ca::ObservationSelector::Job selection;selection.operation=job->token;selection.revision=job->revision;
    selection.map=job->map;selection.current=job->start;selection.frontier=job->frontier;selection.task=job->task;
    selection.prefer_clearance=prefer_clearance_;selector_.submit(std::move(selection));job->selecting=true;
  }
  void plan(const std::shared_ptr<Job>&job){
    Plan::Goal goal;goal.planner_id="ExplorationGrid";goal.use_start=true;
    goal.start.header.frame_id=goal.goal.header.frame_id="map";goal.start.header.stamp=goal.goal.header.stamp=now();
    goal.start.pose.position.x=job->start.x;goal.start.pose.position.y=job->start.y;
    goal.start.pose.orientation.z=std::sin(job->start.yaw/2);goal.start.pose.orientation.w=std::cos(job->start.yaw/2);
    goal.goal.pose.position.x=job->target.x;goal.goal.pose.position.y=job->target.y;goal.goal.pose.orientation.w=1;
    job->pending=true;job->plan_at=seconds();rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback=[this,job](Handle::SharedPtr handle){
      if(job_!=job||!job->pending){if(handle)client_->async_cancel_goal(handle);return;}
      if(!handle){next(job);return;}job->handle=handle;
    };
    options.result_callback=[this,job](const Handle::WrappedResult&result){
      if(job_!=job||!job->pending)return;
      job->pending=false;job->handle.reset();
      const size_t accepted=job->results.size();
      if(result.code==rclcpp_action::ResultCode::SUCCEEDED&&result.result){
        try{score(job,result.result->path);}catch(const std::exception&e){RCLCPP_WARN(get_logger(),"前沿评分输入无效：%s",e.what());}
      }
      if(job->results.size()==accepted)job->rejected.push_back({{"frontier",{job->frontier.x,job->frontier.y}},
        {"reason","该观测点未通过真实规划或连续硬净空检查"}});
      next(job);
    };
    try{client_->async_send_goal(goal,options);}catch(const std::exception&e){reply(job,std::string("只读规划请求异常：")+e.what());}
  }
  void score(const std::shared_ptr<Job>&job,const nav_msgs::msg::Path&path){
    if(!healthy(job->request)||path.header.frame_id!="map"||path.poses.empty()||path.poses.size()>20000)return;
    ca::Pose previous=job->start;double length=0,direction=job->start.yaw;
    std::vector<ca::Pose> planned_points;planned_points.reserve(path.poses.size());
    for(const auto&p:path.poses){ca::Pose next{p.pose.position.x,p.pose.position.y,0};
      if(!job->task.inside(next)||!ca::free_connector(*map_,previous,next))return;
      const double distance=std::hypot(next.x-previous.x,next.y-previous.y);if(!std::isfinite(distance))return;
      planned_points.push_back(next);length+=distance;previous=next;
    }
    if(length<=.05||std::hypot(previous.x-job->target.x,previous.y-job->target.y)>.15)return;
    // 从真实安全路径中比较观测前缀；能提前看清未知端部时，不必先走到前沿旁边。
    // 前缀仍是已知自由区内的规划路径，不能跨未知区或用直线替代实际绕行长度。
    ca::Json best;double best_score=std::numeric_limits<double>::infinity(),prefix_length=0,prefix_turn=0,last_sample=-1;
    previous=job->start;direction=job->start.yaw;
    for(size_t i=0;i<path.poses.size();++i){const auto&p=path.poses[i].pose.position;ca::Pose point{p.x,p.y,0};
      const double d=std::hypot(point.x-previous.x,point.y-previous.y);
      if(d>.005){const double angle=std::atan2(point.y-previous.y,point.x-previous.x);prefix_turn+=std::abs(std::remainder(angle-direction,2*M_PI));direction=angle;}
      prefix_length+=d;previous=point;const bool final=i+1==path.poses.size();
      if((prefix_length<.45&&!final)||prefix_length<=.25||(!final&&prefix_length-last_sample<.25)||
         std::hypot(point.x-job->start.x,point.y-job->start.y)<=.25||
         std::hypot(point.x-job->frontier.x,point.y-job->frontier.y)>3.0||
         !ca::planning_footprint(*map_,point)||!ca::observation_visible(*map_,point,job->frontier))continue;
      last_sample=prefix_length;
      // 在同一条已验证路径的各前缀中先应用执行端真实禁区，再挑选剩余最优位置。
      if(ca::observation_blocked_by_regions(job->request.value("blocked_observations",ca::Json::array()),point,now().seconds()))continue;
      const auto visible=ca::visible_unknown_cells(*job->map,point);
      // 原图仍为未知就是尚未完成的观测，不能把“当前理论可见”当作已经写入地图而扣掉。
      double revisit=0;const auto visits=job->request.value("visits",ca::Json::array());
      if(visits.is_array()&&visits.size()<=64)for(const auto&v:visits){
        const auto position=v.is_object()?v.value("point",ca::Json::array()):v;
        if(!position.is_array()||position.size()!=2)continue;
        const double separation=std::hypot(point.x-position[0].get<double>(),point.y-position[1].get<double>());
        revisit+=std::max(0.,1.-separation/.8);
      }
      const double estimated_turn=ca::path_turn_estimate(planned_points,job->start,i,heading_window_m_);
      const double gain=visible.size()*.01,cost=ca::frontier_score(prefix_length,gain,estimated_turn,revisit,turn_cost_m_per_rad_);
      if(cost<best_score){best_score=cost;best={{"frontier",{job->frontier.x,job->frontier.y}}, {"observation",{point.x,point.y}},
        {"path_m",prefix_length},{"full_path_m",length},{"path_prefix",!final},{"gain_m2",gain},
        {"turn_rad",estimated_turn},{"grid_turn_rad",prefix_turn},{"heading_window_m",heading_window_m_},
        {"turn_cost_m_per_rad",turn_cost_m_per_rad_},{"revisit",revisit},{"score",cost}};}
    }
    if(!best.empty())job->results.push_back(std::move(best));
  }

  void tick(){
    const auto job=job_;if(!job)return;const auto pose=current();
    if(!healthy(job->request)||!pose||std::hypot(pose->x-job->start.x,pose->y-job->start.y)>.08){
      job->results=ca::Json::array();reply(job,"排序期间任务、定位或起点变化，丢弃过期结果");return;
    }
    if(seconds()-job->began>5){reply(job,"排序达到5秒预算，返回已完成安全检查的候选");return;}
    if(job->pending&&seconds()-job->plan_at>1.5){reply(job,"规划接受或结果超过1.5秒，取消只读请求并返回已检查候选");return;}
    if(job->selecting){const auto result=selector_.take();if(!result||result->job.operation!=job->token)return;
      job->selecting=false;if(!result->target){
        job->rejected.push_back({{"frontier",{job->frontier.x,job->frontier.y}},{"reason",result->reason}});next(job);return;
      }job->target=*result->target;plan(job);
    }
  }
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;ca::ObservationSelector selector_;
  rclcpp_action::Client<Plan>::SharedPtr client_;std::shared_ptr<Job>job_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;ca::Json state_=ca::Json::object();
  uint64_t revision_=0,sequence_=0;double state_at_=-1e9,map_at_=-1e9;
  double heading_window_m_=.30,turn_cost_m_per_rad_=.10/.22;bool prefer_clearance_=true;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_,requests_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_,progress_;rclcpp::TimerBase::SharedPtr timer_;
};
#ifndef FRONTIER_RANKER_NO_MAIN
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FrontierRanker>());rclcpp::shutdown();return 0;}
#endif
