#include "carcar_mapping_autonomy/exploration_route.hpp"
#include "carcar_mapping_autonomy/observation_selector.hpp"
#include "carcar_mapping_autonomy/route_validation.hpp"
#include <explore/frontier_search.h>
#include <chrono>
#include <map>
#include <queue>
#include <set>
#include <unordered_set>
namespace ca=carcar_mapping_autonomy;
namespace {
constexpr double inf=std::numeric_limits<double>::infinity();
double distance(const ca::Pose&a,const ca::Pose&b){return std::hypot(a.x-b.x,a.y-b.y);}
// 无量纲角代价只影响访问顺序，不改变可通行性或控制器限速。
double turn_cost(const ca::Pose&a,const ca::Pose&b){return 1-std::cos(std::atan2(b.y-a.y,b.x-a.x)-a.yaw);}
std::vector<double> distances(const nav_msgs::msg::OccupancyGrid & map,const ca::Pose & start){
  std::vector<double> costs(map.data.size(),inf);auto index=ca::ObservationSelector::Search::cell(map,start);
  if(!index||map.data[*index]!=0)return costs;
  using Entry=std::pair<double,size_t>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  costs[*index]=0;queue.push({0,*index});const int width=map.info.width,height=map.info.height;
  while(!queue.empty()){
    const auto [cost,i]=queue.top();queue.pop();if(cost>costs[i])continue;
    const int x=i%width,y=i/width;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
      const int nx=x+dx,ny=y+dy;if((!dx&&!dy)||nx<0||ny<0||nx>=width||ny>=height)continue;
      const size_t next=size_t(ny)*width+nx;if(map.data[next]!=0)continue;
      if(dx&&dy&&(map.data[size_t(y)*width+nx]!=0||map.data[size_t(ny)*width+x]!=0))continue;
      const double value=cost+map.info.resolution*(dx&&dy?std::sqrt(2.):1.);
      if(value<costs[next]){costs[next]=value;queue.push({value,next});}
    }
  }
  return costs;
}
std::unordered_set<uint64_t> gain_cells(const nav_msgs::msg::OccupancyGrid & map,const ca::Pose&point){
  std::unordered_set<uint64_t> unknown;
  // 按栅格边界逐格行进，不能用0.1米跳采样跨过仅0.05米厚的已知墙。
  const double resolution=map.info.resolution,ox=map.info.origin.position.x,oy=map.info.origin.position.y;
  auto occupied=[&](int x,int y){return x>=0&&y>=0&&x<int(map.info.width)&&y<int(map.info.height)&&map.data[size_t(y)*map.info.width+x]>=65;};
  // 3米处一度约5厘米，避免稀疏射线平移后产生大量虚假的“新增”采样格。
  for(int i=0;i<360;++i){const double angle=2*M_PI*i/360,dx=std::cos(angle),dy=std::sin(angle);
    int x=int(std::floor((point.x-ox)/resolution)),y=int(std::floor((point.y-oy)/resolution));
    const int sx=dx>=0?1:-1,sy=dy>=0?1:-1;
    const double step_x=std::abs(dx)>1e-12?resolution/std::abs(dx):inf,step_y=std::abs(dy)>1e-12?resolution/std::abs(dy):inf;
    double next_x=std::abs(dx)>1e-12?(ox+(x+(sx>0))*resolution-point.x)/dx:inf;
    double next_y=std::abs(dy)>1e-12?(oy+(y+(sy>0))*resolution-point.y)/dy:inf,range=0;
    for(size_t hops=0;hops<2000&&range<=3.;++hops){
      if(occupied(x,y))break;
      if(x<0||y<0||x>=int(map.info.width)||y>=int(map.info.height)||map.data[size_t(y)*map.info.width+x]<0){
        const auto gx=uint32_t(int32_t(std::floor((ox+(x+.5)*resolution)/.1)));
        const auto gy=uint32_t(int32_t(std::floor((oy+(y+.5)*resolution)/.1)));
        unknown.insert((uint64_t(gx)<<32)|gy);
      }
      range=std::min(next_x,next_y);
      if(std::abs(next_x-next_y)<1e-10){
        if(occupied(x+sx,y)||occupied(x,y+sy))break;
        x+=sx;y+=sy;next_x+=step_x;next_y+=step_y;
      }else if(next_x<next_y){x+=sx;next_x+=step_x;}else{y+=sy;next_y+=step_y;}
    }
  }
  return unknown;
}
}
ca::ExplorationRouteResult ca::exploration_route(ExplorationRouteInput input){
  const auto began=std::chrono::steady_clock::now();ExplorationRouteResult result;result.input=std::move(input);
  auto & job=result.input;
  auto finish=[&](std::string reason){result.reason=std::move(reason);result.computation_ms=
    std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();return result;};
  if(!job.map||!grid_valid(*job.map)||job.map->data.size()>2000000||std::abs(grid_yaw(*job.map))>1e-6)
    {result.active_safe=false;return finish("地图无效、超过本轮两百万格资源上限或网格旋转不受支持；未生成路线");}
  if(job.active_path&&!job.prefix.empty()){
    RouteValidator::Job check;check.map=job.map;check.path=job.active_path;check.task=job.task;
    check.current=job.current;check.target=job.prefix.back().target;check.remaining=true;
    if(!RouteValidator::check(check,&result.reason)){result.active_safe=false;return finish("活动路线净空失效："+result.reason);}
  }
  const auto & map=*job.map;auto mask=planning_keepout_mask(map);
  // 用世界坐标采样格追踪预计未知区域后续变为已知的面积；不冒充真值覆盖率。
  std::unordered_set<uint64_t> newly_known;
  std::vector<double> observed_gain(job.visits.size(),0.);
  for(size_t vi=0;vi<job.visits.size();++vi){const auto&visit=job.visits[vi];if(!visit.reached||visit.predicted_cells.empty())continue;
    size_t observed=0;
    for(const auto key:visit.predicted_cells){
      const double x=(int32_t(uint32_t(key>>32))+.5)*.1,y=(int32_t(uint32_t(key))+.5)*.1;
      const auto index=ObservationSelector::Search::cell(map,{x,y,0});
      if(index&&map.data[*index]>=0){++observed;newly_known.insert(key);}
    }
    result.observations.push_back({{"target",{visit.target.x,visit.target.y}},
      {"predicted_unknown_m2",visit.gain},{"observed_known_m2",observed*.01}});
    observed_gain[vi]=observed*.01;
  }
  result.newly_known_m2=newly_known.size()*.01;
  if(!mask||!free_footprint(map,job.current)){
    result.active_safe=false;return finish("车位或规划掩膜不满足原净空条件；未进行前沿完成判定");
  }
  allow_planning_departure(map,job.current,*mask);
  // 新增收益必须来自改变视点；当前扫描视点已经覆盖的预测格不能再次计奖。
  // 这只是选点收益去重，不修改地图的未知状态，也不用于净空或完成判定。
  const auto current_view=gain_cells(map,job.current);
  auto additional_view=[&](const Pose&pose){auto cells=gain_cells(map,pose);
    for(const auto cell:current_view)cells.erase(cell);return cells;};
  // 稳定的是仍有新增观测收益的承诺；地图已观测到的区域不能继续领取旧收益。
  std::unordered_set<uint64_t> committed_coverage;
  for(const auto&point:job.prefix){
    const auto cells=additional_view(point.target);size_t added=0;
    for(const auto cell:cells)if(!committed_coverage.count(cell))++added;
    if(added*.01<.10)continue;
    result.route.push_back(point);committed_coverage.insert(cells.begin(),cells.end());
  }
  const auto start=result.route.empty()?job.current:result.route.back().target;
  const auto from_start=distances(*mask,start);
  // 复用固定explore_lite前沿搜索；图框外一格未知避免裁剪边缘误报建图完成。
  const auto width=map.info.width+2,height=map.info.height+2;
  nav2_costmap_2d::Costmap2D costmap(width,height,map.info.resolution,
    map.info.origin.position.x-map.info.resolution,map.info.origin.position.y-map.info.resolution,255);
  for(unsigned y=0;y<map.info.height;++y)for(unsigned x=0;x<map.info.width;++x){const auto value=map.data[size_t(y)*map.info.width+x];
    costmap.setCost(x+1,y+1,value<0?255:(value>=65?254:0));}
  frontier_exploration::FrontierSearch search(&costmap,.001,1.,.20,rclcpp::get_logger("continuous_frontier"));
  geometry_msgs::msg::Point position;position.x=job.current.x;position.y=job.current.y;
  const auto frontiers=search.searchFrom(position);result.frontiers=frontiers.size();
  ObservationSelector::Search observations;std::vector<ExplorationPoint> candidates;
  Json rejected={{"无安全观测点",0},{"距离不足",0},{"重复候选",0},{"静态路径不可达",0},{"无未知收益",0},{"到访抑制",0}};
  std::set<std::pair<int,int>> bins;
  for(const auto & frontier:frontiers){
    // 同一连通前沿可环绕整间房，按空间采样保留多个观测方向，避免只看簇均值。
    std::vector<geometry_msgs::msg::Point> representatives{frontier.middle};
    for(const auto&p:frontier.points){if(bins.insert({int(std::floor(p.x/.8)),int(std::floor(p.y/.8))}).second)representatives.push_back(p);}
    std::stable_sort(representatives.begin(),representatives.end(),[&](const auto&a,const auto&b){
      return std::hypot(a.x-start.x,a.y-start.y)<std::hypot(b.x-start.x,b.y-start.y);});
    for(const auto & p:representatives){
      ObservationSelector::Job selected;selected.map=job.map;selected.revision=job.revision;
      selected.current=job.current;selected.frontier={p.x,p.y,0};selected.task=job.task;
      const auto target=observations.select(selected);
      if(!target){rejected["无安全观测点"]=rejected["无安全观测点"].get<int>()+1;continue;}
      // 首步至少覆盖0.15米到点容差、0.10米SLAM移动门槛和半个5厘米栅格余量。
      // 原0.45米过滤会把起点附近唯一安全的约0.30米观测点全部排除。
      const double minimum_step=result.route.empty()?.275:.45;
      if(distance(*target,start)<minimum_step){rejected["近点距离上界_m"]=std::max(rejected.value("近点距离上界_m",0.),distance(*target,start));rejected["距离不足"]=rejected["距离不足"].get<int>()+1;continue;}
      if(std::any_of(result.route.begin(),result.route.end(),[&](const auto&q){return distance(q.target,*target)<.60;}))continue;
      if(std::any_of(candidates.begin(),candidates.end(),[&](const auto&q){return distance(q.target,*target)<.70;})){rejected["重复候选"]=rejected["重复候选"].get<int>()+1;continue;}
      const auto index=ObservationSelector::Search::cell(map,*target);
      if(!index||!std::isfinite(from_start[*index])){rejected["静态路径不可达"]=rejected["静态路径不可达"].get<int>()+1;continue;}
      const auto cells=additional_view(*target);const auto predicted=cells.size()*.01;
      if(predicted<.10){rejected["无未知收益"]=rejected["无未知收益"].get<int>()+1;continue;}
      bool blocked=false;
      for(size_t vi=0;vi<job.visits.size();++vi){const auto&v=job.visits[vi];const double separation=distance(v.target,*target);
        if(separation>=.65)continue;
        if(v.reached){
          // 真正获得新地图的到访不能封锁整个邻域；只禁止同一视点或无收益区域重复领取旧收益。
          blocked=(separation<.20||observed_gain[vi]<.10)&&predicted<=v.gain*1.15+.05;
        }else{
          // 地图变化后允许重新审查临时失败点，至少等待5秒，避免沿用旧图的60秒黑名单。
          blocked=job.now<v.until&&(v.revision==job.revision||job.now<v.until-55.);
        }
        if(blocked)break;
      }
      if(blocked){rejected["到访抑制"]=rejected["到访抑制"].get<int>()+1;continue;}
      const double score=(from_start[*index]+.5*turn_cost(start,*target))/std::sqrt(predicted);
      candidates.push_back({*target,{p.x,p.y,0},predicted,score,{cells.begin(),cells.end()}});
      if(candidates.size()>=24)break;
    }
    if(candidates.size()>=24)break;
  }
  std::stable_sort(candidates.begin(),candidates.end(),[](const auto&a,const auto&b){return a.cost<b.cost;});
  if(candidates.size()>8)candidates.resize(8);
  for(const auto&c:candidates)result.candidates.push_back({{"target",{c.target.x,c.target.y}},
    {"frontier",{c.frontier.x,c.frontier.y}},{"predicted_unknown_m2",c.gain},{"start_score",c.cost}});
  if(candidates.empty())return finish(frontiers.empty()?"没有原始前沿":"前沿仍存在，但本轮无新增可达收益候选；等待新图，未宣布完成；候选拒绝="+rejected.dump());
  // 只追加一个当前点和一个后续点；扣除已承诺观测区域的重复收益。
  std::unordered_set<uint64_t> covered;
  for(const auto&p:result.route){const auto cells=additional_view(p.target);covered.insert(cells.begin(),cells.end());}
  while(result.route.size()<2&&!candidates.empty()){
    auto anchor=result.route.empty()?job.current:result.route.back().target;
    if(!result.route.empty())anchor.yaw=std::atan2(anchor.y-job.current.y,anchor.x-job.current.x);
    const auto costs=distances(*mask,anchor);size_t best=candidates.size();double best_cost=inf,best_gain=0;
    for(size_t i=0;i<candidates.size();++i){const auto index=ObservationSelector::Search::cell(map,candidates[i].target);
      if(!index||!std::isfinite(costs[*index]))continue;
      const auto&cells=candidates[i].predicted_cells;size_t added=0;
      for(auto cell:cells)if(!covered.count(cell))++added;
      const double marginal=added*.01;if(marginal<.10)continue;
      // 当前点尚未通过时不预先承诺大于120度的折返；先等其新增扫描再决定回溯。
      // 当前点完成后仍允许重新选反向点，因此死胡同和穿门回溯不会被永久禁用。
      if(!result.route.empty()&&turn_cost(anchor,candidates[i].target)>1.5)continue;
      const double cost=(costs[*index]+.5*turn_cost(anchor,candidates[i].target))/std::sqrt(marginal);
      if(cost<best_cost){best=i;best_cost=cost;best_gain=marginal;}
    }
    if(best==candidates.size())break;
    auto selected=candidates[best];selected.cost=best_cost;
    selected.target.yaw=std::atan2(selected.target.y-anchor.y,selected.target.x-anchor.x);
    // 访问抑制保留原始预计收益，边际收益另写决策日志。
    covered.insert(selected.predicted_cells.begin(),selected.predicted_cells.end());
    result.candidates.push_back({{"selected",{selected.target.x,selected.target.y}},
      {"marginal_unknown_m2",best_gain},{"route_score",best_cost}});
    result.route.push_back(selected);candidates.erase(candidates.begin()+best);
  }
  return finish("稳定两点路线完成；已扣除当前位置和承诺点的重复观测区域，仍须审查实际执行路径");
}
