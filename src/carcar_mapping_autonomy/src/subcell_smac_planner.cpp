#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "carcar_mapping_autonomy/subcell_smac_planner.hpp"

namespace carcar_mapping_autonomy {
void SubcellSmacPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
  std::string name,std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  SmacPlanner2D::configure(parent,std::move(name),tf,costmap_ros);
  costmap_ros_=std::move(costmap_ros);
  // 使用Nav2实际足迹的外接圆，加独立的2厘米动态障碍余量；保证所有转向都覆盖车身。
  // 原始地图/未知区域仍保留22厘米要求，避免将标障栅格的量化误差再次算作车体尺寸。
  double inscribed=0,circumscribed=0;
  nav2_costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint(),inscribed,circumscribed);
  if(!std::isfinite(circumscribed)||circumscribed<=0)throw std::runtime_error("规划器配置失败：Nav2足迹无效");
  dynamic_radius_=circumscribed+.02;
  RCLCPP_INFO(_logger,"规划动态包络：Nav2足迹外接圆%.4f米，加0.0200米余量，实际半径%.4f米",circumscribed,dynamic_radius_);
  diagnostics_=parent.lock()->create_publisher<std_msgs::msg::String>("/mapping/planner_diagnostic",rclcpp::QoS(10));
  diagnostics_->on_activate();
  maps_=parent.lock()->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",
    rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      std::lock_guard<std::mutex> lock(map_mutex_);map_=m;map_at_=std::chrono::steady_clock::now();
    });
}
void SubcellSmacPlanner::cleanup() {
  diagnostics_.reset();
  maps_.reset();
  {std::lock_guard<std::mutex> lock(map_mutex_);map_.reset();}
  SmacPlanner2D::cleanup();costmap_ros_.reset();
}
nav_msgs::msg::Path SubcellSmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) {
  try{return checkedPlan(start,goal);}
  catch(const std::exception&e){
    const std::string reason=e.what();
    const bool blocked=reason.find("实际车位存在动态障碍")!=std::string::npos||
      reason.find("实际车位不满足原图")!=std::string::npos;
    std_msgs::msg::String message;
    message.data=Json{{"planner_id",_name},{"goal_stamp_ns",rclcpp::Time(goal.header.stamp).nanoseconds()},
      {"target",{goal.pose.position.x,goal.pose.position.y}},{"code",blocked?"START_BLOCKED":"PLAN_FAILED"},
      {"reason",reason},{"dynamic_radius_m",dynamic_radius_}}.dump();
    if(diagnostics_)diagnostics_->publish(message);
    throw;
  }
}
nav_msgs::msg::Path SubcellSmacPlanner::checkedPlan(
  const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) {
  std::lock_guard<std::mutex> operation(planning_mutex_);
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  {std::lock_guard<std::mutex> lock(map_mutex_);
    if(!map_||std::chrono::duration<double>(std::chrono::steady_clock::now()-map_at_).count()>10)
      throw std::runtime_error("规划未执行：原始地图缺失或过期；检查/map更新");
    map=map_;
  }
  const Pose current{start.pose.position.x,start.pose.position.y,0};
  if(start.header.frame_id!="map"||goal.header.frame_id!="map"||!grid_valid(*map)||
    std::abs(grid_yaw(*map))>1e-6||!free_footprint(*map,current))
    throw std::runtime_error("规划未执行：实际车位不满足原图0.22米净空；保持停车检查地图与定位");
  // 整个检查与Smac搜索共用Nav2的递归代价图锁，不修改任何格值。
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
  if(_costmap->getSizeInCellsX()!=map->info.width||_costmap->getSizeInCellsY()!=map->info.height||
    std::abs(_costmap->getResolution()-map->info.resolution)>1e-6||
    std::abs(_costmap->getOriginX()-map->info.origin.position.x)>1e-6||
    std::abs(_costmap->getOriginY()-map->info.origin.position.y)>1e-6)
    throw std::runtime_error("规划未执行：原图与代价图尚未对齐；等待地图更新后重试");
  unsigned sx=0,sy=0,gx=0,gy=0;
  if(!_costmap->worldToMap(current.x,current.y,sx,sy)||
    !_costmap->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy))
    throw std::runtime_error("规划未执行：车位或目标超出代价图");

  std::vector<nav_msgs::msg::OccupancyGrid> obstacles;
  for(const auto&layer:*costmap_ros_->getLayeredCostmap()->getPlugins()) {
    auto obstacle=std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(layer);
    if(!obstacle||!obstacle->isEnabled())continue;
    // 原始动态障碍层与禁行过滤层分开检查，避免将禁行原因误当成可忽略的实物障碍。
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id="map";grid.info.width=obstacle->getSizeInCellsX();
    grid.info.height=obstacle->getSizeInCellsY();grid.info.resolution=obstacle->getResolution();
    grid.info.origin.position.x=obstacle->getOriginX();grid.info.origin.position.y=obstacle->getOriginY();
    grid.info.origin.orientation.w=1;grid.data.resize(size_t(grid.info.width)*grid.info.height);
    for(size_t i=0;i<grid.data.size();++i) {
      const auto v=obstacle->getCharMap()[i];
      // 动态层NO_INFORMATION表示没有该层观测，原始地图的未知区仍由另一项检查拒绝。
      grid.data[i]=(v>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE&&v!=nav2_costmap_2d::NO_INFORMATION)?100:0;
    }
    obstacles.push_back(std::move(grid));
  }
  auto connector_safe=[&](const Pose&a,const Pose&b) {
    if(!free_connector(*map,a,b))return false;
    for(const auto&obstacle:obstacles)if(!free_connector(obstacle,a,b,dynamic_radius_))return false;
    return true;
  };
  auto checked=[&](nav_msgs::msg::Path path) {
    if(path.poses.empty())return path;
    Pose previous=current;
    for(const auto&p:path.poses) {
      const Pose next{p.pose.position.x,p.pose.position.y,0};
      if(!connector_safe(previous,next))return nav_msgs::msg::Path{};
      previous=next;
    }
    auto exact=start;exact.header=path.header;
    if(std::hypot(path.poses.front().pose.position.x-current.x,
                  path.poses.front().pose.position.y-current.y)>1e-6)
      path.poses.insert(path.poses.begin(),exact);
    {std::lock_guard<std::mutex> guard(map_mutex_);
      if(!map_||!same_grid(*map,*map_))return nav_msgs::msg::Path{};
    }
    return path;
  };
  const auto center=cell_world(*map,sx,sy);
  if(_costmap->getCost(sx,sy)<nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE&&
    connector_safe(current,center)) {
    auto path=checked(SmacPlanner2D::createPlan(start,goal));
    if(path.poses.empty())throw std::runtime_error("规划路径未通过连续净空检查，未下发路径；检查原图及动态障碍");
    return path;
  }
  // 只处理额外规划余量/取整格心造成的堵塞；真实当前位置不安全时绝不寻找逃逸路径。
  if(!connector_safe(current,current))
    throw std::runtime_error("规划未执行：实际车位存在动态障碍，保持停车检查激光与现场");
  if(planning_footprint(*map,center))
    throw std::runtime_error("规划未执行：起点被实时代价图阻挡，不属于地图离散起点问题");
  auto available=*map;
  for(size_t i=0;i<available.data.size();++i)
    available.data[i]=_costmap->getCharMap()[i]<nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE?0:100;
  const auto entries=departure_entries(*map,current,available);
  size_t attempts=0;
  const auto began=std::chrono::steady_clock::now();
  for(const auto&entry:entries) {
    if(attempts>=4||std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>1.)break;
    if(!connector_safe(current,entry))continue;
    ++attempts;
    auto seed=start;seed.pose.position.x=entry.x;seed.pose.position.y=entry.y;
    nav_msgs::msg::Path path;
    try {path=checked(SmacPlanner2D::createPlan(seed,goal));}
    catch(const std::runtime_error&){continue;}
    if(path.poses.empty())continue;
    RCLCPP_INFO(_logger,"规划安全衔接：实际车位[%.4f,%.4f]经入口[%.4f,%.4f]接入Smac，连接%.3f米；原图/禁行层未改，0.22米及动态障碍检查通过",
      current.x,current.y,entry.x,entry.y,std::hypot(entry.x-current.x,entry.y-current.y));
    return path;
  }
  throw std::runtime_error("规划未执行：实际车位安全，但附近没有通过净空和动态障碍检查的规划入口；保持停车检查地图与通道");
}
}  // namespace carcar_mapping_autonomy
PLUGINLIB_EXPORT_CLASS(carcar_mapping_autonomy::SubcellSmacPlanner,nav2_core::GlobalPlanner)
