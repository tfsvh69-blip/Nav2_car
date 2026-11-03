#define CARCAR_POINT_FIXTURE_ONLY
#include "test_point_navigation.cpp"
#include <xtensor/xrandom.hpp>
#include <nav2_mppi_controller/controller.hpp>
#include "carcar_mapping_autonomy/auto_motion.hpp"
#include "carcar_mapping_autonomy/mapping_rotation_shim.hpp"
namespace {
std::vector<geometry_msgs::msg::Point> obstacle_points(const nav_msgs::msg::OccupancyGrid&map){
  std::vector<geometry_msgs::msg::Point> result;
  for(unsigned y=0;y<map.info.height;++y)for(unsigned x=0;x<map.info.width;++x){
    if(map.data[y*map.info.width+x]<50)continue;
    for(double dx:{0.,1.})for(double dy:{0.,1.}){
      geometry_msgs::msg::Point p;p.x=map.info.origin.position.x+(x+dx)*map.info.resolution;
      p.y=map.info.origin.position.y+(y+dy)*map.info.resolution;result.push_back(p);
    }
  }
  return result;
}
struct ControlScene {
  nav_msgs::msg::OccupancyGrid map;
  nav_msgs::msg::Path route;
  ca::Pose initial;Twist initial_velocity;
  bool heading=false,updates=false,temporary=false;
};
geometry_msgs::msg::PoseStamped point(double x,double y,double yaw=0){
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);return p;
}
ControlScene scene(const std::string&name){
  ControlScene s;
  if(name.rfind("recorded",0)==0){
    std::ifstream file(environment("CARCAR_RECORDED_FIXTURE"));if(!file)throw std::runtime_error("缺少本轮录制夹具");
    const auto j=Json::parse(file);const auto&m=j.at("map");s.map.header.frame_id="map";
    s.map.info.width=m["info"]["width"];s.map.info.height=m["info"]["height"];s.map.info.resolution=m["info"]["resolution"];
    s.map.info.origin.position.x=m["info"]["origin"]["position"]["x"];s.map.info.origin.position.y=m["info"]["origin"]["position"]["y"];
    s.map.info.origin.orientation.w=1;s.map.data=m["data"].get<std::vector<int8_t>>();s.route=path(j.at("route"));
    s.initial={j["initial_pose"][0],j["initial_pose"][1],j["initial_pose"][2]};
    s.initial_velocity.linear.x=j["initial_velocity"][0];s.initial_velocity.angular.z=j["initial_velocity"][1];return s;
  }
  s.map.header.frame_id="map";s.map.info.width=s.map.info.height=160;s.map.info.resolution=.05;
  s.map.info.origin.position.x=s.map.info.origin.position.y=-3;s.map.info.origin.orientation.w=1;
  s.map.data.assign(160*160,0);s.route.header.frame_id="map";
  if(name=="behind")s.initial.yaw=M_PI;
  if(name=="short_return"){s.heading=true;for(int i=0;i<=19;++i)s.route.poses.push_back(point(-i*.01,0,i==19?0:M_PI));}
  else if(name=="s_curve"||name=="path_updates"){
    s.updates=name=="path_updates";
    for(int i=0;i<=100;++i){const double x=i*.02,y=.22*std::sin(M_PI*x);s.route.poses.push_back(point(x,y,std::atan(.22*M_PI*std::cos(M_PI*x))));}
  }else if(name=="right_angle"||name=="known_corner"){
    for(int i=0;i<=50;++i)s.route.poses.push_back(point(i*.02,0,0));
    for(int i=1;i<=50;++i)s.route.poses.push_back(point(1,i*.02,M_PI/2));
  }else for(int i=0;i<=100;++i)s.route.poses.push_back(point(i*.02,0,0));
  s.temporary=name=="temporary_obstacle";
  for(unsigned y=0;y<s.map.info.height;++y)for(unsigned x=0;x<s.map.info.width;++x){
    const auto p=ca::cell_world(s.map,x,y);auto&v=s.map.data[y*s.map.info.width+x];
    if(name=="narrow"&&std::abs(p.y)>.35)v=100;
    if(name=="wall"&&p.y<-.35)v=100;
    if(name=="known_corner"){
      const bool horizontal=p.x>=-.6&&p.x<=1.35&&std::abs(p.y)<=.35;
      const bool vertical=p.y>=-.35&&p.y<=1.6&&std::abs(p.x-1)<=.35;
      if(!horizontal&&!vertical)v=-1;
    }
  }
  return s;
}
// 同一软膨胀场供所有控制器使用；离线生成，不计入控制器计算耗时。
std::vector<unsigned char> inflation(const nav_msgs::msg::OccupancyGrid&map,double radius){
  std::vector<unsigned char> costs(map.data.size());const int reach=std::ceil(radius/map.info.resolution);
  for(int y=0;y<int(map.info.height);++y)for(int x=0;x<int(map.info.width);++x){
    const auto index=y*map.info.width+x;const auto value=map.data[index];
    if(value<0){costs[index]=255;continue;}if(value>=50){costs[index]=254;continue;}
    double nearest=1e9;
    for(int dy=-reach;dy<=reach;++dy)for(int dx=-reach;dx<=reach;++dx){
      const int nx=x+dx,ny=y+dy;if(nx<0||ny<0||nx>=int(map.info.width)||ny>=int(map.info.height))continue;
      if(map.data[ny*map.info.width+nx]>=50)nearest=std::min(nearest,std::hypot(dx,dy)*map.info.resolution);
    }
    costs[index]=nearest<=.13?253:nearest<=radius?static_cast<unsigned char>(252*std::exp(-5*(nearest-.13))):0;
  }
  return costs;
}
TEST_F(PointNavigation, SamePathControllersUseRealSmootherAndDelayedExecution){
  const auto profile=environment("CARCAR_CONTROLLER","rpp"),name=environment("CARCAR_SCENE","s_curve");
  const unsigned seed=std::stoul(environment("CARCAR_SEED","17"));xt::random::seed(seed);
  auto source=scene(name);const auto original=source.map;
  const double radius=std::stod(environment("CARCAR_INFLATION","0.20"));auto costs=inflation(source.map,radius);auto world_obstacles=obstacle_points(source.map);
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("comparison_local");
  for(const auto&r:local->set_parameters({{"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
      {"global_frame","map"},{"robot_base_frame","base_footprint"},{"rolling_window",false},{"track_unknown_space",true},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
  local->configure();local->getCostmap()->resizeMap(60,60,.05,-1.5,-1.5);
  std::vector<std::string>args{"--ros-args","--params-file",NAV_BASE_CONFIG,"--params-file",MAPPING_CONFIG,"--params-file",RPP_CONFIG};
  if(profile=="shim"||profile=="stock_shim")args.insert(args.end(),{"--params-file",SHIM_CONFIG});
  if(profile=="mppi")args.insert(args.end(),{"--params-file",MPPI_CONFIG});
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",rclcpp::NodeOptions().arguments(args).automatically_declare_parameters_from_overrides(true));
  std::unique_ptr<nav2_core::Controller>controller;
  if(profile=="shim")controller=std::make_unique<ca::MappingRotationShim>();
  else if(profile=="stock_shim")controller=std::make_unique<nav2_rotation_shim_controller::RotationShimController>();
  else if(profile=="mppi")controller=std::make_unique<nav2_mppi_controller::MPPIController>();
  else controller=std::make_unique<RecordedRpp>();
  std::unique_ptr<nav2_core::GoalChecker>checker;
  if(source.heading)checker=std::make_unique<nav2_controller::SimpleGoalChecker>();
  else checker=std::make_unique<nav2_controller::PositionGoalChecker>();
  checker->initialize(node,source.heading?"general_goal_checker":"exploration_goal_checker",local);
  ManualProgressChecker progress;progress.start(node);
  controller->configure(node,"FollowPath",local->getTfBuffer(),local);controller->activate();controller->setPlan(source.route);
  auto smoother=std::make_shared<ManualSmoother>(rclcpp::NodeOptions().arguments({"--ros-args","--params-file",NAV_BASE_CONFIG,"--params-file",MAPPING_CONFIG}));
  Twist actual=source.initial_velocity,request;smoother->start(actual);std::deque<Twist>commands(4,actual);
  ca::Pose current=source.initial;const auto target=source.route.poses.back();
  bool reached=false,safe=true,finite=true,progress_failed=false,obstacle_present=false;unsigned stops=0,stopped=0,exceptions=0;
  bool was_moving=false;double zero_since=-1,distance=0,last_good=0,time=0;
  std::vector<double>durations,sweep_durations;size_t sweep_budget_failures=0;Json trace=Json::array();Pattern pattern;
  constexpr double dt=.05;
  for(unsigned step=0;step<2400;++step){
    time=step*dt;auto pose=point(current.x,current.y,current.yaw);pose.header.stamp=node->now();
    const bool obstacle=source.temporary&&time>=6&&time<9;
    if(obstacle!=obstacle_present){
      obstacle_present=obstacle;source.map=original;
      if(obstacle)for(unsigned y=0;y<source.map.info.height;++y)for(unsigned x=0;x<source.map.info.width;++x){
        const auto p=ca::cell_world(source.map,x,y);if(std::hypot(p.x-.95,p.y)<.08)source.map.data[y*source.map.info.width+x]=100;
      }
      costs=inflation(source.map,radius);world_obstacles=obstacle_points(source.map);
    }
    auto*cm=local->getCostmap();cm->updateOrigin(current.x-1.5,current.y-1.5);
    for(unsigned y=0;y<60;++y)for(unsigned x=0;x<60;++x){double wx,wy;cm->mapToWorld(x,y,wx,wy);
      const int gx=std::floor((wx-source.map.info.origin.position.x)/.05),gy=std::floor((wy-source.map.info.origin.position.y)/.05);
      cm->setCost(x,y,gx<0||gy<0||gx>=int(source.map.info.width)||gy>=int(source.map.info.height)?255:costs[gy*source.map.info.width+gx]);
    }
    geometry_msgs::msg::TransformStamped tf;tf.header=pose.header;tf.child_frame_id="base_footprint";
    tf.transform.translation.x=current.x;tf.transform.translation.y=current.y;tf.transform.rotation=pose.pose.orientation;
    ASSERT_TRUE(local->getTfBuffer()->setTransform(tf,"comparison_wheel",true));
    if(step%2==0){
      if(checker->isGoalReached(pose.pose,target.pose,actual))reached=true;
      if(reached)request=Twist();
      else if(!progress.at(pose,time)){progress_failed=true;request=Twist();}
      else{
        if(source.updates&&step%20==0){
          auto update=source.route;size_t nearest=0;double closest=1e9;
          for(size_t i=0;i<update.poses.size();++i){const auto&p=update.poses[i].pose.position;const double d=std::hypot(current.x-p.x,current.y-p.y);if(d<closest){closest=d;nearest=i;}}
          update.poses.erase(update.poses.begin(),update.poses.begin()+nearest);update.poses.insert(update.poses.begin(),pose);controller->setPlan(update);
        }
        const auto began=std::chrono::steady_clock::now();
        try{request=controller->computeVelocityCommands(pose,actual,checker.get()).twist;last_good=time;}
        catch(const std::exception&){request=Twist();++exceptions;}
        durations.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count());
      }
      pattern.accept(request,.1);
    }
    auto smoothed=smoother->step(request);
    // 复现正式监督器位于平滑器之后的最终速度包络，所有候选使用同一限制。
    smoothed.linear.x=std::clamp(smoothed.linear.x,0.,.10);smoothed.angular.z=std::clamp(smoothed.angular.z,-.36,.36);
    std::vector<ca::Pose> points;
    for(const auto&p:world_obstacles)if(std::hypot(p.x-current.x,p.y-current.y)<1.)points.push_back({p.x,p.y,0});
    const auto protected_command=ca::auto_motion_check(source.map,ca::Task(),current,points,smoothed,actual);
    sweep_durations.push_back(protected_command.compute_ms);
    if(protected_command.reason.find("8毫秒")!=std::string::npos)++sweep_budget_failures;
    smoothed=protected_command.velocity;
    commands.push_back(smoothed);actual=commands.front();commands.pop_front();
    finite=std::isfinite(actual.linear.x)&&std::isfinite(actual.angular.z)&&actual.linear.x>=-1e-9&&actual.linear.x<=.10+1e-6&&
      std::abs(actual.angular.z)<=.36+1e-6&&std::abs(actual.linear.y)<1e-9;
    const auto before=current;current.x+=actual.linear.x*dt*std::cos(current.yaw+actual.angular.z*dt/2);
    current.y+=actual.linear.x*dt*std::sin(current.yaw+actual.angular.z*dt/2);current.yaw=std::remainder(current.yaw+actual.angular.z*dt,2*M_PI);
    distance+=std::abs(actual.linear.x)*dt;
    safe=ca::free_connector(source.map,before,current,.22);
    if(actual.linear.x>.02){was_moving=true;zero_since=-1;}
    else if(was_moving&&!reached&&actual.linear.x<.005){if(zero_since<0)zero_since=time;if(time-zero_since>.4){++stops;was_moving=false;}}
    if(reached&&std::abs(actual.linear.x)+std::abs(actual.angular.z)<1e-6)++stopped;else stopped=0;
    if(step%2==0)trace.push_back({{"t",time},{"pose",{current.x,current.y,current.yaw}},{"raw",{request.linear.x,request.angular.z}},
      {"actual",{actual.linear.x,actual.angular.z}},{"safe",safe}});
    if(stopped>=10||!safe||!finite||progress_failed||(!source.temporary&&time-last_good>5))break;
  }
  std::sort(durations.begin(),durations.end());const double p99=durations.empty()?0:durations[std::min(durations.size()-1,size_t(std::ceil(durations.size()*.99)-1))];
  std::sort(sweep_durations.begin(),sweep_durations.end());
  const double sweep_p99=sweep_durations.empty()?0:sweep_durations[std::min(sweep_durations.size()-1,size_t(std::ceil(sweep_durations.size()*.99)-1))];
  Json result={{"sweep_p99_ms",sweep_p99},{"sweep_max_ms",sweep_durations.empty()?0:sweep_durations.back()},{"sweep_budget_failures",sweep_budget_failures},{"profile",profile},{"scene",name},{"seed",seed},{"hardware",false},{"real_controller",true},{"real_smoother",true},{"project_execution_sweep",true},
    {"map_resolution",.05},{"local_inflation",radius},{"speed_limit",.10},{"actuation_delay_seconds",.20},
    {"safe",safe},{"limits",finite},{"progress_failed",progress_failed},{"reached",reached},{"stopped",stopped>=10},
    {"seconds",time},{"distance_m",distance},{"stop_cycles",stops},{"exceptions",exceptions},{"pattern",pattern.state()},
    {"compute_p99_ms",p99},{"compute_max_ms",durations.empty()?0:durations.back()},{"trace",trace}};
  const auto output=environment("CARCAR_COMPARISON_OUTPUT","controller-comparison.json");std::ofstream(output)<<result.dump(2);
  result.erase("trace");std::cout<<"CONTROLLER_COMPARISON "<<result.dump()<<std::endl;
  smoother->stop();controller->deactivate();controller->cleanup();local->cleanup();
  EXPECT_EQ(sweep_budget_failures,0U);EXPECT_TRUE(safe);EXPECT_TRUE(finite);EXPECT_FALSE(progress_failed);EXPECT_TRUE(reached);EXPECT_GE(stopped,10U);
  EXPECT_LE(p99,50);ASSERT_FALSE(durations.empty());EXPECT_LE(durations.back(),100);
}
TEST_F(PointNavigation, ShimUsesAvailablePoseTimeAndRejectsMissingOrStaleTransforms){
  auto local=std::make_shared<nav2_costmap_2d::Costmap2DROS>("pose_time_local");
  for(const auto&r:local->set_parameters({{"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
      {"global_frame","odom"},{"robot_base_frame","base_footprint"},{"rolling_window",false},
      {"footprint",std::string("[[-0.14,-0.13],[-0.14,0.13],[0.14,0.13],[0.14,-0.13]]")},{"footprint_padding",0.0}}))ASSERT_TRUE(r.successful);
  local->configure();local->getCostmap()->resizeMap(60,60,.05,-1.5,-1.5);
  for(unsigned y=0;y<60;++y)for(unsigned x=0;x<60;++x)local->getCostmap()->setCost(x,y,0);
  std::vector<std::string>args{"--ros-args","--params-file",NAV_BASE_CONFIG,"--params-file",MAPPING_CONFIG,
    "--params-file",RPP_CONFIG,"--params-file",SHIM_CONFIG};
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",rclcpp::NodeOptions().arguments(args).automatically_declare_parameters_from_overrides(true));
  nav2_controller::PositionGoalChecker checker;checker.initialize(node,"exploration_goal_checker",local);
  ca::MappingRotationShim controller;controller.configure(node,"FollowPath",local->getTfBuffer(),local);controller.activate();
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(int i=0;i<=20;++i)route.poses.push_back(point(i*.05,0));
  controller.setPlan(route);
  // 仅提供早于now的两帧真实时间TF；路径map、控制位姿odom，旋转90度防止漏变换也能通过。
  const auto stamp=node->now()-rclcpp::Duration::from_seconds(.05);
  geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="map";transform.child_frame_id="odom";
  transform.transform.rotation.z=std::sin(M_PI/4);transform.transform.rotation.w=std::cos(M_PI/4);
  transform.header.stamp=stamp-rclcpp::Duration::from_seconds(.05);
  ASSERT_TRUE(local->getTfBuffer()->setTransform(transform,"pose_time_map",false));
  transform.header.stamp=stamp;ASSERT_TRUE(local->getTfBuffer()->setTransform(transform,"pose_time_map",false));
  geometry_msgs::msg::TransformStamped wheel;wheel.header.frame_id="odom";wheel.child_frame_id="base_footprint";wheel.transform.rotation.w=1;
  wheel.header.stamp=stamp-rclcpp::Duration::from_seconds(.05);ASSERT_TRUE(local->getTfBuffer()->setTransform(wheel,"pose_time_wheel",false));
  wheel.header.stamp=stamp;ASSERT_TRUE(local->getTfBuffer()->setTransform(wheel,"pose_time_wheel",false));
  auto pose=point(0,0);pose.header.frame_id="odom";pose.header.stamp=stamp;Twist velocity;
  auto began=std::chrono::steady_clock::now();
  const auto command=controller.computeVelocityCommands(pose,velocity,&checker);
  const auto valid_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  EXPECT_DOUBLE_EQ(command.twist.linear.x,0.);EXPECT_LT(command.twist.angular.z,0.);
  EXPECT_LE(std::abs(command.twist.angular.z),.22+1e-6);EXPECT_LT(valid_ms,50.);
  // 当前时间没有对应TF：必须快速拒绝，不改查最新TF，也不等待未来数据来掩盖时序错误。
  pose.header.stamp=node->now();controller.setPlan(route);began=std::chrono::steady_clock::now();
  EXPECT_THROW(controller.computeVelocityCommands(pose,velocity,&checker),nav2_core::PlannerException);
  const auto missing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  EXPECT_LT(missing_ms,50.);
  pose.header.stamp=node->now()-rclcpp::Duration::from_seconds(1.0);
  EXPECT_THROW(controller.computeVelocityCommands(pose,velocity,&checker),nav2_core::PlannerException);
  pose.header.stamp=builtin_interfaces::msg::Time();
  EXPECT_THROW(controller.computeVelocityCommands(pose,velocity,&checker),nav2_core::PlannerException);
  pose.header.stamp=node->now()+rclcpp::Duration::from_seconds(.10);
  EXPECT_THROW(controller.computeVelocityCommands(pose,velocity,&checker),nav2_core::PlannerException);
  std::cout<<"POSE_TIME_CONTROLLER "<<Json{{"valid_ms",valid_ms},{"missing_ms",missing_ms},
    {"valid_vx",command.twist.linear.x},{"valid_wz",command.twist.angular.z}}.dump()<<std::endl;
  controller.deactivate();controller.cleanup();local->cleanup();
}

}
