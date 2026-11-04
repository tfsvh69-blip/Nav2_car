#define CARCAR_SUPERVISOR_FIXTURE_ONLY
#include "test_supervisor.cpp"
#include "carcar_mapping_autonomy/navigation_speed.hpp"

class LiveSupervisorIntegration:public SupervisorIntegration {
protected:
  void SetUp()override{live_mode=recovery_mode=restricted_manual_enabled=true;slam_sequence=4;SupervisorIntegration::SetUp();}
  Json walls(double x=1.5){return Json::array({{{"id","glass"},{"points",Json::array({Json::array({x,-1.0}),Json::array({x,1.0})})}}});}
  void wall_request(const std::string&action,const std::string&id,Json items=Json()){
    Json data={{"navigation_generation",navigation_generation},{"wall_revision",live_scene->state().value("revision",uint64_t{})}};
    if(action=="set_walls")data["walls"]=items;
    request(action,id,data);
  }
  void install_wall(double x=1.5){wall_request("set_walls","initial-wall",walls(x));
    // 场景已更新不能代替监督器收到同版本ACK；旧空墙的READY不得提前结束等待。
    ASSERT_TRUE(until([this]{return live_scene->state().value("revision",0)==1&&
      state["walls"].value("applied_revision",0)==1&&state["walls"].value("navigation_ready",false)&&
      !state["walls"].value("pending",false)&&result("initial-wall","accepted")&&state.value("phase","")=="MANUAL";},4))<<state.dump();}
  bool result(const std::string&id,const std::string&value){
    for(const auto&r:state.value("request_results",Json::array()))if(r.value("request_id","")==id)return r.value("request_result","")==value;
    return false;}
};

class LiveAutomaticStartupIntegration:public LiveSupervisorIntegration {
protected:
  void SetUp()override{
    automatic_on_boot=true;
    tuning_state.update({{"ready",false},{"pending",true},{"requires_stop",false}});
    LiveSupervisorIntegration::SetUp();
  }
  void tuning_ready(){tuning_state.update({{"ready",true},{"pending",false},{"requires_stop",false}});}
  void cancel_waiting_start(const std::string&action,const std::string&target){
    ASSERT_TRUE(state.value("auto_start_pending",false))<<state.dump();
    request(action,"cancel-startup-"+action);
    ASSERT_TRUE(until([&]{return result("cancel-startup-"+action,"accepted")&&state.value("phase","")==target;},3))<<state.dump();
    EXPECT_FALSE(state.value("auto_start_pending",true));EXPECT_DOUBLE_EQ(last_velocity,0);
    const auto stopped_outputs=nonzero;
    if(action=="manual"){
      auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);double desired=.05;
      auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist velocity;velocity.linear.x=desired;manual->publish(velocity);});
      // 允许取消自动等待，不等于允许参数尚未核验时遥控行驶。
      until([]{return false;},.7);EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,stopped_outputs);
      desired=0;until([]{return false;},.2);heartbeat.reset();
    }
    // READY只能解除初始化等待，不能覆盖操作者已撤销的自动启动请求。
    tuning_ready();until([]{return false;},.7);
    EXPECT_EQ(state.value("phase",""),target);EXPECT_FALSE(state.value("auto_start_pending",true));
    EXPECT_EQ(nonzero,stopped_outputs);EXPECT_DOUBLE_EQ(last_velocity,0);
    EXPECT_EQ(nav_requests,0U);EXPECT_FALSE(state["fault"].value("active",false));
  }
};

class MappingReadyAutomaticStartupIntegration:public LiveAutomaticStartupIntegration {
protected:
  void SetUp()override{
    session_ready=false;session_fields["mapping_ready"]=true;
    LiveAutomaticStartupIntegration::SetUp();
  }
};

class MappingReadyManualIntegration:public LiveSupervisorIntegration {
protected:
  void SetUp()override{
    session_ready=false;session_fields["mapping_ready"]=true;
    LiveSupervisorIntegration::SetUp();
  }
};

TEST_F(MappingReadyAutomaticStartupIntegration, MappingReadinessStartsAutomaticWhileTeleopPermissionIsFalse){
  ASSERT_TRUE(state.value("auto_start_pending",false));
  EXPECT_EQ(state.value("phase",""),"MANUAL");EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));
  tuning_ready();
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3))<<state.dump();
  EXPECT_FALSE(state.value("auto_start_pending",true));EXPECT_TRUE(state.value("healthy",false));
}

TEST_F(MappingReadyManualIntegration, MappingReadinessBootstrapsHealthyManualWithoutTeleopPermissionFeedback){
  ASSERT_TRUE(until([this]{return state["manual_policy"].value("motion_allowed",false);},2))<<state.dump();
  EXPECT_TRUE(state.value("healthy",false));EXPECT_FALSE(state["restricted_manual"].value("active",true));
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=.07;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.05;},2))<<state.dump();
  heartbeat.reset();ASSERT_TRUE(until([this]{return last_velocity==0;},1));
  manual_owner_id.clear();
  ASSERT_TRUE(until([this]{return !state["manual_policy"].value("motion_allowed",true);},2))<<state.dump();
  EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, MappingReadinessFalseOverridesTrueLegacyReadiness){
  session_fields["mapping_ready"]=false;
  ASSERT_TRUE(until([this]{return !state.value("healthy",true);},2))<<state.dump();
  EXPECT_NE(state.value("health_reason","").find("建图会话未就绪"),std::string::npos);
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));
  request("start","mapping-not-ready");
  ASSERT_TRUE(until([this]{return result("mapping-not-ready","rejected");},2))<<state.dump();
  EXPECT_EQ(nav_requests,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, PointTwentySpeedRetentionUsesControllerReadbackAndRejectsHigherWithoutNewGoal){
  using Speed=carcar_mapping_autonomy::MappingNavigationSpeed;
  double controller_speed=.05;size_t writes=0,reads=0;
  auto set_speed=n->create_service<Speed::Set>("/controller_server/set_parameters_atomically",
    [&](Speed::Set::Request::SharedPtr request,Speed::Set::Response::SharedPtr response){
      ++writes;EXPECT_EQ(request->parameters.size(),1u);
      if(request->parameters.size()!=1){response->result.successful=false;return;}
      EXPECT_EQ(request->parameters.front().name,"FollowPath.desired_linear_vel");
      controller_speed=request->parameters.front().value.double_value;response->result.successful=true;
    });
  auto get_speed=n->create_service<Speed::Get>("/controller_server/get_parameters",
    [&](Speed::Get::Request::SharedPtr request,Speed::Get::Response::SharedPtr response){
      ++reads;EXPECT_EQ(request->names,std::vector<std::string>({"FollowPath.desired_linear_vel"}));
      rcl_interfaces::msg::ParameterValue value;value.type=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      value.double_value=controller_speed;response->values={value};
    });
  ASSERT_TRUE(until([this]{return state.value("speed",Json::object()).value("phase","")=="READY";},3))<<state.dump();
  EXPECT_DOUBLE_EQ(controller_speed,.1);start_goal();const auto goals_before=nav_requests;
  const auto writes_before=writes,reads_before=reads;
  request("set_speed","retain-point-twenty",{{"speed_mps",.20}});
  ASSERT_TRUE(until([this]{const auto speed=state.value("speed",Json::object());
    return speed.value("request_id","")=="retain-point-twenty"&&!speed.value("pending",true);},2))<<state.dump();
  // 调速以控制器真实写入和读回为成功依据，不把请求送达或旧任务回执当作accepted。
  EXPECT_EQ(state["speed"].value("phase",""),"READY");
  EXPECT_DOUBLE_EQ(state["speed"].value("confirmed_mps",0.),.20);
  EXPECT_DOUBLE_EQ(state["speed"].value("cap_mps",0.),.20);EXPECT_DOUBLE_EQ(controller_speed,.20);
  EXPECT_EQ(writes,writes_before+1);EXPECT_EQ(reads,reads_before+1);EXPECT_EQ(nav_requests,goals_before);
  const auto confirmed_writes=writes;
  request("set_speed","reject-speed-above-point-twenty",{{"speed_mps",.201}});
  ASSERT_TRUE(until([this]{const auto speed=state.value("speed",Json::object());
    return speed.value("request_id","")=="reject-speed-above-point-twenty"&&speed.value("phase","")=="FAILED";},2))<<state.dump();
  EXPECT_DOUBLE_EQ(state["speed"].value("cap_mps",0.),.20);EXPECT_DOUBLE_EQ(controller_speed,.20);
  EXPECT_EQ(writes,confirmed_writes);EXPECT_EQ(nav_requests,goals_before);EXPECT_EQ(state.value("phase",""),"EXPLORING");
}

class InvalidMappingReadinessIntegration:public LiveSupervisorIntegration,
    public testing::WithParamInterface<Json> {
public:
  static void SetUpTestSuite(){SupervisorIntegration::SetUpTestSuite();}
  static void TearDownTestSuite(){SupervisorIntegration::TearDownTestSuite();}
};

TEST_P(InvalidMappingReadinessIntegration, MappingReadinessInvalidValueCannotFallBackToLegacyReadiness){
  SCOPED_TRACE(GetParam().dump());session_fields["mapping_ready"]=GetParam();
  ASSERT_TRUE(until([this]{return !state.value("healthy",true);},2))<<state.dump();
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));EXPECT_DOUBLE_EQ(last_velocity,0);
  request("start","invalid-mapping-readiness");
  ASSERT_TRUE(until([this]{return result("invalid-mapping-readiness","rejected");},2))<<state.dump();
  EXPECT_EQ(nav_requests,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
}

INSTANTIATE_TEST_SUITE_P(InvalidMappingReadiness,InvalidMappingReadinessIntegration,
  testing::Values(Json(nullptr),Json(0),Json(1),Json("true"),Json::array(),Json::object()));

TEST_F(LiveSupervisorIntegration, MappingReadinessMissingFieldPreservesLegacyReadinessAndStorageGate){
  EXPECT_TRUE(session_fields.empty());EXPECT_TRUE(state.value("healthy",false));
  session_ready=false;
  ASSERT_TRUE(until([this]{return !state.value("healthy",true);},2))<<state.dump();
  EXPECT_DOUBLE_EQ(last_velocity,0);
  // 会话故障后的冻结保留；单独核对磁盘拒绝原因，不把输入恢复当作自动复验。
  storage_ready=false;
  ASSERT_TRUE(until([this]{return state.value("health_reason","").find("磁盘余量")!=std::string::npos;},2))<<state.dump();
  EXPECT_FALSE(state.value("healthy",true));
  EXPECT_DOUBLE_EQ(last_velocity,0);
}


// 真实监督器进程仍运行；夹具分别控制全局TF、本地TF、轮式输入和近场反馈，避免把地图故障混成传感器故障。
class LocalManualSupervisorIntegration:public LiveSupervisorIntegration {
protected:
  bool publish_map_transform=true,publish_odom_transform=true,publish_local_odometry=true,collision_feedback=true;
  double last_yaw=0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr yaw_output;
  void SetUp()override{
    LiveSupervisorIntegration::SetUp();timer.reset();
    timer=n->create_wall_timer(std::chrono::milliseconds(50),[this]{feed_local();});
    input.reset();input=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/collision_input",1,
      [this](const geometry_msgs::msg::Twist&m){if(collision_feedback)collision->publish(m);});
    yaw_output=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/cmd_vel",1,
      [this](const geometry_msgs::msg::Twist&m){last_yaw=m.angular.z;});
  }
  void TearDown()override{yaw_output.reset();LiveSupervisorIntegration::TearDown();}
  void fail_global(){
    publish_map_transform=false;nav2_active=false;session_ready=false;live_mask=live_map=live_state=false;
    recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true},{"reason","定位几何未通过"}});
    until([]{return false;},2.2);
  }
  void feed_local(){
    const auto now=n->now();
    geometry_msgs::msg::TransformStamped t;t.header.stamp=now;t.header.frame_id="map";t.child_frame_id="odom";t.transform.rotation.w=1;if(publish_map_transform)tf->sendTransform(t);t.header.frame_id="odom";t.child_frame_id="base_footprint";t.transform.translation.x=odom_x;t.transform.translation.y=odom_y;if(publish_odom_transform)tf->sendTransform(t);
    sensor_msgs::msg::LaserScan s;s.header.stamp=now;s.header.frame_id="laser_frame";s.angle_min=-M_PI;s.angle_increment=2*M_PI/360;s.angle_max=M_PI;s.range_min=.05;s.range_max=12;s.ranges.assign(360,2.0);if(near_obstacle){s.ranges[180]=.1;s.ranges[181]=.1;}if(feed_scan){scan->publish(s);filtered->publish(s);}
    nav_msgs::msg::Odometry o;o.header.stamp=now;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.twist.twist.linear.y=sideways_speed;o.pose.pose.orientation.w=1;o.pose.pose.position.x=odom_x;o.pose.pose.position.y=odom_y;if(publish_local_odometry&&now.seconds()>=odom_feed_after)odom->publish(o);
    if(base_diagnostics){diagnostic_msgs::msg::DiagnosticArray d;d.header.stamp=now;
      diagnostic_msgs::msg::DiagnosticStatus status;status.name="carcar/rosmaster";status.level=0;
      diagnostic_msgs::msg::KeyValue value;value.key="wheel_odometry_status";value.value="encoder_odometry_active";status.values.push_back(value);
      d.status.push_back(status);base_diagnostics->publish(d);}
    nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.header.stamp=now;m.info.resolution=.05;m.info.width=grid_width;m.info.height=100;m.info.origin.position.x=grid_origin;m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1;m.data.assign(grid_width*100,0);if(unknown_start)m.data[50*grid_width+53]=-1;if(block_route)m.data[50*grid_width+58]=100;
    if(origin_obstacle)m.data[50*grid_width+50]=100;
    if(fill_return_neighborhood)for(int y=0;y<100;++y)for(int x=0;x<grid_width;++x)
      if(std::hypot(grid_origin+(x+.5)*.05,-2.5+(y+.5)*.05)<.95)m.data[y*grid_width+x]=100;
    if(block_distant)m.data[80*grid_width+80]=100;
    if(observation_wall)for(int y=0;y<100;++y){m.data[y*grid_width+63]=100;for(int x=64;x<grid_width;++x)m.data[y*grid_width+x]=-1;}
    // 激光可穿过0.35米门缝，0.22米圆包络及规划余量不能通过。
    if(narrow_door)for(int y=0;y<100;++y)if(y<47||y>53)m.data[y*grid_width+63]=100;
    if(closed_outer_walls){for(unsigned x=0;x<m.info.width;++x){m.data[x]=100;m.data[(m.info.height-1)*m.info.width+x]=100;}
      for(unsigned y=0;y<m.info.height;++y){m.data[y*m.info.width]=100;m.data[y*m.info.width+m.info.width-1]=100;}}
    if(feed_map)map->publish(m);
    nav_msgs::msg::OccupancyGrid navigation=m;
    if(live_scene){
      if(feed_map)live_scene->map(std::make_shared<nav_msgs::msg::OccupancyGrid>(m));
      if(live_scene->output())navigation=live_scene->output()->navigation;
      if(live_map&&live_scene->ready())navigation_map_pub->publish(navigation);
      std_msgs::msg::String message;if(live_state){message.data=live_scene->state().dump();wall_state_pub->publish(message);}
      tuning_state["task_id"]=task_id;message.data=tuning_state.dump();tuning_state_pub->publish(message);
    }
    if(mask_pub&&(!live_mode||live_mask)){
      const auto&m=navigation;
      mask_pub->publish(m);std_msgs::msg::String status;status.data=Json{{"source_fingerprint",std::to_string(carcar_mapping_autonomy::map_content_fingerprint(m))},{"navigation_generation",navigation_generation}}.dump();mask_status_pub->publish(status);
      nav2_msgs::msg::Costmap raw;raw.header=m.header;raw.metadata.size_x=m.info.width;raw.metadata.size_y=m.info.height;
      raw.metadata.layer="master";raw.metadata.resolution=m.info.resolution;raw.metadata.origin=m.info.origin;raw.metadata.update_time=now;
      raw.data.assign(m.data.size(),0);global_raw_pub->publish(raw);raw.header.frame_id="odom";local_raw_pub->publish(raw);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped p;p.header.frame_id="map";p.header.stamp=now;p.pose.pose.orientation.w=1;p.pose.pose.position.x=pose_x;p.pose.pose.position.y=pose_y;if(feed_pose)pose->publish(p);
    if(!freeze_nav_pose)nav_pose_stamp=now;
    geometry_msgs::msg::PolygonStamped fp;fp.header.stamp=nav_pose_stamp;fp.polygon.points.resize(4);fp.header.frame_id="odom";local_pose->publish(fp);fp.header.frame_id="map";global_pose->publish(fp);
    std_msgs::msg::String j;j.data=Json{{"admission_enabled",force_slam_admission_open||!recovery_state.value("gate_write",false)},{"sequence",slam_sequence},{"accepted",true},{"accepted_stamp_ns",now.nanoseconds()},{"startup_closed",slam_startup_closed},{"loop_generation",loop_generation},{"loop_active",loop_active},{"loop_guard",{{"rejections",loop_rejections},{"rolled_back",loop_rejections>0}}}}.dump();slam->publish(j);j.data=session_message().dump();if(publish_session)ready->publish(j);
    if(recovery_pub&&!task_id.empty()){recovery_state["task_id"]=task_id;j.data=recovery_state.dump();recovery_pub->publish(j);}
    geometry_msgs::msg::Twist v;v.linear.x=.06;nav_velocity->publish(v);
    if(active&&active->is_canceling()&&ack_cancel){active->canceled(std::make_shared<Nav::Result>());active.reset();}
    else if(active&&((succeed_return&&std::abs(active->get_goal()->pose.pose.position.x)<.01)||succeed_any_return)){
      if(!fake_return_success){pose_x=odom_x=active->get_goal()->pose.pose.position.x;pose_y=odom_y=active->get_goal()->pose.pose.position.y;}
      active->succeed(std::make_shared<Nav::Result>());active.reset();}
  }
};

TEST_F(LiveSupervisorIntegration, FailedParametersBlockRestrictedManualAndExplainReason){
  start_goal();
  tuning_state.update({{"phase","FAILED"},{"ready",false},{"pending",false},{"requires_stop",true},
    {"restore_available",true},{"error","现场回退读回超时"}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
  request("manual","failed-parameter-manual",{{"restricted",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL";},3))<<state.dump();
  until([]{return false;},.5);
  EXPECT_FALSE(state["restricted_manual"].value("ready",true));
  EXPECT_NE(state["restricted_manual"].value("reason","").find("参数安全核验"),std::string::npos);
  EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, HealthyManualIgnoresVirtualWallAndKeepsMappingOpen){
  install_wall(.26);
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=.08;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.06;},2))<<state.dump();
  EXPECT_EQ(state.value("phase",""),"MANUAL");EXPECT_FALSE(state["restricted_manual"].value("active",true));
  EXPECT_FALSE(recovery_state.value("gate_write",true));
  EXPECT_EQ(state.value("manual_policy",Json::object()).value("version",0),1);
  heartbeat.reset();ASSERT_TRUE(until([this]{return last_velocity==0;},1));
}

TEST_F(LiveSupervisorIntegration, StrongerLocalRequestDuringNormalHandoffIsRejectedWithoutRestartingCancellation){
  start_goal();ack_cancel=false;request("manual","normal-handoff");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="STOPPING"&&state.value("stop_target","")=="MANUAL"&&last_velocity==0;},2));
  const auto generation=state.value("generation",uint64_t{});
  request("manual","stronger-local-handoff",{{"restricted",true}});
  ASSERT_TRUE(until([this]{return result("stronger-local-handoff","rejected");},1))<<state.dump();
  EXPECT_EQ(state.value("generation",uint64_t{}),generation);EXPECT_EQ(nav_requests,1U);
  EXPECT_FALSE(state["restricted_manual"].value("pending",true));EXPECT_FALSE(recovery_state.value("gate_write",true));
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));EXPECT_DOUBLE_EQ(last_velocity,0);
  ack_cancel=true;ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL";},2));
}

TEST_F(LiveSupervisorIntegration, SingleManualRequestUsesLocalSafetyWhenGeometryAndWallMaskAreUnavailable){
  install_wall(.26);recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true},{"reason","几何匹配不足"}});
  live_mask=live_map=false;session_ready=false;
  ASSERT_TRUE(until([this]{return !state.value("healthy",true);},1));
  EXPECT_TRUE(state["actions"]["manual"].value("enabled",false))<<state.dump();
  request("manual","single-local-manual");
  ASSERT_TRUE(until([this]{return result("single-local-manual","accepted")&&state["restricted_manual"].value("ready",false);},3))<<state.dump();
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=.12;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.01;},2))<<state.dump();EXPECT_LE(last_velocity,.05);
  EXPECT_TRUE(recovery_state.value("gate_write",false));EXPECT_FALSE(state.value("healthy",true));
  request("start","local-cannot-auto");ASSERT_TRUE(until([this]{return result("local-cannot-auto","rejected");},1));
  EXPECT_EQ(nav_requests,0U);heartbeat.reset();ASSERT_TRUE(until([this]{return last_velocity==0;},1));
}

TEST_F(LiveSupervisorIntegration, ExplicitParameterRestoreKeepsAutomaticTaskPaused){
  start_goal();
  tuning_state.update({{"phase","FAILED"},{"ready",false},{"pending",false},{"requires_stop",true},
    {"restore_available",true},{"error","现场回退读回超时"}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED"&&state.value("still",false)&&state.value("navigation_terminal",false);},3));
  Json forwarded;
  auto listener=n->create_subscription<std_msgs::msg::String>("/mapping/tuning/request",10,
    [&forwarded](const std_msgs::msg::String&m){forwarded=Json::parse(m.data);});
  until([]{return false;},.2);request("restore_tuning","restore-failed-parameters");
  ASSERT_TRUE(until([&]{return !forwarded.empty();},2))<<state.dump();
  EXPECT_EQ(forwarded.value("action",""),"restore");EXPECT_EQ(forwarded.value("task_id",""),task_id);
  tuning_state.update({{"phase","READY"},{"ready",true},{"pending",false},{"requires_stop",false}});
  until([]{return false;},.7);
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(nav_requests,1U);EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveAutomaticStartupIntegration, DelayedTuningAndSceneKeepBootRequestUntilBothAreReady){
  until([]{return false;},.3);
  ASSERT_TRUE(state.value("auto_start_pending",false))<<state.dump();
  EXPECT_EQ(state.value("phase",""),"MANUAL");EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_NE(state.value("automatic_start_reason","").find("动态参数"),std::string::npos);
  // 参数即将就绪时再制造真实跨话题地图同步等待，验证两道门槛均不会消费请求。
  live_state=live_mask=false;++grid_width;grid_origin-=.05;
  ASSERT_TRUE(until([this]{return !state["walls"].value("navigation_ready",true);},1));
  tuning_ready();
  ASSERT_TRUE(until([this]{return state.value("automatic_start_reason","").find("人工墙")!=std::string::npos;},1))<<state.dump();
  EXPECT_TRUE(state.value("auto_start_pending",false));EXPECT_EQ(state.value("phase",""),"MANUAL");
  EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nonzero,0U);EXPECT_EQ(nav_requests,0U);
  live_state=live_mask=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},3))<<state.dump();
  EXPECT_FALSE(state.value("auto_start_pending",true));EXPECT_TRUE(state["walls"].value("navigation_ready",false));
  EXPECT_TRUE(state.value("healthy",false));EXPECT_FALSE(state["fault"].value("active",false));
}

TEST_F(LiveAutomaticStartupIntegration, HaltDuringStartupWaitCannotBeOverriddenByReady){
  cancel_waiting_start("halt","PAUSED");
}

TEST_F(LiveAutomaticStartupIntegration, PauseDuringStartupWaitCannotBeOverriddenByReady){
  EXPECT_TRUE(state["actions"]["pause"].value("enabled",false));
  cancel_waiting_start("pause","PAUSED");
}

TEST_F(LiveAutomaticStartupIntegration, ManualDuringStartupWaitCancelsBootWithoutBypassingReadiness){
  EXPECT_TRUE(state["actions"]["manual"].value("enabled",false));
  cancel_waiting_start("manual","MANUAL");
}

TEST_F(LiveSupervisorIntegration, NormalMapReorderingHoldsOutputWithoutFaultAndRecovers){
  start_goal();const auto goal=active;
  live_state=false;live_mask=false;++grid_width;grid_origin-=.05;
  ASSERT_TRUE(until([this]{return !state["walls"].value("navigation_ready",true)&&last_velocity==0;},1));
  until([]{return false;},.45);EXPECT_EQ(state["phase"],"EXPLORING");EXPECT_FALSE(state["fault"].value("active",false));
  live_state=live_mask=true;ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false)&&last_velocity>0;},2));
  EXPECT_EQ(active,goal);EXPECT_EQ(nav_requests,1U);
}

TEST_F(LiveSupervisorIntegration, WallAckCannotResumeBeforeDerivedMapAndMaskMatch){
  start_goal();live_map=live_mask=false;wall_request("set_walls","replace-wait-map",walls());
  ASSERT_TRUE(until([this]{return live_scene->state().value("revision",0)==1;},3));
  until([]{return false;},.4);EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_FALSE(state["walls"].value("navigation_ready",true));EXPECT_TRUE(state["walls"].value("pending",false));
  live_map=live_mask=true;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING"&&state["walls"].value("navigation_ready",false);},3))<<state.dump();
  EXPECT_TRUE(result("replace-wait-map","accepted"));
}

TEST_F(LiveSupervisorIntegration, HumanStopCancelsWallAutomaticResumeAndRejectedUpdatePreservesOldWalls){
  install_wall();start_goal();live_requests=false;
  wall_request("set_walls","stopped-change",walls(1.6));
  ASSERT_TRUE(until([this]{return !wall_commands.empty()&&wall_commands.back().value("request_id","")=="stopped-change";},3));
  request("halt","human-stop");ASSERT_TRUE(until([this]{return result("human-stop","accepted");},2));
  live_scene->request(wall_commands.back());live_requests=true;
  ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false)&&!state["walls"].value("pending",false);},3));
  EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
  live_reject=true;const auto old=live_scene->state()["walls"];wall_request("set_walls","rejected-change",walls(1.8));
  ASSERT_TRUE(until([this]{return result("rejected-change","rejected");},3))<<state.dump();
  EXPECT_EQ(live_scene->state()["walls"],old);EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, ParameterRollbackPausesWithoutLocalizationFaultAndHumanStopWins){
  start_goal();tuning_state.update({{"ready",false},{"requires_stop",true},{"pending",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));EXPECT_FALSE(state["fault"].value("active",false));
  request("halt","stop-during-rollback");ASSERT_TRUE(until([this]{return result("stop-during-rollback","accepted");},2));
  tuning_state.update({{"ready",true},{"requires_stop",false},{"pending",false}});
  until([]{return false;},.7);EXPECT_EQ(state["phase"],"PAUSED");EXPECT_DOUBLE_EQ(last_velocity,0);
  request("start","after-rollback");ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));
  tuning_state.update({{"ready",false},{"requires_stop",true},{"pending",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED";},3));
  tuning_state.update({{"ready",true},{"requires_stop",false},{"pending",false}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},2));EXPECT_FALSE(state["fault"].value("active",false));
}

TEST_F(LiveSupervisorIntegration, ObsoleteWallBypassCannotChangeManualOrAutomaticPolicy){
  install_wall(.26);request("set_wall_bypass","obsolete-bypass",{{"enabled",true},{"bypass_owner_id",manual_owner_id}});
  ASSERT_TRUE(until([this]{return result("obsolete-bypass","rejected");},2));
  EXPECT_FALSE(state["walls"].value("bypass_enabled",true));EXPECT_FALSE(recovery_state.value("gate_write",true));
  live_mask=live_map=false;++grid_width;grid_origin-=.05;
  ASSERT_TRUE(until([this]{return !state["walls"].value("navigation_ready",true);},1));
  request("start","walls-still-block-auto");ASSERT_TRUE(until([this]{return result("walls-still-block-auto","rejected");},1));
  EXPECT_EQ(nav_requests,0U);EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, RecoveryUsesRawMapThenWallReviewThenNavigationCommit){
  install_wall();start_goal();const auto origin=state["origin"];
  recovery_state.update({{"phase","DISTURBED"},{"requires_stop",true},{"gate_write",true}});
  ASSERT_TRUE(until([this]{return state.value("phase","")=="RECOVERING";},3))<<state.dump();
  ASSERT_TRUE(until([this]{return recovery_state["phase"]=="WAITING_STABLE";},2));
  recovery_state.update({{"phase","VERIFY_READY"},{"commit_ready",true}});
  ASSERT_TRUE(until([this]{return state["walls"].value("review_allowed",false);},2))<<state.dump();
  EXPECT_FALSE(state["walls"].value("navigation_ready",true));EXPECT_DOUBLE_EQ(last_velocity,0);
  until([]{return false;},.5);EXPECT_EQ(state["phase"],"RECOVERING");
  EXPECT_EQ(std::find(recovery_actions.begin(),recovery_actions.end(),"commit_verified"),recovery_actions.end());
  wall_request("confirm_walls","review-after-restore");
  ASSERT_TRUE(until([this]{return state["walls"].value("navigation_ready",false);},3))<<state.dump();
  const auto reviewed_generation=navigation_generation;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="EXPLORING";},6))<<state.dump();
  EXPECT_EQ(navigation_generation,reviewed_generation);EXPECT_EQ(state["origin"],origin);
  EXPECT_EQ(live_scene->state()["phase"],"READY");EXPECT_EQ(nav_requests,1U);
  EXPECT_NE(std::find(recovery_actions.begin(),recovery_actions.end(),"commit_verified"),recovery_actions.end());
}

TEST_F(LiveSupervisorIntegration, SuccessfulLoopWallReviewCannotEscalateIntoLocalizationRecovery){
  install_wall();start_goal();++loop_generation;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED"&&
    state["walls"].value("review_allowed",false)&&state.value("still",false)&&state.value("navigation_terminal",false);},4))<<state.dump();
  EXPECT_TRUE(state.value("healthy",false));EXPECT_EQ(state.value("quality_fault",""),"");
  EXPECT_NE(state.value("reason","").find("位置已核对，复核墙"),std::string::npos);
  EXPECT_FALSE(state["actions"]["recheck"].value("enabled",true));
  EXPECT_NE(state["actions"]["recheck"].value("reason","").find("仅需复核人工墙位置"),std::string::npos);
  const auto recovery_count=recovery_actions.size(),navigation_count=nav_requests;
  request("recheck","wall-review-is-not-relocalization");
  ASSERT_TRUE(until([this]{return result("wall-review-is-not-relocalization","rejected");},2))<<state.dump();
  EXPECT_EQ(recovery_actions.size(),recovery_count);EXPECT_EQ(nav_requests,navigation_count);EXPECT_DOUBLE_EQ(last_velocity,0);
  wall_request("confirm_walls","confirm-position-after-successful-loop");
  ASSERT_TRUE(until([this]{return result("confirm-position-after-successful-loop","accepted")&&
    state["walls"].value("navigation_ready",false)&&!state["walls"].value("pending",false);},3))<<state.dump();
  EXPECT_EQ(state.value("phase",""),"PAUSED");EXPECT_EQ(recovery_actions.size(),recovery_count);
  EXPECT_EQ(nav_requests,navigation_count);EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, HealthyManualWallReviewExplainsExistingWallConfirmationInsteadOfRelocalization){
  install_wall();++loop_generation;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL"&&state["walls"].value("review_allowed",false);},3))<<state.dump();
  EXPECT_FALSE(state["actions"]["recheck"].value("enabled",true));
  EXPECT_NE(state["actions"]["recheck"].value("reason","").find("仅需复核人工墙位置"),std::string::npos);
  EXPECT_NE(state["actions"]["recheck"].value("reason","").find("无需重新定位"),std::string::npos);
  const auto recovery_count=recovery_actions.size();request("recheck","manual-wall-review-is-not-relocalization");
  ASSERT_TRUE(until([this]{return result("manual-wall-review-is-not-relocalization","rejected");},2));
  EXPECT_EQ(recovery_actions.size(),recovery_count);EXPECT_EQ(state.value("phase",""),"MANUAL");EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, ARealDisturbanceDuringWallReviewStillRequiresFullLocalizationRecovery){
  install_wall();start_goal();++loop_generation;
  ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED"&&state["walls"].value("review_allowed",false);},4))<<state.dump();
  // 同时发生的真实定位冻结不能被“仅需墙复核”的引导掩盖，仍走原恢复事务。
  recovery_state.update({{"phase","DISTURBED"},{"requires_stop",true},{"gate_write",true}});
  ASSERT_TRUE(until([this]{return !state.value("healthy",true)&&state["actions"]["recheck"].value("enabled",false);},2))<<state.dump();
  EXPECT_EQ(state["actions"]["recheck"].value("reason","").find("仅需复核人工墙位置"),std::string::npos);
  request("recheck","disturbed-wall-review-needs-relocalization");
  ASSERT_TRUE(until([this]{return result("disturbed-wall-review-needs-relocalization","accepted")&&
    state.value("phase","")=="RECOVERING"&&std::find(recovery_actions.begin(),recovery_actions.end(),"recover")!=recovery_actions.end();},3))<<state.dump();
  EXPECT_DOUBLE_EQ(last_velocity,0);
}

TEST_F(LiveSupervisorIntegration, LocalManualOwnerHandoffStopsAndNeedsNewRequestAndGesture){
  recovery_state.update({{"phase","FAILED"},{"gate_write",true},{"requires_stop",true}});session_ready=false;
  ASSERT_TRUE(until([this]{return !state.value("healthy",true);},1));
  request("manual","first-local-owner");ASSERT_TRUE(until([this]{return state["restricted_manual"].value("ready",false);},3))<<state.dump();
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);double desired=.05;
  rclcpp::TimerBase::SharedPtr heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=desired;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  manual_owner_id.clear();ASSERT_TRUE(until([this]{return state.value("phase","")=="PAUSED"&&last_velocity==0;},2));
  request("manual","replacement-local-owner");
  ASSERT_TRUE(until([this]{return state.value("phase","")=="MANUAL";},3))<<state.dump();
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));EXPECT_DOUBLE_EQ(last_velocity,0);
  desired=0;heartbeat.reset();manual_owner_id="replacement-owner";
  ASSERT_TRUE(until([this]{return state["manual_policy"].value("motion_allowed",false);},2));
  until([]{return false;},.4);EXPECT_DOUBLE_EQ(last_velocity,0);
  desired=.05;heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=desired;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.01;},2));heartbeat.reset();
}

TEST_F(LiveSupervisorIntegration, SupervisedEscapeTrialRequiresExplicitRequestAndKeepsValidationFalse){
 request("set_escape_enabled","trial-missing",{{"enabled",true}});
 ASSERT_TRUE(until([this]{return result("trial-missing","rejected");},1));
 request("set_escape_enabled","trial-accepted",{{"enabled",true},{"supervised_trial",true}});
 ASSERT_TRUE(until([this]{return result("trial-accepted","accepted")&&state["obstacle_escape"].value("enabled",false);},2))<<state.dump();
 EXPECT_FALSE(state["obstacle_escape"].value("field_validated",true));
 EXPECT_TRUE(state["obstacle_escape"].value("trial_authorized",false));
 EXPECT_EQ(last_velocity,0);EXPECT_EQ(nav_requests,0U);
 request("set_escape_enabled","trial-close",{{"enabled",false}});
 ASSERT_TRUE(until([this]{return !state["obstacle_escape"].value("enabled",true);},1));
}

TEST_F(LocalManualSupervisorIntegration, NoGlobalLocalizationOrWallReviewCannotBlockLocalManualButLocalFailuresStop){
  install_wall(.26);fail_global();
  request("manual","no-global-local-manual");ASSERT_TRUE(until([this]{return state["restricted_manual"].value("ready",false);},3))<<state.dump();
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);double desired=.12;
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=desired;v.angular.z=desired>0?.25:0;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.01&&last_yaw>.01;},2))<<state.dump();
  EXPECT_LE(last_velocity,.05);EXPECT_LE(last_yaw,.10);EXPECT_FALSE(state.value("healthy",true));
  const auto policy=state.value("manual_policy",Json::object());EXPECT_EQ(policy.value("mode",""),"local_safety");
  EXPECT_TRUE(policy.value("ignores_virtual_walls",false));EXPECT_FALSE(policy.value("requires_global_localization",true));
  request("return","bad-global-return");ASSERT_TRUE(until([this]{return result("bad-global-return","rejected");},1));EXPECT_EQ(nav_requests,0U);
  near_obstacle=true;ASSERT_TRUE(until([this]{return last_velocity==0&&state["manual_policy"].value("direction_blocked",false);},1))<<state.dump();
  EXPECT_TRUE(state["manual_policy"].value("motion_allowed",false));
  // 已由原始扫掠阻断的零候选不应因为近场模块停止重复零而撤销手动资格。
  collision_feedback=false;until([]{return false;},.4);
  EXPECT_TRUE(state["manual_policy"].value("motion_allowed",false));
  collision_feedback=true;
  near_obstacle=false;ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  feed_scan=false;ASSERT_TRUE(until([this]{return last_velocity==0&&!state["manual_policy"].value("motion_allowed",true);},1));
  feed_scan=true;ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  publish_local_odometry=false;ASSERT_TRUE(until([this]{return last_velocity==0&&!state["manual_policy"].value("motion_allowed",true);},1));
  publish_local_odometry=true;ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  publish_odom_transform=false;ASSERT_TRUE(until([this]{return last_velocity==0&&!state["manual_policy"].value("motion_allowed",true);},1));
  publish_odom_transform=true;ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  collision_feedback=false;ASSERT_TRUE(until([this]{return last_velocity==0&&!state["manual_policy"].value("motion_allowed",true);},1));
  collision_feedback=true;until([]{return false;},.4);EXPECT_DOUBLE_EQ(last_velocity,0);
  desired=0;ASSERT_TRUE(until([this]{return state["manual_policy"].value("motion_allowed",false);},1));
  desired=.12;ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  desired=0;heartbeat.reset();ASSERT_TRUE(until([this]{return last_velocity==0&&last_yaw==0;},1));
}

TEST_F(LocalManualSupervisorIntegration, ManualCandidateWaitsForFreshCollisionFeedbackAndExpiresAfterRelease){
  double candidate_x=0,candidate_yaw=0;
  auto candidates=n->create_subscription<geometry_msgs::msg::Twist>("/mapping/collision_input",1,
    [&](const geometry_msgs::msg::Twist&m){candidate_x=m.linear.x;candidate_yaw=m.angular.z;});
  collision_feedback=false;until([]{return false;},.4);
  ASSERT_TRUE(state["manual_policy"].value("motion_allowed",false))<<state.dump();
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);double desired=.07;
  rclcpp::TimerBase::SharedPtr heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=desired;manual->publish(v);});
  ASSERT_TRUE(until([&]{return candidate_x>.05&&state["manual_policy"].value("direction_blocked",false);},2))<<state.dump();
  EXPECT_NE(state["manual_policy"].value("direction_reason","").find("等待近场保护"),std::string::npos);
  EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_DOUBLE_EQ(last_yaw,0);
  const auto stopped_outputs=nonzero;
  ASSERT_TRUE(until([&]{return candidate_x==0&&!state["manual_policy"].value("motion_allowed",true);},1))<<state.dump();
  EXPECT_NE(state["manual_policy"].value("blocked_reason","").find("松手"),std::string::npos);
  EXPECT_EQ(nonzero,stopped_outputs);
  // 首次送审超时也锁存；反馈恢复不能复活持续按住的旧手势。
  collision_feedback=true;until([]{return false;},.4);
  EXPECT_EQ(nonzero,stopped_outputs);EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));
  // 原输入期限失效后可重新提交新手势，旧非零反馈不能绕过当前零候选。
  heartbeat.reset();
  ASSERT_TRUE(until([&]{return candidate_x==0&&candidate_yaw==0&&last_velocity==0&&state["manual_policy"].value("motion_allowed",false);},1));
  EXPECT_EQ(nonzero,stopped_outputs);EXPECT_DOUBLE_EQ(last_velocity,0);
  heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=desired;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.05;},2))<<state.dump();
  collision_feedback=false;
  ASSERT_TRUE(until([&]{return candidate_x==0&&last_velocity==0&&!state["manual_policy"].value("motion_allowed",true);},1))<<state.dump();
  const auto interrupted_outputs=nonzero;
  collision_feedback=true;until([]{return false;},.4);
  EXPECT_EQ(nonzero,interrupted_outputs);EXPECT_DOUBLE_EQ(last_velocity,0);
  EXPECT_FALSE(state["manual_policy"].value("motion_allowed",true));
  desired=0;ASSERT_TRUE(until([&]{return candidate_x==0&&state["manual_policy"].value("motion_allowed",false);},1));
  desired=.07;ASSERT_TRUE(until([this]{return last_velocity>.05;},2))<<state.dump();
  heartbeat.reset();ASSERT_TRUE(until([&]{return candidate_x==0&&last_velocity==0;},1));
}

TEST_F(LocalManualSupervisorIntegration, ParameterRestoreWithoutWallMaskReleasesOnlyLocalManualAndDisconnectStops){
  install_wall(.26);fail_global();
  tuning_state.update({{"ready",false},{"requires_stop",true},{"pending",false}});
  request("manual","local-during-parameter-failure");
  ASSERT_TRUE(until([this]{return result("local-during-parameter-failure","accepted")&&state.value("phase","")=="MANUAL"&&
    !state["restricted_manual"].value("ready",true);},3))<<state.dump();
  tuning_state.update({{"ready",true},{"requires_stop",false},{"pending",false}});
  ASSERT_TRUE(until([this]{return state["restricted_manual"].value("ready",false);},2))<<state.dump();
  auto manual=n->create_publisher<geometry_msgs::msg::Twist>("/mapping/manual_cmd_vel",1);
  auto heartbeat=n->create_wall_timer(std::chrono::milliseconds(50),[&]{geometry_msgs::msg::Twist v;v.linear.x=.05;manual->publish(v);});
  ASSERT_TRUE(until([this]{return last_velocity>.01;},2));
  publish_session=false;ASSERT_TRUE(until([this]{return last_velocity==0&&state.value("phase","")=="PAUSED";},2));
  publish_session=true;until([]{return false;},.6);EXPECT_DOUBLE_EQ(last_velocity,0);EXPECT_EQ(nav_requests,0U);heartbeat.reset();
}
