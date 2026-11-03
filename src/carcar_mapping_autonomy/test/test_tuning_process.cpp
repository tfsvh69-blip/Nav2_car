#include <gtest/gtest.h>
#include <limits>
#include <thread>
#define CARCAR_TUNING_NO_MAIN
#include "../src/tuning.cpp"
namespace ca=carcar_mapping_autonomy;
namespace {
using Json=nlohmann::json;
class TuningProcess:public testing::Test {
protected:
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{
    fixture=std::make_shared<rclcpp::Node>("tuning_fixture");
    ack=fixture->create_publisher<std_msgs::msg::String>("/mapping/tuning/inflation_ack",rclcpp::QoS(20).reliable().transient_local());
    autonomy=fixture->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
    requests=fixture->create_publisher<std_msgs::msg::String>("/mapping/tuning/request",rclcpp::QoS(10));
    endpoints[0]=std::make_shared<rclcpp::Node>("global_costmap","/global_costmap");
    endpoints[1]=std::make_shared<rclcpp::Node>("local_costmap","/local_costmap");endpoints[2]=std::make_shared<rclcpp::Node>("controller_server");
    for(size_t i=0;i<3;++i){
      if(i<2){endpoints[i]->declare_parameter("inflation_layer.inflation_radius",i==0?.5:.3);
        endpoints[i]->declare_parameter("inflation_layer.cost_scaling_factor",i==0?3.:5.);endpoints[i]->declare_parameter<int64_t>("inflation_layer.tuning_revision",0);
      }else{endpoints[i]->declare_parameter("FollowPath.inflation_cost_scaling_factor",5.);endpoints[i]->declare_parameter("FollowPath.cost_scaling_dist",.3);endpoints[i]->declare_parameter<int64_t>("FollowPath.tuning_revision",0);}
      handlers[i]=endpoints[i]->add_on_set_parameters_callback([this,i](const std::vector<rclcpp::Parameter>&p){
        rcl_interfaces::msg::SetParametersResult r;r.successful=true;int64_t rev=-1;
        for(const auto&x:p){if(x.get_name()==(i<2?"inflation_layer.tuning_revision":"FollowPath.tuning_revision"))rev=x.as_int();
          if(i==1&&fail_local_any_once&&x.get_name()=="inflation_layer.cost_scaling_factor"){fail_local_any_once=false;r.successful=false;r.reason="注入一次局部写入拒绝";}
          if(i==1&&fail_local_once&&x.get_name()=="inflation_layer.cost_scaling_factor"&&x.as_double()==4.){fail_local_once=false;r.successful=false;r.reason="注入单次局部拒绝";}}
        if(rev<=last_revision[i]){r.successful=false;r.reason="旧代次";}if(r.successful)last_revision[i]=rev;
        writes.push_back({{"node",i},{"revision",rev},{"accepted",r.successful}});return r;
      });executor.add_node(endpoints[i]);
      lifecycle[i]=endpoints[i]->create_service<ca::MappingTuningNode::Life>(std::string(endpoints[i]->get_fully_qualified_name())+"/get_state",
        [](ca::MappingTuningNode::Life::Request::SharedPtr,ca::MappingTuningNode::Life::Response::SharedPtr response){response->current_state.id=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;});
    }
    rclcpp::NodeOptions options;options.parameter_overrides({{"task_id",std::string("task-test")},{"defaults_file",std::string("/tmp/carcar-tuning-test-absent-defaults.json")},{"service_timeout_seconds",.35}});
    tuning=std::make_shared<ca::MappingTuningNode>(options);executor.add_node(tuning);executor.add_node(fixture);
    ASSERT_TRUE(until([this]{return tuning->state().at("ready").get<bool>();},6))<<tuning->state().dump();writes.clear();
  }
  void TearDown()override{executor.remove_node(tuning);executor.remove_node(fixture);for(const auto&e:endpoints)executor.remove_node(e);
    tuning.reset();for(auto&s:lifecycle)s.reset();for(auto&h:handlers)h.reset();for(auto&e:endpoints)e.reset();fixture.reset();}
  void spin(){
    std_msgs::msg::String a;a.data=Json{{"task_id","task-test"},{"generation",100},{"navigation_generation",generation},
      {"phase",stopped?"PAUSED":"EXPLORING"},{"tuning",Json{{"request_id",tuning->state().at("request_id")},{"requires_stop",acknowledge_stop&&tuning->state().at("requires_stop").get<bool>()}}},
      {"still",stopped},{"navigation_terminal",stopped},{"output",stopped?Json::array({0.,0.}):Json::array({.1,0.})}}.dump();autonomy->publish(a);
    for(size_t i=0;i<2;++i)if(!hold_ack[i]){auto&e=endpoints[i];const auto revision=e->get_parameter("inflation_layer.tuning_revision").as_int();
      if(revision==sent_revision[i])continue;
      sent_revision[i]=revision;std_msgs::msg::String m;
      m.data=Json{{"node",e->get_fully_qualified_name()},{"layer","inflation_layer"},{"revision",e->get_parameter("inflation_layer.tuning_revision").as_int()},
        {"radius",e->get_parameter("inflation_layer.inflation_radius").as_double()},{"factor",e->get_parameter("inflation_layer.cost_scaling_factor").as_double()},
        {"completed",true},{"full_rebuild",true}}.dump();ack->publish(m);}
    executor.spin_some();if(tuning->state().at("phase")=="APPLYING"&&tuning->state().at("request_id")!=""){
      if(tuning->state().at("request_id").get<std::string>().rfind("startup-",0)!=0)applying_ready=applying_ready&&tuning->state().at("ready").get<bool>();}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  template<class Predicate>bool until(Predicate predicate,double limit){const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(limit);
    while(std::chrono::steady_clock::now()<end){spin();if(predicate())return true;}return predicate();}
  void apply(const ca::TuningValues&v,const std::string&id="apply-1"){
    apply_values(v.json(),id);
  }
  void apply_values(const Json&values,const std::string&id){
    std_msgs::msg::String m;m.data=Json{{"task_id","task-test"},{"request_id",id},{"generation",generation},{"action","apply"},{"values",values}}.dump();requests->publish(m);
  }
  void restart_startup(){
    executor.remove_node(tuning);tuning.reset();rclcpp::NodeOptions options;
    options.parameter_overrides({{"task_id",std::string("task-test")},{"defaults_file",std::string("/tmp/carcar-tuning-test-absent-defaults.json")},{"service_timeout_seconds",.35}});
    tuning=std::make_shared<ca::MappingTuningNode>(options);executor.add_node(tuning);writes.clear();
  }
  rclcpp::executors::SingleThreadedExecutor executor;rclcpp::Node::SharedPtr fixture;std::array<rclcpp::Node::SharedPtr,3>endpoints;
  std::array<rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr,3>handlers;
  std::array<rclcpp::Service<ca::MappingTuningNode::Life>::SharedPtr,3>lifecycle;
  std::shared_ptr<ca::MappingTuningNode>tuning;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack,autonomy,requests;
  std::array<int64_t,3>last_revision{{0,0,0}};std::array<bool,2>hold_ack{{false,false}};std::vector<Json>writes;
  std::array<int64_t,2>sent_revision{{-1,-1}};
  bool stopped=false,fail_local_once=false,fail_local_any_once=false,applying_ready=true,acknowledge_stop=true;uint64_t generation=1;
};
TEST_F(TuningProcess,ApplyWhileMovingKeepsReadyAndCoordinatesRppAfterExactMapAck){
  ca::TuningValues target;target.global_radius=.65;target.local_radius=.22;target.local_factor=3.;apply(target);
  ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="apply-1"&&tuning->state().at("phase")=="READY";},5));
  EXPECT_TRUE(applying_ready);EXPECT_FALSE(tuning->state().at("requires_stop").get<bool>());EXPECT_FALSE(stopped);
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.inflation_cost_scaling_factor").as_double(),3.);
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.cost_scaling_dist").as_double(),.22);
  EXPECT_EQ(tuning->state().at("confirmed"),target.json());ASSERT_GE(writes.size(),4U);
  EXPECT_EQ(writes.front().at("node"),2);EXPECT_EQ(writes.back().at("node"),2);
}
TEST_F(TuningProcess,SingleFieldRequestKeepsOtherConfirmedValues){
  apply_values(Json{{"global_inflation_radius",.7}},"single-radius");
  ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="single-radius"&&tuning->state().at("phase")=="READY";},3))<<tuning->state().dump();
  const Json expected{{"global_inflation_radius",.7},{"global_cost_scaling_factor",3.},
    {"local_inflation_radius",.3},{"local_cost_scaling_factor",5.}};
  EXPECT_EQ(tuning->state().at("confirmed"),expected);EXPECT_EQ(tuning->state().at("requested"),expected);
  EXPECT_DOUBLE_EQ(endpoints[0]->get_parameter("inflation_layer.inflation_radius").as_double(),.7);
  EXPECT_DOUBLE_EQ(endpoints[0]->get_parameter("inflation_layer.cost_scaling_factor").as_double(),3.);
  EXPECT_DOUBLE_EQ(endpoints[1]->get_parameter("inflation_layer.inflation_radius").as_double(),.3);
  EXPECT_DOUBLE_EQ(endpoints[1]->get_parameter("inflation_layer.cost_scaling_factor").as_double(),5.);
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.inflation_cost_scaling_factor").as_double(),5.);
  EXPECT_TRUE(applying_ready);EXPECT_FALSE(stopped);
}
TEST_F(TuningProcess,SequentialPartialRequestsKeepThePrecedingConfirmedChange){
  apply_values(Json{{"global_inflation_radius",.7}},"first-field");
  ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="first-field"&&tuning->state().at("phase")=="READY";},3));
  // 第二次请求不带第一项；车端必须从已确认的新快照补齐，不能回到启动值或依赖网页补齐。
  apply_values(Json{{"local_cost_scaling_factor",6.}},"second-field");
  ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="second-field"&&tuning->state().at("phase")=="READY";},3))<<tuning->state().dump();
  const Json expected{{"global_inflation_radius",.7},{"global_cost_scaling_factor",3.},
    {"local_inflation_radius",.3},{"local_cost_scaling_factor",6.}};
  EXPECT_EQ(tuning->state().at("confirmed"),expected);EXPECT_EQ(tuning->state().at("requested"),expected);
  EXPECT_DOUBLE_EQ(endpoints[0]->get_parameter("inflation_layer.inflation_radius").as_double(),.7);
  EXPECT_DOUBLE_EQ(endpoints[1]->get_parameter("inflation_layer.cost_scaling_factor").as_double(),6.);
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.inflation_cost_scaling_factor").as_double(),6.);
  EXPECT_EQ(tuning->state().at("baseline"),ca::TuningValues{}.json());
  EXPECT_TRUE(applying_ready);EXPECT_FALSE(stopped);
}
TEST_F(TuningProcess,EmptyUnknownAndNonFinitePartialRequestsDoNotWriteParameters){
  auto extra=ca::TuningValues{}.json();extra["unknown"]=1.;
  // JSON序列化会把NaN编码为null；两者都不能被当作有效参数。
  const std::vector<Json>invalid{Json::object(),Json{{"unknown",.7}},
    Json{{"local_cost_scaling_factor",std::numeric_limits<double>::quiet_NaN()}},
    Json{{"global_inflation_radius",true}},extra};
  size_t count=0;
  for(const auto&values:invalid){apply_values(values,"invalid-partial-"+std::to_string(count));++count;
    ASSERT_TRUE(until([this,count]{return tuning->state().at("rejected_requests").size()==count;},1));}
  std_msgs::msg::String malformed;
  malformed.data="{\"task_id\":\"task-test\",\"request_id\":\"literal-nan\",\"generation\":1,\"action\":\"apply\",\"values\":{\"local_cost_scaling_factor\":NaN}}";
  requests->publish(malformed);++count;
  ASSERT_TRUE(until([this,count]{return tuning->state().at("rejected_requests").size()==count;},1));
  EXPECT_TRUE(writes.empty());EXPECT_EQ(tuning->state().at("confirmed"),ca::TuningValues{}.json());
  EXPECT_EQ(tuning->state().at("phase"),"READY");EXPECT_FALSE(tuning->state().at("requires_stop").get<bool>());
}
TEST_F(TuningProcess,FailureWaitsForRealStopThenRollsBackBothMapsAndRpp){
  fail_local_once=true;ca::TuningValues target;target.global_radius=.65;target.local_factor=4.;apply(target);
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},5));
  EXPECT_TRUE(tuning->state().at("requires_stop").get<bool>());EXPECT_DOUBLE_EQ(endpoints[0]->get_parameter("inflation_layer.inflation_radius").as_double(),.65);
  until([]{return false;},.12);EXPECT_EQ(tuning->state().at("phase"),"WAITING_STOP");
  stopped=true;acknowledge_stop=false;until([]{return false;},.12);
  EXPECT_EQ(tuning->state().at("phase"),"WAITING_STOP")<<"仅有停稳而没有同请求停车回执，不能开始回退";
  acknowledge_stop=true;ASSERT_TRUE(until([this]{return tuning->state().at("rollback_ready").get<bool>();},5));
  EXPECT_EQ(tuning->state().at("confirmed"),ca::TuningValues{}.json());EXPECT_EQ(tuning->state().at("request_result"),"failed");
  EXPECT_DOUBLE_EQ(endpoints[0]->get_parameter("inflation_layer.inflation_radius").as_double(),.5);
}
TEST_F(TuningProcess,ParameterReadbackWithoutCompletedMapRevisionCannotSucceed){
  hold_ack[1]=true;ca::TuningValues target;target.local_radius=.4;apply(target);
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},5));
  EXPECT_FALSE(tuning->state().at("ready").get<bool>());hold_ack[1]=false;
  until([]{return false;},.12);EXPECT_EQ(tuning->state().at("phase"),"WAITING_STOP")<<"迟到成功回执不能重新完成失败事务";
  stopped=true;ASSERT_TRUE(until([this]{return tuning->state().at("rollback_ready").get<bool>();},5));
  EXPECT_EQ(tuning->state().at("confirmed"),ca::TuningValues{}.json());
}
TEST_F(TuningProcess,NavigationGenerationChangeCancelsOldTransaction){
  hold_ack[1]=true;ca::TuningValues target;target.local_radius=.4;apply(target);
  ASSERT_TRUE(until([this]{return endpoints[1]->get_parameter("inflation_layer.inflation_radius").as_double()==.4;},4));
  ++generation;ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},1));
  stopped=true;hold_ack[1]=false;ASSERT_TRUE(until([this]{return tuning->state().at("rollback_ready").get<bool>();},5));
}
TEST_F(TuningProcess,InvalidAndDuplicateRequestsDoNotCauseExtraWrites){
  ca::TuningValues invalid;invalid.local_radius=.1;apply(invalid,"invalid");
  ASSERT_TRUE(until([this]{return !tuning->state().at("rejected_requests").empty();},1));EXPECT_TRUE(writes.empty());
  ca::TuningValues target;target.local_factor=6.;apply(target);ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="apply-1"&&tuning->state().at("phase")=="READY";},5));
  const auto count=writes.size();apply(target);until([]{return false;},.15);EXPECT_EQ(writes.size(),count);
}
TEST_F(TuningProcess,RejectedStartupDefaultsRemainNotReadyEvenAfterVerifiedRollback){
  const auto file=std::filesystem::temp_directory_path()/("carcar-tuning-startup-failure-"+
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  ca::TuningValues target;target.global_radius=.65;target.local_factor=4.;
  {std::ofstream out(file);out<<Json{{"values",target.json()}};}
  executor.remove_node(tuning);tuning.reset();fail_local_once=true;
  rclcpp::NodeOptions options;options.parameter_overrides({{"task_id",std::string("task-test")},{"defaults_file",file.string()},{"service_timeout_seconds",.35}});
  tuning=std::make_shared<ca::MappingTuningNode>(options);std::filesystem::remove(file);executor.add_node(tuning);
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},5));
  stopped=true;ASSERT_TRUE(until([this]{return tuning->state().at("rolled_back").get<bool>();},5));
  EXPECT_EQ(tuning->state().at("phase"),"FAILED");EXPECT_FALSE(tuning->state().at("ready").get<bool>());
  EXPECT_TRUE(tuning->state().at("requires_stop").get<bool>());EXPECT_EQ(tuning->state().at("confirmed"),ca::TuningValues{}.json());
}
TEST_F(TuningProcess,StartupCanBecomeReadyAfterIndependentRollbackSatisfiesRequestedConfiguration){
  fail_local_any_once=true;restart_startup();
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},4));
  EXPECT_FALSE(tuning->state().at("ready").get<bool>());EXPECT_TRUE(tuning->state().at("requires_stop").get<bool>());
  stopped=true;acknowledge_stop=false;until([]{return false;},.12);
  EXPECT_EQ(tuning->state().at("phase"),"WAITING_STOP")<<"没有同请求可信停车回执，配置相同也不得放行";
  acknowledge_stop=true;ASSERT_TRUE(until([this]{return tuning->state().at("rolled_back").get<bool>();},4));
  EXPECT_EQ(tuning->state().at("phase"),"READY")<<tuning->state().dump();
  EXPECT_TRUE(tuning->state().at("ready").get<bool>());EXPECT_FALSE(tuning->state().at("requires_stop").get<bool>());
  EXPECT_EQ(tuning->state().at("request_result"),"failed");EXPECT_FALSE(tuning->state().at("error").get<std::string>().empty());
  EXPECT_EQ(tuning->state().at("confirmed"),ca::TuningValues{}.json());EXPECT_EQ(tuning->state().at("requested"),ca::TuningValues{}.json());
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.inflation_cost_scaling_factor").as_double(),5.);
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.cost_scaling_dist").as_double(),.3);
}
TEST_F(TuningProcess,LateStartupAckCannotUnlockBeforeIndependentStoppedRollback){
  hold_ack[1]=true;restart_startup();
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},4));
  const auto failed_revision=tuning->state().at("expected_revisions").at(1).get<int64_t>();
  hold_ack[1]=false;
  ASSERT_TRUE(until([this,failed_revision]{const auto a=tuning->state().at("inflation_ack").at(1);return a.is_object()&&a.at("revision")==failed_revision;},1));
  EXPECT_EQ(tuning->state().at("phase"),"WAITING_STOP");EXPECT_FALSE(tuning->state().at("ready").get<bool>());
  EXPECT_TRUE(tuning->state().at("requires_stop").get<bool>());EXPECT_FALSE(tuning->state().at("rolled_back").get<bool>());
  stopped=true;ASSERT_TRUE(until([this]{return tuning->state().at("rollback_ready").get<bool>();},4));
  EXPECT_EQ(tuning->state().at("phase"),"READY");EXPECT_EQ(tuning->state().at("request_result"),"failed");
  EXPECT_GT(tuning->state().at("expected_revisions").at(1).get<int64_t>(),failed_revision);
}
TEST_F(TuningProcess,StartupRollbackWithUnmetRequestedRppDistanceRemainsFailed){
  ASSERT_TRUE(endpoints[1]->set_parameters_atomically({rclcpp::Parameter("inflation_layer.inflation_radius",.22),
    rclcpp::Parameter("inflation_layer.tuning_revision",last_revision[1]+1)}).successful);
  fail_local_any_once=true;restart_startup();
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},4));
  stopped=true;ASSERT_TRUE(until([this]{return tuning->state().at("rolled_back").get<bool>();},4));
  EXPECT_EQ(tuning->state().at("confirmed"),tuning->state().at("requested"))<<"四项代价相同也不能忽略RPP距离要求";
  EXPECT_DOUBLE_EQ(endpoints[2]->get_parameter("FollowPath.cost_scaling_dist").as_double(),.3);
  EXPECT_EQ(tuning->state().at("phase"),"FAILED");EXPECT_FALSE(tuning->state().at("ready").get<bool>());
  EXPECT_TRUE(tuning->state().at("requires_stop").get<bool>());EXPECT_EQ(tuning->state().at("request_result"),"failed");
}
TEST_F(TuningProcess,WrongRollbackAckAndLateCorrectAckCannotUnlockFailedStartup){
  fail_local_any_once=true;restart_startup();
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="WAITING_STOP";},4));
  const auto failed_revision=tuning->state().at("expected_revisions").at(1).get<int64_t>();
  hold_ack[1]=true;stopped=true;
  ASSERT_TRUE(until([this,failed_revision]{return tuning->state().at("phase")=="ROLLING_BACK"&&
    endpoints[1]->get_parameter("inflation_layer.tuning_revision").as_int()>failed_revision;},3));
  const auto fresh_revision=endpoints[1]->get_parameter("inflation_layer.tuning_revision").as_int();
  std_msgs::msg::String wrong;wrong.data=Json{{"node","/local_costmap/local_costmap"},{"layer","inflation_layer"},
    {"revision",fresh_revision},{"radius",.3},{"factor",4.},{"completed",true},{"full_rebuild",true}}.dump();ack->publish(wrong);
  ASSERT_TRUE(until([this]{return tuning->state().at("phase")=="FAILED";},2));
  EXPECT_FALSE(tuning->state().at("ready").get<bool>());EXPECT_TRUE(tuning->state().at("requires_stop").get<bool>());
  hold_ack[1]=false;until([]{return false;},.15);
  EXPECT_EQ(tuning->state().at("phase"),"FAILED")<<"失败后的正确迟到ACK也不能重新完成回退";
  EXPECT_FALSE(tuning->state().at("ready").get<bool>());
}
TEST_F(TuningProcess,SuccessfulStartupDefaultsBecomeThisTaskRestoreBaseline){
  const auto file=std::filesystem::temp_directory_path()/("carcar-tuning-startup-baseline-"+
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  ca::TuningValues baseline;baseline.global_radius=.65;baseline.local_factor=6.;
  {std::ofstream out(file);out<<baseline.json();}
  executor.remove_node(tuning);tuning.reset();rclcpp::NodeOptions options;
  options.parameter_overrides({{"task_id",std::string("task-test")},{"defaults_file",file.string()},{"service_timeout_seconds",.35}});
  tuning=std::make_shared<ca::MappingTuningNode>(options);std::filesystem::remove(file);executor.add_node(tuning);
  ASSERT_TRUE(until([this]{return tuning->state().at("ready").get<bool>();},5));EXPECT_EQ(tuning->state().at("baseline"),baseline.json());
  ca::TuningValues modified=baseline;modified.global_radius=.8;apply(modified,"temporary");
  ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="temporary"&&tuning->state().at("phase")=="READY";},5));
  std_msgs::msg::String m;m.data=Json{{"task_id","task-test"},{"generation",generation},{"request_id","restore-default"},{"action","restore"}}.dump();requests->publish(m);
  ASSERT_TRUE(until([this]{return tuning->state().at("request_id")=="restore-default"&&tuning->state().at("phase")=="READY";},5));
  EXPECT_EQ(tuning->state().at("confirmed"),baseline.json());
}
class TuningDefaults:public testing::Test {
protected:
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{file=std::filesystem::temp_directory_path()/("carcar-tuning-defaults-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");}
  void TearDown()override{std::filesystem::remove(file);}
  Json inspect(const Json&value){std::ofstream out(file);out<<value;out.close();rclcpp::NodeOptions options;
    options.parameter_overrides({{"task_id",std::string("default-test")},{"defaults_file",file.string()}});
    auto node=std::make_shared<ca::MappingTuningNode>(options);return node->state();}
  std::filesystem::path file;
};
TEST_F(TuningDefaults,InvalidExistingConfigurationCannotSilentlyFallBackToReady){
  auto bad=ca::TuningValues{}.json();bad["local_inflation_radius"]=.1;const auto state=inspect(bad);
  EXPECT_EQ(state.at("phase"),"FAILED");EXPECT_FALSE(state.at("ready").get<bool>());EXPECT_TRUE(state.at("requires_stop").get<bool>());
}
TEST_F(TuningDefaults,SupportsValuesWrapperWithoutChangingRuntimeBeforeActiveNodes){
  auto values=ca::TuningValues{}.json();values["global_inflation_radius"]=.7;const auto state=inspect(Json{{"values",values}});
  EXPECT_EQ(state.at("phase"),"INITIALIZING");EXPECT_TRUE(state.at("defaults_loaded").get<bool>());EXPECT_FALSE(state.at("ready").get<bool>());
}
}
