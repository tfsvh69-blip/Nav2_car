// 只在隔离DDS域派生真实场景节点，不启动导航、驱动或执行器。
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <chrono>
#include <cstdlib>
#include <thread>
#include "carcar_mapping_autonomy/task_walls.hpp"
extern char**environ;
namespace ca=carcar_mapping_autonomy;
namespace {
class SceneProcess {
public:
  explicit SceneProcess(const std::string&task){
    std::vector<std::string>arguments{MAPPING_TASK_SCENE_EXECUTABLE,"--ros-args"};
    if(!task.empty()){arguments.push_back("-p");arguments.push_back("task_id:="+task);}
    std::vector<char*>pointers;for(auto&arg:arguments)pointers.push_back(arg.data());pointers.push_back(nullptr);
    const int error=posix_spawn(&pid_,pointers[0],nullptr,nullptr,pointers.data(),environ);
    if(error)throw std::runtime_error("隔离场景节点无法启动："+std::to_string(error));
  }
  ~SceneProcess(){stop();}
  bool stop(){
    if(pid_<=0)return true;
    kill(pid_,SIGINT);
    for(int attempt=0;attempt<200;++attempt){int status=0;const auto result=waitpid(pid_,&status,WNOHANG);
      if(result==pid_){pid_=-1;return WIFEXITED(status)&&WEXITSTATUS(status)==0;}
      if(result<0){pid_=-1;return false;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    kill(pid_,SIGKILL);waitpid(pid_,nullptr,0);pid_=-1;return false;
  }
private:pid_t pid_=-1;
};
class TaskSceneProcessTest:public testing::Test {
protected:
  static void SetUpTestSuite(){
    const char*domain=std::getenv("ROS_DOMAIN_ID"),*local=std::getenv("ROS_LOCALHOST_ONLY");
    ASSERT_NE(domain,nullptr);ASSERT_STREQ(domain,"225");ASSERT_NE(local,nullptr);ASSERT_STREQ(local,"1");
    int argc=0;rclcpp::init(argc,nullptr);
  }
  static void TearDownTestSuite(){if(rclcpp::ok())rclcpp::shutdown();}
  void SetUp()override{
    node=std::make_shared<rclcpp::Node>("task_scene_process_fixture");
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    raw=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",qos);
    generations=node->create_publisher<std_msgs::msg::UInt64>("/mapping/navigation_generation",qos);
    requests=node->create_publisher<std_msgs::msg::String>("/mapping/walls/request",rclcpp::QoS(10).reliable());
    state_sub=node->create_subscription<std_msgs::msg::String>("/mapping/walls/state",qos,
      [this](const std_msgs::msg::String&message){state=ca::Json::parse(message.data);++states;});
    map_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/mapping/navigation_map",qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message){map=std::move(message);++maps;});
    mask_sub=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/mapping/walls/mask",qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message){mask=std::move(message);});
    info_sub=node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>("/mapping/walls/filter_info",qos,
      [this](const nav2_msgs::msg::CostmapFilterInfo&message){info=message;have_info=true;});
  }
  template<class Predicate>bool until(Predicate done,double seconds=5){
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<deadline){rclcpp::spin_some(node);if(done())return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));}rclcpp::spin_some(node);return done();
  }
  nav_msgs::msg::OccupancyGrid source(unsigned width=20,double origin=0){
    nav_msgs::msg::OccupancyGrid result;result.header.frame_id="map";result.header.stamp=node->now();
    result.info.width=width;result.info.height=20;result.info.resolution=.1;
    result.info.origin.position.x=origin;result.info.origin.orientation.w=1;result.data.assign(width*20,0);
    result.data[1]=-1;result.data[3]=20;return result;
  }
  void generation(uint64_t value){std_msgs::msg::UInt64 message;message.data=value;generations->publish(message);}
  void request(ca::Json value){std_msgs::msg::String message;message.data=value.dump();requests->publish(message);}
  ca::Json replace(const std::string&id,uint64_t revision,uint64_t generation,const ca::Json&walls){
    return {{"task_id","scene-task"},{"request_id",id},{"action","replace"},{"expected_revision",revision},
      {"navigation_generation",generation},{"walls",walls}};
  }
  bool response(const std::string&id,const std::string&result){return state.value("request_id","")==id&&state.value("request_result","")==result;}
  rclcpp::Node::SharedPtr node;ca::Json state=ca::Json::object();size_t states=0,maps=0;bool have_info=false;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,mask;nav2_msgs::msg::CostmapFilterInfo info;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr generations;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr requests;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub,mask_sub;
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr info_sub;
};
}
TEST_F(TaskSceneProcessTest,RealProcessPublishesAtomicConstraintsRejectsStaleRequestsAndRestartClearsTask){
  auto process=std::make_unique<SceneProcess>("scene-task");
  auto original=source();generation(10);raw->publish(original);
  ASSERT_TRUE(until([&]{return state.value("phase","")=="READY"&&state.value("task_id","")=="scene-task"&&map&&mask&&have_info;}));
  EXPECT_EQ(*map,original);EXPECT_EQ(info.type,0);EXPECT_EQ(info.base,0);EXPECT_EQ(info.multiplier,1);
  EXPECT_EQ(info.filter_mask_topic,"/mapping/walls/mask");
  const double boundary=10.*double(original.info.resolution);
  const auto walls=ca::Json::array({{{"id","glass-boundary"},{"points",{{boundary,.1},{boundary,1.9}}}}});
  request(replace("replace-1",0,10,walls));
  ASSERT_TRUE(until([&]{return response("replace-1","accepted")&&state["revision"]==1&&map&&map->data!=original.data;}));
  const auto accepted=*map;EXPECT_EQ(state["walls"],walls);EXPECT_TRUE(state["temporary"]);
  EXPECT_EQ(state["navigation_fingerprint"],std::to_string(ca::map_content_fingerprint(accepted)));
  EXPECT_EQ(state["source_fingerprint"],std::to_string(ca::map_content_fingerprint(original)));
  EXPECT_EQ(accepted.data[10*20+9],100);EXPECT_EQ(accepted.data[10*20+10],100);
  EXPECT_EQ(accepted.data[1],-1);EXPECT_EQ(accepted.data[3],20);EXPECT_EQ(mask->data[1],0);
  auto bad=replace("wrong-task",1,10,ca::Json::array());bad["task_id"]="old-task";request(bad);
  ASSERT_TRUE(until([&]{return response("wrong-task","rejected");}));EXPECT_EQ(state["revision"],1);EXPECT_EQ(*map,accepted);
  std_msgs::msg::String invalid;invalid.data="{broken";requests->publish(invalid);
  ASSERT_TRUE(until([&]{return state.value("request_result","")=="rejected"&&state.value("request_id","").empty();}));
  EXPECT_EQ(state["phase"],"READY");EXPECT_EQ(*map,accepted);
  invalid.data=std::string(65537,'x');requests->publish(invalid);
  ASSERT_TRUE(until([&]{return state.value("request_error","").find("64KiB")!=std::string::npos;}));
  EXPECT_EQ(state["phase"],"READY");EXPECT_EQ(*map,accepted);
  const size_t previous_maps=maps;generation(11);
  ASSERT_TRUE(until([&]{return state.value("phase","")=="REVIEW_REQUIRED"&&state["navigation_generation"]==11;}));
  auto expanded=source(30,-.5);raw->publish(expanded);
  ASSERT_TRUE(until([&]{return state.contains("source_geometry")&&state["source_geometry"]["width"]==30;}));
  EXPECT_EQ(maps,previous_maps)<<"未确认时不得把旧墙静默套入新地图";
  request({{"task_id","scene-task"},{"request_id","confirm-old"},{"action","confirm"},{"navigation_generation",10},{"revision",1}});
  ASSERT_TRUE(until([&]{return response("confirm-old","rejected");}));EXPECT_EQ(state["phase"],"REVIEW_REQUIRED");
  request({{"task_id","scene-task"},{"request_id","confirm-ok"},{"action","confirm"},{"navigation_generation",11},{"revision",1}});
  ASSERT_TRUE(until([&]{return response("confirm-ok","accepted")&&state.value("phase","")=="READY"&&map->info.width==30;}));
  EXPECT_EQ(state["revision"],1);EXPECT_EQ(state["applied_navigation_generation"],11);
  const auto confirmed=*map;auto late=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/mapping/navigation_map",
    rclcpp::QoS(1).reliable().transient_local(),[&](const nav_msgs::msg::OccupancyGrid&value){if(value==confirmed)have_info=false;});
  ASSERT_TRUE(until([&]{return !have_info;}))<<"迟订阅者必须接收当前真实约束地图";
  request(replace("clear",1,11,ca::Json::array()));
  ASSERT_TRUE(until([&]{return response("clear","accepted")&&state["revision"]==2&&*map==expanded;}));
  EXPECT_TRUE(std::all_of(mask->data.begin(),mask->data.end(),[](int value){return value==0;}));
  request(replace("replace-again",2,11,walls));ASSERT_TRUE(until([&]{return response("replace-again","accepted");}));
  const size_t heartbeats=states;ASSERT_TRUE(until([&]{return states>heartbeats;},1.6));
  ASSERT_TRUE(process->stop());process.reset();
  state=ca::Json::object();map.reset();mask.reset();
  process=std::make_unique<SceneProcess>("new-or-resumed-task");
  ASSERT_TRUE(until([&]{return state.value("task_id","")=="new-or-resumed-task"&&state.value("phase","")=="READY"&&map&&mask;}));
  EXPECT_EQ(state["revision"],0);EXPECT_TRUE(state["walls"].empty());EXPECT_EQ(*map,expanded);
  auto old_request=replace("old-task-request",0,11,walls);request(old_request);
  ASSERT_TRUE(until([&]{return response("old-task-request","rejected");}));EXPECT_TRUE(state["walls"].empty());
  ASSERT_TRUE(process->stop());
}
TEST_F(TaskSceneProcessTest,DefaultEmptyTaskAdoptsOnlyFirstValidTaskAndReplaysLatchedInputs){
  auto original=source();generation(7);raw->publish(original);
  SceneProcess process("");
  ASSERT_TRUE(until([&]{return state.contains("task_id")&&state["task_id"]=="";}));EXPECT_FALSE(map);
  auto autonomy=node->create_publisher<std_msgs::msg::String>("/mapping/autonomy/state",rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::String message;message.data="{\"task_id\":\"\"}";autonomy->publish(message);
  until([]{return false;},.1);EXPECT_EQ(state["task_id"],"");
  message.data="{\"task_id\":\"first-default-task\"}";autonomy->publish(message);
  ASSERT_TRUE(until([&]{return state.value("task_id","")=="first-default-task"&&state.value("phase","")=="READY"&&map;}));
  EXPECT_EQ(state["navigation_generation"],7);EXPECT_EQ(*map,original);EXPECT_TRUE(state["walls"].empty());
  message.data="{\"task_id\":\"different-later-task\"}";autonomy->publish(message);
  until([]{return false;},1.1);EXPECT_EQ(state["task_id"],"first-default-task");EXPECT_EQ(state["revision"],0);
  ASSERT_TRUE(process.stop());
}
