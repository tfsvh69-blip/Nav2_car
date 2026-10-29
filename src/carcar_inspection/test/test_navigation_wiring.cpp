// 只提取实际巡检入口中的导航组，在隔离域运行；不启动任何底盘/雷达/相机驱动。
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tinyxml2.h>
#include <spawn.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <set>
#include <thread>
extern char ** environ;
namespace {
using namespace std::chrono_literals;
tinyxml2::XMLElement * navigation_group(tinyxml2::XMLElement * root) {
  for(auto * item=root->FirstChildElement();item;item=item->NextSiblingElement()) {
    if(std::string(item->Name())=="include") {
      const char * file=item->Attribute("file");
      if(file&&std::string(file).find("/navigation_experimental.launch.xml")!=std::string::npos)
        return item->Parent()->ToElement();
    }
    if(auto * result=navigation_group(item))return result;
  }
  return nullptr;
}
struct NavigationOnly {
  std::filesystem::path root;
  pid_t pid=-1;
  NavigationOnly(tinyxml2::XMLElement * group,tinyxml2::XMLElement * source) {
    char name[]="/tmp/carcar-navigation-wiring-XXXXXX";
    root=mkdtemp(name);
    tinyxml2::XMLDocument doc;
    auto * launch=doc.NewElement("launch");doc.InsertEndChild(launch);
    // 实际组现在引用入口参数；夹具须复制其声明，仍不复制任何硬件组。
    for(auto*arg=source->FirstChildElement("arg");arg;arg=arg->NextSiblingElement("arg"))launch->InsertEndChild(arg->DeepClone(&doc));
    launch->InsertEndChild(group->DeepClone(&doc));
    const auto file=(root/"navigation.launch.xml").string();
    if(doc.SaveFile(file.c_str())!=tinyxml2::XML_SUCCESS)throw std::runtime_error("无法写测试Launch");
    std::vector<std::string> args{"ros2","launch",file};
    std::vector<char *> raw;for(auto & arg:args)raw.push_back(arg.data());raw.push_back(nullptr);
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions,STDOUT_FILENO,(root/"launch.log").c_str(),O_CREAT|O_WRONLY|O_TRUNC,0600);
    posix_spawn_file_actions_adddup2(&actions,STDOUT_FILENO,STDERR_FILENO);
    posix_spawnattr_t attr;posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attr,0);
    const int rc=posix_spawnp(&pid,raw[0],&actions,&attr,raw.data(),environ);
    posix_spawnattr_destroy(&attr);posix_spawn_file_actions_destroy(&actions);
    if(rc)throw std::runtime_error("无法启动隔离导航Launch");
  }
  ~NavigationOnly() {
    if(pid>0) {
      kill(-pid,SIGINT);bool exited=false;
      for(int i=0;i<50;++i) {
        if(waitpid(pid,nullptr,WNOHANG)==pid){exited=true;break;}
        std::this_thread::sleep_for(100ms);
      }
      kill(-pid,SIGKILL);if(!exited)waitpid(pid,nullptr,0);
    }
    if(::testing::Test::HasFailure())std::cerr<<"隔离导航证据："<<root<<'\n';
    else std::filesystem::remove_all(root);
  }
};
std::set<std::string> publishers(const rclcpp::Node::SharedPtr & node,const std::string & topic) {
  std::set<std::string> result;
  for(const auto & endpoint:node->get_publishers_info_by_topic(topic))result.insert(endpoint.node_name());
  return result;
}
std::set<std::string> subscribers(const rclcpp::Node::SharedPtr & node,const std::string & topic) {
  std::set<std::string> result;
  for(const auto & endpoint:node->get_subscriptions_info_by_topic(topic))result.insert(endpoint.node_name());
  return result;
}
TEST(NavigationWiring, ActualInspectionGroupKeepsRawSmoothedAndGatedVelocitySeparate) {
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"226");
  ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"),"1");
  tinyxml2::XMLDocument source;ASSERT_EQ(source.LoadFile(ROBOT_LAUNCH),tinyxml2::XML_SUCCESS);
  auto * group=navigation_group(source.RootElement());ASSERT_NE(group,nullptr);
  rclcpp::init(0,nullptr);
  {
    NavigationOnly child(group,source.RootElement());
    auto node=std::make_shared<rclcpp::Node>("inspection_wiring_observer");
    const auto until=std::chrono::steady_clock::now()+15s;
    while(std::chrono::steady_clock::now()<until) {
      rclcpp::spin_some(node);
      if(publishers(node,"/inspection/navigation/cmd_vel").count("navigation_motion_gate")&&
        subscribers(node,"/cmd_vel_smoothed_raw").count("navigation_motion_gate")&&
        publishers(node,"/cmd_vel_smoothed_raw").count("velocity_smoother")&&
        (publishers(node,"/cmd_vel_nav").count("behavior_server")||
         publishers(node,"/inspection/navigation/cmd_vel").count("behavior_server"))&&
        (subscribers(node,"/cmd_vel").count("nav_event_logger")||
         subscribers(node,"/inspection/navigation/cmd_vel").count("nav_event_logger")))break;
      std::this_thread::sleep_for(100ms);
    }
    EXPECT_EQ(publishers(node,"/inspection/navigation/cmd_vel"),std::set<std::string>{"navigation_motion_gate"})
      <<"原始速度与最终门控输出合并，会引入零速竞争和反馈回路";
    EXPECT_EQ(publishers(node,"/cmd_vel_nav"),(std::set<std::string>{"controller_server","behavior_server"}));
    EXPECT_TRUE(subscribers(node,"/cmd_vel_nav").count("velocity_smoother"));
    EXPECT_FALSE(subscribers(node,"/inspection/navigation/cmd_vel").count("velocity_smoother"));
    EXPECT_TRUE(subscribers(node,"/cmd_vel").count("nav_event_logger"))<<"日志须读取真正底盘出口";
    EXPECT_TRUE(publishers(node,"/cmd_vel").empty())<<"隔离导航组不能绕过巡检控制台发到底盘出口";
  }
  rclcpp::shutdown();
}
}  // namespace
