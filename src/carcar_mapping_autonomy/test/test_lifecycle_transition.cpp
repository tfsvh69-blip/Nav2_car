#include <gtest/gtest.h>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_srvs/srv/trigger.hpp>
extern char **environ;
class LifecycleTransition:public testing::Test {
protected:
 rclcpp::Node::SharedPtr node;
 rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change;
 rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr state;
 rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready;
 pid_t pid=-1;uint8_t current=1;size_t requests=0;bool drop_first=true,apply_first=true;
 static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 template<class F>bool until(F f,double timeout){auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
  do{rclcpp::spin_some(node);if(f())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}while(std::chrono::steady_clock::now()<end);return f();}
 void SetUp()override{
  node=std::make_shared<rclcpp::Node>("lifecycle_transition_fixture");
  change=node->create_service<lifecycle_msgs::srv::ChangeState>("/fake_lifecycle/change_state",
   [this](std::shared_ptr<rmw_request_id_t>header,std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>request){
    ++requests;const auto transition=request->transition.id;
    if(!(requests==1&&!apply_first))current=transition==1?2:(transition==3?3:(transition==2?1:4));
    if(requests==1&&drop_first)return;  // 故意不回包，复现实节点已过渡而应答丢失。
    auto response=std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();response->success=true;change->send_response(*header,*response);
   });
  state=node->create_service<lifecycle_msgs::srv::GetState>("/fake_lifecycle/get_state",
   [this](std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,std::shared_ptr<lifecycle_msgs::srv::GetState::Response>response){response->current_state.id=current;});
  ready=node->create_client<std_srvs::srv::Trigger>("/lifecycle_manager_navigation/is_active");
 }
 void launch(){
  char executable[]=LIFECYCLE_EXE,ros[]="--ros-args",p[]="-p",r[]="-r",name[]="__node:=lifecycle_manager_navigation";
  char nodes[]="node_names:=[fake_lifecycle]",automatic[]="autostart:=true",bond[]="bond_timeout:=0.0",timeout[]="transition_response_timeout:=1.0";
  char*args[]={executable,ros,r,name,p,nodes,p,automatic,p,bond,p,timeout,nullptr};
  ASSERT_EQ(posix_spawn(&pid,executable,nullptr,nullptr,args,environ),0);
 }
 void TearDown()override{if(pid>0){kill(pid,SIGINT);for(int i=0;i<100;++i){rclcpp::spin_some(node);if(waitpid(pid,nullptr,WNOHANG)==pid){pid=-1;break;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);}}
  ready.reset();state.reset();change.reset();node.reset();}
 bool is_active(){if(!ready->wait_for_service(std::chrono::seconds(2)))return false;
  auto f=ready->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if(!until([&]{return f.wait_for(std::chrono::seconds(0))==std::future_status::ready;},2))return false;
  return f.get()->success;}
};
TEST_F(LifecycleTransition, LostResponseUsesRealStateWithoutRepeatingTransition){launch();
 ASSERT_TRUE(until([this]{return requests==2&&current==3;},6));EXPECT_TRUE(is_active());EXPECT_EQ(requests,2U);}
TEST_F(LifecycleTransition, LostResponseWithoutActualTransitionDoesNotActivate){apply_first=false;launch();
 ASSERT_TRUE(until([this]{return requests==1;},3));until([]{return false;},1.5);
 EXPECT_FALSE(is_active());EXPECT_EQ(requests,1U);EXPECT_EQ(current,1U);}
TEST_F(LifecycleTransition, NormalResponsesStillUseOfficialManager){drop_first=false;launch();
 ASSERT_TRUE(until([this]{return requests==2&&current==3;},4));EXPECT_TRUE(is_active());EXPECT_EQ(requests,2U);}
