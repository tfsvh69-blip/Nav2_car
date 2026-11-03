// 仅域225、空managed_nodes夹具；真实ROS服务证明preload调用原函数，不启动受管节点/硬件。
#include <chrono>
#include <cstdlib>
#include <dlfcn.h>
#include <iostream>
#include <thread>
#include <vector>
#include <nav2_lifecycle_manager/lifecycle_manager.hpp>

class FixtureManager:public nav2_lifecycle_manager::LifecycleManager {
public:
  FixtureManager(const rclcpp::NodeOptions&options,bool active):LifecycleManager(options){system_active_=active;}
};
int main(int argc,char**argv){
  const char*domain=std::getenv("ROS_DOMAIN_ID");const char*local=std::getenv("ROS_LOCALHOST_ONLY");
  if(!domain||std::string(domain)!="225"||!local||std::string(local)!="1"){std::cerr<<"夹具仅允许隔离域225与localhost\n";return 2;}
  auto count=reinterpret_cast<uint64_t(*)()>(dlsym(RTLD_DEFAULT,"carcar_health_observer_original_calls"));
  if(!count){std::cerr<<"未加载观测器，拒绝伪造通过\n";return 2;}
  rclcpp::init(argc,argv);bool passed=true;std::vector<std::shared_ptr<FixtureManager>>retained;
  for(const bool expected:{false,true}){
    const std::string name=expected?"health_fixture_active":"health_fixture_inactive";
    rclcpp::NodeOptions options;options.arguments({"--ros-args","-r","__node:="+name});
    options.parameter_overrides({{"autostart",false},{"node_names",std::vector<std::string>{}}});
    auto manager=std::make_shared<FixtureManager>(options,expected);
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(manager);
    std::thread thread([&]{executor.spin();});
    auto client_node=std::make_shared<rclcpp::Node>("health_fixture_client");
    auto client=client_node->create_client<std_srvs::srv::Trigger>("/"+name+"/is_active");
    const auto before=count();bool actual=!expected,returned=false;
    if(client->wait_for_service(std::chrono::seconds(3))){
      auto future=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      if(rclcpp::spin_until_future_complete(client_node,future,std::chrono::seconds(3))==rclcpp::FutureReturnCode::SUCCESS){actual=future.get()->success;returned=true;}
    }
    const bool correct=returned&&actual==expected&&count()==before+1;
    std::cout<<"HEALTH_OBSERVER_FIXTURE expected="<<expected<<" actual="<<actual<<" returned="<<returned<<" original_calls="<<count()<<" passed="<<correct<<std::endl;
    passed=passed&&correct;executor.cancel();thread.join();executor.remove_node(manager);
    // 官方manager注册了context preshutdown回调；夹具也遵守先shutdown、再销毁manager的生命周期。
    retained.push_back(std::move(manager));
  }
  rclcpp::shutdown();retained.clear();return passed?0:1;
}
