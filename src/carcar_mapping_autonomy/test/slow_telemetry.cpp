// 仅隔离回归：域205的软件夹具可注入一次600毫秒，正式域31不生效。
#include <rcl/publisher.h>
#include <std_msgs/msg/string.hpp>
#include <dlfcn.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <thread>
namespace {
bool armed(const char*mode){
 const auto*domain=std::getenv("ROS_DOMAIN_ID");const auto*local=std::getenv("ROS_LOCALHOST_ONLY");
 const auto*requested=std::getenv("CARCAR_TEST_TELEMETRY_DELAY");const auto*path=std::getenv("CARCAR_TEST_TELEMETRY_ARM");
 return domain&&local&&requested&&path&&std::strcmp(domain,"205")==0&&std::strcmp(local,"1")==0&&
   std::strcmp(requested,mode)==0&&access(path,F_OK)==0;
}
void delay(){
 const std::string marker=std::getenv("CARCAR_TEST_TELEMETRY_ARM");
 std::ofstream(marker+".fired")<<"已进入隔离延迟注入\n";
 std::this_thread::sleep_for(std::chrono::milliseconds(600));
}
}
extern "C" void rcl_logging_external_log(int severity,const char*name,const char*message){
 using Function=void(*)(int,const char*,const char*);
 static auto original=reinterpret_cast<Function>(dlsym(RTLD_NEXT,"rcl_logging_external_log"));
 static std::atomic_bool delayed{false};
 if(armed("file")&&name&&message&&std::strcmp(name,"mapping_autonomy")==0&&
    std::strstr(message,"自动建图状态")&&std::strstr(message,"\"phase\":\"EXPLORING\"")&&!delayed.exchange(true))delay();
 original(severity,name,message);
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t*publisher,const void*message,rmw_publisher_allocation_t*allocation){
 using Function=rcl_ret_t(*)(const rcl_publisher_t*,const void*,rmw_publisher_allocation_t*);
 static auto original=reinterpret_cast<Function>(dlsym(RTLD_NEXT,"rcl_publish"));
 static std::atomic_bool delayed{false};
 const auto result=original(publisher,message,allocation);
 const char*topic=publisher?rcl_publisher_get_topic_name(publisher):nullptr;
 if(armed("state")&&topic&&std::strcmp(topic,"/mapping/autonomy/state")==0&&!delayed.exchange(true))delay();
 return result;
}
