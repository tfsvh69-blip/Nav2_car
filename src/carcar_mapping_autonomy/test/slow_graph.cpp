// 仅隔离回归：让真实ROS图查询延迟400ms，验证它不能阻塞速度/停车循环。
#include <rmw/get_topic_endpoint_info.h>
#include <dlfcn.h>
#include <chrono>
#include <cstring>
#include <thread>
#include <atomic>
#include <algorithm>
#include <cstdlib>
extern "C" rmw_ret_t rmw_get_publishers_info_by_topic(
 const rmw_node_t*node,rcutils_allocator_t*allocator,const char*topic,bool no_mangle,
 rmw_topic_endpoint_info_array_t*info){
 using Function=rmw_ret_t(*)(const rmw_node_t*,rcutils_allocator_t*,const char*,bool,rmw_topic_endpoint_info_array_t*);
 static auto original=reinterpret_cast<Function>(dlsym(RTLD_NEXT,"rmw_get_publishers_info_by_topic"));
 static const auto began=std::chrono::steady_clock::now();static std::atomic_bool delayed{false};
 if(node&&node->name&&topic&&std::strcmp(node->name,"mapping_autonomy")==0&&std::strcmp(topic,"/mapping/cmd_vel")==0&&
    std::chrono::steady_clock::now()-began>std::chrono::seconds(2)&&!delayed.exchange(true)){
   const auto*value=std::getenv("CARCAR_TEST_GRAPH_DELAY_MS");
   const int milliseconds=value?std::clamp(std::atoi(value),1,2000):400;
   std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
 }
 return original(node,allocator,topic,no_mangle,info);
}
