// 仅用于独立诊断的LD_PRELOAD观察器；不链接进运行包，不读取消息正文，不改变原函数参数或返回值。
#include <rmw/rmw.h>
#include <rcl/publisher.h>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace {
std::atomic<uint64_t> next_call{1};
uint64_t monotonic_ns() noexcept {
  timespec value{};if(::clock_gettime(CLOCK_MONOTONIC,&value))return 0;
  return uint64_t(value.tv_sec)*1000000000ULL+uint64_t(value.tv_nsec);
}
template<class Function> Function original(const char*name) noexcept {
  void*symbol=::dlsym(RTLD_NEXT,name);
  if(!symbol){
    char line[256];const int count=std::snprintf(line,sizeof(line),"DEBUG-rpc-20261103 fatal=missing_original symbol=%s pid=%ld\n",name,long(::getpid()));
    if(count>0){const auto written=::write(STDERR_FILENO,line,size_t(count)<sizeof(line)?size_t(count):sizeof(line)-1);(void)written;}
    // 找不到原符号就终止诊断进程，绝不伪造一个RMW或RCL返回值继续运行。
    ::_exit(127);
  }
  return reinterpret_cast<Function>(symbol);
}
bool watched(const char*name) noexcept {
  return name&&(std::strstr(name,"/lifecycle_manager_navigation/is_active")||
    std::strstr(name,"/local_costmap/local_costmap/set_parameters_atomically")||
    std::strstr(name,"/global_costmap/global_costmap/set_parameters_atomically")||
    std::strstr(name,"/controller_server/set_parameters_atomically"));
}
bool watched_topic(const char*name) noexcept {
  return name&&!std::strstr(name,"footprint")&&(!std::strcmp(name,"/mapping/tuning/inflation_ack")||!std::strcmp(name,"/parameter_events")||
    std::strstr(name,"/local_costmap/costmap")==name||std::strstr(name,"/global_costmap/costmap")==name);
}
void record(const char*op,const char*edge,const char*name,uint64_t id,uint64_t stamp,uint64_t duration,
            const rmw_request_id_t*header,const char*taken,const char*result,void*target,
            const void*handle=nullptr,const int64_t*sequence=nullptr) noexcept {
  char guid[33]{};
  if(header)for(size_t i=0;i<16;++i)std::snprintf(guid+2*i,3,"%02x",static_cast<unsigned char>(header->writer_guid[i]));
  char line[1024];
  const int count=std::snprintf(line,sizeof(line),
    "DEBUG-rpc-20261103 op=%s edge=%s monotonic_ns=%" PRIu64 " call_id=%" PRIu64 " pid=%ld tid=%ld name=%s handle=%p seq_valid=%d seq=%" PRId64 " guid=%s taken=%s rc=%s duration_ns=%" PRIu64 " original=%p\n",
    op,edge,stamp,id,long(::getpid()),long(::syscall(SYS_gettid)),name?name:"-",const_cast<void*>(handle),header||sequence?1:0,
    sequence?*sequence:header?header->sequence_number:int64_t(0),header?guid:"-",taken,result,duration,target);
  if(count>0){const auto written=::write(STDERR_FILENO,line,size_t(count)<sizeof(line)?size_t(count):sizeof(line)-1);(void)written;}
}
void completed(const char*op,const char*name,uint64_t id,uint64_t end,uint64_t begin,
               const rmw_request_id_t*header,const char*taken,int result,void*target,
               const void*handle=nullptr,const int64_t*sequence=nullptr) noexcept {
  char code[32];std::snprintf(code,sizeof(code),"%d",result);
  record(op,"exit",name,id,end,end>=begin?end-begin:0,header,taken,code,target,handle,sequence);
}
}

extern "C" __attribute__((visibility("default"))) rmw_ret_t rmw_send_request(
  const rmw_client_t*client,const void*request,int64_t*sequence_id){
  const int incoming_errno=errno;static const auto real=original<decltype(&rmw_send_request)>("rmw_send_request");
  const char*name=client?client->service_name:nullptr;
  if(!watched(name)){errno=incoming_errno;return real(client,request,sequence_id);}
  const auto id=next_call.fetch_add(1,std::memory_order_relaxed);
  // sequence_id是输出参数，进入时不读取；按client handle区分同服务的多个序号空间。
  record("rmw_send_request","enter",name,id,monotonic_ns(),0,nullptr,"na","na",reinterpret_cast<void*>(real),client);
  const auto begin=monotonic_ns();errno=incoming_errno;const auto result=real(client,request,sequence_id);const int returned_errno=errno;const auto end=monotonic_ns();
  completed("rmw_send_request",name,id,end,begin,nullptr,"na",int(result),reinterpret_cast<void*>(real),client,result==RMW_RET_OK?sequence_id:nullptr);
  errno=returned_errno;return result;
}

// 本机Humble的take ABI使用rmw_service_info_t，request_id位于其中；不是旧版rmw_request_id_t*。
extern "C" __attribute__((visibility("default"))) rmw_ret_t rmw_take_request(
  const rmw_service_t*service,rmw_service_info_t*header,void*request,bool*taken){
  const int incoming_errno=errno;static const auto real=original<decltype(&rmw_take_request)>("rmw_take_request");
  const char*name=service?service->service_name:nullptr;
  if(!watched(name)){errno=incoming_errno;return real(service,header,request,taken);}
  const auto id=next_call.fetch_add(1,std::memory_order_relaxed);
  record("rmw_take_request","enter",name,id,monotonic_ns(),0,nullptr,"na","na",reinterpret_cast<void*>(real),service);
  const auto begin=monotonic_ns();errno=incoming_errno;const auto result=real(service,header,request,taken);const int returned_errno=errno;const auto end=monotonic_ns();
  const bool valid=result==RMW_RET_OK&&taken;
  completed("rmw_take_request",name,id,end,begin,valid&&*taken&&header?&header->request_id:nullptr,valid?(*taken?"1":"0"):"na",int(result),reinterpret_cast<void*>(real),service);
  errno=returned_errno;return result;
}
extern "C" __attribute__((visibility("default"))) rmw_ret_t rmw_send_response(
  const rmw_service_t*service,rmw_request_id_t*header,void*response){
  const int incoming_errno=errno;static const auto real=original<decltype(&rmw_send_response)>("rmw_send_response");
  const char*name=service?service->service_name:nullptr;
  if(!watched(name)){errno=incoming_errno;return real(service,header,response);}
  const auto id=next_call.fetch_add(1,std::memory_order_relaxed);
  record("rmw_send_response","enter",name,id,monotonic_ns(),0,header,"na","na",reinterpret_cast<void*>(real),service);
  const auto begin=monotonic_ns();errno=incoming_errno;const auto result=real(service,header,response);const int returned_errno=errno;const auto end=monotonic_ns();
  completed("rmw_send_response",name,id,end,begin,header,"na",int(result),reinterpret_cast<void*>(real),service);
  errno=returned_errno;return result;
}
extern "C" __attribute__((visibility("default"))) rmw_ret_t rmw_take_response(
  const rmw_client_t*client,rmw_service_info_t*header,void*response,bool*taken){
  const int incoming_errno=errno;static const auto real=original<decltype(&rmw_take_response)>("rmw_take_response");
  const char*name=client?client->service_name:nullptr;
  if(!watched(name)){errno=incoming_errno;return real(client,header,response,taken);}
  const auto id=next_call.fetch_add(1,std::memory_order_relaxed);
  record("rmw_take_response","enter",name,id,monotonic_ns(),0,nullptr,"na","na",reinterpret_cast<void*>(real),client);
  const auto begin=monotonic_ns();errno=incoming_errno;const auto result=real(client,header,response,taken);const int returned_errno=errno;const auto end=monotonic_ns();
  const bool valid=result==RMW_RET_OK&&taken;
  completed("rmw_take_response",name,id,end,begin,valid&&*taken&&header?&header->request_id:nullptr,valid?(*taken?"1":"0"):"na",int(result),reinterpret_cast<void*>(real),client);
  errno=returned_errno;return result;
}
extern "C" __attribute__((visibility("default"))) rcl_ret_t rcl_publish(
  const rcl_publisher_t*publisher,const void*message,rmw_publisher_allocation_t*allocation){
  const int incoming_errno=errno;static const auto real=original<decltype(&rcl_publish)>("rcl_publish");
  static const auto topic_name=original<decltype(&rcl_publisher_get_topic_name)>("rcl_publisher_get_topic_name");
  const char*name=publisher?topic_name(publisher):nullptr;
  if(!watched_topic(name)){errno=incoming_errno;return real(publisher,message,allocation);}
  const auto id=next_call.fetch_add(1,std::memory_order_relaxed);
  record("rcl_publish","enter",name,id,monotonic_ns(),0,nullptr,"na","na",reinterpret_cast<void*>(real),publisher);
  const auto begin=monotonic_ns();errno=incoming_errno;const auto result=real(publisher,message,allocation);const int returned_errno=errno;const auto end=monotonic_ns();
  completed("rcl_publish",name,id,end,begin,nullptr,"na",int(result),reinterpret_cast<void*>(real),publisher);
  errno=returned_errno;return result;
}
