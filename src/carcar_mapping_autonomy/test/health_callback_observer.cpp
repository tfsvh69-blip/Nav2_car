// 仅离线诊断LD_PRELOAD；保持本机Nav2 1.1.20的原isActiveCallback判定与返回。
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>
#include <nav2_lifecycle_manager/lifecycle_manager.hpp>

namespace {
using Manager=nav2_lifecycle_manager::LifecycleManager;
using Header=std::shared_ptr<rmw_request_id_t>;
using Request=std::shared_ptr<std_srvs::srv::Trigger::Request>;
using Response=std::shared_ptr<std_srvs::srv::Trigger::Response>;
using Original=void(*)(Manager*,Header,Request,Response);
constexpr auto symbol="_ZN22nav2_lifecycle_manager16LifecycleManager16isActiveCallbackESt10shared_ptrI16rmw_request_id_sES1_IN8std_srvs3srv16Trigger_Request_ISaIvEEEES1_INS5_17Trigger_Response_IS7_EEE";
std::atomic<uint64_t> original_calls{0};
uint64_t monotonic_ns(){timespec value{};clock_gettime(CLOCK_MONOTONIC,&value);
  return uint64_t(value.tv_sec)*1000000000ULL+uint64_t(value.tv_nsec);}
Original original(){
  static const auto function=[](){
    dlerror();void*address=dlsym(RTLD_NEXT,symbol);const char*error=dlerror();
    if(error||!address){std::fprintf(stderr,"DEBUG-health-20261103 edge=resolve_failed pid=%ld error=%s\n",long(getpid()),error?error:"null");std::abort();}
    Dl_info location{};dladdr(address,&location);
    std::fprintf(stderr,"DEBUG-health-20261103 edge=resolved pid=%ld monotonic_ns=%" PRIu64 " original_path=%s\n",
      long(getpid()),monotonic_ns(),location.dli_fname?location.dli_fname:"unknown");
    // POSIX dlsym用于本机Itanium C++ ABI同签名成员函数，首参显式传原this。
    return reinterpret_cast<Original>(address);
  }();
  return function;
}
}

extern "C" uint64_t carcar_health_observer_original_calls(){return original_calls.load(std::memory_order_relaxed);}

namespace nav2_lifecycle_manager {
void LifecycleManager::isActiveCallback(const Header header,const Request request,Response response){
  const auto call=original();const auto began=monotonic_ns();
  const int64_t sequence=header?header->sequence_number:-1;
  std::fprintf(stderr,"DEBUG-health-20261103 edge=entry pid=%ld monotonic_ns=%" PRIu64 " sequence=%" PRId64 "\n",long(getpid()),began,sequence);
  try{
    call(this,header,request,response);
    original_calls.fetch_add(1,std::memory_order_relaxed);
    const auto ended=monotonic_ns();
    std::fprintf(stderr,"DEBUG-health-20261103 edge=exit pid=%ld monotonic_ns=%" PRIu64 " sequence=%" PRId64 " success=%d duration_ns=%" PRIu64 "\n",
      long(getpid()),ended,sequence,response?int(response->success):-1,ended-began);
  }catch(...){
    const auto ended=monotonic_ns();
    std::fprintf(stderr,"DEBUG-health-20261103 edge=throw pid=%ld monotonic_ns=%" PRIu64 " sequence=%" PRId64 " duration_ns=%" PRIu64 "\n",long(getpid()),ended,sequence,ended-began);
    throw;
  }
}
}
