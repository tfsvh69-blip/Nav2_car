#pragma once
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <cstring>
#include <streambuf>
#include <sys/mman.h>
#include <unistd.h>
#include <karto_sdk/Mapper.h>
#include <boost/archive/binary_oarchive.hpp>

// 固定SLAM Toolbox 2.6.10已导出这个官方实例；复用它而不复制私有ScanManager布局。
extern template void boost::archive::detail::oserializer<boost::archive::binary_oarchive,karto::Mapper>::save_object_data(boost::archive::detail::basic_oarchive&,const void*)const;

namespace carcar_mapping_autonomy {
// 匿名RAM缓冲区没有磁盘路径；封存后内核只读，析构自动释放。
// 使用匿名映射以允许透明大页；不修改系统大页或CPU设置。
class SnapshotMemoryBuffer {
public:
  SnapshotMemoryBuffer()=default;
  ~SnapshotMemoryBuffer(){if(memory_)munmap(memory_,capacity_);}
  SnapshotMemoryBuffer(const SnapshotMemoryBuffer&)=delete;
  SnapshotMemoryBuffer&operator=(const SnapshotMemoryBuffer&)=delete;
  // 必须在建图锁外准备并触碰页面；锁内只写已经驻留的有界缓冲区。
  void prepare(size_t capacity){
    if(memory_||sealed()||!capacity||capacity>256*1024*1024)throw std::runtime_error("快照内存准备参数无效");
    auto*value=mmap(nullptr,capacity,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);
    if(value==MAP_FAILED)throw std::runtime_error("快照内存映射失败");
    memory_=static_cast<char*>(value);capacity_=capacity;
    madvise(memory_,capacity_,MADV_HUGEPAGE); // 仅建议本进程映射；不假定内核一定满足。
    // 使用volatile确保页面准备不会被优化掉；不在持有建图锁时等待缺页分配。
    volatile char*pages=memory_;const size_t page=static_cast<size_t>(sysconf(_SC_PAGESIZE));
    for(size_t i=0;i<capacity_;i+=page)pages[i]=0;
  }
  char*buffer()const{return sealed_?nullptr:memory_;}
  const char*data()const{if(!sealed_)throw std::runtime_error("快照尚未封存");return memory_;}
  size_t capacity()const{return capacity_;}
  void finish(size_t used){
    const auto began=std::chrono::steady_clock::now();
    if(!memory_||!used||used>capacity_)throw std::runtime_error("快照内存写入长度无效");
    used_=used;
    seal();
    finish_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  }
  double finish_ms=0;
  size_t size()const{
    if(!used_||used_>256*1024*1024)
      throw std::runtime_error("快照为空或超过256MiB资源上限");
    return used_;
  }
  void seal(){
    size();
    if(mprotect(memory_,capacity_,PROT_READ))
      throw std::runtime_error("快照内存封存失败");
    sealed_=true;
  }
  bool sealed()const{return sealed_;}
  // 仅供锁外落盘或验证使用，捕获过程不复制整份缓冲区。
  std::string read()const{
    return std::string(data(),size());
  }
private:
  char*memory_=nullptr;size_t capacity_=0,used_=0;bool sealed_=false;
};
struct GraphSnapshot {
  std::unique_ptr<SnapshotMemoryBuffer> graph,data;
  double capture_ms=0,graph_ms=0,data_ms=0,prepare_ms=0;
};
inline GraphSnapshot prepare_graph_snapshot(size_t capacity=32*1024*1024){
  const auto start=std::chrono::steady_clock::now();
  GraphSnapshot result{std::make_unique<SnapshotMemoryBuffer>(),std::make_unique<SnapshotMemoryBuffer>()};
  result.graph->prepare(capacity);result.data->prepare(capacity);
  result.prepare_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  return result;
}
class SnapshotOutputBuffer:public std::streambuf {
public:
  explicit SnapshotOutputBuffer(SnapshotMemoryBuffer&file):data_(file.buffer()),capacity_(file.capacity()){
    if(!data_||!capacity_)throw std::runtime_error("快照缓冲区未在锁外准备");
  }
  size_t size()const{return used_;}
protected:
  std::streamsize xsputn(const char*source,std::streamsize count)override{
    if(count<0||size_t(count)>capacity_-used_)return 0;
    // 扫描坐标占绝大多数写入，固定8字节复制能直接编译为加载/存储指令。
    if(count==8)std::memcpy(data_+used_,source,8);else std::memcpy(data_+used_,source,size_t(count));
    used_+=size_t(count);return count;
  }
  int_type overflow(int_type value)override{
    if(traits_type::eq_int_type(value,traits_type::eof()))return traits_type::not_eof(value);
    if(used_==capacity_)return traits_type::eof();
    data_[used_++]=traits_type::to_char_type(value);return value;
  }
private:
  char*data_;size_t capacity_,used_=0;
};
template<class T>inline void serialize_snapshot(T&object,SnapshotMemoryBuffer&file){
  SnapshotOutputBuffer buffer(file);
  {boost::archive::binary_oarchive archive(buffer,boost::archive::no_codecvt);archive<<BOOST_SERIALIZATION_NVP(object);}
  file.finish(buffer.size());
}
// 调用者必须在同一建图锁内调用；返回值不持有生产对象指针。
// 两个独立只读archive共享冻结状态，最多两个计算线程。禁止从工作线程读活动图。
inline GraphSnapshot capture_graph(karto::Mapper & mapper,karto::Dataset & dataset,GraphSnapshot result){
  const auto start=std::chrono::steady_clock::now();
  if(!result.graph||!result.data)throw std::runtime_error("快照捕获缺少准备好的缓冲区");
  auto data_job=std::async(std::launch::async,[&]{
    const auto began=std::chrono::steady_clock::now();serialize_snapshot(dataset,*result.data);
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  });
  const auto graph_start=std::chrono::steady_clock::now();serialize_snapshot(mapper,*result.graph);
  result.graph_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-graph_start).count();
  result.data_ms=data_job.get();
  result.capture_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  return result;
}
}
