#pragma once
#include <chrono>
#include <array>
#include <filesystem>
#include <optional>
#include <vector>
#include <utility>
#include <fcntl.h>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include "carcar_mapping_autonomy/graph_snapshot.hpp"
#include "carcar_mapping_autonomy/snapshot_protocol.hpp"
extern char**environ;
namespace carcar_mapping_autonomy {
struct SnapshotInputs {
  int graph=-1,data=-1;
  SnapshotInputs()=default;
  SnapshotInputs(const SnapshotInputs&)=delete;
  SnapshotInputs&operator=(const SnapshotInputs&)=delete;
  SnapshotInputs(SnapshotInputs&&other)noexcept:graph(std::exchange(other.graph,-1)),data(std::exchange(other.data,-1)){}
  ~SnapshotInputs(){if(graph>=0)close(graph);if(data>=0)close(data);}
};
// 锁外把只读内存副本转成可继承的封存RAM句柄；这里仍不访问磁盘。
inline SnapshotInputs snapshot_process_inputs(GraphSnapshot snapshot){
  SnapshotInputs result;
  auto copy=[](const SnapshotMemoryBuffer&buffer,int&fd){
    fd=memfd_create("carcar-snapshot-worker",MFD_CLOEXEC|MFD_ALLOW_SEALING);if(fd<0)throw std::runtime_error("工作进程快照句柄创建失败");
    size_t offset=0;while(offset<buffer.size()){
      const auto count=write(fd,buffer.data()+offset,buffer.size()-offset);
      if(count<0&&errno==EINTR)continue;
      if(count<=0)throw std::runtime_error("工作进程快照复制失败");
      offset+=size_t(count);
    }
    if(fcntl(fd,F_ADD_SEALS,F_SEAL_WRITE|F_SEAL_GROW|F_SEAL_SHRINK|F_SEAL_SEAL))throw std::runtime_error("工作进程输入封存失败");
  };
  copy(*snapshot.graph,result.graph);copy(*snapshot.data,result.data);return result;
}
class SnapshotProcess {
public:
  ~SnapshotProcess(){cancel("进程退出，取消未完成快照");if(pid_>0)waitpid(pid_,nullptr,WNOHANG);if(report_>=0)close(report_);}
  bool active()const{return pid_>0;}
  const SnapshotJson&state()const{return state_;}
  void start(const std::string&executable,const std::filesystem::path&root,const SnapshotJson&metadata,
    SnapshotInputs inputs,double timeout=25){
    if(active())throw std::runtime_error("已有快照工作进程，拒绝重叠事务");
    int pipefd[2];if(pipe2(pipefd,O_CLOEXEC|O_NONBLOCK))throw std::runtime_error("快照状态管道创建失败");
    const int nullfd=open("/dev/null",O_WRONLY|O_CLOEXEC);if(nullfd<0){close(pipefd[0]);close(pipefd[1]);throw std::runtime_error("快照日志端点创建失败");}
    // 父进程原句柄可能恰好是197～199；先复制到目的范围之外，避免dup2先覆盖另一个输入。
    const std::array<int,4> sources{inputs.graph,inputs.data,pipefd[1],nullfd};
    std::array<int,4> inherited{-1,-1,-1,-1};
    for(size_t i=0;i<sources.size();++i){
      inherited[i]=fcntl(sources[i],F_DUPFD_CLOEXEC,200);
      if(inherited[i]<0){for(const int fd:inherited)if(fd>=0)close(fd);
        close(pipefd[0]);close(pipefd[1]);close(nullfd);throw std::runtime_error("快照子进程句柄隔离失败");}
    }
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions,inherited[0],198);posix_spawn_file_actions_adddup2(&actions,inherited[1],199);
    posix_spawn_file_actions_adddup2(&actions,inherited[2],197);
    posix_spawn_file_actions_adddup2(&actions,inherited[3],STDOUT_FILENO);posix_spawn_file_actions_adddup2(&actions,inherited[3],STDERR_FILENO);
    std::vector<std::string>args{executable,root.string(),metadata.dump()};std::vector<char*>argv;
    for(auto&arg:args)argv.push_back(arg.data());
    argv.push_back(nullptr);
    pid_t child=-1;const int error=posix_spawn(&child,executable.c_str(),&actions,nullptr,argv.data(),environ);
    posix_spawn_file_actions_destroy(&actions);for(const int fd:inherited)close(fd);close(pipefd[1]);close(nullfd);
    if(error){close(pipefd[0]);throw std::runtime_error("快照工作进程启动失败："+std::to_string(error));}
    if(report_>=0)close(report_);
    report_=pipefd[0];pid_=child;cancelled_=false;pending_.clear();
    state_={{"phase","WRITING"}};deadline_=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
  }
  void cancel(const std::string&reason){
    if(!active()||cancelled_)return;
    cancelled_=true;state_={{"phase","FAILED"},{"reason",reason}};kill(pid_,SIGKILL);
  }
  // 只做非阻塞读与waitpid；慢盘、子进程退出或取消不会卡住SLAM回调线程。
  void poll(){
    if(!active())return;
    if(std::chrono::steady_clock::now()>deadline_)cancel("快照后台处理超过时限，旧可信快照保留");
    auto drain=[&]{char buffer[4096];ssize_t count;
    while((count=read(report_,buffer,sizeof(buffer)))>0){
      if(cancelled_)continue;
      pending_.append(buffer,size_t(count));
      if(pending_.size()>16384){cancel("快照工作进程状态超过资源上限");continue;}
      size_t newline;while(!cancelled_&&(newline=pending_.find('\n'))!=std::string::npos){
        try{state_=SnapshotJson::parse(pending_.substr(0,newline));}catch(...){cancel("快照工作进程状态格式无效");}
        pending_.erase(0,newline+1);
      }
    }};
    drain();
    int status=0;const auto done=waitpid(pid_,&status,WNOHANG);
    if(done==pid_||(done<0&&errno==ECHILD)){
      drain(); // 子进程可能在上一次read与waitpid之间写入最终READY。
      pid_=-1;close(report_);report_=-1;
      if(!cancelled_&&(done<0||!WIFEXITED(status)||WEXITSTATUS(status)!=0||state_.value("phase","")!="READY")){
        if(state_.value("phase","")!="FAILED")state_={{"phase","FAILED"},{"reason","快照工作进程退出，未产生完整可信快照"}};
      }
    }
  }
private:
  pid_t pid_=-1;int report_=-1;bool cancelled_=false;std::string pending_;
  SnapshotJson state_=SnapshotJson::object();
  std::chrono::time_point<std::chrono::steady_clock,std::chrono::duration<double>>deadline_;
};
} // namespace carcar_mapping_autonomy
