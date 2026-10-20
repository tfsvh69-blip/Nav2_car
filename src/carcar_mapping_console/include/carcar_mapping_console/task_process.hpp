#pragma once
#include "carcar_mapping_console/common.hpp"
#include <spawn.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <cerrno>
#include <cstring>
extern char ** environ;
namespace carcar_mapping_console {
// 只执行服务端构造的 argv；独立进程组使 Launch 的子进程也能有序退出。
class TaskProcess {
public:
  ~TaskProcess() {stop();}
  void start(std::vector<std::string> args, const std::filesystem::path & log) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pid_ > 0) throw std::runtime_error("上次任务尚未释放");
    std::filesystem::create_directories(log.parent_path());
    const int fd = open(log.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0) throw std::runtime_error("无法创建任务日志");
    posix_spawn_file_actions_t actions; posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_adddup2(&actions, fd, STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, fd, STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, fd);
    // 后台Launch只继承标准输入/日志，不能继续持有网关监听、HTTP或控制socket。
    // 否则网关close后子进程仍持有连接，浏览器收不到真正的TCP断开。
    const int close_error=posix_spawn_file_actions_addclosefrom_np(&actions,3);
    if(close_error){posix_spawn_file_actions_destroy(&actions);close(fd);
      throw std::runtime_error(std::string("无法隔离后台文件描述符: ")+std::strerror(close_error));}
    posix_spawnattr_t attr; posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP); posix_spawnattr_setpgroup(&attr, 0);
    std::vector<char *> argv; for (auto & a : args) argv.push_back(a.data()); argv.push_back(nullptr);
    const int error = posix_spawnp(&pid_, argv[0], &actions, &attr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions); posix_spawnattr_destroy(&attr); close(fd);
    if (error) {pid_ = -1; throw std::runtime_error(std::string("任务启动失败: ") + std::strerror(error));}
  }
  bool alive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pid_ <= 0) return false;
    siginfo_t info{};
    // 暂不回收进程组首进程，避免退出后的 PID 被复用，再由 stop 统一回收。
    return waitid(P_PID, pid_, &info, WEXITED | WNOHANG | WNOWAIT) == 0 && info.si_pid == 0;
  }
  void stop() noexcept {
    pid_t pid;
    {std::lock_guard<std::mutex> lock(mutex_); pid = pid_;}
    if (pid <= 0) return;
    // Launch 会自行向节点转发 SIGINT；给整个组发送会让节点重复收到信号。
    kill(pid, SIGINT);
    const double until = steady() + 8;
    while (alive() && steady() < until) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // 即使 Launch 首进程先退，也清理本工具自己的残留子进程组。
    kill(-pid, SIGTERM);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    kill(-pid, SIGKILL);
    int status; while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    {std::lock_guard<std::mutex> lock(mutex_); pid_ = -1;}
  }
private:
  mutable std::mutex mutex_; pid_t pid_ = -1;
};
inline std::string log_tail(const std::filesystem::path & path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return "暂无任务日志";
  file.seekg(0, std::ios::end); const auto end = file.tellg();
  file.seekg(std::max<std::streamoff>(0, static_cast<std::streamoff>(end) - 6000));
  auto text = std::string(std::istreambuf_iterator<char>(file), {});
  // 按字节截尾可能落在汉字中间；跳过残缺的UTF-8延续字节，保证诊断JSON可编码。
  while (!text.empty() && (static_cast<unsigned char>(text.front()) & 0xc0) == 0x80) text.erase(0, 1);
  return text;
}
}  // namespace carcar_mapping_console
