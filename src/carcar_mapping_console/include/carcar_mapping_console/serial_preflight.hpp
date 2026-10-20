#pragma once
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

extern char ** environ;
namespace carcar_mapping_console {
// 只查看文件/进程占用，不打开串口；检查失败必须拒绝后续操作。
inline void require_unused_serial(const std::filesystem::path & port) {
  if (!std::filesystem::exists(port)) throw std::runtime_error("串口不存在: " + port.string());
  auto resolved = std::filesystem::canonical(port).string();
  int pipefd[2];
  if (pipe(pipefd) != 0) throw std::runtime_error("无法收集串口预检输出");
  posix_spawn_file_actions_t actions; posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addclose(&actions, pipefd[0]);
  posix_spawn_file_actions_adddup2(&actions, pipefd[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, pipefd[1], STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, pipefd[1]);
  // 本机fuser不接受此处的“--”；绝对路径也避免端口名被当成选项。
  char executable[] = "fuser";
  char * argv[] = {executable, resolved.data(), nullptr};
  pid_t pid;
  const int error = posix_spawnp(&pid, argv[0], &actions, nullptr, argv, environ);
  posix_spawn_file_actions_destroy(&actions); close(pipefd[1]);
  if (error) {
    close(pipefd[0]);
    throw std::runtime_error(std::string("串口占用检查工具无法执行: ") + std::strerror(error));
  }
  std::string output; char buffer[1024]; bool read_error = false;
  while (true) {
    const auto count = read(pipefd[0], buffer, sizeof(buffer));
    if (count > 0) {
      if (output.size() < 4096) output.append(buffer, std::min<size_t>(count, 4096 - output.size()));
    } else if (count == 0) break;
    else if (errno != EINTR) {read_error = true; break;}
  }
  close(pipefd[0]); int status; pid_t waited;
  do {waited = waitpid(pid, &status, 0);} while (waited < 0 && errno == EINTR);
  if (waited < 0 || read_error) throw std::runtime_error("串口预检结果读取失败");
  // 不裁剪空白：任何stdout/stderr都不是“确认空闲”。
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 1 || !output.empty())
    throw std::runtime_error("串口占用或无法检查，拒绝继续: " + port.string() + "\n" + output);
}
}  // namespace carcar_mapping_console
