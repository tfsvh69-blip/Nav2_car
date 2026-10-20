#pragma once
#include "carcar_mapping_console/common.hpp"
#include <cerrno>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace carcar_mapping_console {
inline std::string shell_quote(const std::string & text) {
  std::string result = "'";
  for (char c : text) result += c == '\'' ? "'\\''" : std::string(1, c);
  return result + "'";
}
// 同目录、独占创建、0600，完整写入并同步成功后才允许删除原配置。
inline std::filesystem::path backup_profile(const std::filesystem::path & source) {
  const int input = open(source.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
  if (input < 0) throw std::runtime_error("热点配置无法读取，拒绝删除");
  struct stat status{};
  if (fstat(input, &status) || !S_ISREG(status.st_mode)) {
    close(input); throw std::runtime_error("热点配置不是普通文件，拒绝删除");
  }
  const auto backup = source.string() + "." + nonce() + ".backup";
  const int output = open(backup.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (output < 0) {close(input); throw std::runtime_error("无法创建同目录热点备份，拒绝删除");}
  try {
    char buffer[65536];
    for (;;) {
      const auto count = read(input, buffer, sizeof(buffer));
      if (count < 0 && errno == EINTR) continue;
      if (count < 0) throw std::runtime_error("热点备份读取失败，拒绝删除");
      if (!count) break;
      ssize_t offset = 0;
      while (offset < count) {
        const auto written = write(output, buffer + offset, count - offset);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) throw std::runtime_error("热点备份写入失败，拒绝删除");
        offset += written;
      }
    }
    if (fsync(output)) throw std::runtime_error("热点备份同步失败，拒绝删除");
  } catch (...) {close(input); close(output); unlink(backup.c_str()); throw;}
  close(input);
  if (close(output)) {unlink(backup.c_str()); throw std::runtime_error("热点备份关闭失败，拒绝删除");}
  return backup;
}
}  // namespace carcar_mapping_console
