// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <string>
#include <thread>

namespace carcar_inspection {

// 建图状态只在后台查询；控制循环通过 blocks_inspection() 读取缓存。
// start()/stop() 由拥有者线程串行调用，读取缓存可来自任意线程。
// 初始不阻断；空地址禁用查询。异常判定沿用原控制台，不代表运动许可。
class MappingStatusMonitor {
public:
  explicit MappingStatusMonitor(std::string status_url);
  ~MappingStatusMonitor();

  MappingStatusMonitor(const MappingStatusMonitor &) = delete;
  MappingStatusMonitor & operator=(const MappingStatusMonitor &) = delete;

  void start();
  bool blocks_inspection() const noexcept;
  // 重复停止安全，返回前回收后台线程；停止后保留最后一次缓存。
  void stop();

private:
  bool query_busy() const;
  void watch();

  const std::string status_url_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
};

}  // namespace carcar_inspection
