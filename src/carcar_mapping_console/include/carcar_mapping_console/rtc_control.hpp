#pragma once
#include <functional>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>

namespace carcar_mapping_console {
// 只传小型操纵消息；所有权、顺序屏障与300ms授权仍由既有网关/门控判断。
// 回调来自库线程；close()返回后不会再调用调用方，调用方应将回调投递至自己的事件循环。
class RtcControl {
public:
  using Signal = std::function<void(nlohmann::json)>;
  using Receive = std::function<void(std::string)>;
  using State = std::function<void(bool, std::string)>;
  RtcControl(Signal, Receive, State, const std::string & bind_address = {});
  ~RtcControl();
  static bool available();
  void offer(const std::string & sdp);
  void candidate(const std::string & candidate, const std::string & mid);
  bool send(const std::string & message);
  nlohmann::json diagnostics() const;
  void close();
private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace carcar_mapping_console
