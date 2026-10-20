#pragma once
#include <nlohmann/json.hpp>

namespace carcar_mapping_console {
// 快速控制确认仅携带交互必需字段；完整停车证据仍由诊断接口保留。
inline nlohmann::json compact_control(const nlohmann::json & full) {
  nlohmann::json value = nlohmann::json::object();
  for (const auto key : {"challenge", "armed", "reason", "gear", "x", "yaw", "health",
       "supported_gears", "require_video", "control_protocol", "control_lease_ms",
       "accepted_sequence", "ack_sequence", "control_revision", "control_epoch", "issued_steady_ms", "idle_heartbeat_required"})
    if (full.contains(key)) value[key] = full[key];
  if (full.contains("last_stop") && full["last_stop"].is_object()) {
    const auto & stop = full["last_stop"];
    if (stop.contains("server_steady")) value["last_stop"] = {
      {"server_steady", stop["server_steady"]}, {"reason", stop.value("reason", "")}};
  }
  return value;
}
}  // namespace carcar_mapping_console
