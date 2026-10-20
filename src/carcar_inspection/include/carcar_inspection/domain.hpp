// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace carcar_inspection {
using json = nlohmann::json;
inline void require(bool condition, const std::string & reason) {
  if (!condition) throw std::runtime_error(reason);
}
inline void valid_id(const std::string & id) {
  require(!id.empty() && id.size() <= 100 &&
    id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos,
    "编号无效");
}
inline double number(const json & value, const char * key) {
  const auto result = value.at(key).get<double>();
  require(std::isfinite(result), std::string(key) + "必须为有限数值");
  return result;
}
inline int servo_command(double target) {
  require(std::isfinite(target) && target >= 85.0 && target <= 160.0,
    "S1进行目标必须在85°～160°内");
  // 最上端量化不能把160°变为板端107（标称160.5°）。实际位置仍无反馈。
  return std::clamp(static_cast<int>(std::lround(target / 1.5)), 57, 106);
}
inline void validate_pose(const json & value) {
  number(value, "x"); number(value, "y");
  require(std::abs(number(value, "yaw")) <= M_PI, "朝向必须在-pi～pi弧度内");
}
inline void validate_approach(const json & approach) {
  validate_pose(approach.at("pose"));
  require(servo_command(number(approach, "target_degrees")) == approach.at("board_command").get<int>(),
    "目标角与板端档位不一致");
}
inline json approach_list(const json & point) {
  if (point.contains("approaches")) {
    require(point.at("approaches").is_array(), "停车位列表无效");
    return point.at("approaches");
  }
  json approach{{"pose", point.at("pose")}, {"target_degrees", number(point, "target_degrees")},
    {"board_command", point.at("board_command")}, {"verified", point.value("verified", false)}};
  if (point.contains("reference")) approach["reference"] = point["reference"];
  return json::array({std::move(approach)});
}
inline json normalize_point(json point) {
  require(!point.contains("views"), "同一停车位不支持多个拍摄角度");
  if (!point.contains("approaches")) {
    json approach{{"pose", point.at("pose")}, {"target_degrees", number(point, "target_degrees")},
      {"board_command", point.at("board_command")}, {"verified", point.value("verified", false)}};
    if (point.contains("reference")) approach["reference"] = point["reference"];
    point["approaches"] = json::array({std::move(approach)});
  }
  const auto approaches = point.at("approaches");
  require(approaches.is_array() && approaches.size() >= 1 && approaches.size() <= 3, "每个拍摄目标需要1～3个停车位");
  bool verified = true;
  for (const auto & approach : approaches) {validate_approach(approach); verified = verified && approach.value("verified", false);}
  const auto & first = approaches.at(0);
  point["pose"] = first.at("pose");
  point["target_degrees"] = first.at("target_degrees");
  point["board_command"] = first.at("board_command");
  point["verified"] = verified;
  return point;
}
inline void validate_point(const json & point) {
  valid_id(point.at("id")); valid_id(point.at("map_id"));
  require(point.at("name").is_string() && !point.at("name").get<std::string>().empty() &&
    point.at("name").get<std::string>().size() <= 180, "名称须为1～180字节");
  require(point.at("map_version").is_string() && !point.at("map_version").get<std::string>().empty(), "缺少地图版本");
  require(!point.contains("views"), "同一停车位不支持多个拍摄角度");
  const auto normalized = normalize_point(point);
  require(normalized.at("approaches").size() >= 1 && normalized.at("approaches").size() <= 3, "每个拍摄目标需要1～3个停车位");
}
inline void validate_task(const json & task) {
  valid_id(task.at("id")); valid_id(task.at("map_id"));
  require(task.at("name").is_string() && !task.at("name").get<std::string>().empty(), "任务名称不能为空");
  require(task.at("point_ids").is_array() && !task.at("point_ids").empty() && task.at("point_ids").size() <= 100, "请选择1～100个巡检点");
  std::vector<std::string> seen;
  for (const auto & id : task.at("point_ids")) {
    const auto text = id.get<std::string>(); valid_id(text);
    require(std::find(seen.begin(), seen.end(), text) == seen.end(), "同一任务不能重复添加相同点位");
    seen.push_back(text);
  }
  validate_pose(task.at("final_parking_pose"));
}
}  // namespace carcar_inspection
