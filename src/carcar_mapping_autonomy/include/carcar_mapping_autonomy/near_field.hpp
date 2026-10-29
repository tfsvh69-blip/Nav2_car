#pragma once
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace carcar_mapping_autonomy {
// 与 collision.yaml 的停车框、nav2.yaml 的 footprint 同一矩形。
// 整车实测长 0.235 m、宽 0.216 m；框是 0.28 m × 0.26 m，略大于车身。
inline constexpr double kBodyMinX = -0.14, kBodyMaxX = 0.14, kBodyMinY = -0.13, kBodyMaxY = 0.13;

struct NearField {
  bool valid = false, inside = false;
  int inside_count = 0;
  double range = 0, x = 0, y = 0;
  std::string side;
};

inline std::string body_side(double x, double y) {
  if (y > 0.04 && std::abs(y) >= std::abs(x)) return "左侧";
  if (y < -0.04 && std::abs(y) >= std::abs(x)) return "右侧";
  if (x > 0.04) return "前方";
  if (x < -0.04) return "后方";
  return "车身附近";
}

inline std::string format_meters(double value) {
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "%.2f", value);
  return buffer;
}

// ranges 是激光坐标系下的 (距离, 方位角)。laser_* 是激光原点在 base_footprint 下的位姿。
inline NearField assess_near_field(const std::vector<std::pair<double, double>> & ranges, double laser_x, double laser_y, double laser_yaw) {
  NearField nearest_any, nearest_inside;
  int inside_count = 0;
  const double c = std::cos(laser_yaw), s = std::sin(laser_yaw);
  for (const auto & sample : ranges) {
    const double range = sample.first, angle = sample.second;
    if (!std::isfinite(range) || !std::isfinite(angle) || range < 0.05 || range > 1.0) continue;
    const double lx = range * std::cos(angle), ly = range * std::sin(angle);
    const double x = laser_x + c * lx - s * ly, y = laser_y + s * lx + c * ly;
    if (!std::isfinite(x) || !std::isfinite(y)) continue;
    const bool in = x >= kBodyMinX && x <= kBodyMaxX && y >= kBodyMinY && y <= kBodyMaxY;
    auto closer = [&](NearField & slot) {
      if (!slot.valid || range < slot.range) {slot.valid = true; slot.range = range; slot.x = x; slot.y = y; slot.side = body_side(x, y);}
    };
    closer(nearest_any);
    if (in) {++inside_count; closer(nearest_inside);}
  }
  NearField result = nearest_inside.valid ? nearest_inside : nearest_any;
  result.inside = nearest_inside.valid;
  result.inside_count = inside_count;
  return result;
}

inline std::string motion_block_note(bool commanded, bool cleared, const NearField & field) {
  if (!commanded || !cleared) return {};
  if (field.inside) return "导航已经给出速度，但激光进入车身停车框：" + field.side + " " + format_meters(field.range) +
    " 米处有回波。近场保护把发给轮子的速度清成零，所以车停在原地。";
  if (field.valid) return "导航已经给出速度，但发给轮子的速度仍是零。最近激光在" + field.side + " " + format_meters(field.range) +
    " 米，位于车身停车框外。请看全局和局部代价是否把路堵住，或碰撞监视器是否因扫描超时停车。";
  return "导航已经给出速度，但发给轮子的速度是零，当前没有可用的车身坐标系激光来判断远近。";
}
}  // namespace carcar_mapping_autonomy
