#ifndef EXPLORE_DIRECTION_PREFERENCE_H_
#define EXPLORE_DIRECTION_PREFERENCE_H_

#include <cmath>
#include <geometry_msgs/msg/point.hpp>

namespace explore
{
// 方向只参与候选排序，不替代路径规划或把后方未知区域判为不可达。
// 保存成功行驶的地图方向；地图坐标发生校正时由监督器增代清空。
class DirectionPreference
{
public:
  double penalty(const geometry_msgs::msg::Point & start, double yaw,
    const geometry_msgs::msg::Point & candidate, double scale) const
  {
    const double dx = candidate.x - start.x, dy = candidate.y - start.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1e-6 || scale == 0.0) {return 0.0;}
    const double facing = 1.0 - (dx * std::cos(yaw) + dy * std::sin(yaw)) / distance;
    const double continuation = valid_ ? 1.0 - (dx * x_ + dy * y_) / distance : facing;
    return scale * (0.35 * facing + 0.65 * continuation);
  }

  void succeeded(const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & reached)
  {
    double dx = reached.x - start.x, dy = reached.y - start.y;
    const double distance = std::hypot(dx, dy);
    // 未真正行驶的近点成功不创建方向证据，拒绝/取消也不调用本入口。
    if (!std::isfinite(distance) || distance < 0.20) {return;}
    dx /= distance; dy /= distance;
    if (valid_) {dx = 0.65 * x_ + 0.35 * dx; dy = 0.65 * y_ + 0.35 * dy;}
    const double length = std::hypot(dx, dy);
    x_ = dx / length; y_ = dy / length; valid_ = true;
  }

  void clear() {valid_ = false;}

private:
  bool valid_ = false;
  double x_ = 1.0, y_ = 0.0;
};
}  // namespace explore
#endif
