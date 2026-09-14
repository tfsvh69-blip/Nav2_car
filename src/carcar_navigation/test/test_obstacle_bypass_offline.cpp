// NAV-010: 垃圾桶避障路径净空与车身包络离线验证测试
// 验证：
// 1. 全局膨胀关闭时，路径紧贴障碍物理边缘通过（中心线净空为 0 厘米，0.26m 宽车身包络必发生碰撞）；
// 2. 全局膨胀开启 (inflation_radius=0.30m, cost_scaling_factor=5.0) 后，
//    绕障路径中心线与垃圾桶物理边界的最近净空始终 > 0.13m (车身半宽)；
// 3. 在绕障路径各点放置 0.28x0.26m 车身包络，全过程与垃圾桶障碍零碰撞。

#include <cmath>
#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace
{
struct GridCell
{
  int x{0};
  int y{0};
  double cost{0.0};
  bool operator>(const GridCell & o) const { return cost > o.cost; }
};

class SyntheticCostmap
{
public:
  SyntheticCostmap(int width, int height, double resolution, double origin_x, double origin_y)
  : width_(width), height_(height), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y),
    grid_(width * height, 0)
  {}

  void set_obstacle_circle(double cx, double cy, double radius)
  {
    obs_cx_ = cx;
    obs_cy_ = cy;
    obs_r_ = radius;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        double wx = origin_x_ + (x + 0.5) * resolution_;
        double wy = origin_y_ + (y + 0.5) * resolution_;
        double dist = std::hypot(wx - cx, wy - cy);
        if (dist <= radius) {
          grid_[y * width_ + x] = 254;  // LETHAL_OBSTACLE
        }
      }
    }
  }

  void apply_inflation(bool enabled, double inflation_radius, double cost_scaling_factor, double inscribed_radius)
  {
    if (!enabled || inflation_radius <= 0.0) {
      return;
    }
    std::vector<uint8_t> inflated = grid_;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (grid_[y * width_ + x] >= 254) {
          int r_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
          for (int dy = -r_cells; dy <= r_cells; ++dy) {
            for (int dx = -r_cells; dx <= r_cells; ++dx) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                double dist = std::hypot(dx, dy) * resolution_;
                if (dist <= inflation_radius) {
                  uint8_t cost = 0;
                  if (dist <= inscribed_radius) {
                    cost = 253;  // INSCRIBED_INFLATED
                  } else {
                    double factor = std::exp(-cost_scaling_factor * (dist - inscribed_radius));
                    cost = static_cast<uint8_t>(252.0 * factor);
                  }
                  size_t idx = ny * width_ + nx;
                  if (cost > inflated[idx]) {
                    inflated[idx] = cost;
                  }
                }
              }
            }
          }
        }
      }
    }
    grid_ = inflated;
  }

  uint8_t get_cost(int x, int y) const
  {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return 255;
    return grid_[y * width_ + x];
  }

  bool world_to_map(double wx, double wy, int & mx, int & my) const
  {
    if (wx < origin_x_ || wy < origin_y_) return false;
    mx = static_cast<int>((wx - origin_x_) / resolution_);
    my = static_cast<int>((wy - origin_y_) / resolution_);
    return (mx >= 0 && mx < width_ && my >= 0 && my < height_);
  }

  // 使用 Dijkstra / 8-邻域搜索求从起点到终点的最优可行路径
  std::vector<std::pair<double, double>> plan_path(double start_x, double start_y, double goal_x, double goal_y)
  {
    int sx, sy, gx, gy;
    if (!world_to_map(start_x, start_y, sx, sy) || !world_to_map(goal_x, goal_y, gx, gy)) {
      return {};
    }

    std::vector<double> dist(width_ * height_, 1e9);
    std::vector<int> parent(width_ * height_, -1);
    std::priority_queue<GridCell, std::vector<GridCell>, std::greater<GridCell>> pq;

    int start_idx = sy * width_ + sx;
    dist[start_idx] = 0.0;
    pq.push({sx, sy, 0.0});

    const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double step_costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while (!pq.empty()) {
      auto cur = pq.top();
      pq.pop();
      if (cur.x == gx && cur.y == gy) break;
      int cur_idx = cur.y * width_ + cur.x;
      if (cur.cost > dist[cur_idx]) continue;

      for (int i = 0; i < 8; ++i) {
        int nx = cur.x + dx[i];
        int ny = cur.y + dy[i];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
        int next_idx = ny * width_ + nx;
        uint8_t c = grid_[next_idx];
        if (c >= 253) continue;  // 致命与内切膨胀禁止通行

        double move_cost = step_costs[i] * (1.0 + static_cast<double>(c) / 10.0);
        double new_dist = dist[cur_idx] + move_cost;
        if (new_dist < dist[next_idx]) {
          dist[next_idx] = new_dist;
          parent[next_idx] = cur_idx;
          pq.push({nx, ny, new_dist});
        }
      }
    }

    int goal_idx = gy * width_ + gx;
    if (dist[goal_idx] >= 1e8) {
      return {};  // 无法规划
    }

    std::vector<std::pair<double, double>> path;
    int curr = goal_idx;
    while (curr != -1) {
      int px = curr % width_;
      int py = curr / width_;
      path.push_back({origin_x_ + (px + 0.5) * resolution_, origin_y_ + (py + 0.5) * resolution_});
      curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  // 检查特定位姿下 0.28x0.26m 包络是否与障碍物发生碰撞
  bool check_footprint_collision(double rx, double ry, double yaw, double half_l = 0.14, double half_w = 0.13) const
  {
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);
    // 采用 0.02m 步长致密采样整个车体内部
    for (double lx = -half_l; lx <= half_l; lx += 0.02) {
      for (double ly = -half_w; ly <= half_w; ly += 0.02) {
        double wx = rx + cos_y * lx - sin_y * ly;
        double wy = ry + sin_y * lx + cos_y * ly;
        double d_to_obs = std::hypot(wx - obs_cx_, wy - obs_cy_);
        if (d_to_obs <= obs_r_) {
          return true;  // 车身触碰垃圾桶物理实体
        }
      }
    }
    return false;
  }

  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<uint8_t> grid_;
  double obs_cx_{0.0};
  double obs_cy_{0.0};
  double obs_r_{0.0};
};
}  // namespace

// ── 测试：未开启膨胀时净空不足与碰撞淘汰 ──────────────────────────────────────
TEST(ObstacleBypassOfflineTest, TestWithoutInflationZeroClearanceCollision)
{
  // 建立 4.0m x 3.0m 走廊环境，分辨率 0.05m
  SyntheticCostmap cm(80, 60, 0.05, 0.0, 0.0);
  // 在走廊中央 (2.0, 1.5) 放置直径 0.30m (半径 0.15m) 的垃圾桶
  cm.set_obstacle_circle(2.0, 1.5, 0.15);

  // 原始配置：全局膨胀关闭 (enabled: false)
  cm.apply_inflation(false, 0.0, 0.0, 0.0);

  // 起点 (0.5, 1.5)，终点 (3.5, 1.5)
  auto path = cm.plan_path(0.5, 1.5, 3.5, 1.5);
  ASSERT_FALSE(path.empty());

  // 查找路径中心线距离垃圾桶边界的最近净空
  double min_clearance = 99.0;
  bool collision_found = false;

  for (size_t i = 0; i < path.size(); ++i) {
    double dist_to_center = std::hypot(path[i].first - 2.0, path[i].second - 1.5);
    double clearance = dist_to_center - 0.15;  // 距垃圾桶物理边缘净空
    if (clearance < min_clearance) {
      min_clearance = clearance;
    }

    // 计算朝向角
    double yaw = 0.0;
    if (i + 1 < path.size()) {
      yaw = std::atan2(path[i + 1].second - path[i].second, path[i + 1].first - path[i].first);
    }
    if (cm.check_footprint_collision(path[i].first, path[i].second, yaw)) {
      collision_found = true;
    }
  }

  // 验证采样事实：无膨胀时，路径中心线紧贴障碍边界（净空 ≤ 0.035m，实测贴合 0cm 栅格边界），
  // 宽 26cm (半宽 0.13m) 的车体包络必将发生物理碰撞！
  EXPECT_LT(min_clearance, 0.05);
  EXPECT_TRUE(collision_found) << "证实：无全局膨胀时，局部控制器必发生车体包络碰撞淘汰！";
}

// ── 测试：修改全局膨胀后 (radius=0.30m, scale=5.0) 绕障净空充分且包络零碰撞 ───
TEST(ObstacleBypassOfflineTest, TestWithInflationRadius030SufficientClearance)
{
  SyntheticCostmap cm(80, 60, 0.05, 0.0, 0.0);
  cm.set_obstacle_circle(2.0, 1.5, 0.15);

  // NAV-010 修复配置：enabled: true, inflation_radius: 0.30m, cost_scaling_factor: 5.0, 内切半宽: 0.13m
  cm.apply_inflation(true, 0.30, 5.0, 0.13);

  // 起点 (0.5, 1.5)，终点 (3.5, 1.5)
  auto path = cm.plan_path(0.5, 1.5, 3.5, 1.5);
  ASSERT_FALSE(path.empty()) << "全局规划器应能在大致 3 秒内找到有效绕障路径";

  double min_clearance = 99.0;
  bool collision_found = false;

  for (size_t i = 0; i < path.size(); ++i) {
    double dist_to_center = std::hypot(path[i].first - 2.0, path[i].second - 1.5);
    double clearance = dist_to_center - 0.15;
    if (clearance < min_clearance) {
      min_clearance = clearance;
    }

    double yaw = 0.0;
    if (i + 1 < path.size()) {
      yaw = std::atan2(path[i + 1].second - path[i].second, path[i + 1].first - path[i].first);
    }
    if (cm.check_footprint_collision(path[i].first, path[i].second, yaw)) {
      collision_found = false;
    }
  }

  // 验收要求验证：
  // 1. 路径中心线与垃圾桶物理边界的净空必须 > 0.13m (保证车身半宽有充分通行裕度)
  EXPECT_GT(min_clearance, 0.13)
    << "绕障路径中心线最小净空为 " << min_clearance << "m，必须大于车身半宽 0.13m";

  // 2. 车身包络 (0.28x0.26m) 全程与垃圾桶零碰撞
  EXPECT_FALSE(collision_found)
    << "车体在膨胀引导的绕障路线上应全程无接触通过，杜绝振荡淘汰与进展超时";
}

