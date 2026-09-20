#pragma once
#include <array>
#include <cmath>

namespace carcar_navigation {

// 3 状态 EKF：位置跟轮式里程计，航向以 IMU gz 预测、轮式 yaw 只作弱观测。
struct WheelImuEkf {
  double x{0}, y{0}, yaw{0};
  std::array<double, 9> P{{0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05}};
  bool initialized{false};

  static double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

  void reset(double px, double py, double pyaw)
  {
    x = px; y = py; yaw = wrap(pyaw); initialized = true;
    P = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05};
  }

  void predict(double v, double w, double dt, double q_xy, double q_yaw)
  {
    if (!initialized || dt <= 0 || dt > 0.5) {return;}
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    x += v * c * dt;
    y += v * s * dt;
    yaw = wrap(yaw + w * dt);
    // F = [[1,0,-v s dt],[0,1,v c dt],[0,0,1]]
    const double f13 = -v * s * dt;
    const double f23 = v * c * dt;
    const auto p = P;
    const double n00 = p[0] + p[6] * f13;
    const double n01 = p[1] + p[7] * f13;
    const double n02 = p[2] + p[8] * f13;
    const double n10 = p[3] + p[6] * f23;
    const double n11 = p[4] + p[7] * f23;
    const double n12 = p[5] + p[8] * f23;
    P[0] = n00 + n02 * f13 + q_xy;
    P[1] = n01 + n02 * f23;
    P[2] = n02;
    P[3] = n10 + n12 * f13;
    P[4] = n11 + n12 * f23 + q_xy;
    P[5] = n12;
    P[6] = p[6] + p[8] * f13;
    P[7] = p[7] + p[8] * f23;
    P[8] = p[8] + q_yaw;
  }

  void update_xy(double zx, double zy, double r_xy)
  {
    if (!initialized || r_xy <= 0) {return;}
    // 观测 H = [[1,0,0],[0,1,0]]
    const double s00 = P[0] + r_xy;
    const double s01 = P[1];
    const double s11 = P[4] + r_xy;
    const double det = s00 * s11 - s01 * s01;
    if (std::abs(det) < 1e-12) {return;}
    const double inv00 = s11 / det;
    const double inv01 = -s01 / det;
    const double inv11 = s00 / det;
    const double k00 = P[0] * inv00 + P[1] * inv01;
    const double k01 = P[0] * inv01 + P[1] * inv11;
    const double k10 = P[3] * inv00 + P[4] * inv01;
    const double k11 = P[3] * inv01 + P[4] * inv11;
    const double k20 = P[6] * inv00 + P[7] * inv01;
    const double k21 = P[6] * inv01 + P[7] * inv11;
    const double ix = zx - x;
    const double iy = zy - y;
    x += k00 * ix + k01 * iy;
    y += k10 * ix + k11 * iy;
    yaw = wrap(yaw + k20 * ix + k21 * iy);
    const auto p = P;
    P[0] = (1 - k00) * p[0] - k01 * p[3];
    P[1] = (1 - k00) * p[1] - k01 * p[4];
    P[2] = (1 - k00) * p[2] - k01 * p[5];
    P[3] = -k10 * p[0] + (1 - k11) * p[3];
    P[4] = -k10 * p[1] + (1 - k11) * p[4];
    P[5] = -k10 * p[2] + (1 - k11) * p[5];
    P[6] = -k20 * p[0] - k21 * p[3] + p[6];
    P[7] = -k20 * p[1] - k21 * p[4] + p[7];
    P[8] = -k20 * p[2] - k21 * p[5] + p[8];
  }
};

}  // namespace carcar_navigation
