#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

// 离线读取快照；不按高度删点，不连接 ROS，不把统计结果自动判为地面验收通过。
struct Point {double x, y, z;};

void write_stats(std::ostream & out, const std::string & frame, size_t band,
  const std::vector<Point> & points)
{
  constexpr std::array<double, 5> edges{0.5, 1.0, 1.5, 2.0, 3.0};
  out << frame << ',' << edges[band] << ',' << edges[band + 1] << ',' << points.size();
  if (points.empty()) {out << ",nan,nan,0,nan,nan,nan,nan,nan,nan,nan,0,0,0,nan\n"; return;}
  double mean = 0, m2 = 0;
  double min_x = points.front().x, max_x = min_x;
  size_t count = 0, above = 0, left = 0, right = 0;
  std::set<int> cells;
  std::vector<double> heights;
  for (const auto & p : points) {
    const double delta = p.z - mean;
    mean += delta / ++count;
    m2 += delta * (p.z - mean);
    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
    cells.insert(static_cast<int>(std::floor((p.x - edges[band]) / 0.1)));
    heights.push_back(p.z);
    if (p.z >= 0.03) {++above;}
    if (p.y > 0) {++left;} else {++right;}
  }
  std::sort(heights.begin(), heights.end());
  const auto quantile = [&heights](double q) {
      const double index = q * (heights.size() - 1);
      const size_t lo = static_cast<size_t>(index);
      const size_t hi = std::min(lo + 1, heights.size() - 1);
      return heights[lo] + (index - lo) * (heights[hi] - heights[lo]);
    };
  out << ',' << min_x << ',' << max_x << ',' << cells.size()
      << ',' << mean << ',' << std::sqrt(m2 / count)
      << ',' << heights.front() << ',' << quantile(0.01) << ',' << quantile(0.5)
      << ',' << quantile(0.99) << ',' << heights.back() << ',' << above
      << ',' << left << ',' << right << ',' << quantile(0.999) << '\n';
}

void write_plane(std::ostream & planes, const std::string & frame,
  const std::vector<Point> & points)
{
  planes << frame << ',' << points.size();
  if (points.size() < 3) {planes << ",nan,nan,nan,nan,nan,nan,nan,nan\n"; return;}
  double min_x = points.front().x, max_x = min_x;
  double min_y = points.front().y, max_y = min_y;
  cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
  cv::Mat heights(static_cast<int>(points.size()), 1, CV_64F), coefficients;
  for (size_t i = 0; i < points.size(); ++i) {
    design.at<double>(i, 0) = points[i].x;
    design.at<double>(i, 1) = points[i].y;
    design.at<double>(i, 2) = 1;
    heights.at<double>(i, 0) = points[i].z;
    min_x = std::min(min_x, points[i].x); max_x = std::max(max_x, points[i].x);
    min_y = std::min(min_y, points[i].y); max_y = std::max(max_y, points[i].y);
  }
  planes << ',' << min_x << ',' << max_x << ',' << min_y << ',' << max_y;
  if (max_x - min_x < 0.2 || max_y - min_y < 0.2 ||
    !cv::solve(design, heights, coefficients, cv::DECOMP_SVD))
  {planes << ",nan,nan,nan,nan\n"; return;}
  const double rms = cv::norm(design * coefficients - heights) / std::sqrt(points.size());
  planes << ',' << coefficients.at<double>(0) << ',' << coefficients.at<double>(1)
         << ',' << coefficients.at<double>(2) << ',' << rms << '\n';
}

int main(int argc, char ** argv)
{
  try {
    if (argc < 3) {
      throw std::invalid_argument("Usage: depth_floor_profile OUTPUT_DIR SNAPSHOT_CSV...");
    }
    const std::filesystem::path output(argv[1]);
    std::filesystem::create_directories(output);
    // 独立目录避免覆盖先前的原始结果。
    if (std::filesystem::exists(output / "distance_stats.csv")) {
      throw std::runtime_error("Output already exists; use a new directory");
    }
    std::set<std::filesystem::path> inputs;
    for (int i = 2; i < argc; ++i) {
      const auto path = std::filesystem::canonical(argv[i]);
      if (!inputs.insert(path).second) {throw std::runtime_error("Duplicate input frame");}
    }
    std::ofstream stats(output / "distance_stats.csv"), planes(output / "near_plane.csv");
    std::ofstream full_planes(output / "full_plane.csv");
    stats << std::setprecision(10)
          << "frame,min_band_x,max_band_x,count,min_x,max_x,x_cells_0p1m,mean_z_m,"
          << "std_z_m,min_z_m,p01_z_m,p50_z_m,p99_z_m,max_z_m,above_3cm,left_count,right_count,p999_z_m\n";
    planes << std::setprecision(10)
           << "frame,count,min_x,max_x,min_y,max_y,slope_x,slope_y,intercept_m,rms_m\n";
    full_planes << std::setprecision(10)
                << "frame,count,min_x,max_x,min_y,max_y,slope_x,slope_y,intercept_m,rms_m\n";
    std::array<std::vector<Point>, 4> pooled;
    std::array<size_t, 4> populated{};
    for (const auto & path : inputs) {
      std::ifstream input(path);
      std::string line;
      std::getline(input, line);
      if (line != "u,v,depth_m,base_x,base_y,base_z") {
        throw std::runtime_error("Expected depth_snapshot image CSV: " + path.string());
      }
      std::array<std::vector<Point>, 4> bands;
      std::vector<Point> near, full;
      while (std::getline(input, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        double u, v, depth;
        Point p;
        if (!(row >> u >> v >> depth >> p.x >> p.y >> p.z) ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {throw std::runtime_error("Invalid CSV row: " + path.string());}
        if (std::abs(p.y) > 0.15) {continue;}
        if (p.x >= 0.5 && p.x < 1.2) {near.push_back(p);}
        if (p.x < 0.5 || p.x >= 3.0) {continue;}
        full.push_back(p);
        const size_t band = p.x < 1.0 ? 0 : p.x < 1.5 ? 1 : p.x < 2.0 ? 2 : 3;
        bands[band].push_back(p);
      }
      if (input.bad()) {throw std::runtime_error("Read failed: " + path.string());}
      for (size_t b = 0; b < bands.size(); ++b) {
        write_stats(stats, path.string(), b, bands[b]);
        if (!bands[b].empty()) {++populated[b];}
        pooled[b].insert(pooled[b].end(), bands[b].begin(), bands[b].end());
      }
      write_plane(planes, path.string(), near);
      write_plane(full_planes, path.string(), full);
    }
    for (size_t b = 0; b < pooled.size(); ++b) {
      write_stats(stats, "ALL", b, pooled[b]);
      std::cout << "距离段" << b + 1 << "：有点帧=" << populated[b]
                << '/' << inputs.size() << "，点数=" << pooled[b].size() << '\n';
    }
    std::ofstream note(output / "说明.md");
    note << "# 地面分段统计的解释边界\n\n"
         << "只读原始快照，中央 |y|≤0.15 m；各段按 base_footprint 的 x 分组，"
         << "不是传感器斜距。不按 z 删除异常值。输入必须来自同一外参与固定姿态，"
         << "且现场确认空地；软件不能从 CSV 证明这些前提。\n\n"
         << "每帧及合并统计保留点数、实际 x 覆盖、10 cm 分箱、左右点数、"
         << "单点高度分布的总体标准差、P99/P99.9及极值（不是帧均值的标准差）。"
         << "无点记 nan；有点不等于连续地面覆盖。"
         << "合并点受空间/时间相关性影响，不能视为独立重复样本或只用 3σ 自动定阈值。\n\n"
         << "near_plane.csv 使用 OpenCV solve/DECOMP_SVD 拟合 z=ax+by+c，"
         << "范围 x=0.5～1.2 m；缺足够横纵跨度时输出 nan。它只反映近距残差，"
         << "full_plane.csv 同样拟合全部0.5～3 m输入点，近距密集点会主导结果，\n"
         << "必须同时检查远距分段均值与稀疏覆盖，不能仅凭整段RMS判定线性或深度偏差。\n"
         << "不验收远处、yaw、水平平移、绝对 roll 或实体舵机平视。"
         << "来源：https://docs.opencv.org/4.10.0/d2/de8/group__core__array.html"
         << "；复用本项目已有 OpenCV 接口，不复制上游代码。\n";
    if (!stats.good() || !planes.good() || !full_planes.good() || !note.good()) {
      throw std::runtime_error("Output write failed");
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
