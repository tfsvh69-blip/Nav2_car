#include <opencv2/imgcodecs.hpp>
#include <karto_sdk/Mapper.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

// 离线读取既有 SLAM 数据；不启动 ROS、不打开串口、不修改原工程。
int main(int argc, char **argv) try {
  if (argc != 3) return 2;
  const std::filesystem::path bundle(argv[1]), output(argv[2]);
  if (!std::filesystem::is_regular_file(bundle / "graph.data")) return 2;
  std::filesystem::create_directories(output);
  const auto grid = cv::imread((bundle / "map.pgm").string(), cv::IMREAD_GRAYSCALE);
  if (grid.empty() || !cv::imwrite((output / "map.png").string(), grid)) return 1;
  std::cout << "map_pixels=" << grid.cols << 'x' << grid.rows << std::endl;
  // 同一序列化工程的 Mapper/Dataset 各自持有传感器；单次只读进程交由 OS 回收，
  // 避免此版本两份反序列化对象析构时重复注销同名传感器。
  auto &mapper = *new karto::Mapper;
  mapper.LoadFromFile((bundle / "graph.posegraph").string());
  auto &dataset = *new karto::Dataset;
  dataset.LoadFromFile((bundle / "graph.data").string());
  double maximum = 0;
  for (auto *object : dataset.GetLasers()) {
    if (auto *laser = dynamic_cast<karto::LaserRangeFinder *>(object)) {
      maximum = laser->GetMaximumRange();
      std::cout << "laser_threshold_m=" << laser->GetRangeThreshold()
                << " declared_max_m=" << maximum << '\n';
    }
  }
  std::ofstream csv(output / "scans.csv");
  csv << "id,time,odom_x,odom_y,odom_yaw,corrected_x,corrected_y,corrected_yaw,near3,near6,valid,max_range,heading_correction_deg\n";
  std::vector<int> near3, near6;
  size_t scans = 0, raw = 0, within3 = 0, within6 = 0;
  double first = 0, last = 0, biggest_step = 0, previous = 0;
  karto::Pose2 previous_odom, previous_corrected;
  double odom_signed = 0, corrected_signed = 0, odom_abs = 0, corrected_abs = 0;
  double odom_distance = 0, corrected_distance = 0;
  int biggest_id = -1;
  for (const auto &[id, object] : dataset.GetData()) {
    auto *scan = dynamic_cast<karto::LocalizedRangeScan *>(object);
    if (!scan) continue;
    int count3 = 0, count6 = 0, valid = 0;
    double farthest = 0;
    for (const double r : scan->GetRangeReadingsVector()) {
      if (!std::isfinite(r) || r < 0.1 || r >= maximum) continue;
      ++valid;
      if (r <= 3) ++count3;
      if (r <= 6) ++count6;
      farthest = std::max(farthest, r);
    }
    const auto &odom = scan->GetOdometricPose();
    const auto &corrected = scan->GetCorrectedPose();
    const double correction = std::remainder(corrected.GetHeading() - odom.GetHeading(), 2 * M_PI);
    if (!scans) first = scan->GetTime();
    else {
      const double odom_turn = std::remainder(odom.GetHeading() - previous_odom.GetHeading(), 2 * M_PI);
      const double corrected_turn = std::remainder(corrected.GetHeading() - previous_corrected.GetHeading(), 2 * M_PI);
      odom_signed += odom_turn; corrected_signed += corrected_turn;
      odom_abs += std::abs(odom_turn); corrected_abs += std::abs(corrected_turn);
      odom_distance += std::hypot(odom.GetX() - previous_odom.GetX(), odom.GetY() - previous_odom.GetY());
      corrected_distance += std::hypot(corrected.GetX() - previous_corrected.GetX(), corrected.GetY() - previous_corrected.GetY());
      const double step = std::abs(std::remainder(correction - previous, 2 * M_PI));
      if (step > biggest_step) {biggest_step = step; biggest_id = id;}
    }
    last = scan->GetTime(); previous = correction;
    previous_odom = odom; previous_corrected = corrected;
    ++scans; raw += valid; within3 += count3; within6 += count6;
    near3.push_back(count3); near6.push_back(count6);
    csv << std::setprecision(15) << id << ',' << scan->GetTime() << ','
        << odom.GetX() << ',' << odom.GetY() << ',' << odom.GetHeading() << ','
        << corrected.GetX() << ',' << corrected.GetY() << ',' << corrected.GetHeading() << ','
        << count3 << ',' << count6 << ',' << valid << ',' << farthest << ','
        << correction * 180 / M_PI << '\n';
  }
  if (!scans || !raw) return 1;
  auto quantile = [](std::vector<int> values, double fraction) {
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>((values.size() - 1) * fraction)];
  };
  std::cout << "scans=" << scans << " first=" << std::setprecision(15) << first
            << " last=" << last << " duration_s=" << last - first << '\n'
            << "valid_beams=" << raw << " within3=" << within3 << " within6=" << within6 << '\n'
            << "near3_min/p10/median=" << quantile(near3, 0) << '/' << quantile(near3, .1) << '/' << quantile(near3, .5) << '\n'
            << "near6_min/p10/median=" << quantile(near6, 0) << '/' << quantile(near6, .1) << '/' << quantile(near6, .5) << '\n'
            << "odom/corrected_signed_turn_deg=" << odom_signed * 180 / M_PI << '/' << corrected_signed * 180 / M_PI << '\n'
            << "odom/corrected_abs_turn_deg=" << odom_abs * 180 / M_PI << '/' << corrected_abs * 180 / M_PI << '\n'
            << "odom/corrected_distance_m=" << odom_distance << '/' << corrected_distance << '\n'
            << "max_heading_correction_step_deg=" << biggest_step * 180 / M_PI << " at_id=" << biggest_id << '\n';
  return csv.good() ? 0 : 1;
} catch (const karto::Exception &error) {
  std::cerr << error.GetErrorMessage() << '\n';
  return 1;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
