#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// 只读长测：保存时间信息和诊断，不保存大点云，不发布或控制任何硬件。
class DepthHealthWindow : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
public:
  DepthHealthWindow() : Node("depth_health_window")
  {
    output_ = declare_parameter("output_dir", std::string("/tmp/depth_health_window"));
    duration_ = declare_parameter("duration_s", 600.0);
    min_hz_ = declare_parameter("min_hz", 5.0);
    max_gap_ = declare_parameter("max_gap_s", 0.50);
    if (!std::isfinite(duration_) || duration_ < 10 || duration_ > 3600 ||
      !std::isfinite(min_hz_) || min_hz_ <= 0 ||
      !std::isfinite(max_gap_) || max_gap_ <= 0)
    {throw std::invalid_argument("Invalid health window limits");}
    std::filesystem::create_directories(output_);
    if (!std::filesystem::is_empty(output_)) {
      throw std::invalid_argument("Use a new output directory");
    }
    raw_file_.open(output_ + "/raw_frames.csv");
    diag_file_.open(output_ + "/diagnostics.csv");
    rate_file_.open(output_ + "/rates.csv");
    raw_file_ << "phase,receive_wall_s,stamp_sec,stamp_ns,stamp_age_s,gap_s,points\n";
    diag_file_ << "phase,receive_wall_s,stamp_sec,stamp_ns,level,reason,raw_age_s\n";
    rate_file_ << "elapsed_s,period_s,frames,hz\n";
    raw_file_ << std::setprecision(12); diag_file_ << std::setprecision(12);
    rate_file_ << std::setprecision(12);
    initial_ = Clock::now();
    raw_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        const auto now = Clock::now();
        const double gap = raw_seen_ ? seconds(now, last_raw_) : 0.0;
        const double age = (this->now() - rclcpp::Time(msg->header.stamp)).seconds();
        raw_file_ << (armed_ ? "RUN" : "WARMUP") << ',' << this->now().seconds()
                  << ',' << msg->header.stamp.sec << ',' << msg->header.stamp.nanosec
                  << ',' << age << ',' << gap << ','
                  << static_cast<size_t>(msg->width) * msg->height << '\n';
        if (armed_) {
          ++frames_; ++period_frames_;
          peak_gap_ = std::max(peak_gap_, gap);
          peak_age_ = std::max(peak_age_, age);
          if (gap > max_gap_ || !std::isfinite(age) || age < -0.1 || age > max_gap_) {
            failed_ = true;
          }
        }
        raw_seen_ = true; last_raw_ = now;
      });
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/navigation/diagnostics", rclcpp::QoS(100),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.name != "D435DepthFilter") {continue;}
          std::string reason = status.message, raw_age;
          for (const auto & value : status.values) {
            if (value.key == "reason_code") {reason = value.value;}
            if (value.key == "raw_age_s") {raw_age = value.value;}
          }
          diag_file_ << (armed_ ? "RUN" : "WARMUP") << ',' << this->now().seconds()
                     << ',' << msg->header.stamp.sec << ',' << msg->header.stamp.nanosec
                     << ',' << static_cast<int>(status.level) << ',' << reason
                     << ',' << raw_age << '\n';
          healthy_ = status.level == 0 && reason == "NONE";
          diag_seen_ = true; last_diag_ = Clock::now();
          if (armed_) {
            ++diagnostics_;
            if (!healthy_) {++bad_diagnostics_; failed_ = true;}
          }
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {tick();});
  }

  bool finish()
  {
    if (finished_) {return passed_;}
    finished_ = true;
    const double elapsed = armed_ ? seconds(Clock::now(), started_) : 0;
    const double hz = elapsed > 0 ? frames_ / elapsed : 0;
    raw_file_.flush(); diag_file_.flush(); rate_file_.flush();
    passed_ = complete_ && !failed_ && diagnostics_ > 0 && hz >= min_hz_ &&
      raw_file_.good() && diag_file_.good() && rate_file_.good();
    std::ofstream summary(output_ + "/summary.txt");
    summary << std::setprecision(12) << "verdict=" << (passed_ ? "PASS" : "FAIL")
            << "\ncompleted=" << complete_ << "\nduration_s=" << elapsed
            << "\nframes=" << frames_ << "\nhz=" << hz
            << "\ndiagnostics=" << diagnostics_ << "\nbad_diagnostics=" << bad_diagnostics_
            << "\nmax_gap_s=" << peak_gap_ << "\nmax_stamp_age_s=" << peak_age_
            << "\nmin_observed_period_hz=" << min_period_hz_
            << "\nlimits: min_hz=" << min_hz_ << " max_gap_s=" << max_gap_ << '\n';
    raw_file_.flush(); diag_file_.flush(); rate_file_.flush();
    if (!summary.good() || !raw_file_.good() || !diag_file_.good() || !rate_file_.good()) {
      passed_ = false;
    }
    RCLCPP_INFO(get_logger(), "健康窗口 %s：%.1fs，%zu帧，%.3fHz，异常诊断%zu，最大间隔%.3fs",
      passed_ ? "PASS" : "FAIL", elapsed, frames_, hz, bad_diagnostics_, peak_gap_);
    return passed_;
  }

private:
  static double seconds(Time a, Time b) {return std::chrono::duration<double>(a - b).count();}
  void tick()
  {
    const auto now = Clock::now();
    const bool fresh = raw_seen_ && diag_seen_ && healthy_ &&
      seconds(now, last_raw_) <= max_gap_ && seconds(now, last_diag_) <= 0.8;
    if (!armed_) {
      if (!fresh) {stable_ = false;}
      else if (!stable_) {stable_ = true; stable_since_ = now;}
      else if (seconds(now, stable_since_) >= 3.0) {
        armed_ = true; started_ = period_start_ = now;
        RCLCPP_INFO(get_logger(), "连续健康3s后开始%.0fs验收，发生异常不重置计时", duration_);
      }
      if (seconds(now, initial_) > 45 && !armed_) {
        failed_ = true; finish(); rclcpp::shutdown();
      }
      return;
    }
    if (!fresh) {failed_ = true;}
    peak_gap_ = std::max(peak_gap_, seconds(now, last_raw_));
    const double elapsed = seconds(now, started_);
    const double period = seconds(now, period_start_);
    if (period >= 60 || elapsed >= duration_) {
      const double hz = period_frames_ / period;
      rate_file_ << elapsed << ',' << period << ',' << period_frames_ << ',' << hz << '\n';
      if (period >= 10) {
        min_period_hz_ = std::min(min_period_hz_, hz);
        if (hz < min_hz_) {failed_ = true;}
      }
      RCLCPP_INFO(get_logger(), "%.0f/%.0fs：最近%.1fs %.3fHz，异常诊断%zu",
        elapsed, duration_, period, hz, bad_diagnostics_);
      period_frames_ = 0; period_start_ = now;
      raw_file_.flush(); diag_file_.flush(); rate_file_.flush();
    }
    if (elapsed >= duration_) {complete_ = true; finish(); rclcpp::shutdown();}
  }

  std::string output_;
  double duration_{600}, min_hz_{5}, max_gap_{0.5};
  double peak_gap_{0}, peak_age_{0}, min_period_hz_{1e9};
  size_t frames_{0}, period_frames_{0}, diagnostics_{0}, bad_diagnostics_{0};
  bool raw_seen_{false}, diag_seen_{false}, healthy_{false}, stable_{false};
  bool armed_{false}, failed_{false}, complete_{false}, finished_{false}, passed_{false};
  Time initial_, stable_since_, started_, last_raw_, last_diag_, period_start_;
  std::ofstream raw_file_, diag_file_, rate_file_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool passed = false;
  try {
    auto node = std::make_shared<DepthHealthWindow>();
    rclcpp::spin(node);
    passed = node->finish();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_health_window"), "%s", error.what());
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return passed ? 0 : 1;
}
