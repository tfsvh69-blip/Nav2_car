#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 手动启动的只读小目标取证：记录原始/过滤点云同一车体ROI，不发布或控制硬件。
// 点数阈值只用于比较，必须结合空场基线、现场位置和独立健康窗口解释。
class DepthTargetWindow : public rclcpp::Node
{
  using Steady = std::chrono::steady_clock;
public:
  DepthTargetWindow() : Node("depth_target_window"), buffer_(get_clock()), listener_(buffer_)
  {
    output_ = declare_parameter<std::string>("output_dir", "/tmp/depth_target_window");
    duration_ = declare_parameter("duration_s", 60.0);
    center_x_ = declare_parameter("center_x", 0.65);
    center_y_ = declare_parameter("center_y", 0.0175);
    half_width_ = declare_parameter("half_width", 0.035);
    min_z_ = declare_parameter("min_z", 0.01);
    max_z_ = declare_parameter("max_z", 0.04);
    max_age_ = declare_parameter("max_age_s", 0.5);
    if (!std::isfinite(duration_) || duration_ < 10 || duration_ > 600 ||
      !std::isfinite(center_x_) || !std::isfinite(center_y_) ||
      !std::isfinite(half_width_) || half_width_ <= 0 || half_width_ > 0.2 ||
      !std::isfinite(min_z_) || !std::isfinite(max_z_) || min_z_ >= max_z_ ||
      !std::isfinite(max_age_) || max_age_ <= 0)
    {throw std::invalid_argument("Invalid target ROI or duration");}
    std::filesystem::create_directories(output_);
    if (!std::filesystem::is_empty(output_)) {
      throw std::invalid_argument("Use a new output directory");
    }
    file_.open(output_ + "/frames.csv");
    file_ << std::setprecision(12)
          << "phase,source,receive_s,stamp_sec,stamp_ns,age_s,gap_s,usable,roi_points,"
          << "min_z,max_z,processing_ms\n";
    for (size_t i = 0; i < streams_.size(); ++i) {
      subscriptions_[i] = create_subscription<sensor_msgs::msg::PointCloud2>(
        i == 0 ? "/camera/depth/points" : "/camera/navigation/obstacles",
        rclcpp::SensorDataQoS().keep_last(1),
        [this, i](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {capture(i, *msg);});
    }
    initial_ = started_ = Steady::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (!armed_ && seconds(Steady::now(), initial_) >= 15) {
          finish();
          rclcpp::shutdown();
        } else if (armed_ && seconds(Steady::now(), started_) >= duration_) {
          complete_ = true;
          finish();
          rclcpp::shutdown();
        }
      });
  }

  bool finish()
  {
    if (finished_) {return valid_;}
    finished_ = true;
    const double elapsed = seconds(Steady::now(), started_);
    file_.flush();
    valid_ = complete_ && file_.good();
    std::ofstream out(output_ + "/summary.txt");
    out << std::setprecision(12) << "completed=" << complete_ << "\nduration_s=" << elapsed
        << "\ncenter_x=" << center_x_ << "\ncenter_y=" << center_y_
        << "\nhalf_width=" << half_width_ << "\nmin_z=" << min_z_
        << "\nmax_z=" << max_z_ << '\n';
    for (size_t i = 0; i < streams_.size(); ++i) {
      const auto & s = streams_[i];
      const double tail = s.seen ? seconds(Steady::now(), s.last) : elapsed;
      const double first_gap = s.seen ? s.first_delay : elapsed;
      const double peak_gap = std::max({s.peak_gap, tail, first_gap});
      valid_ = valid_ && s.frames > 0 && s.bad == 0 && s.frames / elapsed >= 5 &&
        peak_gap <= max_age_;
      const std::string prefix = i == 0 ? "raw_" : "filtered_";
      out << prefix << "frames=" << s.frames << '\n'
          << prefix << "unusable=" << s.bad << '\n'
          << prefix << "hz=" << s.frames / elapsed << '\n'
          << prefix << "max_gap_s=" << peak_gap << '\n'
          << prefix << "frames_ge1=" << s.ge1 << '\n'
          << prefix << "frames_ge3=" << s.ge3 << '\n'
          << prefix << "fraction_ge3=" << (s.frames ? double(s.ge3) / s.frames : 0) << '\n'
          << prefix << "longest_without_ge3_s="
          << std::max(s.longest_miss, seconds(Steady::now(), s.last_hit)) << '\n';
    }
    out << "measurement_valid=" << valid_
        << "\nlimitation=ROI counts only; require empty baseline, physical placement and health window\n";
    valid_ = valid_ && out.good();
    return valid_;
  }

private:
  struct Stream {
    bool seen{false};
    size_t frames{0}, bad{0}, ge1{0}, ge3{0};
    double first_delay{0}, peak_gap{0}, longest_miss{0};
    Steady::time_point last{}, last_hit{Steady::now()};
  };
  static double seconds(Steady::time_point a, Steady::time_point b)
  {return std::chrono::duration<double>(a - b).count();}

  void capture(size_t index, const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto received = Steady::now();
    const double age = (now() - rclcpp::Time(msg.header.stamp)).seconds();
    bool usable = std::isfinite(age) && age >= -0.1 && age <= max_age_ &&
      !msg.header.frame_id.empty() && !msg.is_bigendian && msg.height == 1 &&
      msg.point_step >= 12 && msg.row_step == size_t(msg.width) * msg.point_step &&
      msg.data.size() == size_t(msg.height) * msg.row_step;
    for (const auto * name : {"x", "y", "z"}) {
      const auto field = std::find_if(msg.fields.begin(), msg.fields.end(),
        [name](const auto & f) {return f.name == name;});
      usable = usable && field != msg.fields.end() &&
        field->datatype == sensor_msgs::msg::PointField::FLOAT32 && field->count == 1 &&
        size_t(field->offset) + sizeof(float) <= msg.point_step;
    }
    size_t count = 0;
    double low = NAN, high = NAN;
    try {
      if (usable) {
        tf2::Transform transform;
        tf2::fromMsg(buffer_.lookupTransform("base_footprint", msg.header.frame_id,
          rclcpp::Time(msg.header.stamp)).transform, transform);
        if (msg.width > 0) {
          sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
          for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
            const auto p = transform * tf2::Vector3(*x, *y, *z);
            if (std::abs(p.x() - center_x_) <= half_width_ &&
              std::abs(p.y() - center_y_) <= half_width_ &&
              p.z() >= min_z_ && p.z() <= max_z_)
            {
              if (!count) {low = high = p.z();}
              low = std::min(low, p.z()); high = std::max(high, p.z()); ++count;
            }
          }
        }
      }
    } catch (const std::exception &) {usable = false; count = 0; low = high = NAN;}
    if (!armed_) {
      ready_[index] = usable ? received : Steady::time_point{};
      if (ready_[0] != Steady::time_point{} && ready_[1] != Steady::time_point{} &&
        seconds(received, ready_[0]) <= max_age_ && seconds(received, ready_[1]) <= max_age_)
      {
        armed_ = true;
        started_ = received;
        streams_ = {};
        for (auto & stream : streams_) {stream.last_hit = received;}
      }
    }
    double gap = 0;
    if (armed_) {
      auto & s = streams_[index];
      gap = s.seen ? seconds(received, s.last) : seconds(received, started_);
      if (!s.seen) {s.first_delay = gap;}
      s.seen = true; s.last = received; s.peak_gap = std::max(s.peak_gap, gap); ++s.frames;
      if (!usable) {++s.bad;}
      if (usable && count) {++s.ge1;}
      if (usable && count >= 3) {
        ++s.ge3;
        s.longest_miss = std::max(s.longest_miss, seconds(received, s.last_hit));
        s.last_hit = received;
      }
    }
    file_ << (armed_ ? "RUN" : "WARMUP") << ','
          << (index == 0 ? "raw" : "filtered") << ',' << now().seconds() << ','
          << msg.header.stamp.sec << ',' << msg.header.stamp.nanosec << ',' << age << ','
          << gap << ',' << usable << ',' << count << ',' << low << ',' << high << ','
          << seconds(Steady::now(), received) * 1000 << '\n';
  }

  std::string output_;
  double duration_{60}, center_x_{0}, center_y_{0}, half_width_{0}, min_z_{0}, max_z_{0}, max_age_{0};
  bool armed_{false}, complete_{false}, finished_{false}, valid_{false};
  Steady::time_point initial_, started_;
  std::array<Steady::time_point, 2> ready_{};
  std::ofstream file_;
  std::array<Stream, 2> streams_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr, 2> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<DepthTargetWindow>();
    rclcpp::spin(node);
    result = node->finish() ? 0 : 1;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_target_window"), "%s", error.what());
  }
  rclcpp::shutdown();
  return result;
}
