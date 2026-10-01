#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// 只读诊断：原生深度像素及其在车体坐标中的高度，不发布任何 ROS 消息。
class DepthSnapshot : public rclcpp::Node
{
public:
  DepthSnapshot() : Node("depth_snapshot"), buffer_(get_clock()), listener_(buffer_)
  {
    output_ = declare_parameter("output_dir", std::string("/tmp/depth_snapshot"));
    require_color_ = declare_parameter("require_color", true);
    std::filesystem::create_directories(output_);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/depth/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {info_ = msg;});
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (color_saved_ || (msg->encoding != "rgb8" && msg->encoding != "bgr8") ||
          msg->data.size() < static_cast<size_t>(msg->height) * msg->step) {return;}
        cv::Mat source(msg->height, msg->width, CV_8UC3,
          const_cast<uint8_t *>(msg->data.data()), msg->step);
        cv::Mat image;
        if (msg->encoding == "rgb8") {cv::cvtColor(source, image, cv::COLOR_RGB2BGR);}
        else {image = source.clone();}
        color_saved_ = cv::imwrite(output_ + "/color.png", image);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {capture(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (cloud_saved_) {return;}
        tf2::Transform transform;
        try {
          tf2::fromMsg(buffer_.lookupTransform("base_footprint", msg->header.frame_id,
            rclcpp::Time(msg->header.stamp), rclcpp::Duration::from_seconds(0.1)).transform,
            transform);
          sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
          size_t count = 0, obstacles = 0;
          double highest = -1e9;
          std::ofstream data(output_ + "/sdk_central_points.csv");
          data << "base_x,base_y,base_z\n";
          for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z <= 0) {
              continue;
            }
            const auto p = transform * tf2::Vector3(*x, *y, *z);
            if (p.x() < 0.3 || p.x() > 1.2 || std::abs(p.y()) > 0.15) {continue;}
            ++count;
            highest = std::max(highest, p.z());
            if (p.z() >= 0.03 && p.z() <= 0.26) {++obstacles;}
            data << p.x() << ',' << p.y() << ',' << p.z() << '\n';
          }
          cloud_saved_ = data.good();
          if (!cloud_saved_) {failed_ = true;}
          RCLCPP_INFO(get_logger(), "SDK 原始点云中央点=%zu 高度3~26cm=%zu 最高z=%.6f m",
            count, obstacles, highest);
        } catch (const tf2::TransformException &) {return;}
        catch (const std::runtime_error & error) {
          RCLCPP_ERROR(get_logger(), "原始点云读取失败：%s", error.what());
          failed_ = true;
          rclcpp::shutdown();
        }
      });
    timeout_ = create_wall_timer(std::chrono::seconds(15), [this]() {
      RCLCPP_ERROR(get_logger(), "采样超时：深度帧=%d，彩色=%d", frames_, color_saved_);
      failed_ = true;
      rclcpp::shutdown();
    });
  }

  bool failed() const {return failed_;}

private:
  void capture(const sensor_msgs::msg::Image & msg)
  {
    if (frames_ >= 3) {
      if ((!require_color_ || color_saved_) && cloud_saved_) {rclcpp::shutdown();}
      return;
    }
    if (!info_ || msg.encoding != "16UC1" || msg.is_bigendian ||
      info_->width != msg.width || info_->height != msg.height ||
      info_->header.frame_id != msg.header.frame_id || info_->k[0] <= 0 || info_->k[4] <= 0 ||
      msg.step < msg.width * 2 || msg.data.size() < static_cast<size_t>(msg.height) * msg.step)
    {return;}
    tf2::Transform base_from_depth;
    try {
      tf2::fromMsg(buffer_.lookupTransform("base_footprint", msg.header.frame_id,
        rclcpp::Time(msg.header.stamp), rclcpp::Duration::from_seconds(0.1)).transform,
        base_from_depth);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "等待 TF：%s", error.what());
      return;
    }
    const std::string prefix = output_ + "/sample_" + std::to_string(++frames_);
    if (frames_ == 1) {
      std::ofstream pose(output_ + "/base_from_depth.txt");
      const auto & t = base_from_depth.getOrigin();
      const auto q = base_from_depth.getRotation();
      pose << "translation " << t.x() << ' ' << t.y() << ' ' << t.z()
           << "\nquaternion " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
           << '\n';
      if (!pose.good()) {failed_ = true;}
    }
    cv::Mat depth_value(msg.height, msg.width, CV_8UC1, cv::Scalar(0));
    cv::Mat height_value(msg.height, msg.width, CV_8UC1, cv::Scalar(0));
    cv::Mat invalid(msg.height, msg.width, CV_8UC1, cv::Scalar(255));
    cv::Mat depth_mm(msg.height, msg.width, CV_16UC1, cv::Scalar(0));
    std::ofstream points(prefix + "_central_points.csv");
    points << "u,v,depth_m,base_x,base_y,base_z\n";
    size_t valid = 0, central = 0, low = 0, ground = 0, high = 0;
    size_t fit_count = 0;
    double sum_x = 0.0, sum_z = 0.0, sum_xx = 0.0, sum_xz = 0.0, sum_zz = 0.0;
    for (uint32_t v = 0; v < msg.height; ++v) {
      for (uint32_t u = 0; u < msg.width; ++u) {
        uint16_t mm;
        std::memcpy(&mm, &msg.data[static_cast<size_t>(v) * msg.step + u * 2], 2);
        depth_mm.at<uint16_t>(v, u) = mm;
        if (mm == 0) {continue;}
        const double depth = mm * 0.001;
        const tf2::Vector3 p = base_from_depth * tf2::Vector3(
          (u - info_->k[2]) * depth / info_->k[0],
          (v - info_->k[5]) * depth / info_->k[4], depth);
        ++valid;
        invalid.at<uint8_t>(v, u) = 0;
        depth_value.at<uint8_t>(v, u) = cv::saturate_cast<uint8_t>(depth * 255 / 2.0);
        height_value.at<uint8_t>(v, u) = cv::saturate_cast<uint8_t>((p.z() + 0.1) * 255 / 0.5);
        if (p.x() >= 0.3 && p.x() <= 1.2 && std::abs(p.y()) <= 0.15) {
          ++central;
          if (p.z() >= 0.03 && p.z() <= 0.26) {++low;}
          else if (p.z() < 0.03) {++ground;}
          else {++high;}
          points << u << ',' << v << ',' << depth << ',' << p.x() << ',' << p.y()
                 << ',' << p.z() << '\n';
          if (v >= 360 && v <= 450 && p.x() >= 0.4) {
            ++fit_count;
            sum_x += p.x();
            sum_z += p.z();
            sum_xx += p.x() * p.x();
            sum_xz += p.x() * p.z();
            sum_zz += p.z() * p.z();
          }
        }
      }
    }
    cv::Mat depth_color, height_color;
    cv::applyColorMap(depth_value, depth_color, cv::COLORMAP_TURBO);
    cv::applyColorMap(height_value, height_color, cv::COLORMAP_TURBO);
    depth_color.setTo(cv::Scalar(0, 0, 0), invalid);
    height_color.setTo(cv::Scalar(0, 0, 0), invalid);
    if (!cv::imwrite(prefix + "_depth.png", depth_color) ||
      !cv::imwrite(prefix + "_height.png", height_color) ||
      !cv::imwrite(prefix + "_depth_mm.png", depth_mm) || !points.good()) {failed_ = true;}
    RCLCPP_INFO(get_logger(),
      "帧%d 有效=%zu 中央区域=%zu 高度3~26cm=%zu 低于3cm=%zu 高于26cm=%zu",
      frames_, valid, central, low, ground, high);
    const double denominator = fit_count * sum_xx - sum_x * sum_x;
    if (fit_count > 2 && denominator > 1e-9) {
      const double slope = (fit_count * sum_xz - sum_x * sum_z) / denominator;
      const double intercept = (sum_z - slope * sum_x) / fit_count;
      const double squared_error = sum_zz + slope * slope * sum_xx +
        fit_count * intercept * intercept - 2 * slope * sum_xz -
        2 * intercept * sum_z + 2 * slope * intercept * sum_x;
      const double rms = std::sqrt(std::max(0.0, squared_error / fit_count));
      std::ofstream fit(prefix + "_floor_fit.txt");
      fit << "ROI: x=0.4..1.2m, |y|<=0.15m, image v=360..450; "
             "only interpret as floor after checking the scene\n"
          << "count=" << fit_count << " slope=" << slope <<
        " intercept_m=" << intercept << " rms_m=" << rms << '\n';
      if (!fit.good()) {failed_ = true;}
      RCLCPP_INFO(get_logger(),
        "帧%d 候选地面拟合 n=%zu slope=%.5f intercept=%.4fm rms=%.4fm",
        frames_, fit_count, slope, intercept, rms);
    }
  }

  std::string output_;
  bool color_saved_{false}, cloud_saved_{false}, failed_{false};
  bool require_color_{true};
  int frames_{0};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timeout_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthSnapshot>();
  rclcpp::spin(node);
  const bool failed = node->failed();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
