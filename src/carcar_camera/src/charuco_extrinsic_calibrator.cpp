#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numeric>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "carcar_camera/extrinsic_math.hpp"

namespace {
using Steady = std::chrono::steady_clock;

struct Detection
{
  tf2::Transform camera_from_board;
  rclcpp::Time stamp;
  double reprojection_rms{INFINITY};
  int corners{0};
};

struct Candidate
{
  tf2::Transform base_from_camera_optical;
  std::string name;
  bool validation{false};
  double reprojection_rms{INFINITY};
};

struct MeasurementRecord
{
  std::string name;
  std::string role;
  std::string method;
  tf2::Transform base_from_board;
  std::vector<double> reference_points;
};

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

}  // namespace

class CharucoExtrinsicCalibrator : public rclcpp::Node
{
public:
  CharucoExtrinsicCalibrator()
  : Node("charuco_extrinsic_calibrator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    squares_x_ = declare_parameter("squares_x", 7);
    squares_y_ = declare_parameter("squares_y", 5);
    square_length_m_ = declare_parameter("square_length_m", 0.030);
    marker_length_m_ = declare_parameter("marker_length_m", 0.022);
    dictionary_id_ = declare_parameter("dictionary_id", 0);  // DICT_4X4_50
    min_corners_ = declare_parameter("min_corners", 8);
    capture_frames_ = declare_parameter("capture_frames", 10);
    capture_window_s_ = declare_parameter("capture_window_s", 2.0);
    max_reprojection_rms_px_ = declare_parameter("max_reprojection_rms_px", 1.0);
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
    parent_frame_ = declare_parameter("parent_frame", std::string("base_link"));
    camera_link_frame_ = declare_parameter("camera_link_frame", std::string("camera_link"));
    image_topic_ = declare_parameter(
      "image_topic", std::string("/camera/camera/color/image_raw"));
    camera_info_topic_ = declare_parameter(
      "camera_info_topic", std::string("/camera/camera/color/camera_info"));
    output_file_ = declare_parameter(
      "output_file", std::string("/tmp/carcar_camera_extrinsics_candidate.yaml"));
    declare_parameter("sample_name", std::string("pose_1"));
    declare_parameter("sample_role", std::string("calibration"));
    declare_parameter("measurement_mode", std::string("three_points"));
    declare_parameter("board_reference_points", std::vector<double>(9, 0.0));
    max_board_size_error_m_ = declare_parameter("max_board_size_error_m", 0.010);
    servo_target_angle_deg_ = declare_parameter("servo_target_angle_deg", 140.5);
    measurement_quality_ = declare_parameter(
      "measurement_quality", std::string("ruler_coarse"));
    declare_parameter("base_to_board_xyz", std::vector<double>{0.5, 0.0, 0.2});
    declare_parameter("base_to_board_rpy", std::vector<double>{0.0, 0.0, 0.0});

    if (squares_x_ < 3 || squares_y_ < 3 || square_length_m_ <= 0.0 ||
      marker_length_m_ <= 0.0 || marker_length_m_ >= square_length_m_ ||
      min_corners_ < 4 || capture_frames_ < 1 || capture_window_s_ <= 0.0 ||
      max_board_size_error_m_ <= 0.0) {
      throw std::invalid_argument("ChArUco 板参数无效");
    }
    if (!std::isfinite(servo_target_angle_deg_) || servo_target_angle_deg_ < 85.0 ||
      servo_target_angle_deg_ > 160.0 || measurement_quality_ != "ruler_coarse") {
      throw std::invalid_argument("标定元数据无效：目标角度须在 85～160°，当前只支持尺量粗标定");
    }
    if (dictionary_id_ != cv::aruco::DICT_4X4_50) {
      throw std::invalid_argument("本项目当前只接受已确认的 DICT_4X4_50（dictionary_id=0）");
    }
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(dictionary_id_));
    board_ = cv::makePtr<cv::aruco::CharucoBoard>(
      cv::Size(squares_x_, squares_y_), static_cast<float>(square_length_m_),
      static_cast<float>(marker_length_m_), *dictionary_);
    detector_parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();
    detector_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, qos,
      std::bind(&CharucoExtrinsicCalibrator::on_info, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      std::bind(&CharucoExtrinsicCalibrator::on_image, this, std::placeholders::_1));
    capture_service_ = create_service<std_srvs::srv::Trigger>(
      "/camera/calibration/capture_pose",
      std::bind(
        &CharucoExtrinsicCalibrator::capture_pose, this,
        std::placeholders::_1, std::placeholders::_2));
    solve_service_ = create_service<std_srvs::srv::Trigger>(
      "/camera/calibration/solve",
      std::bind(
        &CharucoExtrinsicCalibrator::solve, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/camera/calibration/reset",
      std::bind(
        &CharucoExtrinsicCalibrator::reset, this,
        std::placeholders::_1, std::placeholders::_2));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/camera/calibration/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&CharucoExtrinsicCalibrator::diagnostics, this));
  }

private:
  void on_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    for (size_t index = 0; index < msg->k.size(); ++index) {
      camera_matrix_.at<double>(index / 3, index % 3) = msg->k[index];
    }
    distortion_ = cv::Mat::zeros(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t index = 0; index < msg->d.size(); ++index) {
      distortion_.at<double>(0, static_cast<int>(index)) = msg->d[index];
    }
    have_info_ = camera_matrix_.at<double>(0, 0) > 0.0 &&
      camera_matrix_.at<double>(1, 1) > 0.0;
  }

  bool image_to_bgr(const sensor_msgs::msg::Image & msg, cv::Mat & output)
  {
    if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
      return false;
    }
    if (msg.encoding == "bgr8") {
      output = cv::Mat(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
        const_cast<unsigned char *>(msg.data.data()), msg.step).clone();
      return true;
    }
    if (msg.encoding == "rgb8") {
      cv::Mat rgb(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(rgb, output, cv::COLOR_RGB2BGR);
      return true;
    }
    if (msg.encoding == "mono8") {
      cv::Mat mono(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC1,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(mono, output, cv::COLOR_GRAY2BGR);
      return true;
    }
    last_error_ = "UNSUPPORTED_IMAGE_ENCODING: " + msg.encoding;
    return false;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat camera_matrix;
    cv::Mat distortion;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_image_received_ = Steady::now();
      optical_frame_ = msg->header.frame_id;
      if (!have_info_) {
        last_error_ = "CAMERA_INFO_MISSING";
        return;
      }
      camera_matrix = camera_matrix_.clone();
      distortion = distortion_.clone();
    }

    cv::Mat image;
    if (!image_to_bgr(*msg, image)) {
      return;
    }
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(
      image, dictionary_, marker_corners, marker_ids, detector_parameters_);
    if (marker_ids.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_error_ = "CHARUCO_NOT_DETECTED";
      last_corner_count_ = 0;
      return;
    }
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, image, board_, charuco_corners, charuco_ids,
      camera_matrix, distortion);
    if (static_cast<int>(charuco_ids.size()) < min_corners_) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_error_ = "CHARUCO_CORNERS_INSUFFICIENT";
      last_corner_count_ = static_cast<int>(charuco_ids.size());
      return;
    }
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if (!cv::aruco::estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board_, camera_matrix, distortion,
        rvec, tvec, false))
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_error_ = "PNP_FAILED";
      return;
    }

    std::vector<cv::Point3f> object_points;
    object_points.reserve(charuco_ids.size());
    const auto chessboard_corners = board_->getChessboardCorners();
    for (const int id : charuco_ids) {
      if (id < 0 || id >= static_cast<int>(chessboard_corners.size())) {
        return;
      }
      object_points.push_back(chessboard_corners[static_cast<size_t>(id)]);
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion, projected);
    double squared_error = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
      const cv::Point2f delta = projected[i] - charuco_corners[i];
      squared_error += delta.dot(delta);
    }
    const double rms = std::sqrt(squared_error / projected.size());
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    tf2::Matrix3x3 rotation(
      rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
      rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
      rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
      rotation_matrix.at<double>(2, 2));
    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    Detection detection;
    detection.camera_from_board = tf2::Transform(
      quaternion, tf2::Vector3(tvec[0], tvec[1], tvec[2]));
    detection.stamp = rclcpp::Time(msg->header.stamp);
    detection.reprojection_rms = rms;
    detection.corners = static_cast<int>(charuco_ids.size());

    std::lock_guard<std::mutex> lock(mutex_);
    recent_.push_back(detection);
    while (recent_.size() > 200) {
      recent_.pop_front();
    }
    last_corner_count_ = detection.corners;
    last_rms_ = rms;
    last_error_ = rms <= max_reprojection_rms_px_ ? "" : "REPROJECTION_RMS_TOO_HIGH";
  }

  void capture_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto xyz = get_parameter("base_to_board_xyz").as_double_array();
    const auto rpy = get_parameter("base_to_board_rpy").as_double_array();
    const auto name = get_parameter("sample_name").as_string();
    const auto role = get_parameter("sample_role").as_string();
    const auto measurement_mode = get_parameter("measurement_mode").as_string();
    const auto reference_points = get_parameter("board_reference_points").as_double_array();
    if (xyz.size() != 3 || rpy.size() != 3 || name.empty() ||
      !std::regex_match(name, std::regex("[A-Za-z0-9_-]+")) ||
      (role != "calibration" && role != "validation")) {
      response->message = "采集参数无效：名称只允许字母数字_-，role 为 calibration 或 validation";
      return;
    }
    tf2::Transform base_from_board;
    try {
      if (measurement_mode == "three_points") {
        base_from_board = carcar_camera::transform_from_reference_points(
          reference_points, squares_x_ * square_length_m_,
          squares_y_ * square_length_m_, max_board_size_error_m_);
      } else if (measurement_mode == "xyz_rpy") {
        base_from_board = carcar_camera::transform_from_xyz_rpy(xyz, rpy);
      } else {
        response->message = "measurement_mode 只允许 three_points 或 xyz_rpy";
        return;
      }
    } catch (const std::invalid_argument & error) {
      response->message = error.what();
      return;
    }
    const rclcpp::Time newest_allowed = now() - rclcpp::Duration::from_seconds(capture_window_s_);
    std::vector<Detection> selected;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto it = recent_.rbegin(); it != recent_.rend() &&
        selected.size() < static_cast<size_t>(capture_frames_); ++it)
      {
        if (it->stamp >= newest_allowed &&
          it->reprojection_rms <= max_reprojection_rms_px_) {
          selected.push_back(*it);
        }
      }
    }
    if (selected.size() < static_cast<size_t>(capture_frames_)) {
      response->message = "合格新鲜帧不足，请保持标定板静止、完整入镜后重试";
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    candidates_.erase(
      std::remove_if(
        candidates_.begin(), candidates_.end(),
        [&name](const Candidate & candidate) {return candidate.name == name;}),
      candidates_.end());
    measurements_.erase(
      std::remove_if(
        measurements_.begin(), measurements_.end(),
        [&name](const MeasurementRecord & record) {return record.name == name;}),
      measurements_.end());
    measurements_.push_back(
      MeasurementRecord{name, role, measurement_mode, base_from_board, reference_points});
    for (const auto & detection : selected) {
      Candidate candidate;
      candidate.base_from_camera_optical = carcar_camera::solve_base_from_camera(
        base_from_board, detection.camera_from_board);
      candidate.name = name;
      candidate.validation = role == "validation";
      candidate.reprojection_rms = detection.reprojection_rms;
      candidates_.push_back(candidate);
    }
    response->success = true;
    response->message = "已保存 " + name + " 的 " + std::to_string(selected.size()) + " 帧";
  }

  void solve(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::vector<Candidate> candidates;
    std::vector<MeasurementRecord> measurements;
    std::string optical_frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidates = candidates_;
      measurements = measurements_;
      optical_frame = optical_frame_;
    }
    std::unordered_set<std::string> calibration_names;
    std::unordered_set<std::string> validation_names;
    std::vector<tf2::Transform> calibration_transforms;
    double rms_sum = 0.0;
    size_t rms_count = 0;
    for (const auto & candidate : candidates) {
      if (candidate.validation) {
        validation_names.insert(candidate.name);
      } else {
        calibration_names.insert(candidate.name);
        calibration_transforms.push_back(candidate.base_from_camera_optical);
        rms_sum += candidate.reprojection_rms;
        ++rms_count;
      }
    }
    if (calibration_names.size() < 3 || validation_names.empty() ||
      calibration_transforms.empty() || optical_frame.empty()) {
      response->message = "至少需要 3 个 calibration 板位和 1 个 validation 板位";
      return;
    }
    const tf2::Transform base_from_optical =
      carcar_camera::average_transforms(calibration_transforms);
    tf2::Transform parent_from_base;
    tf2::Transform optical_from_camera_link;
    try {
      if (parent_frame_ == base_frame_) {
        parent_from_base.setIdentity();
      } else {
        const auto transform = tf_buffer_.lookupTransform(
          parent_frame_, base_frame_, tf2::TimePointZero);
        tf2::fromMsg(transform.transform, parent_from_base);
      }
      if (optical_frame == camera_link_frame_) {
        optical_from_camera_link.setIdentity();
      } else {
        const auto transform = tf_buffer_.lookupTransform(
          optical_frame, camera_link_frame_, tf2::TimePointZero);
        tf2::fromMsg(transform.transform, optical_from_camera_link);
      }
    } catch (const tf2::TransformException & error) {
      response->message = std::string("求解所需 TF 不可用: ") + error.what();
      return;
    }
    const tf2::Transform parent_from_camera_link =
      carcar_camera::optical_result_to_camera_link(
      parent_from_base, base_from_optical, optical_from_camera_link);

    double max_calibration_translation = 0.0;
    double max_calibration_angle = 0.0;
    double max_validation_translation = 0.0;
    double max_validation_angle = 0.0;
    for (const auto & candidate : candidates) {
      const double translation =
        (candidate.base_from_camera_optical.getOrigin() - base_from_optical.getOrigin()).length();
      const double angle = carcar_camera::rotation_error(
        base_from_optical.getRotation(), candidate.base_from_camera_optical.getRotation());
      if (candidate.validation) {
        max_validation_translation = std::max(max_validation_translation, translation);
        max_validation_angle = std::max(max_validation_angle, angle);
      } else {
        max_calibration_translation = std::max(max_calibration_translation, translation);
        max_calibration_angle = std::max(max_calibration_angle, angle);
      }
    }
    const auto q = parent_from_camera_link.getRotation();
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::ofstream output(output_file_);
    if (!output) {
      response->message = "无法写入候选外参文件: " + output_file_;
      return;
    }
    output << std::fixed << std::setprecision(9);
    output << "parent_frame: " << parent_frame_ << "\n";
    output << "child_frame: " << camera_link_frame_ << "\n";
    output << "verified: false\n";
    output << "mount_condition:\n";
    output << "  servo_target_angle_deg: " << servo_target_angle_deg_ << "\n";
    output << "  servo_position_feedback: false\n";
    output << "  measurement_quality: " << measurement_quality_ << "\n";
    output << "translation:\n";
    output << "  x: " << parent_from_camera_link.getOrigin().x() << "\n";
    output << "  y: " << parent_from_camera_link.getOrigin().y() << "\n";
    output << "  z: " << parent_from_camera_link.getOrigin().z() << "\n";
    output << "quaternion:\n";
    output << "  x: " << q.x() << "\n  y: " << q.y() << "\n";
    output << "  z: " << q.z() << "\n  w: " << q.w() << "\n";
    output << "rpy:\n";
    output << "  roll: " << roll << "\n  pitch: " << pitch << "\n  yaw: " << yaw << "\n";
    output << "board:\n";
    output << "  squares_x: " << squares_x_ << "\n  squares_y: " << squares_y_ << "\n";
    output << "  square_length_m: " << square_length_m_ << "\n";
    output << "  marker_length_m: " << marker_length_m_ << "\n";
    output << "  dictionary: DICT_4X4_50\n";
    output << "  max_board_size_error_m: " << max_board_size_error_m_ << "\n";
    output << "metrics:\n";
    const double radians_to_degrees = 180.0 / std::acos(-1.0);
    output << "  mean_reprojection_rms_px: " << rms_sum / rms_count << "\n";
    output << "  max_calibration_translation_m: " << max_calibration_translation << "\n";
    output << "  max_calibration_translation_mm: " << max_calibration_translation * 1000.0 << "\n";
    output << "  max_calibration_angle_rad: " << max_calibration_angle << "\n";
    output << "  max_calibration_angle_deg: " << max_calibration_angle * radians_to_degrees << "\n";
    output << "  max_validation_translation_m: " << max_validation_translation << "\n";
    output << "  max_validation_translation_mm: " << max_validation_translation * 1000.0 << "\n";
    output << "  max_validation_angle_rad: " << max_validation_angle << "\n";
    output << "  max_validation_angle_deg: " << max_validation_angle * radians_to_degrees << "\n";
    output << "  calibration_pose_count: " << calibration_names.size() << "\n";
    output << "  validation_pose_count: " << validation_names.size() << "\n";
    output << "measurement_records:\n";
    for (const auto & record : measurements) {
      const auto record_q = record.base_from_board.getRotation();
      output << "  - name: " << record.name << "\n";
      output << "    role: " << record.role << "\n";
      output << "    method: " << record.method << "\n";
      output << "    base_to_board_translation: ["
             << record.base_from_board.getOrigin().x() << ", "
             << record.base_from_board.getOrigin().y() << ", "
             << record.base_from_board.getOrigin().z() << "]\n";
      output << "    base_to_board_quaternion: [" << record_q.x() << ", "
             << record_q.y() << ", " << record_q.z() << ", " << record_q.w() << "]\n";
      if (record.method == "three_points") {
        const tf2::Vector3 origin(
          record.reference_points[0], record.reference_points[1], record.reference_points[2]);
        const tf2::Vector3 x_edge = tf2::Vector3(
          record.reference_points[3], record.reference_points[4], record.reference_points[5]) - origin;
        const tf2::Vector3 y_edge = tf2::Vector3(
          record.reference_points[6], record.reference_points[7], record.reference_points[8]) - origin;
        output << "    measured_width_m: " << x_edge.length() << "\n";
        output << "    measured_height_m: " << y_edge.length() << "\n";
        output << "    measured_edge_dot: " << x_edge.normalized().dot(y_edge.normalized()) << "\n";
        output << "    reference_points_base_m: [";
        for (size_t index = 0; index < record.reference_points.size(); ++index) {
          output << (index == 0 ? "" : ", ") << record.reference_points[index];
        }
        output << "]\n";
      }
    }
    output.close();
    response->success = true;
    response->message = "候选外参已写入 " + output_file_ + "，仍需独立实测验收";
  }

  void reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    candidates_.clear();
    measurements_.clear();
    recent_.clear();
    response->success = true;
    response->message = "已清除本次内存中的标定采样";
  }

  void diagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "D435CharucoCalibration";
    status.hardware_id = "D435";
    status.level = last_error_.empty() ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = last_error_.empty() ? "READY_TO_CAPTURE" : last_error_;
    add_value(status, "board", "7x5, square=0.030m, marker=0.022m, DICT_4X4_50");
    add_value(status, "optical_frame", optical_frame_);
    add_value(status, "corners", std::to_string(last_corner_count_));
    add_value(status, "reprojection_rms_px", std::to_string(last_rms_));
    add_value(status, "stored_frames", std::to_string(candidates_.size()));
    add_value(status, "output_file", output_file_);
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  int squares_x_{7};
  int squares_y_{5};
  int dictionary_id_{0};
  int min_corners_{8};
  double servo_target_angle_deg_{140.5};
  std::string measurement_quality_{"ruler_coarse"};
  int capture_frames_{10};
  double square_length_m_{0.030};
  double marker_length_m_{0.022};
  double capture_window_s_{2.0};
  double max_reprojection_rms_px_{1.0};
  double max_board_size_error_m_{0.010};
  std::string base_frame_;
  std::string parent_frame_;
  std::string camera_link_frame_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_file_;
  std::string optical_frame_;
  std::string last_error_{"CAMERA_INFO_MISSING"};
  int last_corner_count_{0};
  double last_rms_{INFINITY};
  bool have_info_{false};
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  std::mutex mutex_;
  Steady::time_point last_image_received_{};
  std::deque<Detection> recent_;
  std::vector<Candidate> candidates_;
  std::vector<MeasurementRecord> measurements_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr solve_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CharucoExtrinsicCalibrator>());
  rclcpp::shutdown();
  return 0;
}
