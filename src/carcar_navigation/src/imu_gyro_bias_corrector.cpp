#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "carcar_navigation/imu_gyro_bias.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

namespace carcar_navigation
{
class ImuGyroBiasCorrector : public rclcpp::Node
{
public:
  ImuGyroBiasCorrector()
  : Node("imu_gyro_bias_corrector")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/imu/data_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data_calibrated");
    bias_z_ = declare_parameter<double>("gyro_z_bias", 0.0);
    max_abs_z_ = declare_parameter<double>("max_abs_angular_velocity_z", 10.0);
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.20);
    if (!std::isfinite(bias_z_) || !std::isfinite(max_abs_z_) || max_abs_z_ <= 0.0 ||
      !std::isfinite(stale_timeout_) || stale_timeout_ <= 0.0)
    {
      throw std::invalid_argument("IMU 陀螺仪校准参数无效");
    }
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/imu_gyro_bias_corrector/diagnostics", 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr input) {
        const auto corrected=correct_gyro_z(input->angular_velocity.z,bias_z_,max_abs_z_);
        last_received_=std::chrono::steady_clock::now();
        last_reason_=corrected.reason;
        if (!corrected.valid) {++rejected_;return;}
        auto output=*input;
        output.angular_velocity.z=corrected.corrected_z;
        publisher_->publish(output);
        ++published_;
      });
    timer_ = create_wall_timer(1s, std::bind(&ImuGyroBiasCorrector::publish_diagnostics, this));
  }

private:
  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="IMU gyro Z bias corrector";status.hardware_id="carcar_navigation";
    const bool received=last_received_!=std::chrono::steady_clock::time_point{};
    const double age=received ? std::chrono::duration<double>(
      std::chrono::steady_clock::now()-last_received_).count() : -1.0;
    const bool fresh=received && age<=stale_timeout_;
    status.level=fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message=fresh ? "校准后陀螺仪 Z 数据新鲜" : "原始 IMU 数据缺失或过期";
    auto add=[&](const std::string & key,const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;item.key=key;item.value=value;status.values.push_back(item);
    };
    add("input_topic",input_topic_);add("output_topic",output_topic_);
    add("gyro_z_bias",std::to_string(bias_z_));add("age_s",std::to_string(age));
    add("published",std::to_string(published_));add("rejected",std::to_string(rejected_));
    add("last_reason",last_reason_);array.status.push_back(status);diagnostics_->publish(array);
  }

  std::string input_topic_,output_topic_,last_reason_{"NO_DATA"};
  double bias_z_{0.0},max_abs_z_{10.0},stale_timeout_{0.20};
  uint64_t published_{0},rejected_{0};
  std::chrono::steady_clock::time_point last_received_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace carcar_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<carcar_navigation::ImuGyroBiasCorrector>());
  rclcpp::shutdown();
  return 0;
}
