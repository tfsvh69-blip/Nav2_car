// Copyright 2026 carcar maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Command estimate for stationary debugging, never encoder feedback.
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include "carcar_hardware/camera_servo_safety.hpp"

class CameraServoState : public rclcpp::Node
{
public:
  CameraServoState()
  : Node("camera_servo_state")
  {
    const auto topic = declare_parameter<std::string>("joint_topic", "/joint_states");
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(topic, 10);
    subscription_ = create_subscription<std_msgs::msg::Int32>(
      "/hardware/camera_servo/board_command_sent",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        if (msg->data < 57 || msg->data > 107) {
          RCLCPP_ERROR(get_logger(), "Rejected out-of-range S1 command estimate");
          return;
        }
        angle_ = carcar_hardware::camera_tilt_from_board_command(msg->data);
        received_ = true;
        RCLCPP_INFO(
          get_logger(), "S1 board=%d; tilt estimate=%.4f rad; NOT measured",
          msg->data, angle_);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        if (!received_) {return;}
        sensor_msgs::msg::JointState state;
        state.header.stamp = now();
        state.name = {"camera_tilt_joint"};
        state.position = {angle_};
        publisher_->publish(state);
      });
    RCLCPP_WARN(
      get_logger(), "Camera TF follows last SENT command; stationary debug only. "
      "No servo feedback, settling or power-loss detection.");
  }

private:
  bool received_{false};
  double angle_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraServoState>());
  rclcpp::shutdown();
  return 0;
}
