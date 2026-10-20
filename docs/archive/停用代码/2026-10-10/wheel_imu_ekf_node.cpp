// 已停用：接手前自写 EKF 草稿，不构建、不启动。后续融合采用 robot_localization。
// 此草稿缺少传感器时戳对齐、安装 TF、完整协方差和异常数据处理，不作为实车入口。
#include "carcar_navigation/wheel_imu_ekf.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include <mutex>

namespace carcar_navigation
{

class WheelImuEkfNode : public rclcpp::Node
{
public:
  WheelImuEkfNode()
  : Node("wheel_imu_ekf")
  {
    odom_in_ = declare_parameter("odom_topic", std::string("/wheel/odometry"));
    imu_in_ = declare_parameter("imu_topic", std::string("/imu/data_raw"));
    odom_out_ = declare_parameter("odom_out_topic", std::string("/odometry/filtered"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
    publish_tf_ = declare_parameter("publish_tf", true);
    q_xy_ = declare_parameter("q_xy", 0.02);
    q_yaw_ = declare_parameter("q_yaw", 0.01);
    r_xy_ = declare_parameter("r_xy", 0.03);
    imu_max_age_ = declare_parameter("imu_max_age", 0.20);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out_, rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_in_, rclcpp::SensorDataQoS(),
      std::bind(&WheelImuEkfNode::onOdom, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in_, rclcpp::SensorDataQoS(),
      std::bind(&WheelImuEkfNode::onImu, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "轮速+IMU EKF 已启动：只用 IMU gz 与轮式 x/y，不使用加速度/姿态/磁力计");
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_ = msg;
    last_imu_recv_ = now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double wheel_yaw = tf2::getYaw(msg->pose.pose.orientation);
    if (!ekf_.initialized) {
      ekf_.reset(msg->pose.pose.position.x, msg->pose.pose.position.y, wheel_yaw);
      last_odom_stamp_ = msg->header.stamp;
      publishLocked(*msg);
      return;
    }
    const double dt = (rclcpp::Time(msg->header.stamp) - last_odom_stamp_).seconds();
    last_odom_stamp_ = msg->header.stamp;
    double w = msg->twist.twist.angular.z;
    const bool imu_fresh = last_imu_ &&
      (now() - last_imu_recv_).seconds() <= imu_max_age_ &&
      std::isfinite(last_imu_->angular_velocity.z);
    if (imu_fresh) {w = last_imu_->angular_velocity.z;}
    ekf_.predict(msg->twist.twist.linear.x, w, dt, q_xy_, q_yaw_);
    ekf_.update_xy(msg->pose.pose.position.x, msg->pose.pose.position.y, r_xy_);
    publishLocked(*msg);
  }

  void publishLocked(const nav_msgs::msg::Odometry & wheel)
  {
    nav_msgs::msg::Odometry out = wheel;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = ekf_.x;
    out.pose.pose.position.y = ekf_.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, ekf_.yaw);
    out.pose.pose.orientation = tf2::toMsg(q);
    out.pose.covariance[0] = ekf_.P[0];
    out.pose.covariance[7] = ekf_.P[4];
    out.pose.covariance[35] = ekf_.P[8];
    if (last_imu_ && (now() - last_imu_recv_).seconds() <= imu_max_age_) {
      out.twist.twist.angular.z = last_imu_->angular_velocity.z;
    }
    odom_pub_->publish(out);
    if (!publish_tf_) {return;}
    geometry_msgs::msg::TransformStamped tf;
    tf.header = out.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = ekf_.x;
    tf.transform.translation.y = ekf_.y;
    tf.transform.rotation = out.pose.pose.orientation;
    tf_->sendTransform(tf);
  }

  std::mutex mutex_;
  WheelImuEkf ekf_;
  std::string odom_in_, imu_in_, odom_out_, odom_frame_, base_frame_;
  bool publish_tf_{true};
  double q_xy_{0.02}, q_yaw_{0.01}, r_xy_{0.03}, imu_max_age_{0.20};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_recv_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

}  // namespace carcar_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carcar_navigation::WheelImuEkfNode>());
  rclcpp::shutdown();
  return 0;
}
