"""ROS 2 adapter for the Yahboom Rosmaster control board."""

import math
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

from Rosmaster_Lib import Rosmaster

from .math_utils import clamp, integrate_body_twist, yaw_to_quaternion


class RosmasterNode(Node):
    """Translate standard ROS interfaces to and from a Rosmaster board."""

    def __init__(self) -> None:
        super().__init__('rosmaster_base')

        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('car_type', 1)
        self.declare_parameter('holonomic', True)
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.5)
        self.declare_parameter('max_angular_z', 2.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_odom_tf', False)
        self.declare_parameter('battery_min_voltage', 9.6)
        self.declare_parameter('battery_max_voltage', 12.6)
        self.declare_parameter('debug_serial', False)

        self._serial_port = str(self.get_parameter('serial_port').value)
        self._car_type = int(self.get_parameter('car_type').value)
        self._holonomic = bool(self.get_parameter('holonomic').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self._max_linear_x = float(self.get_parameter('max_linear_x').value)
        self._max_linear_y = float(self.get_parameter('max_linear_y').value)
        self._max_angular_z = float(self.get_parameter('max_angular_z').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._imu_frame = str(self.get_parameter('imu_frame').value)
        self._publish_odom_tf = bool(
            self.get_parameter('publish_odom_tf').value
        )
        self._battery_min = float(
            self.get_parameter('battery_min_voltage').value
        )
        self._battery_max = float(
            self.get_parameter('battery_max_voltage').value
        )

        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self._command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')
        if self._battery_max <= self._battery_min:
            raise ValueError(
                'battery_max_voltage must exceed battery_min_voltage'
            )

        self._io_lock = threading.Lock()
        self._driver = Rosmaster(
            car_type=self._car_type,
            com=self._serial_port,
            debug=bool(self.get_parameter('debug_serial').value),
        )
        self._driver.create_receive_threading()
        self._driver.set_auto_report_state(True, forever=False)
        # Never assume the MCU was stationary before this process connected.
        self._driver.set_car_motion(0.0, 0.0, 0.0)

        self._odom_pub = self.create_publisher(Odometry, 'wheel/odometry', 10)
        self._imu_pub = self.create_publisher(
            Imu, 'imu/data_raw', qos_profile_sensor_data
        )
        self._battery_pub = self.create_publisher(
            BatteryState, 'battery_state', 10
        )
        self._encoder_pub = self.create_publisher(
            Int32MultiArray, 'wheel/encoders', 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10
        )
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, 10
        )
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._publish_odom_tf else None
        )

        now_ns = self.get_clock().now().nanoseconds
        self._last_command_ns = now_ns
        self._last_update_ns = now_ns
        self._watchdog_stopped = True
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._timer = self.create_timer(1.0 / self._publish_rate, self._update)
        self.get_logger().info(
            f'Rosmaster connected on {self._serial_port}; '
            f'car_type={self._car_type}, '
            f'holonomic={self._holonomic}'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                'Ignored cmd_vel containing NaN or infinity'
            )
            return

        velocity_x = clamp(
            msg.linear.x, -self._max_linear_x, self._max_linear_x
        )
        velocity_y = 0.0
        if self._holonomic:
            velocity_y = clamp(
                msg.linear.y, -self._max_linear_y, self._max_linear_y
            )
        angular_z = clamp(
            msg.angular.z, -self._max_angular_z, self._max_angular_z
        )
        with self._io_lock:
            self._driver.set_car_motion(velocity_x, velocity_y, angular_z)
        self._last_command_ns = self.get_clock().now().nanoseconds
        self._watchdog_stopped = False

    def _update(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        command_age = (now_ns - self._last_command_ns) * 1e-9
        if command_age >= self._command_timeout and not self._watchdog_stopped:
            with self._io_lock:
                self._driver.set_car_motion(0.0, 0.0, 0.0)
            self._watchdog_stopped = True
            self.get_logger().warning('cmd_vel timeout: motors stopped')

        dt = (now_ns - self._last_update_ns) * 1e-9
        self._last_update_ns = now_ns
        if dt <= 0.0 or dt > 0.25:
            dt = 1.0 / self._publish_rate

        velocity_x, velocity_y, angular_z = self._driver.get_motion_data()
        if not self._holonomic:
            velocity_y = 0.0
        self._x, self._y, self._yaw = integrate_body_twist(
            self._x,
            self._y,
            self._yaw,
            velocity_x,
            velocity_y,
            angular_z,
            dt,
        )

        self._publish_odometry(now.to_msg(), velocity_x, velocity_y, angular_z)
        self._publish_imu(now.to_msg())
        self._publish_board_state(now.to_msg(), command_age)

    def _publish_odometry(
        self,
        stamp,
        velocity_x: float,
        velocity_y: float,
        angular_z: float,
    ) -> None:
        quaternion = yaw_to_quaternion(self._yaw)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        msg.twist.twist.linear.x = velocity_x
        msg.twist.twist.linear.y = velocity_y
        msg.twist.twist.angular.z = angular_z
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = 0.03
        msg.twist.covariance[7] = 0.03
        msg.twist.covariance[35] = 0.05
        self._odom_pub.publish(msg)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

    def _publish_imu(self, stamp) -> None:
        acceleration = self._driver.get_accelerometer_data()
        angular_velocity = self._driver.get_gyroscope_data()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame
        msg.orientation_covariance[0] = -1.0
        (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ) = angular_velocity
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = acceleration
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10
        self._imu_pub.publish(msg)

    def _publish_board_state(self, stamp, command_age: float) -> None:
        voltage = float(self._driver.get_battery_voltage())
        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = voltage
        battery.percentage = clamp(
            (voltage - self._battery_min)
            / (self._battery_max - self._battery_min),
            0.0,
            1.0,
        )
        battery.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        battery.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )
        battery.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        )
        battery.present = voltage > 0.0
        self._battery_pub.publish(battery)

        encoders = Int32MultiArray()
        encoders.data = list(self._driver.get_motor_encoder())
        self._encoder_pub.publish(encoders)

        status = DiagnosticStatus()
        status.name = 'carcar/rosmaster'
        status.hardware_id = self._serial_port
        if voltage <= 0.0:
            status.level = DiagnosticStatus.WARN
            status.message = 'Waiting for board telemetry'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Connected'
        status.values = [
            KeyValue(key='battery_voltage', value=f'{voltage:.2f}'),
            KeyValue(key='last_cmd_age_s', value=f'{command_age:.3f}'),
            KeyValue(
                key='watchdog_stopped',
                value=str(self._watchdog_stopped),
            ),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)

    def stop(self) -> None:
        """Best-effort motor stop used during shutdown."""
        try:
            with self._io_lock:
                self._driver.set_car_motion(0.0, 0.0, 0.0)
                self._driver.set_auto_report_state(False, forever=False)
        except Exception as exc:  # Hardware may already be disconnected.
            self.get_logger().error(f'Failed to stop Rosmaster cleanly: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RosmasterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
