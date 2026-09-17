"""ROS 2 adapter for the Yahboom Rosmaster control board."""

import math
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from tf2_ros import TransformBroadcaster

from Rosmaster_Lib import Rosmaster

from .math_utils import (
    apply_imu_signs,
    clamp,
    integrate_body_delta,
    is_zero_motion_command,
    mecanum_body_delta_from_encoder_counts,
    validate_imu_signs,
    validate_mecanum_odometry_parameters,
    validate_motion_pid,
    yaw_to_quaternion,
)


class RosmasterNode(Node):
    """Translate standard ROS interfaces to and from a Rosmaster board."""

    def __init__(self) -> None:
        super().__init__('rosmaster_base')

        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('car_type', 1)
        self.declare_parameter('drive_mode', 'board_motion_pid')
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
        self.declare_parameter('wheel_odometry_enabled', False)
        self.declare_parameter('wheel_diameter_m', 0.060)
        self.declare_parameter('wheelbase_m', 0.120)
        self.declare_parameter('track_width_m', 0.185)
        self.declare_parameter('wheel_counts_per_revolution', [0.0] * 4)
        self.declare_parameter('wheel_encoder_signs', [1] * 4)
        self.declare_parameter(
            'wheel_port_wheels',
            ['left_front', 'left_rear', 'right_front', 'right_rear'],
        )
        self.declare_parameter('wheel_odom_max_count_delta', 100000)
        self.declare_parameter('imu_gyro_signs', [1, 1, -1])
        self.declare_parameter('imu_accel_signs', [1, 1, -1])
        self.declare_parameter('battery_min_voltage', 9.6)
        self.declare_parameter('battery_max_voltage', 12.6)
        self.declare_parameter('debug_serial', False)
        self.declare_parameter('motion_kp', 0.8)
        self.declare_parameter('motion_ki', 0.06)
        self.declare_parameter('motion_kd', 0.5)
        self.declare_parameter('motion_pid_apply', False)

        self._serial_port = str(self.get_parameter('serial_port').value)
        self._car_type = int(self.get_parameter('car_type').value)
        self._drive_mode = str(self.get_parameter('drive_mode').value)
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
        self._wheel_odometry_enabled = bool(
            self.get_parameter('wheel_odometry_enabled').value
        )
        self._wheel_diameter_m = float(
            self.get_parameter('wheel_diameter_m').value
        )
        self._wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self._track_width_m = float(
            self.get_parameter('track_width_m').value
        )
        self._wheel_counts_per_revolution = tuple(
            float(value) for value in self.get_parameter(
                'wheel_counts_per_revolution').value
        )
        self._wheel_encoder_signs = tuple(int(value) for value in
                                          self.get_parameter(
                                              'wheel_encoder_signs').value)
        self._wheel_port_wheels = tuple(str(value) for value in
                                        self.get_parameter(
                                            'wheel_port_wheels').value)
        self._wheel_odom_max_count_delta = int(
            self.get_parameter('wheel_odom_max_count_delta').value
        )
        self._imu_gyro_signs = validate_imu_signs(
            self.get_parameter('imu_gyro_signs').value
        )
        self._imu_accel_signs = validate_imu_signs(
            self.get_parameter('imu_accel_signs').value
        )

        self._battery_min = float(
            self.get_parameter('battery_min_voltage').value
        )
        self._battery_max = float(
            self.get_parameter('battery_max_voltage').value
        )
        configured_pid = validate_motion_pid(
            self.get_parameter('motion_kp').value,
            self.get_parameter('motion_ki').value,
            self.get_parameter('motion_kd').value,
        )

        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self._drive_mode != 'board_motion_pid':
            raise ValueError(
                'rosmaster_node only supports drive_mode=board_motion_pid'
            )
        if self._command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')
        if self._battery_max <= self._battery_min:
            raise ValueError(
                'battery_max_voltage must exceed battery_min_voltage'
            )
        if self._wheel_odom_max_count_delta <= 0:
            raise ValueError('wheel_odom_max_count_delta must be positive')
        calibration_counts = self._wheel_counts_per_revolution
        if not self._wheel_odometry_enabled:
            calibration_counts = (1.0, 1.0, 1.0, 1.0)
        (
            self._wheel_diameter_m,
            self._wheelbase_m,
            self._track_width_m,
            _,
            self._wheel_encoder_signs,
            self._wheel_port_wheels,
        ) = validate_mecanum_odometry_parameters(
            self._wheel_diameter_m,
            self._wheelbase_m,
            self._track_width_m,
            calibration_counts,
            self._wheel_encoder_signs,
            self._wheel_port_wheels,
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
        for _ in range(3):
            self._driver.set_car_motion(0.0, 0.0, 0.0)

        queried_pid = self._driver.get_motion_pid()
        try:
            self._motion_pid_active = validate_motion_pid(*queried_pid)
            self._motion_pid_status = 'read_from_board'
            self.set_parameters([
                Parameter('motion_kp', value=self._motion_pid_active[0]),
                Parameter('motion_ki', value=self._motion_pid_active[1]),
                Parameter('motion_kd', value=self._motion_pid_active[2]),
            ])
        except (TypeError, ValueError):
            self._motion_pid_active = configured_pid
            self._motion_pid_status = 'board_read_failed_using_config'
            self.get_logger().warning(
                '读取控制板运动 PID 失败；RQT 显示配置备用值，'
                '首次应用前请保持停车。')
        self._motion_pid_staged = self._motion_pid_active

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
        self._encoder_rate_pub = self.create_publisher(
            Float32MultiArray, 'wheel/encoder_rates', 10
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
        self._previous_encoders = None
        self._previous_encoder_ns = None
        self._previous_odom_encoders = None
        self._previous_odom_ns = None
        self._wheel_odom_status = (
            'waiting_for_encoder_baseline'
            if self._wheel_odometry_enabled else 'disabled_pending_calibration'
        )
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # 初始化关闭声光并订阅指示器控制话题
        with self._io_lock:
            self._driver.set_beep(0)
            self._driver.set_colorful_lamps(0xFF, 0, 0, 0)

        self._last_indicator_cmd_ns = 0
        self._indicator_active = False
        try:
            from carcar_interfaces.msg import IndicatorCommand
            self._indicator_sub = self.create_subscription(
                IndicatorCommand,
                '/hardware/indicator/command',
                self._on_indicator_cmd,
                10,
            )
        except ImportError:
            self.get_logger().warning(
                'carcar_interfaces 未找到，声光控制话题未启用'
            )
            self._indicator_sub = None

        self._timer = self.create_timer(1.0 / self._publish_rate, self._update)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.get_logger().info(
            f'Rosmaster connected on {self._serial_port}; '
            f'car_type={self._car_type}, '
            f'holonomic={self._holonomic}, '
            f'drive_mode={self._drive_mode}, '
            f'wheel_odometry_enabled={self._wheel_odometry_enabled}'
        )

    def _on_indicator_cmd(self, msg) -> None:
        """Handle incoming sound and light indicator command safely."""
        self._last_indicator_cmd_ns = self.get_clock().now().nanoseconds

        # 校验并限制 RGB (0..255)
        r = int(clamp(msg.r, 0, 255))
        g = int(clamp(msg.g, 0, 255))
        b = int(clamp(msg.b, 0, 255))

        # 校验蜂鸣时间：绝不接受持续常鸣 1；只允许 0 或 10..3000ms (10倍数)
        beep_ms = int(msg.beep_duration_ms)
        if beep_ms == 1 or beep_ms < 0:
            beep_ms = 0
        elif beep_ms > 3000:
            beep_ms = 3000
        else:
            beep_ms = (beep_ms // 10) * 10

        # 硬件调用（严格互斥保护，且不改变运动看门狗状态）
        with self._io_lock:
            self._driver.set_colorful_lamps(0xFF, r, g, b)
            if msg.beep_trigger and beep_ms >= 10:
                self._driver.set_beep(beep_ms)
        self._indicator_active = True

    def _on_parameters(self, parameters) -> SetParametersResult:
        """Stage PID values and apply them on an explicit commit."""
        staged = list(self._motion_pid_staged)
        apply_requested = False
        names = ('motion_kp', 'motion_ki', 'motion_kd')
        supported = set(names) | {'motion_pid_apply'}
        for parameter in parameters:
            if parameter.name not in supported:
                return SetParametersResult(
                    successful=False,
                    reason=(f'{parameter.name} 不支持运行时修改；'
                            '请修改 YAML 后停车重启驱动'),
                )
            if parameter.name in names:
                staged[names.index(parameter.name)] = float(parameter.value)
            elif parameter.name == 'motion_pid_apply':
                apply_requested = bool(parameter.value)

        try:
            candidate = validate_motion_pid(*staged)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if apply_requested:
            if not self._watchdog_stopped:
                return SetParametersResult(
                    successful=False,
                    reason='必须先用空格/k 停车，才能应用运动 PID',
                )
            with self._io_lock:
                for _ in range(3):
                    self._driver.set_car_motion(0.0, 0.0, 0.0)
                self._driver.set_pid_param(*candidate, forever=False)
                confirmed = self._driver.get_motion_pid()
            try:
                confirmed = validate_motion_pid(*confirmed)
            except (TypeError, ValueError):
                self._motion_pid_status = 'apply_sent_readback_failed'
                return SetParametersResult(
                    successful=False,
                    reason='PID 已临时发送，但控制板回读失败；保持停车并重启控制板恢复',
                )
            if any(abs(actual - requested) > 0.002
                   for actual, requested in zip(confirmed, candidate)):
                self._motion_pid_status = 'apply_readback_mismatch'
                return SetParametersResult(
                    successful=False,
                    reason=f'PID 回读不一致：请求={candidate}，回读={confirmed}',
                )
            self._motion_pid_active = confirmed
            self._motion_pid_status = 'temporary_apply_confirmed'
            self.get_logger().warning(
                '已临时应用控制板运动 PID：'
                f'Kp={confirmed[0]:.3f}, Ki={confirmed[1]:.3f}, '
                f'Kd={confirmed[2]:.3f}；未写 Flash，重启控制板可恢复。')

        self._motion_pid_staged = candidate
        return SetParametersResult(successful=True)

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
        self._watchdog_stopped = is_zero_motion_command(
            velocity_x, velocity_y, angular_z
        )

    def _update(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        command_age = (now_ns - self._last_command_ns) * 1e-9
        if command_age >= self._command_timeout and not self._watchdog_stopped:
            with self._io_lock:
                self._driver.set_car_motion(0.0, 0.0, 0.0)
            self._watchdog_stopped = True
            self.get_logger().warning('cmd_vel timeout: motors stopped')

        # 声光指示器超时保护：若超过 1.0s 未收到 IndicatorCommand，自动熄灭并停音
        if self._indicator_active and self._last_indicator_cmd_ns > 0:
            indicator_age = (now_ns - self._last_indicator_cmd_ns) * 1e-9
            if indicator_age >= 1.0:
                with self._io_lock:
                    self._driver.set_beep(0)
                    self._driver.set_colorful_lamps(0xFF, 0, 0, 0)
                self._indicator_active = False

        self._last_update_ns = now_ns
        with self._io_lock:
            encoder_values = tuple(self._driver.get_motor_encoder())
        self._update_wheel_odometry(now.to_msg(), now_ns, encoder_values)
        self._publish_imu(now.to_msg())
        self._publish_board_state(
            now.to_msg(), command_age, now_ns, encoder_values
        )

    def _update_wheel_odometry(
        self,
        stamp,
        now_ns: int,
        encoder_values: tuple[int, int, int, int],
    ) -> None:
        """Integrate encoder increments after calibration is enabled."""
        if not self._wheel_odometry_enabled:
            return
        if self._previous_odom_encoders is None:
            self._previous_odom_encoders = encoder_values
            self._previous_odom_ns = now_ns
            self._wheel_odom_status = 'encoder_baseline_captured'
            return

        dt = (now_ns - self._previous_odom_ns) * 1e-9
        count_deltas = tuple(
            current - previous for current, previous in zip(
                encoder_values, self._previous_odom_encoders)
        )
        self._previous_odom_encoders = encoder_values
        self._previous_odom_ns = now_ns
        if dt <= 0.0 or dt > 0.25:
            self._wheel_odom_status = 'encoder_interval_rejected'
            return
        if any(abs(delta) > self._wheel_odom_max_count_delta
               for delta in count_deltas):
            self._wheel_odom_status = 'encoder_jump_rejected'
            self.get_logger().warning(
                '轮式里程计丢弃异常编码器跳变：'
                f'{count_deltas}'
            )
            return

        delta_x, delta_y, delta_yaw = (
            mecanum_body_delta_from_encoder_counts(
                count_deltas,
                self._wheel_diameter_m,
                self._wheelbase_m,
                self._track_width_m,
                self._wheel_counts_per_revolution,
                self._wheel_encoder_signs,
                self._wheel_port_wheels,
            )
        )
        if not self._holonomic:
            delta_y = 0.0
        self._x, self._y, self._yaw = integrate_body_delta(
            self._x,
            self._y,
            self._yaw,
            delta_x,
            delta_y,
            delta_yaw,
        )
        self._wheel_odom_status = 'encoder_odometry_active'
        self._publish_odometry(
            stamp,
            delta_x / dt,
            delta_y / dt,
            delta_yaw / dt,
        )

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
        acceleration = apply_imu_signs(
            self._driver.get_accelerometer_data(), self._imu_accel_signs
        )
        angular_velocity = apply_imu_signs(
            self._driver.get_gyroscope_data(), self._imu_gyro_signs
        )
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

    def _publish_board_state(
        self,
        stamp,
        command_age: float,
        now_ns: int,
        encoder_values: tuple[int, int, int, int],
    ) -> None:
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
        encoders.data = list(encoder_values)
        self._encoder_pub.publish(encoders)

        encoder_rates = Float32MultiArray()
        encoder_rates.data = [0.0, 0.0, 0.0, 0.0]
        if (self._previous_encoders is not None
                and self._previous_encoder_ns is not None):
            encoder_dt = (now_ns - self._previous_encoder_ns) * 1e-9
            if 0.0 < encoder_dt <= 0.25:
                encoder_rates.data = [
                    float(current - previous) / encoder_dt
                    for current, previous in zip(
                        encoder_values, self._previous_encoders)
                ]
        self._previous_encoders = encoder_values
        self._previous_encoder_ns = now_ns
        self._encoder_rate_pub.publish(encoder_rates)

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
            KeyValue(key='drive_mode', value=self._drive_mode),
            KeyValue(key='battery_voltage', value=f'{voltage:.2f}'),
            KeyValue(key='last_cmd_age_s', value=f'{command_age:.3f}'),
            KeyValue(
                key='watchdog_stopped',
                value=str(self._watchdog_stopped),
            ),
            KeyValue(
                key='motion_pid_active',
                value=','.join(f'{value:.3f}'
                               for value in self._motion_pid_active),
            ),
            KeyValue(
                key='motion_pid_staged',
                value=','.join(f'{value:.3f}'
                               for value in self._motion_pid_staged),
            ),
            KeyValue(key='motion_pid_status', value=self._motion_pid_status),
            KeyValue(
                key='wheel_odometry_enabled',
                value=str(self._wheel_odometry_enabled),
            ),
            KeyValue(
                key='wheel_odometry_status',
                value=self._wheel_odom_status,
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
                for _ in range(3):
                    self._driver.set_car_motion(0.0, 0.0, 0.0)
                self._driver.set_beep(0)
                self._driver.set_colorful_lamps(0xFF, 0, 0, 0)
                self._driver.set_auto_report_state(False, forever=False)
        except Exception as exc:  # Hardware may already be disconnected.
            self.get_logger().error(f'Failed to stop Rosmaster cleanly: {exc}')
        finally:
            serial_port = getattr(self._driver, 'ser', None)
            if serial_port is not None and serial_port.is_open:
                serial_port.close()


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
