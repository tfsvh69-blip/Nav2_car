"""Safe four-motor PWM driver for bounded mecanum direction tests."""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from Rosmaster_Lib import Rosmaster

from .math_utils import (
    clamp,
    mecanum_port_targets,
    slew_towards,
    velocity_pi_pwm,
)


ZERO_COMMAND = (0, 0, 0, 0)


class FourMotorMotionTestNode(Node):
    """Apply an explicit wheel/port/sign map with bounded direct PWM."""

    def __init__(self) -> None:
        super().__init__('chassis_motion_test_driver')
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('port_wheels', [
            'left_front', 'right_rear', 'right_front', 'left_rear'])
        self.declare_parameter('max_pwm_percent', 25)
        self.declare_parameter('linear_full_scale', 0.08)
        self.declare_parameter('angular_full_scale', 0.40)
        self.declare_parameter('target_encoder_rate', 700.0)
        self.declare_parameter('target_acceleration_limit', 2000.0)
        self.declare_parameter('pwm_slew_rate', 120.0)
        self.declare_parameter('feedforward_pwm', [9.0, 7.0, 21.0, 21.0])
        self.declare_parameter('velocity_kp', 0.008)
        self.declare_parameter('velocity_ki', 0.020)
        self.declare_parameter('integral_limit', 400.0)
        self.declare_parameter('velocity_filter_alpha', 0.50)
        self.declare_parameter('feedback_timeout', 0.35)
        self.declare_parameter('motor_1_sign', 1)
        self.declare_parameter('motor_2_sign', -1)
        self.declare_parameter('motor_3_sign', 1)
        self.declare_parameter('motor_4_sign', -1)
        self.declare_parameter('encoder_1_sign', -1)
        self.declare_parameter('encoder_2_sign', 1)
        self.declare_parameter('encoder_3_sign', -1)
        self.declare_parameter('encoder_4_sign', 1)
        self.declare_parameter('command_timeout', 0.30)
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('debug_serial', False)

        self._serial_port = str(self.get_parameter('serial_port').value)
        self._port_wheels = tuple(self.get_parameter('port_wheels').value)
        self._max_pwm = int(self.get_parameter('max_pwm_percent').value)
        self._linear_full_scale = float(
            self.get_parameter('linear_full_scale').value)
        self._angular_full_scale = float(
            self.get_parameter('angular_full_scale').value)
        self._target_encoder_rate = float(
            self.get_parameter('target_encoder_rate').value)
        self._target_acceleration_limit = float(
            self.get_parameter('target_acceleration_limit').value)
        self._pwm_slew_rate = float(
            self.get_parameter('pwm_slew_rate').value)
        self._feedforward_pwm = tuple(
            float(value) for value in
            self.get_parameter('feedforward_pwm').value)
        self._velocity_kp = float(
            self.get_parameter('velocity_kp').value)
        self._velocity_ki = float(
            self.get_parameter('velocity_ki').value)
        self._integral_limit = float(
            self.get_parameter('integral_limit').value)
        self._velocity_filter_alpha = float(
            self.get_parameter('velocity_filter_alpha').value)
        self._feedback_timeout = float(
            self.get_parameter('feedback_timeout').value)
        self._motor_signs = tuple(
            int(self.get_parameter(f'motor_{number}_sign').value)
            for number in range(1, 5)
        )
        self._encoder_signs = tuple(
            int(self.get_parameter(f'encoder_{number}_sign').value)
            for number in range(1, 5)
        )
        self._command_timeout = float(
            self.get_parameter('command_timeout').value)
        self._publish_rate = float(
            self.get_parameter('publish_rate').value)

        mecanum_port_targets(
            0.0,
            0.0,
            0.0,
            self._target_encoder_rate,
            self._linear_full_scale,
            self._angular_full_scale,
            self._port_wheels,
        )
        if len(self._feedforward_pwm) != 4:
            raise ValueError('feedforward_pwm must contain four values')
        if len(self._encoder_signs) != 4 or any(
                sign not in (-1, 1) for sign in self._encoder_signs):
            raise ValueError(
                'encoder signs must each be either -1 or 1')
        for index in range(4):
            velocity_pi_pwm(
                self._target_encoder_rate,
                0.0,
                0.0,
                self._feedforward_pwm[index],
                self._velocity_kp,
                self._velocity_ki,
                self._max_pwm,
                self._motor_signs[index],
            )
        if self._integral_limit <= 0.0:
            raise ValueError('integral_limit must be greater than zero')
        if self._target_acceleration_limit <= 0.0:
            raise ValueError(
                'target_acceleration_limit must be greater than zero')
        if self._pwm_slew_rate <= 0.0:
            raise ValueError('pwm_slew_rate must be greater than zero')
        if not 0.0 < self._velocity_filter_alpha <= 1.0:
            raise ValueError(
                'velocity_filter_alpha must be within (0, 1]')
        if self._feedback_timeout <= 0.0:
            raise ValueError('feedback_timeout must be greater than zero')
        if self._command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')
        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self._driver = Rosmaster(
            com=self._serial_port,
            debug=bool(self.get_parameter('debug_serial').value),
        )
        self._driver.create_receive_threading()
        self._driver.set_auto_report_state(True, forever=False)
        self._send_zero_repeatedly()

        self._encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoders', 10)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10)
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, 10)

        self._last_command_ns = self.get_clock().now().nanoseconds
        self._stopped = True
        self._last_pwm = ZERO_COMMAND
        self._target_rates = (0.0, 0.0, 0.0, 0.0)
        self._requested_target_rates = (0.0, 0.0, 0.0, 0.0)
        self._measured_rates = [0.0, 0.0, 0.0, 0.0]
        self._integral_errors = [0.0, 0.0, 0.0, 0.0]
        self._last_encoders = None
        self._last_encoder_sample_ns = None
        self._last_encoder_change_ns = [
            self._last_command_ns for _ in range(4)]
        self._control_log_counter = 0
        self._timer = self.create_timer(
            1.0 / self._publish_rate, self._update)
        self.get_logger().warning(
            f'四轮组合测试驱动已连接 {self._serial_port}；'
            f'端口轮位={self._port_wheels}，'
            f'电机符号={self._motor_signs}，编码器符号={self._encoder_signs}，'
            f'目标={self._target_encoder_rate:.0f} count/s，'
            f'PWM 上限={self._max_pwm}%，当前四路为零。')

    def _on_cmd_vel(self, message: Twist) -> None:
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('收到 NaN/Inf 命令；四轮立即归零。')
            self._stop_closed_loop()
            return

        target_rates = mecanum_port_targets(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self._target_encoder_rate,
            self._linear_full_scale,
            self._angular_full_scale,
            self._port_wheels,
        )
        now_ns = self.get_clock().now().nanoseconds
        self._last_command_ns = now_ns
        if not any(target_rates):
            self._stop_closed_loop()
            return

        if target_rates != self._requested_target_rates:
            self._requested_target_rates = target_rates
            self._integral_errors = [0.0, 0.0, 0.0, 0.0]
            self._last_encoder_change_ns = [now_ns for _ in range(4)]
            self._control_log_counter = 0
            self.get_logger().info(
                '闭环请求 [M1,M2,M3,M4] count/s='
                f'{tuple(round(value) for value in target_rates)}')
        self._stopped = False

    def _send_zero_repeatedly(self) -> None:
        for _ in range(3):
            self._driver.set_motor(*ZERO_COMMAND)
        self._last_pwm = ZERO_COMMAND

    def _stop_closed_loop(self) -> None:
        self._send_zero_repeatedly()
        self._target_rates = (0.0, 0.0, 0.0, 0.0)
        self._requested_target_rates = (0.0, 0.0, 0.0, 0.0)
        self._integral_errors = [0.0, 0.0, 0.0, 0.0]
        self._stopped = True

    def _update_encoder_rates(self, encoders, now_ns: int):
        if self._last_encoders is None:
            self._last_encoders = encoders
            self._last_encoder_sample_ns = now_ns
            self._last_encoder_change_ns = [now_ns for _ in range(4)]
            return None

        dt = (now_ns - self._last_encoder_sample_ns) * 1e-9
        if dt <= 0.0 or dt > 0.25:
            self._last_encoders = encoders
            self._last_encoder_sample_ns = now_ns
            return None

        alpha = self._velocity_filter_alpha
        for index in range(4):
            delta = encoders[index] - self._last_encoders[index]
            if delta != 0:
                self._last_encoder_change_ns[index] = now_ns
            raw_rate = delta / dt
            physical_rate = raw_rate * self._encoder_signs[index]
            self._measured_rates[index] = (
                alpha * physical_rate
                + (1.0 - alpha) * self._measured_rates[index]
            )
        self._last_encoders = encoders
        self._last_encoder_sample_ns = now_ns
        return dt

    def _feedback_is_live(self, now_ns: int) -> bool:
        for index, target_rate in enumerate(self._target_rates):
            age = (now_ns - self._last_encoder_change_ns[index]) * 1e-9
            if target_rate != 0.0 and age >= self._feedback_timeout:
                self.get_logger().error(
                    f'M{index + 1} 编码器超过 {age:.3f} 秒无变化；'
                    '四轮闭环已停车。')
                return False
        return True

    def _ramp_target_rates(self, dt: float) -> None:
        """Apply a bounded acceleration before changing a wheel target."""
        max_step = self._target_acceleration_limit * dt
        self._target_rates = tuple(
            slew_towards(current, requested, max_step)
            for current, requested in zip(
                self._target_rates, self._requested_target_rates)
        )

    def _apply_closed_loop(self, dt: float) -> None:
        command = []
        for index in range(4):
            error = self._target_rates[index] - self._measured_rates[index]
            self._integral_errors[index] = clamp(
                self._integral_errors[index] + error * dt,
                -self._integral_limit,
                self._integral_limit,
            )
            requested_pwm = velocity_pi_pwm(
                self._target_rates[index],
                self._measured_rates[index],
                self._integral_errors[index],
                self._feedforward_pwm[index],
                self._velocity_kp,
                self._velocity_ki,
                self._max_pwm,
                self._motor_signs[index],
            )
            command.append(int(round(slew_towards(
                float(self._last_pwm[index]),
                float(requested_pwm),
                self._pwm_slew_rate * dt,
            ))))
        self._last_pwm = tuple(command)
        self._driver.set_motor(*self._last_pwm)
        self._control_log_counter += 1
        if self._control_log_counter % 5 == 1:
            measured = tuple(round(value) for value in self._measured_rates)
            self.get_logger().info(
                '闭环目标='
                f'{tuple(round(value) for value in self._target_rates)}，'
                f'反馈 count/s={measured}，PWM={self._last_pwm}')

    def _update(self) -> None:
        now = self.get_clock().now()
        command_age = (
            now.nanoseconds - self._last_command_ns
        ) * 1e-9
        if not self._stopped and command_age >= self._command_timeout:
            self._stop_closed_loop()
            self.get_logger().warning('命令超时，M1～M4 已归零。')

        voltage = float(self._driver.get_battery_voltage())
        encoders = tuple(self._driver.get_motor_encoder())
        dt = self._update_encoder_rates(encoders, now.nanoseconds)
        if not self._stopped and dt is not None:
            self._ramp_target_rates(dt)
            if self._feedback_is_live(now.nanoseconds):
                self._apply_closed_loop(dt)
            else:
                self._stop_closed_loop()
        if voltage > 0.0:
            encoder_message = Int32MultiArray()
            encoder_message.data = list(encoders)
            self._encoder_pub.publish(encoder_message)

        status = DiagnosticStatus()
        status.name = 'carcar/chassis_motion_test'
        status.hardware_id = self._serial_port
        status.level = (
            DiagnosticStatus.OK if voltage > 0.0 else DiagnosticStatus.WARN)
        status.message = (
            'Connected' if voltage > 0.0 else 'Waiting for board telemetry')
        status.values = [
            KeyValue(key='battery_voltage', value=f'{voltage:.2f}'),
            KeyValue(key='watchdog_stopped', value=str(self._stopped)),
            KeyValue(key='control_mode', value='encoder_velocity_pi'),
            KeyValue(key='target_count_s', value=str(self._target_rates)),
            KeyValue(
                key='requested_count_s',
                value=str(self._requested_target_rates),
            ),
            KeyValue(
                key='measured_count_s',
                value=str(tuple(round(value) for value in
                                self._measured_rates)),
            ),
            KeyValue(key='motor_pwm', value=str(self._last_pwm)),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = now.to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)

    def close(self) -> None:
        """Stop all outputs and release the sole control-board serial port."""
        try:
            self._stop_closed_loop()
            self._driver.set_auto_report_state(False, forever=False)
        finally:
            serial_port = getattr(self._driver, 'ser', None)
            if serial_port is not None and serial_port.is_open:
                serial_port.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FourMotorMotionTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
