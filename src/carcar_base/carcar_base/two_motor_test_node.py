"""Low-speed, M1/M2-only hardware test node."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from Rosmaster_Lib import Rosmaster

from .math_utils import two_motor_pwm


class TwoMotorTestNode(Node):
    """Drive only motor outputs M1 and M2 from a standard cmd_vel topic."""

    def __init__(self) -> None:
        super().__init__('two_motor_test')
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('max_pwm_percent', 20)
        self.declare_parameter('linear_full_scale', 0.5)
        self.declare_parameter('angular_full_scale', 1.0)
        self.declare_parameter('motor_1_sign', 1)
        self.declare_parameter('motor_2_sign', 1)
        self.declare_parameter('enable_steering', False)
        self.declare_parameter('command_timeout', 0.6)
        self.declare_parameter('watchdog_rate', 20.0)

        self._serial_port = str(self.get_parameter('serial_port').value)
        self._max_pwm = int(self.get_parameter('max_pwm_percent').value)
        self._linear_full_scale = float(
            self.get_parameter('linear_full_scale').value
        )
        self._angular_full_scale = float(
            self.get_parameter('angular_full_scale').value
        )
        self._motor_1_sign = int(self.get_parameter('motor_1_sign').value)
        self._motor_2_sign = int(self.get_parameter('motor_2_sign').value)
        self._enable_steering = bool(
            self.get_parameter('enable_steering').value
        )
        self._command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        watchdog_rate = float(self.get_parameter('watchdog_rate').value)

        # Validate all conversion parameters before opening the serial port.
        two_motor_pwm(
            0.0,
            0.0,
            self._max_pwm,
            self._linear_full_scale,
            self._angular_full_scale,
            self._enable_steering,
            self._motor_1_sign,
            self._motor_2_sign,
        )
        if self._command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')
        if watchdog_rate <= 0.0:
            raise ValueError('watchdog_rate must be greater than zero')

        self._driver = Rosmaster(com=self._serial_port)
        self._set_motor(0, 0)
        self._last_command_ns = self.get_clock().now().nanoseconds
        self._stopped = True

        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, 10
        )
        self._watchdog = self.create_timer(
            1.0 / watchdog_rate, self._check_command_timeout
        )
        self.get_logger().warning(
            f'M1/M2 test mode active on {self._serial_port}: '
            f'PWM limited to {self._max_pwm}%; M3/M4 forced to zero'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.get_logger().error('Invalid cmd_vel; stopping M1 and M2')
            self._stop_motors()
            return

        motor_1, motor_2 = two_motor_pwm(
            msg.linear.x,
            msg.angular.z,
            self._max_pwm,
            self._linear_full_scale,
            self._angular_full_scale,
            self._enable_steering,
            self._motor_1_sign,
            self._motor_2_sign,
        )
        self._set_motor(motor_1, motor_2)
        self._last_command_ns = self.get_clock().now().nanoseconds
        self._stopped = motor_1 == 0 and motor_2 == 0
        self.get_logger().info(
            f'cmd_vel x={msg.linear.x:.2f} -> M1={motor_1}%, M2={motor_2}%'
        )

    def _check_command_timeout(self) -> None:
        command_age = (
            self.get_clock().now().nanoseconds - self._last_command_ns
        ) * 1e-9
        if not self._stopped and command_age >= self._command_timeout:
            self._stop_motors()
            self.get_logger().warning('Command timeout: M1 and M2 stopped')

    def _set_motor(self, motor_1: int, motor_2: int) -> None:
        self._driver.set_motor(motor_1, motor_2, 0, 0)

    def _stop_motors(self) -> None:
        self._set_motor(0, 0)
        self._stopped = True

    def stop(self) -> None:
        """Force all four board outputs to zero during shutdown."""
        try:
            self._stop_motors()
        except Exception as exc:
            self.get_logger().error(f'Failed to stop motors cleanly: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TwoMotorTestNode()
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
