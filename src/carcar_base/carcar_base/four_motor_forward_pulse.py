"""Run one bounded positive-PWM pulse on all four motor outputs."""

import time

import rclpy
from rclpy.node import Node

from Rosmaster_Lib import Rosmaster


class FourMotorForwardPulse(Node):
    """Directly exercise M1--M4 together, independently of normal bringup."""

    def __init__(self) -> None:
        super().__init__('four_motor_forward_pulse')
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('pwm_percent', 15)
        self.declare_parameter('pulse_duration', 0.20)
        self.declare_parameter('startup_delay', 2.0)

        serial_port = str(self.get_parameter('serial_port').value)
        self._pwm = int(self.get_parameter('pwm_percent').value)
        self._duration = float(self.get_parameter('pulse_duration').value)
        self._startup_delay = float(self.get_parameter('startup_delay').value)

        if not 1 <= self._pwm <= 20:
            raise ValueError('pwm_percent must be between 1 and 20')
        if not 0.05 <= self._duration <= 1.0:
            raise ValueError('pulse_duration must be between 0.05 and 1.0 s')
        if not 0.0 <= self._startup_delay <= 10.0:
            raise ValueError('startup_delay must be between 0.0 and 10.0 s')

        # Open the serial port only after every parameter has been validated.
        self._driver = Rosmaster(com=serial_port)
        self._stopped = False
        self.stop()
        self.get_logger().warning(
            f'四电机独立测试已连接 {serial_port}；M1～M4 当前保持零输出。'
        )

    def _set_all(self, pwm: int) -> None:
        self._driver.set_motor(pwm, pwm, pwm, pwm)

    def stop(self) -> None:
        """Send redundant zero commands to all four outputs."""
        for _ in range(3):
            self._set_all(0)
        self._stopped = True

    def run_once(self) -> None:
        """Wait for the safety delay, then issue exactly one bounded pulse."""
        self.get_logger().warning(
            f'{self._startup_delay:.1f} 秒后给 M1～M4 同时发送 '
            f'+{self._pwm}% PWM；持续 {self._duration:.2f} 秒。'
        )
        deadline = time.monotonic() + self._startup_delay
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(min(0.02, deadline - time.monotonic()))
        if not rclpy.ok():
            return

        started = time.monotonic()
        self._set_all(self._pwm)
        self._stopped = False
        try:
            deadline = started + self._duration
            while rclpy.ok() and time.monotonic() < deadline:
                time.sleep(min(0.01, deadline - time.monotonic()))
        finally:
            self.stop()
        elapsed = time.monotonic() - started
        self.get_logger().info(
            f'M1～M4 已归零；实际非零窗口约 {elapsed:.3f} 秒。'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FourMotorForwardPulse()
        node.run_once()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
