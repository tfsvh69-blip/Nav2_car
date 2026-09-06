"""Identify the physical wheel and direction of motor ports M1 through M4."""

import threading
import time

import rclpy
from rclpy.node import Node

from Rosmaster_Lib import Rosmaster

from .math_utils import single_motor_command, single_motor_pwm


MOTOR_DURATION_SECONDS = 3.0
ZERO_COMMAND = (0, 0, 0, 0)


class MotorMappingSequence(Node):
    """Run one bounded positive-PWM pulse on each motor port in order."""

    def __init__(
        self,
        node_name: str = 'motor_mapping_sequence',
        test_label: str = '逐轮映射测试',
    ) -> None:
        super().__init__(node_name)
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('pwm_percent', 15)
        self.declare_parameter('startup_delay', 3.0)
        self.declare_parameter('inter_motor_delay', 2.0)

        serial_port = str(self.get_parameter('serial_port').value)
        self._pwm = int(self.get_parameter('pwm_percent').value)
        self._startup_delay = float(self.get_parameter('startup_delay').value)
        self._inter_motor_delay = float(
            self.get_parameter('inter_motor_delay').value
        )

        # Validate values before opening the only control-board serial port.
        single_motor_pwm(1, self._pwm)
        if not 1.0 <= self._startup_delay <= 10.0:
            raise ValueError('startup_delay must be between 1.0 and 10.0 s')
        if not 1.0 <= self._inter_motor_delay <= 10.0:
            raise ValueError(
                'inter_motor_delay must be between 1.0 and 10.0 s'
            )

        self._io_lock = threading.RLock()
        self._driver = Rosmaster(com=serial_port)
        self._watchdog_generation = 0
        self._watchdog_timer = None
        self._watchdog_event = threading.Event()
        self._closed = False
        self.stop()
        self.get_logger().warning(
            f'{test_label}已连接 {serial_port}；M1～M4 当前保持零输出。'
        )

    def stop(self) -> None:
        """Cancel nonzero output and redundantly command all motors to stop."""
        with self._io_lock:
            self._watchdog_generation += 1
            if self._watchdog_timer is not None:
                self._watchdog_timer.cancel()
                self._watchdog_timer = None
            for _ in range(3):
                self._driver.set_motor(*ZERO_COMMAND)
            self._watchdog_event.set()

    def _watchdog_stop(self, generation: int, motor_number: int) -> None:
        with self._io_lock:
            if generation != self._watchdog_generation:
                return
            for _ in range(3):
                self._driver.set_motor(*ZERO_COMMAND)
            self._watchdog_timer = None
        self.get_logger().info(
            f'M{motor_number} 的 {MOTOR_DURATION_SECONDS:.1f} 秒命令已超时，'
            'M1～M4 已归零。'
        )
        self._watchdog_event.set()

    def _wait_zero_output(self, duration: float) -> bool:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return rclpy.ok()

    def _run_motor(
        self,
        motor_number: int,
        pwm: int = None,
        wheel_label: str = None,
    ) -> None:
        command_pwm = self._pwm if pwm is None else pwm
        command = single_motor_command(motor_number, command_pwm)
        display_name = f'M{motor_number}'
        if wheel_label is not None:
            display_name = f'{wheel_label} M{motor_number}'
        self._watchdog_event.clear()
        with self._io_lock:
            self._watchdog_generation += 1
            generation = self._watchdog_generation
            timer = threading.Timer(
                MOTOR_DURATION_SECONDS,
                self._watchdog_stop,
                args=(generation, motor_number),
            )
            timer.daemon = True
            self._watchdog_timer = timer
            timer.start()
            self._driver.set_motor(*command)

        self.get_logger().warning(
            f'现在仅 {display_name} 输出 {command_pwm:+d}% PWM，'
            f'最多持续 {MOTOR_DURATION_SECONDS:.1f} 秒；请记录实际轮位和方向。'
        )
        deadline = time.monotonic() + MOTOR_DURATION_SECONDS + 0.25
        while rclpy.ok() and not self._watchdog_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.get_logger().error(
                    f'M{motor_number} 安全看门狗响应超时，立即执行主线程停车。'
                )
                break
            self._watchdog_event.wait(min(0.05, remaining))
        self.stop()

    def run_once(self) -> None:
        """Run M1, M2, M3 and M4 once, with zero-output gaps."""
        self.get_logger().warning(
            f'{self._startup_delay:.1f} 秒后开始；顺序为 M1→M2→M3→M4，'
            f'每路正 PWM {MOTOR_DURATION_SECONDS:.1f} 秒，路间全零 '
            f'{self._inter_motor_delay:.1f} 秒。'
        )
        if not self._wait_zero_output(self._startup_delay):
            return

        for motor_number in range(1, 5):
            self._run_motor(motor_number)
            if not rclpy.ok():
                return
            if motor_number < 4:
                self.get_logger().info(
                    f'保持全零 {self._inter_motor_delay:.1f} 秒，'
                    f'随后测试 M{motor_number + 1}。'
                )
                if not self._wait_zero_output(self._inter_motor_delay):
                    return

        self.get_logger().info('M1～M4 逐轮测试完成，四路保持零输出。')

    def close(self) -> None:
        """Stop every output and explicitly release the serial port."""
        if self._closed:
            return
        try:
            self.stop()
        finally:
            serial_port = getattr(self._driver, 'ser', None)
            if serial_port is not None and serial_port.is_open:
                serial_port.close()
            self._closed = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MotorMappingSequence()
        node.run_once()
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
