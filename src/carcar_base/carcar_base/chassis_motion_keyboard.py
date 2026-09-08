"""Keyboard entry for bounded chassis motions and encoder calibration."""

from dataclasses import dataclass
import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from .math_utils import (
    counts_per_revolution,
    encoder_deltas,
    encoder_signs,
)


DEFAULT_COMMAND_TOPIC = '/chassis_motion_test/cmd_vel'
DEFAULT_ENCODER_TOPIC = '/chassis_motion_test/encoders'
ENCODER_MAX_AGE_SECONDS = 0.25


@dataclass(frozen=True)
class Motion:
    """One permitted body-frame motion."""

    label: str
    linear_x_scale: float = 0.0
    linear_y_scale: float = 0.0
    angular_z_scale: float = 0.0


MOTIONS = {
    'i': Motion('前进', linear_x_scale=1.0),
    'j': Motion('左横移', linear_y_scale=1.0),
    'u': Motion('逆时针旋转', angular_z_scale=1.0),
}

WHEEL_LABELS = (
    'M1/左前轮',
    'M2/左后轮',
    'M3/右前轮',
    'M4/右后轮',
)


class ChassisMotionKeyboard(Node):
    """Publish only fixed low-speed pulses and display encoder changes."""

    def __init__(self) -> None:
        super().__init__('chassis_motion_keyboard')
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.40)
        self.declare_parameter('pulse_duration', 0.80)
        self.declare_parameter('command_rate', 10.0)
        self.declare_parameter('settle_duration', 0.40)
        self.declare_parameter('cmd_vel_topic', DEFAULT_COMMAND_TOPIC)
        self.declare_parameter('encoder_topic', DEFAULT_ENCODER_TOPIC)

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(
            self.get_parameter('angular_speed').value)
        self._pulse_duration = float(
            self.get_parameter('pulse_duration').value)
        self._command_rate = float(
            self.get_parameter('command_rate').value)
        self._settle_duration = float(
            self.get_parameter('settle_duration').value)
        self._command_topic = str(
            self.get_parameter('cmd_vel_topic').value)
        self._encoder_topic = str(
            self.get_parameter('encoder_topic').value)
        self._validate_parameters()

        self._publisher = self.create_publisher(Twist, self._command_topic, 10)
        self._encoder_subscription = self.create_subscription(
            Int32MultiArray,
            self._encoder_topic,
            self._on_encoders,
            10,
        )
        self._encoders = None
        self._encoder_update_ns = None
        self._calibration_motor = None
        self._calibration_start = None
        self._calibration_results = {}

    def _validate_parameters(self) -> None:
        if not 0.02 <= self._linear_speed <= 0.10:
            raise ValueError('linear_speed must be between 0.02 and 0.10 m/s')
        if not 0.10 <= self._angular_speed <= 0.50:
            raise ValueError(
                'angular_speed must be between 0.10 and 0.50 rad/s')
        if not 0.20 <= self._pulse_duration <= 2.00:
            raise ValueError(
                'pulse_duration must be between 0.20 and 2.00 s')
        if not 5.0 <= self._command_rate <= 20.0:
            raise ValueError('command_rate must be between 5 and 20 Hz')
        if not 0.20 <= self._settle_duration <= 1.00:
            raise ValueError(
                'settle_duration must be between 0.20 and 1.00 s')

    def _on_encoders(self, message: Int32MultiArray) -> None:
        if len(message.data) != 4:
            self.get_logger().error(
                '编码器消息不是 M1～M4 四个值；运动入口保持禁用。')
            self._encoders = None
            return
        self._encoders = tuple(int(value) for value in message.data)
        self._encoder_update_ns = self.get_clock().now().nanoseconds

    def _spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.02, remaining))

    def _publish_stop(self) -> None:
        # Ctrl+C may have already shut down the ROS context.  In that case
        # there is no live publisher to use, and publishing zero would only
        # turn a normal keyboard exit into a traceback.
        if not rclpy.ok():
            return
        message = Twist()
        for _ in range(3):
            self._publisher.publish(message)
            self._spin_for(0.03)

    def _discard_keys_until_released(self) -> bool:
        """Discard key-repeat and report whether a queued quit was seen."""
        quiet_deadline = time.monotonic() + 0.25
        quit_requested = False
        while rclpy.ok() and time.monotonic() < quiet_deadline:
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                key = os.read(sys.stdin.fileno(), 1).decode().lower()
                quit_requested = quit_requested or key == 'q'
                quiet_deadline = time.monotonic() + 0.25
            rclpy.spin_once(self, timeout_sec=0.0)
        return quit_requested

    def _graph_is_safe(self) -> bool:
        command_publishers = self.count_publishers(self._command_topic)
        command_subscribers = self._publisher.get_subscription_count()
        encoder_publishers = self.count_publishers(self._encoder_topic)
        if command_publishers != 1 or command_subscribers != 1:
            self.get_logger().error(
                f'拒绝动作：{self._command_topic} 当前有 '
                f'{command_publishers} 个发布者、{command_subscribers} 个订阅者；'
                '必须分别为 1（本键盘/测试驱动）。')
            return False
        if encoder_publishers != 1:
            self.get_logger().error(
                f'拒绝动作：{self._encoder_topic} 当前有 '
                f'{encoder_publishers} 个发布者，必须为 1。')
            return False
        return True

    def _encoder_is_fresh(self) -> bool:
        if self._encoders is None or self._encoder_update_ns is None:
            self.get_logger().error('尚未收到四轮编码器消息；拒绝动作。')
            return False
        age = (
            self.get_clock().now().nanoseconds - self._encoder_update_ns
        ) * 1e-9
        if age > ENCODER_MAX_AGE_SECONDS:
            self.get_logger().error(
                f'编码器 ROS 消息已超过 {age:.3f} 秒未更新；拒绝动作。')
            return False
        return True

    def _ready(self) -> bool:
        self._spin_for(0.05)
        return self._graph_is_safe() and self._encoder_is_fresh()

    def _motion_message(self, motion: Motion) -> Twist:
        message = Twist()
        message.linear.x = motion.linear_x_scale * self._linear_speed
        message.linear.y = motion.linear_y_scale * self._linear_speed
        message.angular.z = motion.angular_z_scale * self._angular_speed
        return message

    def run_motion(self, motion: Motion) -> None:
        """Run one bounded pulse, stop, then report encoder deltas."""
        if self._calibration_motor is not None:
            self.get_logger().error('10 圈计数正在进行；先完成或按 x 取消。')
            return
        if not self._ready():
            self._publish_stop()
            return

        self._publish_stop()
        self._spin_for(0.10)
        start = self._encoders
        command = self._motion_message(motion)
        self.get_logger().warning(
            f'执行“{motion.label}”：vx={command.linear.x:+.2f} m/s，'
            f'vy={command.linear.y:+.2f} m/s，'
            f'wz={command.angular.z:+.2f} rad/s，'
            f'最多 {self._pulse_duration:.2f} 秒。')

        deadline = time.monotonic() + self._pulse_duration
        period = 1.0 / self._command_rate
        next_publish = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_publish:
                    self._publisher.publish(command)
                    next_publish += period
                rclpy.spin_once(
                    self,
                    timeout_sec=min(0.02, max(0.0, deadline - now)),
                )
        finally:
            self._publish_stop()

        self._spin_for(self._settle_duration)
        if not self._encoder_is_fresh():
            return
        end = self._encoders
        deltas = encoder_deltas(start, end)
        signs = encoder_signs(deltas)
        self.get_logger().info(
            f'{motion.label}已停车；start={start}，end={end}，'
            f'Δ={deltas}，符号[M1,M2,M3,M4]={signs}。')

    def toggle_calibration(self, motor_number: int) -> None:
        """Capture the endpoints of one manual ten-revolution measurement."""
        if not self._ready():
            self._publish_stop()
            return
        if self._calibration_motor not in (None, motor_number):
            active_label = WHEEL_LABELS[self._calibration_motor - 1]
            self.get_logger().error(
                f'{active_label} 的测量尚未结束；'
                '再按对应数字结束，或按 x 取消。')
            return

        self._publish_stop()
        self._spin_for(0.10)
        wheel_label = WHEEL_LABELS[motor_number - 1]
        if self._calibration_motor is None:
            self._calibration_motor = motor_number
            self._calibration_start = self._encoders
            start_count = self._calibration_start[motor_number - 1]
            self.get_logger().warning(
                f'{wheel_label} 起点={start_count}。'
                '沿车辆前进时该轮的转向连续手转恰好 10 圈，然后再按同一数字键。')
            return

        start = self._calibration_start
        end = self._encoders
        deltas = encoder_deltas(start, end)
        delta = deltas[motor_number - 1]
        count_per_rev = counts_per_revolution(delta, 10)
        self._calibration_results[motor_number] = (
            start[motor_number - 1],
            end[motor_number - 1],
            delta,
            count_per_rev,
        )
        self.get_logger().info(
            f'{wheel_label} 10 圈完成：start={start[motor_number - 1]}，'
            f'end={end[motor_number - 1]}，Δ={delta:+d}，'
            f'count/rev={count_per_rev:.1f}；其他通道 Δ={deltas}。')
        self._calibration_motor = None
        self._calibration_start = None

    def cancel_calibration(self) -> None:
        """Discard an unfinished calibration endpoint."""
        if self._calibration_motor is None:
            self.get_logger().info('当前没有进行中的 10 圈计数。')
            return
        wheel_label = WHEEL_LABELS[self._calibration_motor - 1]
        self._calibration_motor = None
        self._calibration_start = None
        self.get_logger().warning(f'已取消 {wheel_label} 本次 10 圈计数。')

    def print_results(self) -> None:
        """Print all completed count-per-revolution results."""
        if not self._calibration_results:
            self.get_logger().info('尚无已完成的 10 圈计数结果。')
            return
        self.get_logger().info('已完成的输出轴计数结果：')
        for motor_number in sorted(self._calibration_results):
            start, end, delta, count_per_rev = self._calibration_results[
                motor_number]
            self.get_logger().info(
                f'  {WHEEL_LABELS[motor_number - 1]}: start={start}, '
                f'end={end}, Δ={delta:+d}, count/rev={count_per_rev:.1f}')

    @staticmethod
    def print_help() -> None:
        """Print the intentionally small key map."""
        print(
            '\n安全组合运动键盘（每次按键只执行一次固定短脉冲）\n'
            '  i：前进    j：左横移    u：逆时针旋转\n'
            '  空格/k：立即重复发送零速    q：停车并退出\n'
            '  1/2/3/4：对应 M1/M2/M3/M4 的 10 圈计数起点/终点\n'
            '  x：取消当前 10 圈计数    p：打印已完成计数    h：帮助\n',
            flush=True,
        )

    def run_keyboard(self) -> None:
        """Read one key at a time while servicing encoder callbacks."""
        if not sys.stdin.isatty():
            raise RuntimeError('必须在交互式终端中运行键盘入口')
        self._publish_stop()
        self.print_help()
        old_settings = termios.tcgetattr(sys.stdin.fileno())
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                readable, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not readable:
                    continue
                key = os.read(sys.stdin.fileno(), 1).decode().lower()
                if key in MOTIONS:
                    self.run_motion(MOTIONS[key])
                    if self._discard_keys_until_released():
                        break
                elif key in (' ', 'k'):
                    self._publish_stop()
                    self.get_logger().info('已重复发送零速。')
                elif key in ('1', '2', '3', '4'):
                    self.toggle_calibration(int(key))
                elif key == 'x':
                    self.cancel_calibration()
                elif key == 'p':
                    self.print_results()
                elif key == 'h':
                    self.print_help()
                elif key == 'q':
                    break
        finally:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
            self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ChassisMotionKeyboard()
        node.run_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
