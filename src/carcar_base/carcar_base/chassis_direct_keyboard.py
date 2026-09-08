"""Direct fixed-speed keyboard control for the board-motion PID test driver."""

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .math_utils import keyboard_speed_level


DEFAULT_COMMAND_TOPIC = '/chassis_motion_test/cmd_vel'


class ChassisDirectKeyboard(Node):
    """Publish one selected body velocity until the operator stops it."""

    def __init__(self) -> None:
        super().__init__('chassis_direct_keyboard')
        self.declare_parameter('linear_speed', 0.02)
        self.declare_parameter('angular_speed', 0.10)
        self.declare_parameter('command_rate', 10.0)
        self.declare_parameter('cmd_vel_topic', DEFAULT_COMMAND_TOPIC)

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(
            self.get_parameter('angular_speed').value)
        self._command_rate = float(
            self.get_parameter('command_rate').value)
        self._command_topic = str(
            self.get_parameter('cmd_vel_topic').value)
        self._validate_parameters()

        self._publisher = self.create_publisher(Twist, self._command_topic, 10)
        self._command = Twist()
        self._active_motion = None
        self._timer = self.create_timer(
            1.0 / self._command_rate, self._publish_selected_command)
        self._publish_stop()
        self.get_logger().info(
            '直接键盘控制已就绪：i 前进，, 后退，j 左移，l 右移；'
            'u/o 逆/顺时针，1～9 切换速度，空格/k 停车，q 退出。'
            '按一次方向键后持续运行。')

    def _validate_parameters(self) -> None:
        if not 0.01 <= self._linear_speed <= 0.45:
            raise ValueError('linear_speed must be between 0.01 and 0.45 m/s')
        if not 0.05 <= self._angular_speed <= 1.00:
            raise ValueError(
                'angular_speed must be between 0.05 and 1.00 rad/s')
        if not 5.0 <= self._command_rate <= 20.0:
            raise ValueError('command_rate must be between 5 and 20 Hz')

    def _publish_selected_command(self) -> None:
        self._publisher.publish(self._command)

    def _publish_stop(self) -> None:
        self._command = Twist()
        self._active_motion = None
        for _ in range(3):
            self._publisher.publish(self._command)
            time.sleep(0.03)

    def _start_motion(self, label: str, linear_x: float = 0.0,
                      linear_y: float = 0.0, angular_z: float = 0.0) -> None:
        self._active_motion = (label, linear_x, linear_y, angular_z)
        command = Twist()
        command.linear.x = linear_x * self._linear_speed
        command.linear.y = linear_y * self._linear_speed
        command.angular.z = angular_z * self._angular_speed
        self._command = command
        self.get_logger().warning(
            f'持续{label}：vx={command.linear.x:+.2f} m/s，'
            f'vy={command.linear.y:+.2f} m/s，'
            f'wz={command.angular.z:+.2f} rad/s；按空格/k 停车。')

    def _select_speed_level(self, key: str) -> None:
        self._linear_speed, self._angular_speed = keyboard_speed_level(key)
        self.get_logger().warning(
            f'速度档位 {key}：直线/横移={self._linear_speed:.2f} m/s，'
            f'旋转={self._angular_speed:.2f} rad/s。')
        if self._active_motion is not None:
            self._start_motion(*self._active_motion)

    def handle_key(self, key: str) -> bool:
        """Handle one key and return True only when the loop should exit."""
        if key == 'i':
            self._start_motion('前进', linear_x=1.0)
        elif key == ',':
            self._start_motion('后退', linear_x=-1.0)
        elif key == 'j':
            self._start_motion('左横移', linear_y=1.0)
        elif key == 'l':
            self._start_motion('右横移', linear_y=-1.0)
        elif key == 'u':
            self._start_motion('逆时针旋转', angular_z=1.0)
        elif key == 'o':
            self._start_motion('顺时针旋转', angular_z=-1.0)
        elif key in ('1', '2', '3', '4', '5', '6', '7', '8', '9'):
            self._select_speed_level(key)
        elif key in (' ', 'k'):
            self._publish_stop()
            self.get_logger().info('已停车。')
        elif key == 'q':
            self._publish_stop()
            self.get_logger().info('已停车并退出。')
            return True
        return False

    def stop(self) -> None:
        self._publish_stop()


def _read_key(settings, timeout: float) -> str:
    tty.setraw(sys.stdin.fileno())
    try:
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if readable else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisDirectKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = _read_key(settings, timeout=0.02)
            if key and node.handle_key(key):
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
