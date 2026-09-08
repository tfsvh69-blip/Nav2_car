"""Bounded encoder probe for the Rosmaster board-motion PID wheel order."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from .math_utils import encoder_deltas, encoder_signs


COMMAND_TOPIC = '/chassis_motion_test/cmd_vel'
ENCODER_TOPIC = '/chassis_motion_test/encoders'
ENCODER_SIGNS = (-1, 1, -1, 1)


class BoardMotionMappingProbe(Node):
    """Publish two tiny body-motion pulses after validating the ROS graph."""

    def __init__(self) -> None:
        super().__init__('board_motion_mapping_probe')
        self.declare_parameter('pulse_duration', 0.25)
        self.declare_parameter('settle_duration', 0.60)
        self.declare_parameter('linear_y', 0.02)
        self.declare_parameter('angular_z', 0.10)
        self.declare_parameter('startup_timeout', 3.0)
        self._pulse_duration = float(
            self.get_parameter('pulse_duration').value)
        self._settle_duration = float(
            self.get_parameter('settle_duration').value)
        self._linear_y = float(self.get_parameter('linear_y').value)
        self._angular_z = float(self.get_parameter('angular_z').value)
        self._startup_timeout = float(
            self.get_parameter('startup_timeout').value)
        if not 0.10 <= self._pulse_duration <= 0.50:
            raise ValueError('pulse_duration must be within [0.10, 0.50]')
        if not 0.20 <= self._settle_duration <= 1.00:
            raise ValueError('settle_duration must be within [0.20, 1.00]')
        if not 0.01 <= self._linear_y <= 0.04:
            raise ValueError('linear_y must be within [0.01, 0.04]')
        if not 0.05 <= self._angular_z <= 0.15:
            raise ValueError('angular_z must be within [0.05, 0.15]')
        if not 1.0 <= self._startup_timeout <= 10.0:
            raise ValueError('startup_timeout must be within [1.0, 10.0]')

        self._publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)
        self._subscription = self.create_subscription(
            Int32MultiArray, ENCODER_TOPIC, self._on_encoders, 10)
        self._encoders = None
        self._encoder_stamp = None

    def _on_encoders(self, message: Int32MultiArray) -> None:
        if len(message.data) != 4:
            self.get_logger().error('编码器通道数不是四路，拒绝探测。')
            self._encoders = None
            return
        self._encoders = tuple(int(value) for value in message.data)
        self._encoder_stamp = time.monotonic()

    def _spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            timeout = min(0.02, deadline - time.monotonic())
            rclpy.spin_once(self, timeout_sec=timeout)

    def _publish_stop(self) -> None:
        if not rclpy.ok():
            return
        message = Twist()
        for _ in range(3):
            self._publisher.publish(message)
            self._spin_for(0.03)

    def _ready(self) -> bool:
        deadline = time.monotonic() + self._startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_for(0.05)
            if (self._publisher.get_subscription_count() == 1
                    and self.count_publishers(ENCODER_TOPIC) == 1
                    and self._encoders is not None
                    and time.monotonic() - self._encoder_stamp <= 0.25):
                return True
        self.get_logger().error(
            '拒绝探测：必须恰有一个板载 PID 驱动订阅命令并发布新鲜四路编码器。')
        return False

    def _run_pulse(self, label: str, command: Twist,
                   expected_raw, expected_physical) -> None:
        self._publish_stop()
        self._spin_for(0.10)
        start = self._encoders
        self.get_logger().warning(
            f'执行 {label}：最多 {self._pulse_duration:.2f} 秒；'
            f'期望原始符号={expected_raw}，'
            f'期望车轮符号={expected_physical}。')
        deadline = time.monotonic() + self._pulse_duration
        while rclpy.ok() and time.monotonic() < deadline:
            self._publisher.publish(command)
            self._spin_for(0.05)
        self._publish_stop()
        self._spin_for(self._settle_duration)
        end = self._encoders
        deltas = encoder_deltas(start, end)
        raw = encoder_signs(deltas)
        physical = encoder_signs(tuple(
            delta * sign for delta, sign in zip(deltas, ENCODER_SIGNS)))
        self.get_logger().warning(
            f'{label}已停车：Δ={deltas}，原始符号={raw}，'
            f'车轮符号={physical}。')

    def run(self) -> None:
        self._publish_stop()
        if not self._ready():
            self._publish_stop()
            return
        try:
            left = Twist()
            left.linear.y = self._linear_y
            self._run_pulse('左横移', left, ('+', '-', '-', '+'),
                            ('-', '-', '+', '+'))
            counter_clockwise = Twist()
            counter_clockwise.angular.z = self._angular_z
            self._run_pulse('逆时针旋转', counter_clockwise,
                            ('+', '+', '-', '-'), ('-', '+', '+', '-'))
        finally:
            self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BoardMotionMappingProbe()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
