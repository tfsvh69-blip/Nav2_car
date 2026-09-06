"""Publish one bounded velocity pulse followed by an explicit zero command."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


COMMAND_TOPIC = '/motor_closed_loop_test/cmd_vel'
LINEAR_X = 0.10
PULSE_DURATION = 0.20
DISCOVERY_TIMEOUT = 5.0


class SafeVelocityPulse(Node):
    """Send a fixed short pulse only to the isolated motor test topic."""

    def __init__(self) -> None:
        super().__init__('safe_velocity_pulse')
        self._publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)

    def wait_for_subscriber(self) -> bool:
        """Wait briefly for the isolated bottom-board test subscriber."""
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        while rclpy.ok() and time.monotonic() < deadline:
            if self._publisher.get_subscription_count() == 1:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def spin_for(self, duration: float) -> None:
        """Keep ROS responsive for a bounded monotonic-clock duration."""
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.02, remaining))

    def publish_velocity(self, linear_x: float) -> None:
        """Publish a longitudinal command with every other component zero."""
        message = Twist()
        message.linear.x = linear_x
        self._publisher.publish(message)

    def publish_zero_repeatedly(self) -> None:
        """Publish redundant explicit zero commands before returning."""
        for _ in range(3):
            self.publish_velocity(0.0)
            self.spin_for(0.03)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeVelocityPulse()
    exit_code = 0
    try:
        if not node.wait_for_subscriber():
            node.get_logger().error(
                f'{COMMAND_TOPIC} 必须恰好有一个订阅者，未发送非零命令。')
            exit_code = 1
        else:
            node.publish_zero_repeatedly()
            node.get_logger().info(
                f'发送 linear.x=+{LINEAR_X:.2f}，计划在 '
                f'{PULSE_DURATION:.2f} 秒后显式归零。')
            started = time.monotonic()
            node.publish_velocity(LINEAR_X)
            node.spin_for(PULSE_DURATION)
            node.publish_velocity(0.0)
            elapsed = time.monotonic() - started
            node.spin_for(0.03)
            node.publish_velocity(0.0)
            node.spin_for(0.03)
            node.publish_velocity(0.0)
            node.spin_for(0.03)
            node.get_logger().info(
                f'已发送显式零速；非零窗口约 {elapsed:.3f} 秒。')
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        if rclpy.ok():
            node.publish_zero_repeatedly()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
