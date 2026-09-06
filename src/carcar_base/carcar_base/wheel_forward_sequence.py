"""Retest wheels in mechanical-forward order with the confirmed mapping."""

import rclpy

from .motor_mapping_sequence import MotorMappingSequence


WHEEL_SEQUENCE = (
    (1, '左上角（左前轮）', 1),
    (3, '右上角（右前轮）', 1),
    (4, '左下角（左后轮）', -1),
    (2, '右下角（右后轮）', -1),
)


class WheelForwardSequence(MotorMappingSequence):
    """Run each physical wheel forward for three seconds in spatial order."""

    def __init__(self) -> None:
        super().__init__('wheel_forward_sequence', '四轮正转复测')

    def run_once(self) -> None:
        """Run left-front, right-front, left-rear and right-rear forward."""
        self.get_logger().warning(
            f'{self._startup_delay:.1f} 秒后开始；顺序为左上→右上→左下→右下，'
            '每个轮子机械正转 3.0 秒。'
        )
        if not self._wait_zero_output(self._startup_delay):
            return

        for index, (motor_number, wheel_label, command_sign) in enumerate(
            WHEEL_SEQUENCE
        ):
            self._run_motor(
                motor_number,
                command_sign * self._pwm,
                wheel_label,
            )
            if not rclpy.ok():
                return
            if index < len(WHEEL_SEQUENCE) - 1:
                next_wheel = WHEEL_SEQUENCE[index + 1][1]
                self.get_logger().info(
                    f'保持全零 {self._inter_motor_delay:.1f} 秒，'
                    f'随后测试{next_wheel}。'
                )
                if not self._wait_zero_output(self._inter_motor_delay):
                    return

        self.get_logger().info('四个轮子正转复测完成，M1～M4 保持零输出。')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WheelForwardSequence()
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
