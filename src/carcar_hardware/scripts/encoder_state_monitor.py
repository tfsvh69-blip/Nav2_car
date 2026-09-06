#!/usr/bin/env python3

# Copyright 2026 carcar maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""以易读格式显示只读 ros2_control 编码器状态."""

from typing import Dict, List, Optional, Tuple

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


EncoderState = Tuple[float, float]


def extract_encoder_state(
        message: DynamicJointState, joint_name: str) -> Optional[EncoderState]:
    """从动态关节消息提取指定关节的累计计数和计数变化率."""
    try:
        joint_index = message.joint_names.index(joint_name)
    except ValueError:
        return None

    if joint_index >= len(message.interface_values):
        return None

    interfaces = message.interface_values[joint_index]
    values: Dict[str, float] = dict(
        zip(interfaces.interface_names, interfaces.values))
    if 'encoder_count' not in values or 'encoder_velocity' not in values:
        return None
    return values['encoder_count'], values['encoder_velocity']


class EncoderStateMonitor(Node):
    """只订阅编码器状态，在数据变化时打印二路或四路反馈."""

    def __init__(self) -> None:
        super().__init__('encoder_state_monitor')
        self._last_state: Optional[Tuple[EncoderState, ...]] = None
        self._missing_interface_reported = False
        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            '只读观察器已启动；不会打开串口，也不会发送电机命令。')

    def _state_callback(self, message: DynamicJointState) -> None:
        four_motor_names = [f'motor_{number}_joint' for number in range(1, 5)]
        if all(name in message.joint_names for name in four_motor_names):
            names = four_motor_names
            labels = [f'M{number}' for number in range(1, 5)]
        else:
            names = ['left_wheel_joint', 'right_wheel_joint']
            labels = ['M1/旧左轮', 'M2/旧右轮']

        extracted: List[Optional[EncoderState]] = [
            extract_encoder_state(message, name) for name in names]
        if any(state is None for state in extracted):
            if not self._missing_interface_reported:
                self.get_logger().error(
                    '消息缺少二路或四路 encoder_count/encoder_velocity 接口。')
                self._missing_interface_reported = True
            return

        state = tuple(item for item in extracted if item is not None)
        if state == self._last_state:
            return

        fields = []
        for index, encoder in enumerate(state):
            delta = 0.0 if self._last_state is None else (
                encoder[0] - self._last_state[index][0])
            fields.append(
                f'{labels[index]} count={encoder[0]:.0f}, '
                f'delta={delta:+.0f}, velocity={encoder[1]:+.1f} count/s')
        label = '初始' if self._last_state is None else '变化'
        self.get_logger().info(f'{label} | ' + ' | '.join(fields))
        self._last_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
