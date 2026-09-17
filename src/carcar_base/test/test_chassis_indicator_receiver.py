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

import unittest
from unittest.mock import patch

from carcar_base.rosmaster_node import RosmasterNode
from carcar_interfaces.msg import IndicatorCommand
import rclpy


class _SerialStub:
    is_open = False


class _RosmasterStub:

    def __init__(self, **kwargs):
        self.ser = _SerialStub()
        self.motion_calls = []
        self.beep_calls = []
        self.lamp_calls = []

    def create_receive_threading(self):
        pass

    def set_auto_report_state(self, enabled, forever=False):
        pass

    def set_car_motion(self, vx, vy, wz):
        self.motion_calls.append((vx, vy, wz))

    def get_motion_pid(self):
        return [0.8, 0.06, 0.5]

    def get_gyroscope_data(self):
        return [0.0, 0.0, 0.0]

    def get_accelerometer_data(self):
        return [0.0, 0.0, -9.81]

    def get_battery_voltage(self):
        return 12.0

    def get_motor_encoder(self):
        return [0, 0, 0, 0]

    def set_beep(self, duration_ms):
        self.beep_calls.append(duration_ms)

    def set_colorful_lamps(self, index, r, g, b):
        self.lamp_calls.append((index, r, g, b))


class ChassisIndicatorReceiverTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        patcher = patch(
            'carcar_base.rosmaster_node.Rosmaster', _RosmasterStub)
        self.addCleanup(patcher.stop)
        patcher.start()
        self.node = RosmasterNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_startup_clears_indicator(self):
        # 验证节点初始化时主动清零蜂鸣与RGB灯
        self.assertIn(0, self.node._driver.beep_calls)
        self.assertIn((0xFF, 0, 0, 0), self.node._driver.lamp_calls)

    def test_indicator_command_handling_and_zero_motion_isolation(self):
        # 记录初始看门狗时间与状态
        initial_cmd_ns = self.node._last_command_ns
        initial_watchdog = self.node._watchdog_stopped
        initial_motion_calls_count = len(self.node._driver.motion_calls)

        cmd = IndicatorCommand()
        cmd.r = 30
        cmd.g = 15
        cmd.b = 0
        cmd.beep_trigger = True
        cmd.beep_duration_ms = 120

        self.node._on_indicator_cmd(cmd)

        # 验证硬件接口调用
        self.assertEqual(self.node._driver.lamp_calls[-1], (0xFF, 30, 15, 0))
        self.assertEqual(self.node._driver.beep_calls[-1], 120)

        # 验证严格零运动输出保证：不触碰运动看门狗与最后速度时间戳
        self.assertEqual(self.node._last_command_ns, initial_cmd_ns)
        self.assertEqual(self.node._watchdog_stopped, initial_watchdog)
        self.assertEqual(
            len(self.node._driver.motion_calls),
            initial_motion_calls_count,
        )

    def test_duration_filtering_and_clamping(self):
        # 持续常鸣值 1 必须被拒绝并重置为 0
        cmd = IndicatorCommand()
        cmd.r = 0
        cmd.g = 30
        cmd.b = 0
        cmd.beep_trigger = True
        cmd.beep_duration_ms = 1

        initial_beeps = len(self.node._driver.beep_calls)
        self.node._on_indicator_cmd(cmd)
        # 蜂鸣器应被过滤，不执行触发
        self.assertEqual(len(self.node._driver.beep_calls), initial_beeps)

        # 非10整数倍向下对齐 (125 -> 120)
        cmd.beep_duration_ms = 125
        self.node._on_indicator_cmd(cmd)
        self.assertEqual(self.node._driver.beep_calls[-1], 120)

        # 超过 3000ms 截断为 3000ms
        cmd.beep_duration_ms = 4500
        self.node._on_indicator_cmd(cmd)
        self.assertEqual(self.node._driver.beep_calls[-1], 3000)

    def test_indicator_auto_shutoff_on_timeout(self):
        cmd = IndicatorCommand()
        cmd.r = 30
        cmd.g = 0
        cmd.b = 0
        cmd.beep_trigger = False
        cmd.beep_duration_ms = 0
        self.node._on_indicator_cmd(cmd)
        self.assertTrue(self.node._indicator_active)

        # 模拟 1.1s 无新指令
        past_ns = self.node.get_clock().now().nanoseconds - int(1.1 * 1e9)
        self.node._last_indicator_cmd_ns = past_ns

        self.node._update()

        # 验证超时后自动清零并熄灭
        self.assertEqual(self.node._driver.beep_calls[-1], 0)
        self.assertEqual(self.node._driver.lamp_calls[-1], (0xFF, 0, 0, 0))
        self.assertFalse(self.node._indicator_active)

    def test_stop_clears_indicator(self):
        self.node.stop()
        self.assertEqual(self.node._driver.beep_calls[-1], 0)
        self.assertEqual(self.node._driver.lamp_calls[-1], (0xFF, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()

