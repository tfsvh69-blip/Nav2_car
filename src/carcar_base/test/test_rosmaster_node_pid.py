import unittest
from unittest.mock import patch

import rclpy
from rclpy.parameter import Parameter

from carcar_base.rosmaster_node import RosmasterNode


class _SerialStub:

    is_open = False


class _RosmasterStub:

    def __init__(self, **kwargs):
        self.pid = [0.8, 0.06, 0.5]
        self.pid_writes = []
        self.ser = _SerialStub()

    def create_receive_threading(self):
        pass

    def set_auto_report_state(self, enabled, forever=False):
        pass

    def set_car_motion(self, vx, vy, wz):
        pass

    def get_motion_pid(self):
        return list(self.pid)

    def set_pid_param(self, kp, ki, kd, forever=False):
        self.pid = [kp, ki, kd]
        self.pid_writes.append((kp, ki, kd, forever))

    def get_gyroscope_data(self):
        return [0.001, -0.002, -0.15]

    def get_accelerometer_data(self):
        return [0.03, -0.04, -9.88]

    def set_beep(self, duration_ms):
        pass

    def set_colorful_lamps(self, index, r, g, b):
        pass


class RosmasterNodePidTest(unittest.TestCase):

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

    def test_pid_is_staged_then_applied_temporarily(self):
        result = self.node.set_parameters_atomically([
            Parameter('motion_kp', value=0.9),
            Parameter('motion_ki', value=0.08),
            Parameter('motion_kd', value=0.5),
        ])
        self.assertTrue(result.successful)
        self.assertEqual(self.node._driver.pid_writes, [])

        result = self.node.set_parameters_atomically([
            Parameter('motion_pid_apply', value=True),
        ])
        self.assertTrue(result.successful)
        self.assertEqual(
            self.node._driver.pid_writes[-1], (0.9, 0.08, 0.5, False))
        self.assertEqual(
            self.node._motion_pid_active, (0.9, 0.08, 0.5))

    def test_pid_apply_is_rejected_while_moving(self):
        self.node._watchdog_stopped = False
        result = self.node.set_parameters_atomically([
            Parameter('motion_kp', value=1.0),
            Parameter('motion_pid_apply', value=True),
        ])
        self.assertFalse(result.successful)
        self.assertEqual(self.node._driver.pid_writes, [])

    def test_pid_outside_safe_range_is_rejected(self):
        result = self.node.set_parameters_atomically([
            Parameter('motion_kp', value=10.1),
        ])
        self.assertFalse(result.successful)
        self.assertEqual(self.node._driver.pid_writes, [])

    def test_unimplemented_runtime_parameter_change_is_rejected(self):
        result = self.node.set_parameters_atomically([
            Parameter('max_linear_x', value=0.12),
        ])
        self.assertFalse(result.successful)
        self.assertEqual(self.node._max_linear_x, 0.5)

    def test_imu_signs_configured_and_published(self):
        self.assertEqual(self.node._imu_gyro_signs, (1, 1, -1))
        self.assertEqual(self.node._imu_accel_signs, (1, 1, -1))

        published_msgs = []
        self.node._imu_pub = type(
            'MockPub', (), {'publish': published_msgs.append})()
        stamp = self.node.get_clock().now().to_msg()
        self.node._publish_imu(stamp)
        self.assertEqual(len(published_msgs), 1)
        msg = published_msgs[0]
        self.assertAlmostEqual(msg.angular_velocity.x, 0.001)
        self.assertAlmostEqual(msg.angular_velocity.y, -0.002)
        # raw gz was -0.15, with sign -1 it should be +0.15
        self.assertAlmostEqual(msg.angular_velocity.z, 0.15)
        self.assertAlmostEqual(msg.linear_acceleration.x, 0.03)
        self.assertAlmostEqual(msg.linear_acceleration.y, -0.04)
        # raw az was -9.88, with sign -1 it should be +9.88
        self.assertAlmostEqual(msg.linear_acceleration.z, 9.88)


if __name__ == '__main__':

    unittest.main()
