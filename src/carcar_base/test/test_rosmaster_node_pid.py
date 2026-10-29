import math
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import Twist

from carcar_base.rosmaster_node import RosmasterNode


class _SerialStub:

    is_open = False

    def close(self):
        self.is_open = False


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

    def set_pwm_servo(self, servo_id, angle):
        self.servo_command = (servo_id, angle)
        return True

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

    def test_startup_sends_camera_level_command(self):
        self.assertEqual(self.node._driver.servo_command, (1, 94))

    def test_failed_camera_startup_aborts_driver(self):
        with patch.object(_RosmasterStub, 'set_pwm_servo', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'S1'):
                RosmasterNode()

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

    def _timing_odometry(self, samples):
        self.node._wheel_odometry_enabled = True
        self.node._wheel_counts_per_revolution = (1000.0,) * 4
        published = []
        self.node._odom_pub = type(
            'CapturePub', (), {'publish': published.append})()
        for seconds, counts in samples:
            stamp = self.node.get_clock().now().to_msg()
            self.node._update_wheel_odometry(
                stamp, int(seconds * 1_000_000_000), (counts,) * 4)
        return published

    def test_delayed_encoder_burst_keeps_distance_without_velocity_spike(self):
        messages = self._timing_odometry([
            (1.0, 0), (1.04, 20), (1.4, 20),
            (1.427, 160), (1.467, 180),
        ])
        self.assertLess(max(abs(m.twist.twist.linear.x)
                            for m in messages), 0.2)
        self.assertAlmostEqual(self.node._x, 180 * math.pi * 0.06 / 1000)
        self.assertEqual(len(messages), 2,
                         '延迟恢复中的未知速度不得伪报为零或发布尖峰')

    def test_long_poll_gap_does_not_discard_valid_encoder_distance(self):
        messages = self._timing_odometry([
            (1.0, 0), (1.04, 20), (1.4, 140),
            (1.427, 160), (1.467, 180),
        ])
        self.assertAlmostEqual(self.node._x, 180 * math.pi * 0.06 / 1000)
        self.assertEqual(len(messages), 2)

    def test_catchup_callbacks_cannot_publish_short_interval_spikes(self):
        messages = self._timing_odometry([
            (1.0, 0), (1.04, 20), (1.041, 40),
            (1.042, 60), (1.082, 80), (1.122, 100),
        ])
        self.assertEqual(len(messages), 2)
        self.assertLess(max(abs(m.twist.twist.linear.x)
                            for m in messages), 0.2)
        self.assertAlmostEqual(self.node._x, 100 * math.pi * 0.06 / 1000)

    def test_normal_fast_encoder_feedback_is_not_clipped(self):
        messages = self._timing_odometry([(1.0, 0), (1.04, 200)])
        self.assertEqual(len(messages), 1)
        self.assertGreater(messages[0].twist.twist.linear.x, 0.9,
                           '有效间隔内的原始速度不得被人为限幅')

    def test_wall_clock_jump_does_not_trigger_command_watchdog(self):
        clock = Mock()
        clock.now.return_value = Time(seconds=100)
        cmd = Twist()
        cmd.linear.x = 0.1
        with patch.object(self.node, 'get_clock', return_value=clock), \
                patch('carcar_base.rosmaster_node.time.monotonic_ns',
                      side_effect=[10_000_000_000, 10_050_000_000,
                                   10_050_000_000]), \
                patch.object(self.node._driver, 'set_car_motion') as motion, \
                patch.object(self.node._driver, 'get_motor_encoder',
                             return_value=(0,) * 4, create=True), \
                patch.object(self.node, '_publish_imu'), \
                patch.object(self.node, '_publish_board_state'):
            self.node._on_cmd_vel(cmd)
            clock.now.return_value = Time(seconds=101)
            self.node._update()
        self.assertEqual(motion.call_count, 1,
                         '只经过50ms，ROS时间前跳不得导致超时停车')

    def test_monotonic_command_timeout_still_stops_with_frozen_ros_clock(self):
        self.node._command_timeout = 0.3
        clock = Mock()
        clock.now.return_value = Time(seconds=100)
        cmd = Twist()
        cmd.linear.x = 0.1
        with patch.object(self.node, 'get_clock', return_value=clock), \
                patch('carcar_base.rosmaster_node.time.monotonic_ns',
                      side_effect=[10_000_000_000, 10_310_000_000,
                                   10_310_000_000]), \
                patch.object(self.node._driver, 'set_car_motion') as motion, \
                patch.object(self.node._driver, 'get_motor_encoder',
                             return_value=(0,) * 4, create=True), \
                patch.object(self.node, '_publish_imu'), \
                patch.object(self.node, '_publish_board_state'):
            self.node._on_cmd_vel(cmd)
            self.node._update()
        self.assertEqual(motion.call_args.args, (0.0, 0.0, 0.0))
        self.assertTrue(self.node._watchdog_stopped)


if __name__ == '__main__':

    unittest.main()
