"""APP-001 isolated serial adapter tests; no real serial or actuator access."""
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from geometry_msgs.msg import Twist
import rclpy
from carcar_interfaces.srv import SetCameraServo, SetMotionHold
from carcar_base.rosmaster_node import RosmasterNode


class BoardStub:
    """Replace the existing vendor library, including all serial writes."""

    def __init__(self, **kwargs):
        self.motion = []
        self.servo = []
        self.ser = type('SerialStub', (), {
            'fileno': lambda self: -1, 'close': lambda self: None})()

    def create_receive_threading(self):
        pass

    def set_auto_report_state(self, *args, **kwargs):
        pass

    def set_car_motion(self, *values):
        self.motion.append(values)

    def set_pwm_servo(self, *values):
        self.servo.append(values)
        return True

    def get_motion_pid(self):
        return [8.0, 1.2, 0.8]

    def set_beep(self, *args):
        pass

    def set_colorful_lamps(self, *args):
        pass


class InspectionServoReceiverTest(unittest.TestCase):
    """Verify hold ownership, bounded writes and request replay rejection."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        patcher = patch('carcar_base.rosmaster_node.Rosmaster', BoardStub)
        self.addCleanup(patcher.stop)
        patcher.start()
        self.node = RosmasterNode()
        self.owner = 'owner_0123456789abcdef'
        self.epoch = self.node._inspection_driver_epoch

    def tearDown(self):
        self.node.destroy_node()

    def hold(self, enabled=True, owner=None, epoch=None):
        req = SetMotionHold.Request()
        req.owner_token = owner or self.owner
        req.driver_epoch = self.epoch if epoch is None else epoch
        req.hold = enabled
        return self.node._on_inspection_hold(req, SetMotionHold.Response())

    def command(self, board=95, expected=94):
        req = SetCameraServo.Request()
        req.owner_token = self.owner
        req.driver_epoch = self.epoch
        req.command_id = 'command_0123456789abcdef'
        req.expected_board_command = expected
        req.board_command = board
        req.deadline_steady_ns = time.monotonic_ns() + 400_000_000
        return req

    def send(self, request):
        return self.node._on_inspection_servo(
            request, SetCameraServo.Response())

    def test_disabled_by_default_and_no_second_serial_owner(self):
        self.assertFalse(hasattr(self.node, '_inspection_servo_service'))
        self.assertFalse(self.send(self.command()).sent)

    def test_hold_blocks_nonzero_velocity_and_stays_latched(self):
        self.assertTrue(self.hold().accepted)
        cmd = Twist()
        cmd.linear.x = 0.1
        self.node._on_cmd_vel(cmd)
        self.assertEqual(self.node._driver.motion[-1], (0.0, 0.0, 0.0))
        self.assertEqual(self.node._inspection_hold_owner, self.owner)

    def test_single_step_and_duplicate_id_write_only_once(self):
        self.hold()
        request = self.command()
        self.assertTrue(self.send(request).sent)
        count = len(self.node._driver.servo)
        self.assertTrue(self.send(request).sent)
        self.assertEqual(len(self.node._driver.servo), count)

    def test_expired_jump_and_upper_quantization_are_rejected(self):
        self.hold()
        request = self.command()
        request.deadline_steady_ns = time.monotonic_ns() - 1
        self.assertFalse(self.send(request).sent)
        self.assertFalse(self.send(self.command(96)).sent)
        self.assertFalse(self.send(self.command(107)).sent)

    def test_wrong_epoch_owner_and_reused_payload_are_rejected(self):
        self.hold()
        request = self.command()
        request.driver_epoch = 'old_epoch'
        self.assertFalse(self.send(request).sent)
        request.driver_epoch = self.epoch
        request.owner_token = 'different_owner_0123456789'
        self.assertFalse(self.send(request).sent)
        request.owner_token = self.owner
        self.assertTrue(self.send(request).sent)
        request.board_command = 94
        self.assertFalse(self.send(request).sent)

    def test_hold_cannot_release_away_from_navigation_angle(self):
        self.hold()
        self.assertTrue(self.send(self.command()).sent)
        self.assertFalse(self.hold(False).accepted)
        request = self.command(94, 95)
        request.command_id = 'return_0123456789abcdef'
        self.assertTrue(self.send(request).sent)
        self.assertTrue(self.hold(False).accepted)
        self.assertFalse(self.hold(
            owner='another_owner_0123456789', epoch='old').accepted)

    def test_failed_serial_write_keeps_previous_command_and_hold(self):
        self.hold()
        with patch.object(
                self.node._driver, 'set_pwm_servo', return_value=False):
            self.assertFalse(self.send(self.command()).sent)
        self.assertEqual(self.node._inspection_board_command, 94)
        self.assertEqual(self.node._inspection_hold_owner, self.owner)


class EnabledInspectionInterfaceTest(unittest.TestCase):
    """Enable real ROS services while replacing every hardware write."""

    def test_real_fuser_allows_unused_path_before_stub_serial_open(self):
        with tempfile.TemporaryDirectory() as directory:
            serial_path = Path(directory) / 'serial-stub'
            serial_path.touch()
            rclpy.init(args=[
                '--ros-args', '-p', 'enable_inspection_interface:=true',
                '-p', f'serial_port:={serial_path}'])
            node = None
            try:
                with patch('carcar_base.rosmaster_node.Rosmaster', BoardStub), \
                        patch('carcar_base.rosmaster_node.fcntl.ioctl'):
                    node = RosmasterNode()
                self.assertTrue(hasattr(node, '_inspection_servo_service'))
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()

    def test_real_fuser_rejects_open_file_before_vendor_constructor(self):
        with tempfile.NamedTemporaryFile() as serial_file:
            rclpy.init(args=[
                '--ros-args', '-p', 'enable_inspection_interface:=true',
                '-p', f'serial_port:={serial_file.name}'])
            try:
                with patch('carcar_base.rosmaster_node.Rosmaster') as board:
                    with self.assertRaisesRegex(RuntimeError, '串口已占用'):
                        RosmasterNode()
                    board.assert_not_called()
            finally:
                rclpy.shutdown()

    def test_explicit_parameter_creates_both_services(self):
        rclpy.init(args=[
            '--ros-args', '-p', 'enable_inspection_interface:=true'])
        node = None
        idle_check = subprocess.CompletedProcess([], 1, '', '')
        try:
            with patch('carcar_base.rosmaster_node.Rosmaster', BoardStub), \
                    patch('carcar_base.rosmaster_node.subprocess.run',
                          return_value=idle_check), \
                    patch('carcar_base.rosmaster_node.fcntl.ioctl') \
                    as exclusive:
                node = RosmasterNode()
                exclusive.assert_called_once()
            self.assertEqual(
                node._inspection_hold_service.srv_name,
                '/hardware/inspection/motion_hold')
            self.assertEqual(
                node._inspection_servo_service.srv_name,
                '/hardware/inspection/camera_servo')
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def test_busy_serial_is_rejected_before_vendor_constructor(self):
        rclpy.init(args=[
            '--ros-args', '-p', 'enable_inspection_interface:=true'])
        busy_check = subprocess.CompletedProcess([], 0, '123', '')
        try:
            with patch('carcar_base.rosmaster_node.Rosmaster') as board, \
                    patch('carcar_base.rosmaster_node.subprocess.run',
                          return_value=busy_check):
                with self.assertRaisesRegex(RuntimeError, '串口已占用'):
                    RosmasterNode()
                board.assert_not_called()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
