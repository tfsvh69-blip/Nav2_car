import math
import unittest

from carcar_base.math_utils import (
    apply_imu_signs,
    clamp,
    counts_per_revolution,
    encoder_deltas,
    encoder_signs,
    integrate_body_delta,
    integrate_body_twist,
    is_zero_motion_command,
    keyboard_speed_level,
    mecanum_body_delta_from_encoder_counts,
    mecanum_motor_pwm,
    mecanum_port_targets,
    slew_towards,
    single_motor_command,
    single_motor_pwm,
    two_motor_pwm,
    velocity_pi_pwm,
    validate_imu_signs,
    validate_motion_pid,
    yaw_to_quaternion,
)


class MathUtilsTest(unittest.TestCase):

    def test_explicit_port_mapping(self) -> None:
        current = ('left_front', 'right_rear', 'right_front', 'left_rear')
        standard = ('left_front', 'left_rear', 'right_front', 'right_rear')
        for command, expected in (
            ((1, 0, 0), (100, 100, 100, 100)),
            ((0, 1, 0), (-100, -100, 100, 100)),
            ((0, 0, 1), (-100, 100, 100, -100)),
        ):
            actual = mecanum_port_targets(*command, 100, 1, 1, current)
            self.assertEqual(actual, expected)
            remapped = mecanum_port_targets(*command, 100, 1, 1, standard)
            self.assertEqual(
                remapped, (expected[0], expected[3], expected[2], expected[1]))

    def test_invalid_port_mapping(self) -> None:
        for mapping in ((), ('left_front',) * 4,
                        ('left_front', 'right_front', 'left_rear', 'unknown')):
            with self.assertRaises(ValueError):
                mecanum_port_targets(0, 0, 0, 100, 1, 1, mapping)

    def test_clamp(self) -> None:
        self.assertEqual(clamp(2.0, -1.0, 1.0), 1.0)
        self.assertEqual(clamp(-2.0, -1.0, 1.0), -1.0)
        self.assertEqual(clamp(0.25, -1.0, 1.0), 0.25)

    def test_slew_towards_limits_each_step(self) -> None:
        self.assertEqual(slew_towards(0.0, 10.0, 3.0), 3.0)
        self.assertEqual(slew_towards(9.0, 10.0, 3.0), 10.0)
        self.assertEqual(slew_towards(5.0, -5.0, 4.0), 1.0)
        with self.assertRaises(ValueError):
            slew_towards(0.0, 1.0, -0.1)

    def test_integrate_forward_at_quarter_turn(self) -> None:
        x, y, yaw = integrate_body_twist(
            0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0, 1.0
        )
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_integrate_body_delta_uses_midpoint_heading(self) -> None:
        x, y, yaw = integrate_body_delta(
            0.0, 0.0, 0.0, 1.0, 0.0, math.pi / 2
        )
        self.assertAlmostEqual(x, math.sqrt(0.5))
        self.assertAlmostEqual(y, math.sqrt(0.5))
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_mecanum_encoder_counts_produce_forward_delta(self) -> None:
        delta = mecanum_body_delta_from_encoder_counts(
            (100, 100, 100, 100),
            1.0 / math.pi,
            1.0,
            1.0,
            (100.0, 100.0, 100.0, 100.0),
            (1, 1, 1, 1),
            ('left_front', 'left_rear', 'right_front', 'right_rear'),
        )
        self.assertEqual(delta, (1.0, 0.0, 0.0))

    def test_mecanum_encoder_counts_produce_lateral_delta(self) -> None:
        delta = mecanum_body_delta_from_encoder_counts(
            (-100, 100, 100, -100),
            1.0 / math.pi,
            1.0,
            1.0,
            (100.0, 100.0, 100.0, 100.0),
            (1, 1, 1, 1),
            ('left_front', 'left_rear', 'right_front', 'right_rear'),
        )
        self.assertEqual(delta, (0.0, 1.0, 0.0))

    def test_mecanum_encoder_counts_produce_counterclockwise_delta(self):
        delta = mecanum_body_delta_from_encoder_counts(
            (-100, -100, 100, 100),
            1.0 / math.pi,
            1.0,
            1.0,
            (100.0, 100.0, 100.0, 100.0),
            (1, 1, 1, 1),
            ('left_front', 'left_rear', 'right_front', 'right_rear'),
        )
        self.assertEqual(delta, (0.0, 0.0, 1.0))

    def test_mecanum_encoder_odometry_rejects_missing_calibration(self):
        with self.assertRaises(ValueError):
            mecanum_body_delta_from_encoder_counts(
                (1, 1, 1, 1),
                0.06,
                0.12,
                0.185,
                (0.0, 0.0, 0.0, 0.0),
                (1, 1, 1, 1),
                ('left_front', 'left_rear', 'right_front', 'right_rear'),
            )

    def test_yaw_quaternion_is_normalized(self) -> None:
        quaternion = yaw_to_quaternion(1.23)
        self.assertAlmostEqual(sum(value * value for value in quaternion), 1.0)

    def test_two_motor_forward_and_reverse(self) -> None:
        arguments = (20, 0.5, 1.0, False, 1, 1)
        self.assertEqual(two_motor_pwm(0.5, 0.0, *arguments), (20, 20))
        self.assertEqual(two_motor_pwm(-0.5, 0.0, *arguments), (-20, -20))
        self.assertEqual(two_motor_pwm(0.0, 0.0, *arguments), (0, 0))

    def test_two_motor_sign_calibration(self) -> None:
        self.assertEqual(
            two_motor_pwm(0.5, 0.0, 20, 0.5, 1.0, False, 1, -1),
            (20, -20),
        )

    def test_two_motor_outputs_stay_bounded(self) -> None:
        motor_1, motor_2 = two_motor_pwm(
            0.5, 1.0, 20, 0.5, 1.0, True, 1, 1
        )
        self.assertLessEqual(abs(motor_1), 20)
        self.assertLessEqual(abs(motor_2), 20)

    def test_single_motor_pwm_sequence(self) -> None:
        self.assertEqual(single_motor_pwm(1, 15), (15, 0, 0, 0))
        self.assertEqual(single_motor_pwm(2, 15), (0, 15, 0, 0))
        self.assertEqual(single_motor_pwm(3, 15), (0, 0, 15, 0))
        self.assertEqual(single_motor_pwm(4, 15), (0, 0, 0, 15))

    def test_single_motor_pwm_rejects_unsafe_values(self) -> None:
        with self.assertRaises(ValueError):
            single_motor_pwm(0, 15)
        with self.assertRaises(ValueError):
            single_motor_pwm(5, 15)
        with self.assertRaises(ValueError):
            single_motor_pwm(1, 0)
        with self.assertRaises(ValueError):
            single_motor_pwm(1, 21)

    def test_single_motor_signed_commands_follow_wheel_mapping(self) -> None:
        self.assertEqual(single_motor_command(1, 15), (15, 0, 0, 0))
        self.assertEqual(single_motor_command(3, 15), (0, 0, 15, 0))
        self.assertEqual(single_motor_command(4, -15), (0, 0, 0, -15))
        self.assertEqual(single_motor_command(2, -15), (0, -15, 0, 0))

    def test_single_motor_command_rejects_unsafe_values(self) -> None:
        with self.assertRaises(ValueError):
            single_motor_command(1, 0)
        with self.assertRaises(ValueError):
            single_motor_command(1, -21)

    def test_zero_motion_command_detection(self) -> None:
        self.assertTrue(is_zero_motion_command(0.0, -0.0, 0.0))
        self.assertFalse(is_zero_motion_command(0.01, 0.0, 0.0))
        self.assertFalse(is_zero_motion_command(0.0, 0.01, 0.0))
        self.assertFalse(is_zero_motion_command(0.0, 0.0, -0.01))

    def test_keyboard_speed_levels(self) -> None:
        self.assertEqual(keyboard_speed_level('1'), (0.03, 0.10))
        self.assertEqual(keyboard_speed_level('5'), (0.15, 0.40))
        self.assertEqual(keyboard_speed_level('9'), (0.45, 1.00))
        with self.assertRaises(ValueError):
            keyboard_speed_level('0')

    def test_motion_pid_validation(self) -> None:
        self.assertEqual(validate_motion_pid(0.8, 0.06, 0.5),
                         (0.8, 0.06, 0.5))
        self.assertEqual(validate_motion_pid(10.0, 10.0, 10.0),
                         (10.0, 10.0, 10.0))
        for values in ((-0.1, 0.0, 0.0), (10.1, 0.0, 0.0),
                       (0.8, 10.1, 0.5), (0.8, 0.06, 10.1),
                       (float('nan'), 0.0, 0.0)):
            with self.assertRaises(ValueError):
                validate_motion_pid(*values)

    def test_encoder_delta_and_sign_summary(self) -> None:
        deltas = encoder_deltas(
            (100, -200, 300, -400),
            (110, -220, 300, -360),
        )
        self.assertEqual(deltas, (10, -20, 0, 40))
        self.assertEqual(encoder_signs(deltas), ('+', '-', '0', '+'))

    def test_encoder_helpers_reject_wrong_channel_count(self) -> None:
        with self.assertRaises(ValueError):
            encoder_deltas((1, 2, 3), (4, 5, 6))
        with self.assertRaises(ValueError):
            encoder_signs((1, 2, 3))

    def test_ten_revolution_count(self) -> None:
        self.assertEqual(counts_per_revolution(-13205, 10), 1320.5)
        with self.assertRaises(ValueError):
            counts_per_revolution(13200, 0)

    def test_mecanum_commands_use_confirmed_port_mapping(self) -> None:
        arguments = (15, 0.08, 0.40, (1, -1, 1, -1))
        self.assertEqual(
            mecanum_motor_pwm(0.08, 0.0, 0.0, *arguments),
            (15, -15, 15, -15),
        )
        self.assertEqual(
            mecanum_motor_pwm(0.0, 0.08, 0.0, *arguments),
            (-15, 15, 15, -15),
        )
        self.assertEqual(
            mecanum_motor_pwm(0.0, 0.0, 0.40, *arguments),
            (-15, -15, 15, 15),
        )

    def test_mecanum_commands_are_normalized_and_bounded(self) -> None:
        command = mecanum_motor_pwm(
            0.08, 0.08, 0.40, 15, 0.08, 0.40, (1, -1, 1, -1))
        self.assertEqual(command, (-5, -5, 15, -5))
        self.assertLessEqual(max(abs(value) for value in command), 15)

    def test_mecanum_rejects_invalid_motor_signs(self) -> None:
        with self.assertRaises(ValueError):
            mecanum_motor_pwm(
                0.0, 0.0, 0.0, 15, 0.08, 0.40, (1, -1, 1, 0))

    def test_mecanum_count_rate_targets_follow_port_order(self) -> None:
        arguments = (700.0, 0.08, 0.40)
        self.assertEqual(
            mecanum_port_targets(0.08, 0.0, 0.0, *arguments),
            (700.0, 700.0, 700.0, 700.0),
        )
        self.assertEqual(
            mecanum_port_targets(0.0, 0.08, 0.0, *arguments),
            (-700.0, -700.0, 700.0, 700.0),
        )
        self.assertEqual(
            mecanum_port_targets(0.0, 0.0, 0.40, *arguments),
            (-700.0, 700.0, 700.0, -700.0),
        )

    def test_velocity_pi_uses_feedback_and_motor_sign(self) -> None:
        self.assertEqual(
            velocity_pi_pwm(700.0, 0.0, 0.0, 10.0, 0.01, 0.02, 25, 1),
            17,
        )
        self.assertEqual(
            velocity_pi_pwm(700.0, 0.0, 0.0, 10.0, 0.01, 0.02, 25, -1),
            -17,
        )
        self.assertEqual(
            velocity_pi_pwm(-700.0, 0.0, 0.0, 10.0, 0.01, 0.02, 25, 1),
            -17,
        )
        self.assertEqual(
            velocity_pi_pwm(700.0, 700.0, 0.0, 10.0, 0.01, 0.02, 25, 1),
            10,
        )

    def test_velocity_pi_never_reverses_to_brake(self) -> None:
        self.assertEqual(
            velocity_pi_pwm(700.0, 3000.0, 0.0, 5.0, 0.02, 0.0, 25, 1),
            0,
        )
        self.assertEqual(
            velocity_pi_pwm(-700.0, -3000.0, 0.0, 5.0, 0.02, 0.0, 25, 1),
            0,
        )

    def test_velocity_pi_allows_high_output_probe_limit(self) -> None:
        self.assertEqual(
            velocity_pi_pwm(1000.0, 0.0, 0.0, 34.0, 0.025, 0.04, 45, 1),
            45,
        )
        with self.assertRaises(ValueError):
            velocity_pi_pwm(1000.0, 0.0, 0.0, 34.0, 0.025, 0.04, 46, 1)

    def test_validate_imu_signs(self) -> None:
        self.assertEqual(validate_imu_signs((1, 1, -1)), (1, 1, -1))
        self.assertEqual(validate_imu_signs([1, -1, 1]), (1, -1, 1))
        for invalid in ((1, 1), (1, 1, 1, 1), (1, 0, -1), (1, 2, -1)):
            with self.assertRaises(ValueError):
                validate_imu_signs(invalid)

    def test_apply_imu_signs(self) -> None:
        raw_gyro = (0.01, -0.02, -0.5)
        corrected_gyro = apply_imu_signs(raw_gyro, (1, 1, -1))
        self.assertAlmostEqual(corrected_gyro[0], 0.01)
        self.assertAlmostEqual(corrected_gyro[1], -0.02)
        self.assertAlmostEqual(corrected_gyro[2], 0.5)

        raw_accel = (0.04, -0.07, -9.88)
        corrected_accel = apply_imu_signs(raw_accel, (1, 1, -1))
        self.assertAlmostEqual(corrected_accel[0], 0.04)
        self.assertAlmostEqual(corrected_accel[1], -0.07)
        self.assertAlmostEqual(corrected_accel[2], 9.88)


if __name__ == '__main__':

    unittest.main()
