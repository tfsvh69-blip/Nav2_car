import math
import unittest

from carcar_base.math_utils import (
    clamp,
    integrate_body_twist,
    two_motor_pwm,
    yaw_to_quaternion,
)


class MathUtilsTest(unittest.TestCase):

    def test_clamp(self) -> None:
        self.assertEqual(clamp(2.0, -1.0, 1.0), 1.0)
        self.assertEqual(clamp(-2.0, -1.0, 1.0), -1.0)
        self.assertEqual(clamp(0.25, -1.0, 1.0), 0.25)

    def test_integrate_forward_at_quarter_turn(self) -> None:
        x, y, yaw = integrate_body_twist(
            0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0, 1.0
        )
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(yaw, math.pi / 2)

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


if __name__ == '__main__':
    unittest.main()
