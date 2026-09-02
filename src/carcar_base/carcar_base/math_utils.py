"""Small, ROS-independent helpers used by the base driver."""

import math


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to an inclusive range."""
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return x, y, z, w for a rotation about Z."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def integrate_body_twist(
    x: float,
    y: float,
    yaw: float,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    dt: float,
) -> tuple[float, float, float]:
    """Integrate a body-frame planar twist into an odometry-frame pose."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x += (linear_x * cos_yaw - linear_y * sin_yaw) * dt
    y += (linear_x * sin_yaw + linear_y * cos_yaw) * dt
    yaw = math.atan2(
        math.sin(yaw + angular_z * dt),
        math.cos(yaw + angular_z * dt),
    )
    return x, y, yaw


def two_motor_pwm(
    linear_x: float,
    angular_z: float,
    max_pwm: int,
    linear_full_scale: float,
    angular_full_scale: float,
    enable_steering: bool,
    motor_1_sign: int,
    motor_2_sign: int,
) -> tuple[int, int]:
    """Convert a planar command into bounded M1/M2 PWM percentages."""
    if linear_full_scale <= 0.0 or angular_full_scale <= 0.0:
        raise ValueError('full-scale values must be greater than zero')
    if max_pwm < 0 or max_pwm > 100:
        raise ValueError('max_pwm must be between 0 and 100')
    if motor_1_sign not in (-1, 1) or motor_2_sign not in (-1, 1):
        raise ValueError('motor signs must be either -1 or 1')

    forward = clamp(linear_x / linear_full_scale, -1.0, 1.0)
    turn = 0.0
    if enable_steering:
        turn = clamp(angular_z / angular_full_scale, -1.0, 1.0)

    motor_1 = forward - turn
    motor_2 = forward + turn
    scale = max(1.0, abs(motor_1), abs(motor_2))
    motor_1 = int(round(max_pwm * motor_1 / scale)) * motor_1_sign
    motor_2 = int(round(max_pwm * motor_2 / scale)) * motor_2_sign
    return motor_1, motor_2
