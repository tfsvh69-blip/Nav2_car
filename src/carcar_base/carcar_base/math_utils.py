"""Small, ROS-independent helpers used by the base driver."""

import math


KEYBOARD_SPEED_LEVELS = {
    '1': (0.03, 0.10),
    '2': (0.05, 0.15),
    '3': (0.08, 0.20),
    '4': (0.10, 0.30),
    '5': (0.15, 0.40),
    '6': (0.20, 0.50),
    '7': (0.25, 0.65),
    '8': (0.35, 0.80),
    '9': (0.45, 1.00),
}


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to an inclusive range."""
    return max(lower, min(upper, value))


def slew_towards(current: float, target: float, max_step: float) -> float:
    """Move one bounded step toward a target without overshooting it."""
    if max_step < 0.0:
        raise ValueError('max_step must not be negative')
    return current + clamp(target - current, -max_step, max_step)


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


WHEEL_NAMES = ('left_front', 'right_front', 'left_rear', 'right_rear')


def validate_mecanum_odometry_parameters(
    wheel_diameter_m: float,
    wheelbase_m: float,
    track_width_m: float,
    counts_per_revolution: tuple[float, float, float, float],
    encoder_signs: tuple[int, int, int, int],
    port_wheels: tuple[str, str, str, str],
) -> tuple[
    float,
    float,
    float,
    tuple[float, float, float, float],
    tuple[int, int, int, int],
    tuple[str, str, str, str],
]:
    """Validate the physical calibration needed for encoder odometry.

    ``encoder_signs`` maps a raw positive M1--M4 count to a wheel rolling
    forward in the vehicle's +X direction.  ``port_wheels`` maps the same
    port order to physical wheel names.
    """
    geometry = tuple(float(value) for value in (
        wheel_diameter_m, wheelbase_m, track_width_m))
    if (not all(math.isfinite(value) and value > 0.0 for value in geometry)):
        raise ValueError('wheel diameter, wheelbase and track width must be positive')

    counts = tuple(float(value) for value in counts_per_revolution)
    if len(counts) != 4 or not all(
            math.isfinite(value) and value > 0.0 for value in counts):
        raise ValueError('counts_per_revolution must contain four positive values')

    signs = tuple(int(value) for value in encoder_signs)
    if len(signs) != 4 or any(value not in (-1, 1) for value in signs):
        raise ValueError('encoder_signs must contain four values of -1 or 1')

    wheels = tuple(str(value) for value in port_wheels)
    if len(wheels) != 4 or set(wheels) != set(WHEEL_NAMES):
        raise ValueError('port_wheels must contain each physical wheel exactly once')

    return (*geometry, counts, signs, wheels)


def validate_imu_signs(
    signs: tuple[int, ...] | list[int],
) -> tuple[int, int, int]:
    """Validate a 3-element tuple of IMU axis signs (+1 or -1)."""
    values = tuple(int(value) for value in signs)
    if len(values) != 3 or any(value not in (-1, 1) for value in values):
        raise ValueError('IMU axis signs must contain three values of -1 or 1')
    return (values[0], values[1], values[2])


def apply_imu_signs(
    values: tuple[float, float, float] | list[float],
    signs: tuple[int, ...] | list[int],
) -> tuple[float, float, float]:
    """Apply 3-axis signs to IMU linear acceleration or angular velocity."""
    validated_signs = validate_imu_signs(signs)
    if len(values) != 3:
        raise ValueError('IMU measurement must contain three values')
    return (
        float(values[0]) * validated_signs[0],
        float(values[1]) * validated_signs[1],
        float(values[2]) * validated_signs[2],
    )


def mecanum_body_delta_from_encoder_counts(
    count_deltas: tuple[int, int, int, int],
    wheel_diameter_m: float,
    wheelbase_m: float,
    track_width_m: float,
    counts_per_revolution: tuple[float, float, float, float],
    encoder_signs: tuple[int, int, int, int],
    port_wheels: tuple[str, str, str, str],
) -> tuple[float, float, float]:
    """Convert four signed count changes into body-frame dx, dy and dyaw.

    The inverse kinematics use an X-roller mecanum convention: +X is forward,
    +Y is left and positive yaw is counter-clockwise.  Wheel travel is first
    converted per port, then associated with its physical wheel through
    ``port_wheels`` so a future wiring change remains explicit in the YAML.
    """
    if len(count_deltas) != 4:
        raise ValueError('count_deltas must contain four values')
    (wheel_diameter_m, wheelbase_m, track_width_m, counts, signs,
     wheels) = validate_mecanum_odometry_parameters(
         wheel_diameter_m,
         wheelbase_m,
         track_width_m,
         counts_per_revolution,
         encoder_signs,
         port_wheels,
     )
    if not all(math.isfinite(float(value)) for value in count_deltas):
        raise ValueError('count_deltas must be finite')

    circumference_m = math.pi * wheel_diameter_m
    wheel_travel = {
        wheel: float(delta) * sign / count * circumference_m
        for delta, count, sign, wheel in zip(
            count_deltas, counts, signs, wheels)
    }
    left_front = wheel_travel['left_front']
    right_front = wheel_travel['right_front']
    left_rear = wheel_travel['left_rear']
    right_rear = wheel_travel['right_rear']

    delta_x = (left_front + right_front + left_rear + right_rear) * 0.25
    delta_y = (-left_front + right_front + left_rear - right_rear) * 0.25
    rotation_lever_m = (wheelbase_m + track_width_m) * 0.5
    delta_yaw = (
        -left_front + right_front - left_rear + right_rear
    ) / (4.0 * rotation_lever_m)
    return delta_x, delta_y, delta_yaw


def integrate_body_delta(
    x: float,
    y: float,
    yaw: float,
    delta_x: float,
    delta_y: float,
    delta_yaw: float,
) -> tuple[float, float, float]:
    """Integrate one measured body displacement using its midpoint heading."""
    midpoint_yaw = yaw + delta_yaw * 0.5
    cos_yaw = math.cos(midpoint_yaw)
    sin_yaw = math.sin(midpoint_yaw)
    x += delta_x * cos_yaw - delta_y * sin_yaw
    y += delta_x * sin_yaw + delta_y * cos_yaw
    yaw = math.atan2(
        math.sin(yaw + delta_yaw),
        math.cos(yaw + delta_yaw),
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


def single_motor_pwm(motor_number: int, pwm: int) -> tuple[int, int, int, int]:
    """Return one positive PWM command with all other motor outputs at zero."""
    if motor_number not in (1, 2, 3, 4):
        raise ValueError('motor_number must be between 1 and 4')
    if pwm < 1 or pwm > 20:
        raise ValueError('pwm must be between 1 and 20')

    command = [0, 0, 0, 0]
    command[motor_number - 1] = pwm
    return tuple(command)


def single_motor_command(
    motor_number: int,
    pwm: int,
) -> tuple[int, int, int, int]:
    """Return one signed PWM command with every other motor held at zero."""
    if motor_number not in (1, 2, 3, 4):
        raise ValueError('motor_number must be between 1 and 4')
    if pwm == 0 or abs(pwm) > 20:
        raise ValueError('pwm must be between -20 and 20, excluding zero')

    command = [0, 0, 0, 0]
    command[motor_number - 1] = pwm
    return tuple(command)


def is_zero_motion_command(
    linear_x: float,
    linear_y: float,
    angular_z: float,
) -> bool:
    """Return whether every effective motion component is exactly zero."""
    return linear_x == 0.0 and linear_y == 0.0 and angular_z == 0.0


def keyboard_speed_level(key: str) -> tuple[float, float]:
    """Return linear and angular speeds for one tuning keyboard level."""
    if key not in KEYBOARD_SPEED_LEVELS:
        raise ValueError('speed level key must be between 1 and 9')
    return KEYBOARD_SPEED_LEVELS[key]


def validate_motion_pid(
    kp: float,
    ki: float,
    kd: float,
) -> tuple[float, float, float]:
    """Validate board-PID tuning limits (matches Rosmaster_Lib [0, 10])."""
    values = (float(kp), float(ki), float(kd))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('motion PID values must be finite')
    if any(value < 0.0 or value > 10.0 for value in values):
        raise ValueError(
            'motion PID requires 0<=Kp<=10, 0<=Ki<=10, 0<=Kd<=10')
    return values


def encoder_deltas(
    start: tuple[int, int, int, int],
    end: tuple[int, int, int, int],
) -> tuple[int, int, int, int]:
    """Return M1--M4 signed count changes between two snapshots."""
    if len(start) != 4 or len(end) != 4:
        raise ValueError('encoder snapshots must each contain four values')
    return tuple(end[index] - start[index] for index in range(4))


def encoder_signs(
    deltas: tuple[int, int, int, int],
) -> tuple[str, str, str, str]:
    """Return compact signs for four signed encoder deltas."""
    if len(deltas) != 4:
        raise ValueError('encoder deltas must contain four values')
    return tuple('+' if value > 0 else '-' if value < 0 else '0'
                 for value in deltas)


def counts_per_revolution(delta: int, revolutions: int = 10) -> float:
    """Calculate absolute output-shaft encoder counts per revolution."""
    if revolutions <= 0:
        raise ValueError('revolutions must be greater than zero')
    return abs(delta) / revolutions


def mecanum_motor_pwm(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    max_pwm: int,
    linear_full_scale: float,
    angular_full_scale: float,
    motor_signs: tuple[int, int, int, int],
) -> tuple[int, int, int, int]:
    """Map a body twist to M1--M4 PWM using the confirmed wheel layout."""
    if max_pwm < 1 or max_pwm > 20:
        raise ValueError('max_pwm must be between 1 and 20')
    if len(motor_signs) != 4 or any(
            sign not in (-1, 1) for sign in motor_signs):
        raise ValueError('motor_signs must contain four values of -1 or 1')

    port_motion = mecanum_port_targets(
        linear_x,
        linear_y,
        angular_z,
        1.0,
        linear_full_scale,
        angular_full_scale,
    )
    return tuple(
        int(round(max_pwm * value)) * motor_signs[index]
        for index, value in enumerate(port_motion)
    )


def mecanum_port_targets(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    full_scale_target: float,
    linear_full_scale: float,
    angular_full_scale: float,
    port_wheels=('left_front', 'right_rear', 'right_front', 'left_rear'),
) -> tuple[float, float, float, float]:
    """Return normalized M1--M4 physical-forward wheel targets."""
    wheel_names = ('left_front', 'right_front', 'left_rear', 'right_rear')
    if len(port_wheels) != 4 or set(port_wheels) != set(wheel_names):
        raise ValueError('port_wheels must contain each wheel exactly once')
    if full_scale_target <= 0.0:
        raise ValueError('full_scale_target must be greater than zero')
    if linear_full_scale <= 0.0 or angular_full_scale <= 0.0:
        raise ValueError('full-scale values must be greater than zero')

    forward = clamp(linear_x / linear_full_scale, -1.0, 1.0)
    left = clamp(linear_y / linear_full_scale, -1.0, 1.0)
    turn = clamp(angular_z / angular_full_scale, -1.0, 1.0)

    # Standard X-roller wheel order: left-front, right-front,
    # left-rear, right-rear. Ports are M1=LF, M2=RR, M3=RF, M4=LR.
    left_front = forward - left - turn
    right_front = forward + left + turn
    left_rear = forward + left - turn
    right_rear = forward - left + turn
    scale = max(
        1.0,
        abs(left_front),
        abs(right_front),
        abs(left_rear),
        abs(right_rear),
    )
    wheels = dict(zip(
        wheel_names, (left_front, right_front, left_rear, right_rear)))
    port_motion = tuple(wheels[name] for name in port_wheels)
    return tuple(
        full_scale_target * value / scale for value in port_motion
    )


def velocity_pi_pwm(
    target_rate: float,
    measured_rate: float,
    integral_error: float,
    feedforward_pwm: float,
    kp: float,
    ki: float,
    max_pwm: int,
    motor_sign: int,
) -> int:
    """Calculate one direction-preserving wheel velocity PI output."""
    if feedforward_pwm < 0.0 or feedforward_pwm > max_pwm:
        raise ValueError('feedforward_pwm must be within the PWM limit')
    if kp < 0.0 or ki < 0.0:
        raise ValueError('PI gains must not be negative')
    if max_pwm < 1 or max_pwm > 45:
        raise ValueError('max_pwm must be between 1 and 45')
    if motor_sign not in (-1, 1):
        raise ValueError('motor_sign must be either -1 or 1')
    if target_rate == 0.0:
        return 0

    direction = 1.0 if target_rate > 0.0 else -1.0
    error = target_rate - measured_rate
    physical_pwm = (
        direction * feedforward_pwm + kp * error + ki * integral_error
    )
    # Do not reverse a wheel merely to brake during this bounded test.
    directed_pwm = clamp(direction * physical_pwm, 0.0, float(max_pwm))
    return int(round(direction * directed_pwm)) * motor_sign
