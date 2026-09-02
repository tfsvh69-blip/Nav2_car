"""Launch the isolated M1/M2 low-speed test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    max_pwm = LaunchConfiguration('max_pwm_percent')
    motor_1_sign = LaunchConfiguration('motor_1_sign')
    motor_2_sign = LaunchConfiguration('motor_2_sign')
    default_params = PathJoinSubstitution([
        FindPackageShare('carcar_base'), 'config', 'motor_test.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        DeclareLaunchArgument('max_pwm_percent', default_value='20'),
        DeclareLaunchArgument('motor_1_sign', default_value='1'),
        DeclareLaunchArgument('motor_2_sign', default_value='1'),
        Node(
            package='carcar_base',
            executable='two_motor_test_node',
            name='two_motor_test',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': serial_port,
                    'max_pwm_percent': ParameterValue(max_pwm, value_type=int),
                    'motor_1_sign': ParameterValue(
                        motor_1_sign, value_type=int
                    ),
                    'motor_2_sign': ParameterValue(
                        motor_2_sign, value_type=int
                    ),
                },
            ],
        ),
    ])
