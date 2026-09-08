"""Launch the isolated low-speed three-motion chassis test driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([
        FindPackageShare('carcar_base'),
        'config',
        'chassis_motion_test.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        Node(
            package='carcar_base',
            executable='four_motor_motion_test_node',
            name='chassis_motion_test_driver',
            output='screen',
            parameters=[params_file, {'serial_port': serial_port}],
            remappings=[
                ('cmd_vel', '/chassis_motion_test/cmd_vel'),
                ('encoders', '/chassis_motion_test/encoders'),
                ('diagnostics', '/chassis_motion_test/diagnostics'),
            ],
        ),
    ])
