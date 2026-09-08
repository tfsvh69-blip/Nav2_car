"""启动隔离的 Rosmaster X3 板载运动闭环测试。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([
        FindPackageShare('carcar_base'),
        'config',
        'board_closed_loop_test.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        Node(
            package='carcar_base',
            executable='rosmaster_node',
            name='board_closed_loop_test_driver',
            output='screen',
            parameters=[params_file, {'serial_port': serial_port}],
            remappings=[
                ('cmd_vel', '/chassis_motion_test/cmd_vel'),
                ('wheel/encoders', '/chassis_motion_test/encoders'),
                ('wheel/encoder_rates', '/chassis_motion_test/encoder_rates'),
                ('diagnostics', '/chassis_motion_test/diagnostics'),
                ('wheel/odometry', '/board_closed_loop_test/wheel/odometry'),
                ('imu/data_raw', '/board_closed_loop_test/imu/data_raw'),
                ('battery_state', '/board_closed_loop_test/battery_state'),
            ],
        ),
    ])
