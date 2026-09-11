"""启动用于 IMU 单项检查的隔离驱动节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([
        FindPackageShare('carcar_base'),
        'config',
        'imu_test.yaml',
    ])
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        Node(
            package='carcar_base',
            executable='rosmaster_node',
            name='imu_test_driver',
            output='screen',
            parameters=[params_file, {'serial_port': serial_port}],
            remappings=[
                ('cmd_vel', '/imu_test/cmd_vel_disabled'),
                ('imu/data_raw', '/imu/data_raw'),
            ],
        ),
    ])
