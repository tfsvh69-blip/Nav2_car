"""Bring up hardware access and the robot description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    description_launch = PathJoinSubstitution([
        FindPackageShare('carcar_description'),
        'launch',
        'description.launch.py',
    ])
    base_launch = PathJoinSubstitution([
        FindPackageShare('carcar_base'), 'launch', 'base.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'serial_port': serial_port}.items(),
        ),
    ])
