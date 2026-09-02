"""Start asynchronous slam_toolbox; bring up the robot separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_params = PathJoinSubstitution([
        FindPackageShare('carcar_navigation'), 'config', 'slam_toolbox.yaml'
    ])
    upstream_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': params_file,
            }.items(),
        ),
    ])
