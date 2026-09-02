"""Start Nav2 localization and navigation with an existing map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_params = PathJoinSubstitution([
        FindPackageShare('carcar_navigation'), 'config', 'nav2.yaml'
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to a map YAML file'
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'slam': 'False',
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
