"""[已停用 / DEPRECATED] 早期 Nav2 Python 启动入口。

根据 AGENTS.md 规范与 NAV-003 方案，本项目 Launch 文件统一采用 XML 格式，
并解耦为独立的定位与导航入口：
1. 定位入口：launch/localization.launch.xml
2. 导航入口：launch/navigation.launch.xml
本文件仅保留作为历史底层测试资产，不再作为当前启动入口。
"""

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
