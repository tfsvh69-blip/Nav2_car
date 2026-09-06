# Copyright 2026 carcar maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""启动四电机端口级 ros2_control 测试，运动锁默认关闭."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    motion_enabled = LaunchConfiguration('motion_enabled')
    package_share = FindPackageShare('carcar_hardware')
    description_file = PathJoinSubstitution(
        [package_share, 'urdf', 'rosmaster_four_motor_test.urdf.xacro'])
    controller_file = PathJoinSubstitution(
        [package_share, 'config', 'four_motor_test_controllers.yaml'])
    robot_description = ParameterValue(
        Command([
            'xacro ', description_file,
            ' serial_port:=', serial_port,
            ' motion_enabled:=', motion_enabled,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        DeclareLaunchArgument(
            'motion_enabled',
            default_value='false',
            description='四路 motor_pwm 硬件运动锁；反馈测试保持 false',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_file],
            remappings=[('~/robot_description', '/robot_description')],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager',
                       '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['imu_sensor_broadcaster', '--controller-manager',
                       '/controller_manager'],
            output='screen',
        ),
    ])
