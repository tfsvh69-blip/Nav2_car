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

"""启动使用临时几何参数的 diff_drive_controller 架空测试入口."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    motion_enabled = LaunchConfiguration('motion_enabled')
    left_command_sign = LaunchConfiguration('left_command_sign')
    right_command_sign = LaunchConfiguration('right_command_sign')
    left_encoder_sign = LaunchConfiguration('left_encoder_sign')
    right_encoder_sign = LaunchConfiguration('right_encoder_sign')
    description_file = PathJoinSubstitution(
        [FindPackageShare('carcar_hardware'), 'urdf',
         'rosmaster_command_test.urdf.xacro']
    )
    controller_file = PathJoinSubstitution(
        [FindPackageShare('carcar_hardware'), 'config',
         'diff_drive_test_controllers.yaml']
    )
    robot_description = ParameterValue(
        Command([
            'xacro ', description_file,
            ' serial_port:=', serial_port,
            ' motion_enabled:=', motion_enabled,
            ' left_command_sign:=', left_command_sign,
            ' right_command_sign:=', right_command_sign,
            ' left_encoder_sign:=', left_encoder_sign,
            ' right_encoder_sign:=', right_encoder_sign,
        ]),
        value_type=str,
    )

    launch_arguments = [
        DeclareLaunchArgument('serial_port', default_value='/dev/myserial'),
        DeclareLaunchArgument('motion_enabled', default_value='false'),
        DeclareLaunchArgument('left_command_sign', default_value='1'),
        DeclareLaunchArgument('right_command_sign', default_value='1'),
        DeclareLaunchArgument('left_encoder_sign', default_value='1'),
        DeclareLaunchArgument('right_encoder_sign', default_value='1'),
    ]
    nodes = [
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
    ]
    for controller in (
            'joint_state_broadcaster',
            'imu_sensor_broadcaster',
            'diff_drive_controller'):
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager',
                       '/controller_manager'],
            output='screen',
        ))

    return LaunchDescription(launch_arguments + nodes)
