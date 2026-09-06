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

"""启动有标准命令接口但硬件运动锁关闭的测试入口."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    description_file = PathJoinSubstitution(
        [FindPackageShare('carcar_hardware'), 'urdf',
         'rosmaster_command_test.urdf.xacro']
    )
    controller_file = PathJoinSubstitution(
        [FindPackageShare('carcar_hardware'), 'config',
         'command_locked_controllers.yaml']
    )
    robot_description = ParameterValue(
        Command([
            'xacro ', description_file,
            ' serial_port:=', serial_port,
            ' motion_enabled:=false',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/myserial',
            description='Rosmaster control board serial device',
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
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'imu_sensor_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        ),
    ])
