"""在 ROS 2 Humble 中启动 RPLIDAR A1，不启动 RViz 或测试用 TF。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')
    scan_frequency = LaunchConfiguration('scan_frequency')

    default_config = PathJoinSubstitution([
        FindPackageShare('carcar_lidar'),
        'config',
        'rplidar_a1.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
            ),
            description='RPLIDAR A1 的串口设备；不要填写底板 CH340 端口',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='RPLIDAR A1 串口波特率',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame',
            description='LaserScan 消息使用的坐标系',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='是否反转扫描方向',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='是否启用角度补偿',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Express',
            description='扫描模式；兼容性排查时可改为 Standard',
        ),
        DeclareLaunchArgument(
            'scan_frequency',
            default_value='10.0',
            description='用于角度补偿计算的目标扫描频率',
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                default_config,
                {
                    'serial_port': serial_port,
                    'serial_baudrate': ParameterValue(
                        serial_baudrate, value_type=int
                    ),
                    'frame_id': frame_id,
                    'inverted': ParameterValue(inverted, value_type=bool),
                    'angle_compensate': ParameterValue(
                        angle_compensate, value_type=bool
                    ),
                    'scan_mode': scan_mode,
                    'scan_frequency': ParameterValue(
                        scan_frequency, value_type=float
                    ),
                },
            ],
        ),
    ])
