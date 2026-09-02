"""Bring up an RPLIDAR A1 and optionally visualize /scan in RViz2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    scan_mode = LaunchConfiguration("scan_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    base_frame = LaunchConfiguration("base_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")

    rviz_config = str(
        Path(get_package_share_directory("inspection_bringup"))
        / "rviz"
        / "lidar_test.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device connected to the lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value="115200",
                description="RPLIDAR A1 serial baud rate",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="laser",
                description="Frame assigned to LaserScan messages",
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value="false",
                description="Invert scan direction",
            ),
            DeclareLaunchArgument(
                "angle_compensate",
                default_value="true",
                description="Enable angular compensation",
            ),
            DeclareLaunchArgument(
                "scan_mode",
                default_value="Express",
                description="Lidar scan mode (Express gives denser samples on this A1 firmware)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 together with the lidar",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Parent frame used during standalone lidar testing",
            ),
            DeclareLaunchArgument("laser_x", default_value="0.0"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.0"),
            DeclareLaunchArgument("laser_roll", default_value="0.0"),
            DeclareLaunchArgument("laser_pitch", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="laser_static_tf_publisher",
                output="screen",
                arguments=[
                    "--x",
                    laser_x,
                    "--y",
                    laser_y,
                    "--z",
                    laser_z,
                    "--roll",
                    laser_roll,
                    "--pitch",
                    laser_pitch,
                    "--yaw",
                    laser_yaw,
                    "--frame-id",
                    base_frame,
                    "--child-frame-id",
                    frame_id,
                ],
            ),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": frame_id,
                        "inverted": inverted,
                        "angle_compensate": angle_compensate,
                        "scan_mode": scan_mode,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
