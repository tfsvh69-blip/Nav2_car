#!/usr/bin/env python3
"""
NAV-001: SLAM Toolbox 建图链路只读检查脚本
严禁发布 /cmd_vel 或任何驱动指令。
只读检查：
  1. 运行节点与发布者状态
  2. 关键话题消息帧（frame_id）与时间戳
  3. 话题更新频率（/scan, /wheel/odometry, /map）
  4. 完整 TF 树连通性：
     map -> odom -> base_footprint -> base_link -> laser_frame
"""

import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数计算偏航角 Yaw (rad)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SlamLinkChecker(Node):
    def __init__(self):
        super().__init__('slam_link_checker')

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 采样数据存储
        self.scan_msg: Optional[LaserScan] = None
        self.scan_count = 0
        self.scan_times: List[float] = []

        self.odom_msg: Optional[Odometry] = None
        self.odom_count = 0
        self.odom_times: List[float] = []

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_count = 0
        self.map_times: List[float] = []

        # 订阅关键话题（只读）
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.create_subscription(Odometry, '/wheel/odometry', self._odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, 10)

    def _scan_callback(self, msg: LaserScan):
        self.scan_msg = msg
        self.scan_count += 1
        self.scan_times.append(time.time())

    def _odom_callback(self, msg: Odometry):
        self.odom_msg = msg
        self.odom_count += 1
        self.odom_times.append(time.time())

    def _map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_count += 1
        self.map_times.append(time.time())


def compute_rate(times: List[float]) -> float:
    if len(times) < 2:
        return 0.0
    duration = times[-1] - times[0]
    if duration <= 0.0:
        return 0.0
    return (len(times) - 1) / duration


def run_checks(timeout_sec: float = 6.0) -> int:
    print("=" * 68)
    print("  NAV-001: SLAM Toolbox 建图链路只读检查")
    print("  安全承诺：严禁发布 /cmd_vel 或任何电机驱动命令")
    print("=" * 68)

    rclpy.init()
    node = SlamLinkChecker()

    # 收集 3.5 秒数据
    sample_duration = 3.5
    print(f"正在监听 ROS 2 话题并采样 ({sample_duration:.1f} 秒)...")
    start_t = time.time()
    while (time.time() - start_t) < sample_duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    all_passed = True

    # 1. 检查活跃节点
    print("\n" + "-" * 68)
    print("【1. 核心节点存活状态】")
    active_nodes = node.get_node_names()

    expected_nodes = {
        "底盘驱动节点 (carcar_base)": ["rosmaster_base"],
        "模型状态发布 (robot_state_publisher)": ["robot_state_publisher"],
        "激光雷达节点 (sllidar_ros2)": ["sllidar_node"],
        "SLAM 算法节点 (slam_toolbox)": ["slam_toolbox", "async_slam_toolbox_node"],
    }

    for desc, candidates in expected_nodes.items():
        matched = [c for c in candidates if c in active_nodes]
        if matched:
            print(f"  [✓ 正常] {desc}: {matched[0]}")
        else:
            print(f"  [✗ 缺失] {desc}: 未检测到 ({', '.join(candidates)})")
            all_passed = False

    # 2. 检查关键话题与发布者
    print("\n" + "-" * 68)
    print("【2. 关键话题与单发布者独占】")

    topic_names_and_types = dict(node.get_topic_names_and_types())

    topics_to_check = [
        ('/scan', 'sensor_msgs/msg/LaserScan', 1),
        ('/wheel/odometry', 'nav_msgs/msg/Odometry', 1),
        ('/map', 'nav_msgs/msg/OccupancyGrid', 1),
        ('/map_metadata', 'nav_msgs/msg/MapMetaData', 1),
    ]

    for topic_name, expected_type, expected_pubs in topics_to_check:
        if topic_name not in topic_names_and_types:
            print(f"  [✗ 缺失] 话题 {topic_name} 不存在")
            all_passed = False
            continue

        pub_info = node.get_publishers_info_by_topic(topic_name)
        pub_count = len(pub_info)

        if pub_count == expected_pubs:
            pub_names = [p.node_name for p in pub_info]
            print(f"  [✓ 正常] 话题 {topic_name}: {pub_count} 个发布者 ({', '.join(pub_names)})")
        elif pub_count == 0:
            print(f"  [✗ 异常] 话题 {topic_name}: 无发布者")
            all_passed = False
        else:
            pub_names = [p.node_name for p in pub_info]
            print(f"  [! 警告] 话题 {topic_name}: 存在 {pub_count} 个发布者 (需独占): {pub_names}")
            all_passed = False

    # 3. 检查消息内容、Frame ID 与更新频率
    print("\n" + "-" * 68)
    print("【3. 消息帧定义与采样频率】")

    # /scan
    if node.scan_msg is not None:
        scan_hz = compute_rate(node.scan_times)
        frame_id = node.scan_msg.header.frame_id
        valid_ranges = [r for r in node.scan_msg.ranges if node.scan_msg.range_min <= r <= node.scan_msg.range_max]
        if frame_id == 'laser_frame':
            print(f"  [✓ 正常] /scan: frame_id='{frame_id}', 频率={scan_hz:.1f} Hz (预期约 10 Hz), "
                  f"有效点数={len(valid_ranges)}/{len(node.scan_msg.ranges)}")
        else:
            print(f"  [✗ 异常] /scan: frame_id='{frame_id}' (预期 'laser_frame')")
            all_passed = False
    else:
        print("  [✗ 异常] /scan: 未收到任何激光雷达消息")
        all_passed = False

    # /wheel/odometry
    if node.odom_msg is not None:
        odom_hz = compute_rate(node.odom_times)
        frame_id = node.odom_msg.header.frame_id
        child_frame_id = node.odom_msg.child_frame_id
        pos = node.odom_msg.pose.pose.position
        if frame_id == 'odom' and child_frame_id == 'base_footprint':
            print(f"  [✓ 正常] /wheel/odometry: header='{frame_id}', child='{child_frame_id}', "
                  f"频率={odom_hz:.1f} Hz (预期约 25 Hz), 静止位置=(x={pos.x:.3f}, y={pos.y:.3f})")
        else:
            print(f"  [✗ 异常] /wheel/odometry: header='{frame_id}', child='{child_frame_id}' "
                  f"(预期 header='odom', child='base_footprint')")
            all_passed = False
    else:
        print("  [✗ 异常] /wheel/odometry: 未收到任何里程计消息")
        all_passed = False

    # /map
    if node.map_msg is not None:
        frame_id = node.map_msg.header.frame_id
        info = node.map_msg.info
        occupied_cells = sum(1 for c in node.map_msg.data if c > 0)
        free_cells = sum(1 for c in node.map_msg.data if c == 0)
        unknown_cells = sum(1 for c in node.map_msg.data if c < 0)
        if frame_id == 'map':
            print(f"  [✓ 正常] /map: frame_id='{frame_id}', 分辨率={info.resolution:.3f} m/cell, "
                  f"尺寸={info.width}x{info.height}, 已探明栅格(空闲={free_cells}, 占用={occupied_cells}, 未知={unknown_cells})")
        else:
            print(f"  [✗ 异常] /map: frame_id='{frame_id}' (预期 'map')")
            all_passed = False
    else:
        print("  [✗ 异常] /map: 未收到栅格地图消息 (SLAM Toolbox 尚未输出或未运行)")
        all_passed = False

    # 4. 检查 TF 树连通性
    print("\n" + "-" * 68)
    print("【4. 坐标变换 TF 树连通性检查】")
    tf_links: List[Tuple[str, str, str]] = [
        ("map", "odom", "SLAM Toolbox (异步建图)"),
        ("odom", "base_footprint", "rosmaster_base (纯轮式里程计)"),
        ("base_footprint", "base_link", "robot_state_publisher (静态变换)"),
        ("base_link", "laser_frame", "robot_state_publisher (静态变换)"),
    ]

    # 等待 TF 缓存填充
    tf_check_start = time.time()
    while (time.time() - tf_check_start) < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    for parent, child, publisher_desc in tf_links:
        try:
            # 查找最新变换
            tf = node.tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=1.5))
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw_deg = math.degrees(quat_to_yaw(r.x, r.y, r.z, r.w))
            print(f"  [✓ 连通] {parent} -> {child} ({publisher_desc}): "
                  f"xyz=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}] m, yaw={yaw_deg:.1f}°")
        except Exception as e:
            print(f"  [✗ 断链] {parent} -> {child} ({publisher_desc}): 查找失败 ({e})")
            all_passed = False

    # 全链路贯通检查: map -> laser_frame
    print("\n  >> 全链路测试 (map -> laser_frame):")
    try:
        full_tf = node.tf_buffer.lookup_transform("map", "laser_frame", Time(), timeout=Duration(seconds=2.0))
        t = full_tf.transform.translation
        r = full_tf.transform.rotation
        yaw_deg = math.degrees(quat_to_yaw(r.x, r.y, r.z, r.w))
        print(f"  [✓ 贯通] map -> laser_frame: xyz=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}] m, yaw={yaw_deg:.1f}°")
    except Exception as e:
        print(f"  [✗ 失败] map -> laser_frame 无法贯通 ({e})")
        all_passed = False

    # 5. 总结
    print("\n" + "=" * 68)
    if all_passed:
        print("  【检查结论：全部通过 (PASS)】")
        print("  SLAM Toolbox 建图链路（底盘 Odom + RPLIDAR A1 + SLAM + TF）完全贯通！")
        print("  可在 RViz2 中订阅 /map、/scan 及 TF 观察原地建图效果。")
    else:
        print("  【检查结论：存在异常项 (FAIL)】")
        print("  请根据上述输出排查对应的终端进程、串口连接与 TF 发布者。")
    print("=" * 68 + "\n")

    node.destroy_node()
    rclpy.shutdown()

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(run_checks())
