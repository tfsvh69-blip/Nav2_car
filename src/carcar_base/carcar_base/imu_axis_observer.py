"""Collect read-only IMU samples for a single mounting-axis check phase."""

import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


PHASE_HINTS = {
    'level': '水平静止：期望 ax≈0、ay≈0、az≈+9.81 m/s²。',
    'nose_up': '车头抬高约 20～30°后静止：车体 ax 应为正，az 仍为正。',
    'left_up': '左侧抬高约 20～30°后静止：车体 ay 应为正，az 仍为正。',
    'ccw': '水平缓慢逆时针旋转：期望角速度主轴为 Z，且符号为正。',
}


def summarize_samples(samples):
    """Return mean, population deviation, minimum and maximum per IMU axis."""
    if not samples:
        raise ValueError('至少需要一帧 IMU 数据')
    if any(len(sample) != 6 or not all(math.isfinite(v) for v in sample)
           for sample in samples):
        raise ValueError('每帧必须包含六个有限数值')

    names = ('ax', 'ay', 'az', 'gx', 'gy', 'gz')
    result = {}
    for index, name in enumerate(names):
        values = [sample[index] for sample in samples]
        result[name] = {
            'mean': statistics.fmean(values),
            'std': statistics.pstdev(values),
            'min': min(values),
            'max': max(values),
        }
    return result


class ImuAxisObserver(Node):
    """Subscribe to an existing IMU publisher without opening the serial port."""

    def __init__(self) -> None:
        super().__init__('imu_axis_observer')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('phase', 'level')
        self.declare_parameter('sample_seconds', 30.0)
        self.declare_parameter('startup_timeout', 10.0)

        self.topic = self.get_parameter('topic').value
        self.phase = self.get_parameter('phase').value
        self.sample_seconds = float(
            self.get_parameter('sample_seconds').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout').value)
        if self.phase not in PHASE_HINTS:
            valid = ', '.join(PHASE_HINTS)
            raise ValueError(f'phase 必须是：{valid}')
        if not all(math.isfinite(value) and 0.0 < value <= 300.0
                   for value in (self.sample_seconds, self.startup_timeout)):
            raise ValueError('采样时间和启动超时必须为 (0, 300] 秒内的有限值')

        self.samples = []
        self.last_sample_time = None
        self.frame_id = None
        self.create_subscription(
            Imu, self.topic, self._imu_callback, qos_profile_sensor_data)

    def _imu_callback(self, message: Imu) -> None:
        sample = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if not all(math.isfinite(value) for value in sample):
            raise ValueError('收到 NaN/Inf，采样无效')
        if not message.header.frame_id:
            raise ValueError('IMU frame_id 为空，无法确认数据坐标系')
        if self.frame_id is not None and self.frame_id != message.header.frame_id:
            raise ValueError('采样期间 IMU frame_id 发生变化，采样无效')
        self.frame_id = message.header.frame_id
        self.last_sample_time = time.monotonic()
        self.samples.append(sample)

    def collect(self) -> bool:
        """Wait for the topic and collect for the configured bounded duration."""
        self.get_logger().info(
            f'阶段={self.phase}；{PHASE_HINTS[self.phase]}')
        self.get_logger().info(
            f'等待 {self.topic}，收到首帧后采样 {self.sample_seconds:.1f} 秒。')

        startup_deadline = time.monotonic() + self.startup_timeout
        while rclpy.ok() and not self.samples:
            if time.monotonic() >= startup_deadline:
                self.get_logger().error(
                    f'{self.startup_timeout:.1f} 秒内未收到 {self.topic}。')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        sample_deadline = time.monotonic() + self.sample_seconds
        while rclpy.ok() and time.monotonic() < sample_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - self.last_sample_time > 1.0:
                self.get_logger().error('IMU 已超过 1 秒无新消息，本阶段无效。')
                return False
        if len(self.samples) < 2:
            self.get_logger().error('有效样本少于 2 帧，无法形成统计结论。')
            return False
        return rclpy.ok()

    def report(self) -> None:
        """Print stable, copyable statistics for the current phase."""
        summary = summarize_samples(self.samples)
        self.get_logger().info(
            f'阶段={self.phase}，frame_id={self.frame_id}，样本数={len(self.samples)}；'
            f'{PHASE_HINTS[self.phase]}')
        self.get_logger().info(
            '加速度单位 m/s²、角速度单位 rad/s；统计为消息原坐标系数值，'
            '未应用 TF，也不自动判定标定通过。')
        for group in (('ax', 'ay', 'az'), ('gx', 'gy', 'gz')):
            for name in group:
                values = summary[name]
                self.get_logger().info(
                    f'{name}: mean={values["mean"]:+.6f}, '
                    f'std={values["std"]:.6f}, '
                    f'min={values["min"]:+.6f}, '
                    f'max={values["max"]:+.6f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ImuAxisObserver()
        if not node.collect():
            exit_code = 1
        else:
            node.report()
    except (KeyboardInterrupt, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'IMU 观察器启动失败：{error}', file=sys.stderr)
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
