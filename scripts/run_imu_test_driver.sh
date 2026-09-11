#!/usr/bin/env bash
set -e

# IMU 单项测试隔离驱动启动脚本
# 用途：以零速度上限、隔离运动话题、禁用 Odom/TF 发布的模式启动控制板底盘驱动，仅发布 /imu/data_raw。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash

if [ ! -f "install/setup.bash" ]; then
    echo "[错误] 未检测到 install/setup.bash，请先执行构建。"
    exit 1
fi
source install/setup.bash

# 检查串口设备
if [ ! -e /dev/myserial ]; then
    echo "[错误] 控制板未连接 (/dev/myserial 不存在)，停止启动！"
    exit 1
fi

# 检查串口是否被其他进程占用
if fuser /dev/myserial >/dev/null 2>&1; then
    echo "[错误] 串口 /dev/myserial 已被其他进程占用："
    fuser -v /dev/myserial 2>/dev/null || true
    echo "请先正常关闭其他占用串口的节点后再启动。"
    exit 1
fi

echo "=========================================="
echo "正在启动 IMU 隔离驱动 (imu_test_driver)..."
echo "配置模式："
echo "  - 速度限制：max_linear_x=0.0, max_linear_y=0.0, max_angular_z=0.0"
echo "  - 运动隔离：cmd_vel 重映射至 /imu_test/cmd_vel_disabled"
echo "  - 状态隔离：wheel_odometry_enabled=false, publish_odom_tf=false"
echo "  - 数据输出：/imu/data_raw"
echo "安全提示："
echo "  - 12V 电机电源开关必须随时可触及"
echo "  - 任何异常请立即断开 12V 电源"
echo "  - 退出请按 Ctrl+C"
echo "=========================================="

exec ros2 launch carcar_base imu_test.launch.py

