#!/usr/bin/env bash

# IMU 测试停止与串口状态检查脚本
# 用途：确认测试节点已退出，并验证 /dev/myserial 串口完全释放。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "install/setup.bash" ] && source install/setup.bash 2>/dev/null || true

echo "=========================================="
echo "检查 IMU 测试节点退出状态与串口占用情况"
echo "=========================================="

# 检查当前 ROS 节点
echo "当前活跃的 ROS 节点："
ros2 node list 2>/dev/null || echo "（无活跃节点或 ROS 守护进程未启动）"

echo "------------------------------------------"
echo "检查底盘串口 /dev/myserial 占用："
if [ ! -e /dev/myserial ]; then
    echo "注意：/dev/myserial 不存在（控制板未插入或未上电）。"
elif fuser /dev/myserial >/dev/null 2>&1; then
    echo "[警告] 串口 /dev/myserial 仍被以下进程占用："
    fuser -v /dev/myserial
    echo "请在对应运行终端按 Ctrl+C 退出进程。"
else
    echo "✓ 串口 /dev/myserial 已完全释放，无进程占用。"
fi

echo "=========================================="

