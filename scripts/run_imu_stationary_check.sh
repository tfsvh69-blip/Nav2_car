#!/usr/bin/env bash
set -e

# IMU 水平静止 30 秒采样脚本
# 用途：确认单个发布者后，调用 imu_axis_observer 采样 30 秒静止数据，并自动保存至 log 目录。

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

echo "=========================================="
echo "准备执行 IMU 水平静止 30 秒数据采样"
echo "=========================================="

# 1. 检查话题是否存在
echo "正在检测话题 /imu/data_raw..."
if ! ros2 topic list 2>/dev/null | grep -qx "/imu/data_raw"; then
    echo "[错误] 未检测到 /imu/data_raw 话题！"
    echo "请确认终端 1 中隔离驱动 (run_imu_test_driver.sh) 是否已成功启动。"
    exit 1
fi

# 2. 检查发布者数量（要求有且仅有 1 个发布者）
PUB_COUNT=$(ros2 topic info /imu/data_raw 2>/dev/null | grep -i "Publisher count" | awk '{print $3}')
if [ -z "${PUB_COUNT}" ] || [ "${PUB_COUNT}" -lt 1 ]; then
    echo "[错误] /imu/data_raw 话题当前无发布者！"
    exit 1
elif [ "${PUB_COUNT}" -gt 1 ]; then
    echo "[错误] /imu/data_raw 话题存在 ${PUB_COUNT} 个发布者！必须保持单发布者独占。"
    ros2 topic info /imu/data_raw --verbose
    exit 1
fi
echo "✓ 确认 /imu/data_raw 发布者数量为 1 (单发布者正常)"

# 3. 准备日志文件
mkdir -p log
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="log/imu_level_30s_${TIMESTAMP}.log"

echo "------------------------------------------"
echo "【现场操作要求】"
echo "  1. 确认小车已平稳放置在水平地面上；"
echo "  2. 控制板已牢固固定，未受外力晃动；"
echo "  3. 采样期间严禁触碰车体、地面或线缆，保持完全静止；"
echo "  4. 采样时长为 30.0 秒（预期约 750 帧，25 Hz）。"
echo "------------------------------------------"
echo "3 秒后开始采样..."
sleep 3

echo ">>> 开始采样，日志将同步保存至: ${LOG_FILE}"
# 运行观察器并同步记录日志
ros2 run carcar_base imu_axis_observer --ros-args \
    -p topic:=/imu/data_raw \
    -p phase:=level \
    -p sample_seconds:=30.0 2>&1 | tee "${LOG_FILE}"

EXIT_CODE="${PIPESTATUS[0]}"
if [ "${EXIT_CODE}" -eq 0 ]; then
    echo "=========================================="
    echo "✓ IMU 水平静止 30 秒采样完成！"
    echo "完整统计日志已保存至: ${LOG_FILE}"
    echo "请将上方终端打印的六轴统计结论复制反馈。"
    echo "=========================================="
else
    echo "=========================================="
    echo "✗ 采样未完成或异常退出 (退出码: ${EXIT_CODE})，请检查日志与硬件连接。"
    echo "=========================================="
    exit "${EXIT_CODE}"
fi

