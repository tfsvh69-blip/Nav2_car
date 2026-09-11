#!/usr/bin/env bash
set -e

# IMU 水平逆时针旋转 15 秒采样脚本
# 用途：确认单个发布者后，调用 imu_axis_observer 采样 15 秒逆时针旋转数据，确认角速度主轴与符号。

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
echo "准备执行 IMU 水平缓慢逆时针旋转采样 (15 秒)"
echo "=========================================="

# 1. 检查话题是否存在
if ! ros2 topic list 2>/dev/null | grep -qx "/imu/data_raw"; then
    echo "[错误] 未检测到 /imu/data_raw 话题！"
    echo "请确认终端 1 中隔离驱动 (run_imu_test_driver.sh) 是否已成功运行。"
    exit 1
fi

# 2. 检查发布者数量（要求有且仅有 1 个发布者）
PUB_COUNT=$(ros2 topic info /imu/data_raw 2>/dev/null | grep -i "Publisher count" | awk '{print $3}')
if [ -z "${PUB_COUNT}" ] || [ "${PUB_COUNT}" -ne 1 ]; then
    echo "[错误] /imu/data_raw 话题发布者数量异常 (当前为 ${PUB_COUNT})！必须为 1。"
    exit 1
fi
echo "✓ 确认 /imu/data_raw 发布者数量为 1 (单发布者正常)"

# 3. 准备日志文件
mkdir -p log
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="log/imu_ccw_15s_${TIMESTAMP}.log"

echo "------------------------------------------"
echo "【现场操作指导】"
echo "  1. 保持小车平放在水平地面或手扶平稳；"
echo "  2. 3 秒倒计时后开始采样（总采样时长 15.0 秒）；"
echo "  3. 采样开始后，用双手缓慢、平稳地在水平面逆时针旋转小车；"
echo "     - 旋转速度保持平稳（约 0.2～0.5 rad/s，即几秒转一圈即可，切勿剧烈晃动）；"
echo "     - 尽量保持车身水平，不要明显俯仰或倾斜；"
echo "  4. 15 秒结束前平稳停下。"
echo "------------------------------------------"
echo "3 秒后开始采样..."
sleep 3

echo ">>> 开始采样，请平稳逆时针转动小车！日志将同步保存至: ${LOG_FILE}"
ros2 run carcar_base imu_axis_observer --ros-args \
    -p topic:=/imu/data_raw \
    -p phase:=ccw \
    -p sample_seconds:=15.0 2>&1 | tee "${LOG_FILE}"

EXIT_CODE="${PIPESTATUS[0]}"
if [ "${EXIT_CODE}" -eq 0 ]; then
    echo "=========================================="
    echo "✓ IMU 水平逆时针旋转采样完成！"
    echo "完整统计日志已保存至: ${LOG_FILE}"
    echo "请将上方终端打印的六轴统计结论复制反馈。"
    echo "=========================================="
else
    echo "=========================================="
    echo "✗ 采样未完成或异常退出 (退出码: ${EXIT_CODE})。"
    echo "=========================================="
    exit "${EXIT_CODE}"
fi

