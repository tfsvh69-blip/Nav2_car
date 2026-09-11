#!/usr/bin/env bash
set -e

# NAV-001: SLAM Toolbox 建图链路只读检查启动脚本
# 用途：只读检查节点、话题、频率、消息帧和完整 TF 链，并将结果存入 log 目录。
# 安全保证：严禁发布 /cmd_vel 或任何电机驱动命令。

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

mkdir -p log
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="log/slam_link_check_${TIMESTAMP}.log"

echo "=========================================="
echo "准备执行 NAV-001 SLAM Toolbox 链路只读检查"
echo "日志将保存至: ${LOG_FILE}"
echo "=========================================="

python3 "${SCRIPT_DIR}/check_slam_link.py" 2>&1 | tee "${LOG_FILE}"

EXIT_CODE="${PIPESTATUS[0]}"

echo "=========================================="
if [ "${EXIT_CODE}" -eq 0 ]; then
    echo "✓ 链路检查完成，全部通过！"
else
    echo "✗ 链路检查完成，存在未通过项 (退出码: ${EXIT_CODE})。"
fi
echo "完整检查日志已记录于: ${LOG_FILE}"
echo "=========================================="

exit "${EXIT_CODE}"
