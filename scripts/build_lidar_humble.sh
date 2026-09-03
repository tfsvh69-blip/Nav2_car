#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Humble 的 ament 必须使用 Ubuntu 22.04 系统 Python，避免 Archiconda 3.7
# 抢占解释器并造成 catkin_pkg 等系统模块不可见。
unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$PROJECT_ROOT"

colcon build --symlink-install \
  --packages-up-to carcar_lidar \
  --cmake-clean-cache \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3

echo
echo "Humble 雷达包构建完成。"
echo "下一步：cd $PROJECT_ROOT && ./scripts/run_lidar_humble.sh"
