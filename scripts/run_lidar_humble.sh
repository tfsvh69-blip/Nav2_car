#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
SERIAL_PORT="${1:-$DEFAULT_PORT}"
USE_RVIZ="${2:-true}"

if [[ ! -e "$SERIAL_PORT" ]]; then
  echo "错误：雷达串口不存在：$SERIAL_PORT" >&2
  echo "请先执行：cd $PROJECT_ROOT && ./scripts/check_lidar_humble.sh" >&2
  exit 1
fi

resolved_port="$(readlink -f "$SERIAL_PORT")"
vendor_id="$(udevadm info --query=property --name="$resolved_port" 2>/dev/null |
  sed -n 's/^ID_VENDOR_ID=//p')"
model_id="$(udevadm info --query=property --name="$resolved_port" 2>/dev/null |
  sed -n 's/^ID_MODEL_ID=//p')"

if [[ "${vendor_id}:${model_id}" == "1a86:7523" ]]; then
  echo "拒绝启动：$SERIAL_PORT 是底层控制板 CH340，不是雷达 CP210x。" >&2
  exit 1
fi

if [[ ! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT" ]]; then
  echo "错误：当前用户没有 $SERIAL_PORT 的读写权限。" >&2
  echo "确认用户属于 dialout 组，并注销后重新登录。" >&2
  exit 1
fi

if fuser "$resolved_port" >/dev/null 2>&1; then
  echo "错误：$resolved_port 已被其他进程占用。" >&2
  fuser -v "$resolved_port" 2>&1 || true
  echo "请先在原终端按 Ctrl+C 停止旧雷达驱动。" >&2
  exit 1
fi

unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"
export ROS_LOG_DIR="$PROJECT_ROOT/log/ros/lidar_humble"
mkdir -p "$ROS_LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u
cd "$PROJECT_ROOT"

exec ros2 launch carcar_lidar lidar_rviz_test.launch.py \
  serial_port:="$SERIAL_PORT" \
  serial_baudrate:=115200 \
  scan_mode:=Express \
  use_rviz:="$USE_RVIZ"
