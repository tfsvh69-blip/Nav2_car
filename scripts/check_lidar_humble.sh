#!/usr/bin/env bash
set -euo pipefail

echo "ROS 2 Humble 雷达串口检查"
echo

found_lidar=false
for device in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$device" ]] || continue

  vendor_id="$(udevadm info --query=property --name="$device" 2>/dev/null |
    sed -n 's/^ID_VENDOR_ID=//p')"
  model_id="$(udevadm info --query=property --name="$device" 2>/dev/null |
    sed -n 's/^ID_MODEL_ID=//p')"
  device_id="${vendor_id}:${model_id}"

  if [[ "$device_id" == "10c4:ea60" ]]; then
    found_lidar=true
    echo "[雷达 CP210x] $device"
    udevadm info --query=property --name="$device" 2>/dev/null |
      sed -n -E '/^(ID_MODEL=|ID_SERIAL=|DEVLINKS=)/p'
  elif [[ "$device_id" == "1a86:7523" ]]; then
    echo "[底层板 CH340，请勿作为雷达端口] $device"
  else
    echo "[其他串口 ${device_id}] $device"
  fi

  if fuser "$device" >/dev/null 2>&1; then
    echo "  状态：正在被进程占用"
    fuser -v "$device" 2>&1 || true
  else
    echo "  状态：未发现占用进程"
  fi
  echo
done

if [[ "$found_lidar" == false ]]; then
  echo "未找到 USB ID 10c4:ea60 的 RPLIDAR A1 转接板。" >&2
  exit 1
fi

echo "推荐使用 /dev/serial/by-id 下的 CP2102 稳定路径启动雷达。"
