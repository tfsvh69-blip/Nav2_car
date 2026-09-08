#!/usr/bin/env bash
# D435 空闲状态下 USB 自动挂起 A/B 检查；不启动视频流，不修改相机固件。

set -euo pipefail

SERIAL="${1:-213222078719}"
OBSERVE_SECONDS="${OBSERVE_SECONDS:-20}"

if ! [[ "$OBSERVE_SECONDS" =~ ^[0-9]+$ ]] || (( OBSERVE_SECONDS < 10 || OBSERVE_SECONDS > 60 )); then
  echo "错误：OBSERVE_SECONDS 必须是 10～60 的整数。" >&2
  exit 2
fi

usb_device=""
for serial_file in /sys/bus/usb/devices/*/serial; do
  [[ -r "$serial_file" ]] || continue
  if [[ "$(tr -d '[:space:]' < "$serial_file")" == "$SERIAL" ]]; then
    usb_device="${serial_file%/serial}"
    break
  fi
done

if [[ -z "$usb_device" ]]; then
  d435_candidates=()
  for vendor_file in /sys/bus/usb/devices/*/idVendor; do
    [[ -r "$vendor_file" ]] || continue
    candidate="${vendor_file%/idVendor}"
    [[ -r "$candidate/idProduct" ]] || continue
    vendor="$(tr -d '[:space:]' < "$vendor_file")"
    product="$(tr -d '[:space:]' < "$candidate/idProduct")"
    if [[ "$vendor:$product" == "8086:0b07" ]]; then
      d435_candidates+=("$candidate")
    fi
  done
  if (( ${#d435_candidates[@]} == 1 )); then
    usb_device="${d435_candidates[0]}"
    sysfs_serial="未知"
    [[ -r "$usb_device/serial" ]] && sysfs_serial="$(tr -d '[:space:]' < "$usb_device/serial")"
    echo "警告：SDK 序列号 $SERIAL 未在 USB sysfs 中出现；唯一 D435 的 sysfs 序列号为 $sysfs_serial。" >&2
    echo "将使用唯一的 8086:0b07 设备 $usb_device；此差异只记录，不自动解释为硬件故障。" >&2
  elif (( ${#d435_candidates[@]} == 0 )); then
    echo "错误：未找到序列号为 $SERIAL 或 USB ID 8086:0b07 的 D435。" >&2
    exit 1
  else
    echo "错误：未匹配 SDK 序列号 $SERIAL，且发现多台 8086:0b07；拒绝自动选择。" >&2
    exit 1
  fi
fi

if fuser /dev/video* >/dev/null 2>&1; then
  echo "错误：有进程正在占用 /dev/video*；先关闭 Viewer、ROS 相机节点和测试脚本。" >&2
  fuser -v /dev/video* 2>&1 || true
  exit 1
fi

control_file="$usb_device/power/control"
runtime_file="$usb_device/power/runtime_status"
usb_name="${usb_device##*/}"
if [[ ! -r "$control_file" || ! -r "$runtime_file" ]]; then
  echo "错误：设备没有可读的 USB 电源管理属性：$usb_device" >&2
  exit 1
fi

original_control="$(tr -d '[:space:]' < "$control_file")"
restore_needed=0
restored=0
restore_control() {
  if (( restore_needed == 1 && restored == 0 )); then
    printf '%s\n' "$original_control" | sudo tee "$control_file" >/dev/null || true
    restored=1
  fi
}
trap restore_control EXIT INT TERM

count_errors() {
  local since="$1"
  local label="$2"
  local log
  if ! log="$(journalctl -k --since "$since" --no-pager -o cat 2>&1)"; then
    echo "错误：journalctl 无法读取阶段日志：$log" >&2
    return 1
  fi
  log="$(printf '%s\n' "$log" | grep -F "$usb_name" || true)"
  printf '%s: GET_CUR_-32=%s, completion_-71=%s\n' \
    "$label" \
    "$(printf '%s\n' "$log" | grep -c 'GET_CUR.*-32' || true)" \
    "$(printf '%s\n' "$log" | grep -Ec '(-71.*completion handler|completion handler.*-71)' || true)"
}

sysfs_serial="未知"
[[ -r "$usb_device/serial" ]] && sysfs_serial="$(tr -d '[:space:]' < "$usb_device/serial")"
echo "设备：$usb_device，SDK 序列号：$SERIAL，sysfs 序列号：$sysfs_serial"
echo "原始 power/control=$original_control，runtime_status=$(<"$runtime_file")"
echo "本脚本只观察空闲设备，不启动图像流。需要 sudo 仅用于临时切换 power/control。"

baseline_start="$(date '+%Y-%m-%d %H:%M:%S')"
echo "阶段 A：保持 $original_control，观察 ${OBSERVE_SECONDS}s……"
sleep "$OBSERVE_SECONDS"
count_errors "$baseline_start" "阶段 A"

restore_needed=1
printf 'on\n' | sudo tee "$control_file" >/dev/null
on_start="$(date '+%Y-%m-%d %H:%M:%S')"
echo "阶段 B：设置 power/control=on，观察 ${OBSERVE_SECONDS}s……"
sleep "$OBSERVE_SECONDS"
count_errors "$on_start" "阶段 B"

restore_control
echo "已恢复 power/control=$original_control；当前 runtime_status=$(<"$runtime_file")"
echo "请把完整输出发回；在分析结果前不要运行 realsense-viewer 或提高帧率。"
