#!/usr/bin/env bash
set -u

echo "USB serial devices:"
found=false
for device in /dev/ttyUSB* /dev/ttyACM*; do
  if [[ -e "$device" ]]; then
    found=true
    ls -l "$device"
  fi
done

if [[ "$found" == false ]]; then
  echo "  No /dev/ttyUSB* or /dev/ttyACM* device found."
fi

echo
echo "Relevant USB devices:"
if command -v lsusb >/dev/null 2>&1; then
  lsusb 2>&1 | grep -Ei 'CP210|Silicon Labs|RoboPeak|SLAMTEC|USB.*serial' || \
    echo "  No matching USB entry found (or USB access is unavailable)."
else
  echo "  lsusb is not installed."
fi

echo
echo "Expected defaults for an RPLIDAR A1: /dev/ttyUSB0 at 115200 baud."

