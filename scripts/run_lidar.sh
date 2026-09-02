#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SERIAL_PORT="${1:-/dev/ttyUSB0}"
export ROS_LOG_DIR="$PROJECT_ROOT/log/ros"

if [[ ! -e "$SERIAL_PORT" ]]; then
  echo "Error: serial device '$SERIAL_PORT' does not exist." >&2
  echo "Connect the lidar and run ./scripts/check_lidar.sh first." >&2
  exit 1
fi

if [[ ! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT" ]]; then
  echo "Error: current user cannot read and write '$SERIAL_PORT'." >&2
  echo "For a temporary test: sudo chmod a+rw $SERIAL_PORT" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u

exec ros2 launch inspection_bringup lidar_test.launch.py \
  serial_port:="$SERIAL_PORT" \
  serial_baudrate:=115200 \
  use_rviz:=true
