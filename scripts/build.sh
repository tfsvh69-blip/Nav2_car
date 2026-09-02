#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
set -u
cd "$PROJECT_ROOT"
colcon build --symlink-install

echo
echo "Build complete. Run: ./scripts/run_lidar.sh"
