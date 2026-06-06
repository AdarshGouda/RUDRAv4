#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch rudra_base_bridge lidar.launch.py serial_port:="${1:-/dev/ttyUSB0}"
