#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch rudra_base_bridge cmd_vel_to_teensy.launch.py \
  config:="$(pwd)/src/rudra_base_bridge/config/rudra_v4_hardware.yaml"
