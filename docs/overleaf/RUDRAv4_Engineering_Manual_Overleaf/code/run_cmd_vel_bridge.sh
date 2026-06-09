#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
source install/setup.bash
ros2 launch rudra_base_bridge cmd_vel_to_teensy.launch.py \
  config:="$(pwd)/src/rudra_base_bridge/config/rudra_v4_hardware.yaml"
