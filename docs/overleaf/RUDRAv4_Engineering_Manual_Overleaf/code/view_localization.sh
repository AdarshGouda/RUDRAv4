#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
if [ -f /home/rudra/ros2_ws/install/setup.bash ]; then
  source /home/rudra/ros2_ws/install/setup.bash
fi
source install/setup.bash
ros2 launch rudra_base_bridge localization_view.launch.py
