#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
if [ -f /home/rudra/ros2_ws/install/setup.bash ]; then
  source /home/rudra/ros2_ws/install/setup.bash
fi
source install/setup.bash
ros2 launch rudra_base_bridge lidar_view.launch.py
