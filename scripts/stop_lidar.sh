#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/lyrical/setup.bash
if [ -f /home/rudra/ros2_ws/install/setup.bash ]; then
  source /home/rudra/ros2_ws/install/setup.bash
fi
if [ -f /home/rudra/Projects/RUDRAv4/install/setup.bash ]; then
  source /home/rudra/Projects/RUDRAv4/install/setup.bash
fi

stopped_by_service=false
if ros2 service list 2>/dev/null | grep -qx '/stop_scan'; then
  ros2 service call /stop_scan std_srvs/srv/Empty '{}' || true
  stopped_by_service=true
fi

pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
pkill -f 'sllidar_node' 2>/dev/null || true
pkill -f 'ros2 launch rudra_base_bridge lidar.launch.py' 2>/dev/null || true
pkill -f 'ros2 launch rudra_base_bridge sllidar.launch.py' 2>/dev/null || true

if [ "$stopped_by_service" = true ]; then
  echo 'Requested LiDAR stop through /stop_scan and cleaned up driver processes.'
else
  echo 'No /stop_scan service was available; cleaned up any matching driver processes.'
fi
echo 'If the LiDAR still spins now, it is being powered directly by USB. Unplug the LiDAR USB cable or switch off its USB power/hub port.'
