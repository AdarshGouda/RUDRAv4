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
DEFAULT_LIDAR_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
ros2 launch rudra_base_bridge rudra_ps2_lidar_localization.launch.py \
  serial_port:="${1:-$DEFAULT_LIDAR_PORT}" \
  config:="${2:-$(pwd)/src/rudra_base_bridge/config/rudra_v4_hardware.yaml}" \
  lidar_config:="${3:-$(pwd)/src/rudra_base_bridge/config/ydlidar_g2b.yaml}" \
  ekf_config:="${4:-$(pwd)/src/rudra_base_bridge/config/localization_ekf.yaml}" \
  frame_id:="${5:-laser}"
