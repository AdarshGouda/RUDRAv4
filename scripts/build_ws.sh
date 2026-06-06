#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
colcon build --packages-select rudra_base_bridge
