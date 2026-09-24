#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/rby1_ros2_ws/install/setup.bash
source /opt/mtc_ws/install/setup.bash
source /opt/planning_ws/install/setup.bash
exec "$@"
