#!/usr/bin/env bash

set -e

# Nạp môi trường ROS 2 Humble
source /opt/ros/humble/setup.bash

# Nạp workspace rby1-ros2 nếu đã được build
if [ -f "/opt/rby1_ros2_ws/install/setup.bash" ]; then
    source /opt/rby1_ros2_ws/install/setup.bash
fi

# Thêm thư viện rby1-sdk vào đường dẫn runtime
export LD_LIBRARY_PATH="/opt/rby1-sdk/build/src:${LD_LIBRARY_PATH}"

# Chạy lệnh được truyền vào container
exec "$@"