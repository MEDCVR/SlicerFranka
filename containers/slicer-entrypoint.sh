#!/usr/bin/env bash
# Slicer container entrypoint.
# Sources ROS + workspace, builds franka_description on first launch if
# the source dir is mounted but its install tree is missing.
set -e

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="/home/ubuntu/ros2_ws"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -d "${WS}/src/franka_description" ] && [ ! -d "${WS}/install/franka_description" ]; then
    echo "[entrypoint] franka_description mounted but not built — running colcon build..."
    (cd "${WS}" && colcon build --packages-select franka_description --cmake-args -DCMAKE_BUILD_TYPE=Release)
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

exec "$@"
