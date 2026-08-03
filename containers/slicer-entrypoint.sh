#!/usr/bin/env bash
# Slicer container entrypoint.
# Sources ROS + workspace and builds the mounted franka_description package in
# each disposable container.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="/home/ubuntu/ros2_ws"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -d "${WS}/src/franka_description" ] && [ ! -d "${WS}/install/franka_description" ]; then
    echo "[entrypoint] building mounted franka_description..."
    (cd "${WS}" && colcon build --packages-select franka_description --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

exec "$@"
