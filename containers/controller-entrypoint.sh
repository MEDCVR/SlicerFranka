#!/usr/bin/env bash
# Controller container entrypoint.
# Sources ROS + workspace and builds the mounted packages in each disposable
# container.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="/home/ubuntu/ros2_ws"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

NEED_BUILD=0
[ -d "${WS}/src/franka_description" ]  && [ ! -d "${WS}/install/franka_description" ]  && NEED_BUILD=1
[ -d "${WS}/src/franka_controller" ]   && [ ! -d "${WS}/install/franka_controller" ]   && NEED_BUILD=1

if [ "${NEED_BUILD}" -eq 1 ]; then
    echo "[entrypoint] building workspace (franka_description, franka_controller)..."
    (cd "${WS}" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF)
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

exec "$@"
