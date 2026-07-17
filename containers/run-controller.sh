#!/usr/bin/env bash
# Launch the Franka controller container with RT capabilities, host networking
# (for DDS + libfranka socket), and franka_controller + franka_description
# bind-mounted from the host.
#
# Host prerequisites:
#   - PREEMPT_RT-patched kernel
#   - Connected to the Franka Robot
#
# Usage:
#   ./containers/run-controller.sh

set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE="${CONTROLLER_IMAGE:-slicerfranka-controller:local}"

for source_dir in franka_controller franka_description; do
    if [ ! -d "${REPO_ROOT}/${source_dir}" ]; then
        echo "Missing source directory: ${REPO_ROOT}/${source_dir}" >&2
        exit 1
    fi
done

docker run -it --rm --pull=never \
    --net=host \
    --ipc=host \
    --cap-add=SYS_NICE \
    --ulimit rtprio=99 \
    --ulimit memlock=-1 \
    -e ROS_DOMAIN_ID \
    -v "${REPO_ROOT}/franka_controller":/home/ubuntu/ros2_ws/src/franka_controller \
    -v "${REPO_ROOT}/franka_description":/home/ubuntu/ros2_ws/src/franka_description \
    "${IMAGE}"
