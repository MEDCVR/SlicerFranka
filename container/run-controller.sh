#!/usr/bin/env bash
# Launch the Franka controller container with RT capabilities, host networking
# (for DDS + libfranka socket), and franka_controller + franka_description
# bind-mounted from the host.
#
# Host prerequisites:
#   - PREEMPT_RT-patched kernel
#   - Your user in the `realtime` group with rtprio/memlock limits set
#   - Connected to the Franka Robot
#
# Usage:
#   ./container/run-controller.sh

set -e

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE="${CONTROLLER_IMAGE:-ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy}"

docker run -it --rm \
    --net=host \
    --cap-add=SYS_NICE \
    --ulimit rtprio=99 \
    --ulimit memlock=-1 \
    -v "${REPO_ROOT}/franka_controller":/home/ubuntu/ros2_ws/src/franka_controller \
    -v "${REPO_ROOT}/franka_description":/home/ubuntu/ros2_ws/src/franka_description \
    "${IMAGE}"
