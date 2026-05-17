#!/usr/bin/env bash
# Launch the Slicer container with X11, host networking (for DDS), a named
# volume for Slicer settings persistence, and the SlicerFranka module +
# franka_description bind-mounted from the host.
#
# Usage:
#   ./container/run-slicer.sh           # CPU/software-GL rendering
#   ./container/run-slicer.sh --gpu     # NVIDIA GPU passthrough (needs
#                                       # nvidia-container-toolkit on host)

set -e

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE="${SLICER_IMAGE:-ghcr.io/iselein/slicerfranka-slicer:5.10.0-jazzy}"

GPU_ARGS=()
if [ "${1:-}" = "--gpu" ]; then
    GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display)
fi

# Allow the container to reach the host X server (idempotent).
xhost +local:docker > /dev/null

docker run -it --rm \
    --net=host \
    -e DISPLAY="${DISPLAY}" \
    -e XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${XAUTHORITY:-${HOME}/.Xauthority}":"${XAUTHORITY:-${HOME}/.Xauthority}" \
    -v slicerfranka-config:/home/ubuntu/.config/NA-MIC \
    -v "${REPO_ROOT}/SlicerFranka":/home/ubuntu/SlicerFranka \
    -v "${REPO_ROOT}/franka_description":/home/ubuntu/ros2_ws/src/franka_description \
    "${GPU_ARGS[@]}" \
    "${IMAGE}"
