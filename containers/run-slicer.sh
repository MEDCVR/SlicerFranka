#!/usr/bin/env bash
# Launch the Slicer container with X11, host networking (for DDS), a named
# volume for Slicer settings persistence, and the SlicerFranka module +
# franka_description bind-mounted from the host.
#
# Usage:
#   ./containers/run-slicer.sh          # CPU/software-GL rendering
#   ./containers/run-slicer.sh --gpu    # NVIDIA GPU passthrough (needs
#                                       # nvidia-container-toolkit on host)

set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE="${SLICER_IMAGE:-slicerfranka-slicer:local}"

GPU_ARGS=()
if [ "${1:-}" = "--gpu" ]; then
    GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display)
    shift
fi
if [ "$#" -ne 0 ]; then
    echo "Usage: $0 [--gpu]" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set; an X11 or XWayland display is required." >&2
    exit 1
fi

for source_dir in SlicerFranka franka_description; do
    if [ ! -d "${REPO_ROOT}/${source_dir}" ]; then
        echo "Missing source directory: ${REPO_ROOT}/${source_dir}" >&2
        exit 1
    fi
done

if ! command -v xauth >/dev/null; then
    echo "xauth is required for secure X11 forwarding." >&2
    exit 1
fi

XAUTH_FILE=$(mktemp)
trap 'rm -f "${XAUTH_FILE}"' EXIT
xauth nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH_FILE}" nmerge -
if [ ! -s "${XAUTH_FILE}" ]; then
    echo "No X11 authentication cookie found for DISPLAY=${DISPLAY}." >&2
    exit 1
fi
chmod 0644 "${XAUTH_FILE}"

docker run -it --rm --pull=never \
    --net=host \
    --ipc=host \
    -e DISPLAY="${DISPLAY}" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e ROS_DOMAIN_ID \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${XAUTH_FILE}":/tmp/.docker.xauth:ro \
    -v slicerfranka-config:/home/ubuntu/.config/NA-MIC \
    -v "${REPO_ROOT}/SlicerFranka":/home/ubuntu/SlicerFranka \
    -v "${REPO_ROOT}/franka_description":/home/ubuntu/ros2_ws/src/franka_description \
    "${GPU_ARGS[@]}" \
    "${IMAGE}"
