# SlicerFranka containers

Two images, both based on Ubuntu 24.04 with ROS 2 Jazzy:

- **`slicerfranka-slicer`** — 3D Slicer v5.10.0 (built from source), SlicerROS2, and the deps the SlicerFranka module needs.
- **`slicerfranka-controller`** — libfranka 0.9.2, TOPP-RA, and the deps the `franka_controller` package needs.

The two containers communicate over ROS 2 / DDS. They can run on the same host or on two PCs on the same network.

## Quick start (pull prebuilt images)

```bash
# Slicer
docker pull ghcr.io/iselein/slicerfranka-slicer:5.10.0-jazzy
./containers/run-slicer.sh

# Controller (on a PREEMPT_RT host, can be the same or a different PC)
docker pull ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy
./containers/run-controller.sh
```

## Building from source

The Slicer image takes ~30–60 minutes to build (Slicer SuperBuild is the long pole). The controller image takes ~5–10 minutes.

```bash
# From the repo root.
docker build -f containers/slicer.dockerfile     -t slicerfranka-slicer:local .
docker build -f containers/controller.dockerfile -t slicerfranka-controller:local .
```

Both images run as the default `ubuntu` user (UID 1000) that ships in the Ubuntu 24.04 base. If your host UID isn't 1000, files written through bind mounts will be owned by UID 1000 on the host — you'll still be able to read/write them, just check `ls -l` if something looks off.

To launch the locally-built images instead of the registry ones:

```bash
SLICER_IMAGE=slicerfranka-slicer:local         ./containers/run-slicer.sh
CONTROLLER_IMAGE=slicerfranka-controller:local ./containers/run-controller.sh
```

## What's baked vs mounted

| | Baked into image | Mounted from host at run time |
|---|---|---|
| Slicer | ROS Jazzy desktop, Qt5 libs, Slicer 5.10.0 build, SlicerROS2 | `SlicerFranka/` module, `franka_description/` |
| Controller | ROS Jazzy base, libfranka, TOPP-RA, build toolchain | `franka_controller/`, `franka_description/` |

The first time the controller container starts, the entrypoint runs `colcon build` against the mounted sources. Subsequent launches are fast.

The Slicer container persists user settings (including "Additional module paths") in the named docker volume `slicerfranka-config`. First launch you'll still need to add the SlicerFranka and SlicerROS2 module paths via Edit → Application Settings → Modules → Additional module paths; thereafter they're remembered.

## Host prerequisites

**For the Slicer container**
- Docker (Linux host with an X server). Tested on Ubuntu 24.04 desktop.
- For GPU rendering: NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) on the host. Pass `--gpu` to `run-slicer.sh`. Without it, Slicer falls back to software rendering (works but slower).

**For the controller container**
- Docker on a PREEMPT_RT-patched Linux kernel. The container only gets the *capability* (`--cap-add=SYS_NICE`, rtprio ulimit); the realtime scheduler itself is the host kernel's job.
- Your host user should be in a `realtime` group with rtprio + memlock limits set in `/etc/security/limits.d/realtime.conf` so docker can actually grant those limits.
- A network interface routable to the robot's IP (typically `192.168.x.x`).

**For the two containers to talk**
- Same machine: nothing extra. Both use `--net=host`, so DDS discovery is unaffected by docker's bridge networking.
- Two machines on the same LAN: same — `--net=host` and a working multicast route between the two boxes is enough.

## Run command details

The wrappers do this under the hood:

```bash
# run-slicer.sh
xhost +local:docker
docker run -it --rm \
    --net=host \
    -e DISPLAY -e XAUTHORITY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:$XAUTHORITY \
    -v slicerfranka-config:/home/ubuntu/.config/NA-MIC \
    -v <repo>/SlicerFranka:/home/ubuntu/SlicerFranka \
    -v <repo>/franka_description:/home/ubuntu/ros2_ws/src/franka_description \
    ghcr.io/iselein/slicerfranka-slicer:5.10.0-jazzy

# run-controller.sh
docker run -it --rm \
    --net=host \
    --cap-add=SYS_NICE \
    --ulimit rtprio=99 --ulimit memlock=-1 \
    -v <repo>/franka_controller:/home/ubuntu/ros2_ws/src/franka_controller \
    -v <repo>/franka_description:/home/ubuntu/ros2_ws/src/franka_description \
    ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy
```

## Inside the containers

Both shells auto-source ROS and the workspace, so `ros2`, `colcon`, etc. just work.

**Slicer container**
```bash
slicer                                     # alias for the built Slicer binary
```
First launch only: in the Settings dialog, add module paths
- `/home/ubuntu/ros2_ws/install/ROS2/lib/Slicer-5.10/qt-loadable-modules`
- `/home/ubuntu/SlicerFranka`
then restart Slicer. These persist in the `slicerfranka-config` volume.

**Controller container**
```bash
ros2 launch franka_controller controller.launch.py   # or: ctlr
```

## Publishing the images (maintainer notes)

```bash
echo $GHCR_PAT | docker login ghcr.io -u iselein --password-stdin

docker tag slicerfranka-slicer:local         ghcr.io/iselein/slicerfranka-slicer:5.10.0-jazzy
docker tag slicerfranka-controller:local     ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy

docker push ghcr.io/iselein/slicerfranka-slicer:5.10.0-jazzy
docker push ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy
```

The PAT needs the `write:packages` scope.
