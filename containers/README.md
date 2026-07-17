# SlicerFranka containers

Two images, both based on Ubuntu 24.04 with ROS 2 Jazzy:

- **`slicerfranka-slicer`** — 3D Slicer v5.10.0 (built from source), SlicerROS2, and the deps the SlicerFranka module needs.
- **`slicerfranka-controller`** — libfranka 0.9.2, TOPP-RA, and the deps the `franka_controller` package needs.

The two containers communicate over ROS 2 / DDS. They can run on the same host or on two PCs on the same network.

## Build and run

```bash
# From the repository root. The Slicer build can take several hours; the
# controller build usually takes 5-10 minutes.
docker build -f containers/slicer.dockerfile -t slicerfranka-slicer:local .
docker build -f containers/controller.dockerfile -t slicerfranka-controller:local .

./containers/run-slicer.sh
# At the container prompt:
slicer

# On a PREEMPT_RT host connected to the robot:
./containers/run-controller.sh
# At the container prompt:
ctlr
```
Both images run as the Ubuntu base image's `ubuntu` user (UID/GID 1000). The
mounted source is writable, so edits made inside a container immediately affect
the host. On a host where your user is not UID/GID 1000, edit on the host or
adjust the image user before relying on writes from inside the container.

## What's baked vs mounted

| | Baked into image | Mounted from host at run time |
|---|---|---|
| Slicer | ROS Jazzy desktop, Qt5 libs, Slicer 5.10.0 build, SlicerROS2 | `SlicerFranka/` module, `franka_description/` |
| Controller | ROS Jazzy base, libfranka, TOPP-RA, build toolchain | `franka_controller/`, `franka_description/` |

The entrypoints build the mounted ROS packages into the container's writable
layer. Because every container is disposable, `franka_controller` is rebuilt on
every controller launch and `franka_description` is rebuilt on every Slicer
launch. These build, install, and log trees are deleted with the container and
never pollute the host source tree.

The Slicer container persists user settings in the named volume
`slicerfranka-config`. This volume survives container deletion by design. Run
`docker volume rm slicerfranka-config` when you explicitly want to reset Slicer.
The `slicer` alias launches through SlicerROS2 and adds the mounted SlicerFranka
module path automatically.

## Host prerequisites

**For the Slicer container**
- Docker (Linux host with an X server). Tested on Ubuntu 24.04 desktop.
- For GPU rendering: NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) on the host. Pass `--gpu` to `run-slicer.sh`. Without it, Slicer falls back to software rendering (works but slower).

**For the controller container**
- Docker on a PREEMPT_RT-patched Linux kernel. The container only gets the *capability* (`--cap-add=SYS_NICE`, rtprio ulimit); the realtime scheduler itself is the host kernel's job.
- A network interface routable to the robot's IP (typically `192.168.x.x`).

**For the two containers to talk**
- Same machine: both wrappers use host networking and host IPC so Fast DDS can use its shared-memory transport across the containers.
- Two machines on the same LAN: same — `--net=host` and a working multicast route between the two boxes is enough.
- If you use a non-default ROS domain, export the same `ROS_DOMAIN_ID` before running each wrapper.

Both wrappers use `docker run --rm`, so the container is removed whenever its
main process exits normally. Images remain available for the next run. An
unclean daemon or host shutdown can leave a stopped container behind; it is safe
to remove because persistent source and settings live outside it.

## Inside the containers

Both shells auto-source ROS and the workspace, so `ros2`, `colcon`, etc. just work.

**Slicer container**
```bash
slicer  # SlicerROS2 launcher plus the mounted SlicerFranka module path
```

**Controller container**
```bash
ros2 launch franka_controller controller.launch.py   # or: ctlr
```
