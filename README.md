# SlicerFranka

A 3D Slicer module to track and control the Franka Robots using ROS2.
It relies on [SlicerROS2](https://github.com/rosmed/slicer_ros2_module) for ROS2 communication and a [Franka Controller](./franka_controller) for actually moving the robot.

See the video below, and the [paper website](https://medcvr.utm.utoronto.ca/EMBC2026-SlicerFranka.html) for more details.

<div align="center">
  <a href="https://www.youtube.com/watch?v=SxC2sl_dKN4">
    <img src="https://img.youtube.com/vi/SxC2sl_dKN4/maxresdefault.jpg" alt="SlicerFranka demonstration" style="max-width:100%; height:auto;">
  </a>
</div>

## Features

- Joint and Cartesian state control of the robot from a 3D Slicer interface
- Using the Robot as a tracker to collect fiducial points for registration
- Trajectory planning using fiducial nodes and curves and executing them on the robot

## Quick start with Docker

The fastest path to a working SlicerFranka. The prebuilt images include 3D Slicer (built from source), SlicerROS2, libfranka, TOPP-RA, and the SlicerFranka and franka_controller code — no native install needed.

```bash
git clone https://github.com/MEDCVR/SlicerFranka.git
cd SlicerFranka

# Slicer side (any Linux desktop with X11)
docker pull ghcr.io/iselein/slicerfranka-slicer:5.10.0-jazzy
./container/run-slicer.sh

# Controller side (PREEMPT_RT-patched host connected to the Franka robot;
# can be the same machine or a different one on the same network)
docker pull ghcr.io/iselein/slicerfranka-controller:0.9.2-jazzy
./container/run-controller.sh
```

See [container/README.md](./container/README.md) for build-from-source instructions, GPU support (`--gpu` flag), and the full host-prerequisite list.

## Dependencies

- Franka robot
- [Libfranka](https://github.com/frankaemika/libfranka)
- 3D Slicer (built from source)
- ROS2 — tested on **Humble (Ubuntu 22.04)** and **Jazzy (Ubuntu 24.04)**
- [SlicerROS2](https://github.com/rosmed/slicer_ros2_module)

## Manual installation

If you'd rather install everything natively (or are running on a platform without Docker), this is the path. Both supported Ubuntu / ROS combinations have been tested.

The setup supports running 3D Slicer and the Franka controller on different machines, communicating over ROS2. This is the common case because Slicer needs a desktop GUI and the controller needs a PREEMPT_RT-patched kernel — requirements that don't always coexist comfortably. Both can also run on the same PC.

### 3D Slicer setup with extensions

- Build 3D Slicer from source following the [Slicer build instructions](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html). The containers pin v5.10.0.
- Install the SlicerROS2 extension following the [SlicerROS2 documentation](https://slicer-ros2.readthedocs.io/en/v1.0/pages/getting-started.html)
- Clone this repository: `git clone https://github.com/MEDCVR/SlicerFranka.git`
- Copy the [franka_description](./franka_description) repository into the `src` directory of the SlicerROS2 workspace and run `colcon build` again.
- Add the SlicerFranka module directory to the "Additional Module Paths" in 3D Slicer (Go to Edit -> Application Settings -> Modules -> Additional Module Paths -> Add New Module Path: /full/file/path/slicerfranka/SlicerFranka then restart 3D Slicer)

### Franka Controller Setup

- This MUST be done on a real-time kernel
- Install [libfranka](https://github.com/frankaemika/libfranka)
- Follow the instructions in [franka_controller](./franka_controller) to set up the Franka controller.

You're all set up!

## Developer Guide

An extensive developer guide can be found in the [developer guide](docs/slicerfranka.md)

## Citation

If you use SlicerFranka in your research, please cite:

```bibtex
@inproceedings{2026slicerfranka,
  title     = {SlicerFranka: Open-Source Integration of the Franka Robot
               with 3D Slicer via a native ROS2 interface},
  author    = {Iseoluwa, Oluwagbotemi D. and Gondokaryono, Radian
               and Kahrs, Lueder A.},
  booktitle = {Proceedings of the IEEE Engineering in Medicine and Biology
               Society (EMBC)},
  year      = {2026},
  url       = {https://medcvr.utm.utoronto.ca/EMBC2026-SlicerFranka.html}
}
```
