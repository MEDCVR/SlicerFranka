# SlicerFranka

A 3D Slicer module to track and control the Franka Robots using ROS2.
It relies on [SlicerROS2](https://github.com/rosmed/slicer_ros2_module) for ROS2 communication and a [Franka Controller](./medcvr_franka) for actually moving the robot.

<iframe width="560" height="315" src="https://www.youtube.com/embed/SxC2sl_dKN4?si=7wGOILRDNwXEcl17" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

See [paper website](https://medcvr.utm.utoronto.ca/EMBC2026-SlicerFranka.html) for more details.

## Features

- Joint and Cartesian state control of the robot from a 3D Slicer interface
- Using the Robot as a tracker to collect fiducial points for registration
- Trajectory planning using fiducial nodes and curves and executing them on the robot

## Dependencies

- Franka robot
- [Libfranka](https://github.com/frankaemika/libfranka)
- 3D Slicer (recommended version: 5.6.2)
- ROS2 (humble or jazzy)
- [SlicerROS2](https://github.com/rosmed/slicer_ros2_module)

## Installation/Setup

The library is setup such that 3D Slicer and the Franka controller can be run on different machines. This may be desired as running 3D Slicer requires a rich graphical user interface experience while running the Franka controller requires a PREEMPT_RT patched (real time) Linux kernel. These requirements may be at odds. However, both programs can be run on the same PC. Communication via ROS2 makes this feasible.

### 3D Slicer setup with extensions
- This has been tested on Ubuntu 22.04 with ROS2 Humble (and Ubuntu 24.04 with ROS2 Jazzy)
- Build 3D Slicer (version 5.6.2) from source following the instructions provided in the Slicer [documentation](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/linux.html)
- Install the SlicerROS2 extension following the instructions provided in the SlicerROS2 [documentation](https://slicer-ros2.readthedocs.io/en/v1.0/pages/getting-started.html)
- Clone this repository: `git clone https://github.com/MEDCVR/SlicerFranka.git`
- Copy the [franka_description](./franka_description) repository into the `src` directory of the SlicerROS2 workspace and run `colcon build` again.
- Add the SlicerFranka module directory to the "Additional Module Paths" in 3D Slicer (Go to Edit -> Application Settings -> Modules -> Additional Module Paths -> Add New Module Path: /full/file/path/slicerfranka/SlicerFranka then restart 3D Slicer)

### Franka Controller Setup
- This MUST be done on a real-time kernel
- This has been tested on Ubuntu 22.04 with ROS2 Humble (and Ubuntu 24.04 with ROS2 Jazzy)
- Install [libfranka](https://github.com/frankaemika/libfranka)
- Follow the instructions in [medcvr_franka](./medcvr_franka) to set up the Franka controller.

You're all set up!

### Docker Containers

These steps can be avoided by setting up Docker containers:

- 3D Slicer [container](slicer.dockerfile)
- Franka Controller [container](controller.dockerfile)

## Developer Guide

An extensive developer guide can be found in the [developer guide](docs/slicerfranka.md)
