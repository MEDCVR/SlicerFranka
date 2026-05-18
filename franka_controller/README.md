# franka_controller

Minimal ROS 2 controller for the Franka Emika Panda. Implements the topic contract documented in the [SlicerFranka guide](../docs/slicerfranka.md) so it can be driven directly by the [SlicerFranka module](..).

For installation context and the bigger picture, start with the [project README](../README.md).

## Dependencies

This package must run on a PC with:

- **PREEMPT_RT-patched Linux kernel** — required by `libfranka`.
- **`libfranka`** — tested with 0.9.2 against a Panda. Install instructions [here](https://github.com/frankarobotics/libfranka).
- **`toppra`** (C++) — used for joint-space trajectory time-parameterization. Install from [source](https://github.com/hungpham2511/toppra).

Plus the standard ROS 2 setup (`rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`), Eigen3 (`libeigen3-dev`), and yaml-cpp (`libyaml-cpp-dev`).

Tested on Ubuntu 22.04 / ROS 2 Humble and Ubuntu 24.04 / ROS 2 Jazzy.

## Build

### 1. Create the workspace

`franka_controller` lives inside the `slicerfranka` git repository as a subdirectory. You must clone the whole repository into the `src/` folder of your ROS 2 workspace:

```bash
mkdir -p ~/franka_ws/src
cd ~/franka_ws/src
git clone https://github.com/MEDCVR/SlicerFranka.git slicerfranka
```

### 2. Build the package

Run all build commands from the **workspace root** (`~/franka_ws`):

```bash
cd ~/franka_ws
source /opt/ros/<your_distro>/setup.bash
colcon build --packages-select franka_controller
source install/setup.bash
```

## Run

1. Edit `config/controller.yaml` and set `ip_address` to your robot's IP.
2. Power on the arm, unlock the joints in Franka Desk, and enable FCI.
3. Launch:
   ```bash
   ros2 launch franka_controller controller.launch.py
   ```
   Or with a custom config:
   ```bash
   ros2 launch franka_controller controller.launch.py \
     config_path:=/absolute/path/to/your.yaml
   ```

Once running, the controller exposes the topics in the [SlicerFranka guide](../docs/slicerfranka.md#ros-2-topics) and Slicer can connect immediately.
