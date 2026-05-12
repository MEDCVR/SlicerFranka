import logging
import math

import qt
import slicer
import vtk
from slicer.ScriptedLoadableModule import ScriptedLoadableModuleLogic

from .constants import (
    CARTESIAN_ORIENTATION_THRESHOLD,
    CARTESIAN_POSITION_THRESHOLD,
    HOME_CARTESIAN_ORIENTATION,
    HOME_CARTESIAN_POSITION,
    HOME_JOINT_POSITION,
    JOINT_MOVEMENT_THRESHOLD,
    MAX_ALLOWED_JOINT_VELOCITY,
    MAX_ANGULAR_VELOCITY,
    MAX_CARTESIAN_VELOCITY,
    MAX_JOINT_VELOCITY,
    TIMER_INTERVAL_MS,
    UPDATE_RATE_HZ,
)
from .ros_interface import ROS2Interface
from .trajectory import GeometryUtils, TrajectoryPlanner
from .visualization import VisualizationManager


class SlicerFrankaLogic(ScriptedLoadableModuleLogic):
    """This class implements all the actual computation done by the module"""

    def __init__(self):
        ScriptedLoadableModuleLogic.__init__(self)

        self.initialized = False
        self.mode = 0  # simulation
        self.control_mode = "joint"
        self.widget = None

        self.current_joint_positions = list(HOME_JOINT_POSITION)
        self.actual_joint_positions = list(HOME_JOINT_POSITION)
        self.target_joint_positions = list(HOME_JOINT_POSITION)

        self.current_cartesian_position = list(HOME_CARTESIAN_POSITION)
        self.current_cartesian_orientation = list(HOME_CARTESIAN_ORIENTATION)
        self.actual_cartesian_position = list(HOME_CARTESIAN_POSITION)
        self.actual_cartesian_orientation = list(HOME_CARTESIAN_ORIENTATION)
        self.target_cartesian_position = list(HOME_CARTESIAN_POSITION)
        self.target_cartesian_orientation = list(HOME_CARTESIAN_ORIENTATION)

        self.max_joint_velocity = MAX_JOINT_VELOCITY
        self.max_cartesian_velocity = MAX_CARTESIAN_VELOCITY
        self.max_angular_velocity = MAX_ANGULAR_VELOCITY

        self.joint_update_timer = None
        self.cartesian_update_timer = None

        self.ros2_interface = ROS2Interface(self._log_message)
        self.trajectory_planner = TrajectoryPlanner(self._log_message)
        self.visualization_manager = VisualizationManager(self._log_message)
        self.trajectory_status = "idle"

    def set_widget(self, widget_instance):
        """Set the widget instance for UI updates"""
        self.widget = widget_instance

    def _log_message(self, message, level=logging.INFO):
        if level == logging.ERROR:
            logging.error(message)
        elif level == logging.WARNING:
            logging.warning(message)
        else:
            logging.info(message)

        if self.widget:
            self.widget.log_message(message, level)

    # ==========================================================================
    # Initialization and Setup
    # ==========================================================================

    def initialize_robot_connection(self):
        try:
            ros2_logic = slicer.util.getModuleLogic("ROS2")

            self.ros2_interface.initialize_node(ros2_logic)

            self.ros2_interface.setup_publishers()
            self.ros2_interface.setup_subscribers(
                self._joint_state_callback,
                self._tip_pose_callback,
                self._trajectory_status_callback,
            )

            self.visualization_manager.setup_end_effector_markup()
            self.visualization_manager.create_registration_markup_nodes()

            self.visualization_manager.create_workspace_cube(visible=False)

            self.initialized = True

            self._publish_joint_state_for_simulation(HOME_JOINT_POSITION)
            self._log_message("Robot visualization set to home position")

            self._start_joint_update_timer()
            self._start_cartesian_update_timer()

            self._log_message("Robot connection initialization complete")
            return True

        except Exception as e:
            self._log_message(
                f"Failed to initialize robot connection: {str(e)}",
                logging.ERROR,
            )
            return False

    def cleanup(self):
        self._stop_joint_update_timer()
        self._stop_cartesian_update_timer()

        if self.ros2_interface:
            self.ros2_interface.cleanup()

        self._log_message("Logic cleanup complete")

    # ==========================================================================
    # Mode and Control Management
    # ==========================================================================

    def set_mode(self, mode):
        self.mode = mode
        mode_name = "Simulation" if mode == 0 else "Real Robot"
        self._log_message(f"Control mode set to: {mode_name}")

    def set_control_mode(self, mode_type):
        self.control_mode = mode_type

        self.ros2_interface.publish_control_mode(mode_type)
        self._log_message(
            f"Control mode set to: {mode_type}, published as: {mode_type}"
        )

    def set_max_joint_velocity(self, velocity):
        velocity = min(MAX_ALLOWED_JOINT_VELOCITY, max(0.001, velocity))
        self.max_joint_velocity = velocity
        self._log_message(f"Maximum joint velocity set to: {velocity:.3f} rad/s")

        self._stop_joint_update_timer()
        self._start_joint_update_timer()
        self._log_message("Restarted joint update timer with new velocity")

    # ==========================================================================
    # Position and State Access
    # ==========================================================================

    def get_home_joint_position(self):
        return list(HOME_JOINT_POSITION)

    def get_home_cartesian_pose(self):
        return list(HOME_CARTESIAN_POSITION), list(HOME_CARTESIAN_ORIENTATION)

    def get_actual_joint_positions(self):
        return list(self.actual_joint_positions)

    def get_actual_cartesian_position(self):
        return list(self.actual_cartesian_position)

    def get_actual_cartesian_orientation(self):
        return list(self.actual_cartesian_orientation)

    def set_target_joint_position(self, joint_index, value):
        if 0 <= joint_index < 7:
            self.target_joint_positions[joint_index] = value
            if self.mode == 1 and self.control_mode == "joint":
                self._send_joint_angles(self.target_joint_positions)

    def set_target_cartesian_position(self, axis_index, value):
        if 0 <= axis_index < 3:
            self.target_cartesian_position[axis_index] = value
            if self.mode == 1 and self.control_mode == "cartesian":
                self._send_tip_pose(
                    self.target_cartesian_position, self.target_cartesian_orientation
                )

    def set_target_cartesian_orientation(self, axis_index, value):
        if 0 <= axis_index < 3:
            self.target_cartesian_orientation[axis_index] = value
            if self.mode == 1 and self.control_mode == "cartesian":
                self._send_tip_pose(
                    self.target_cartesian_position, self.target_cartesian_orientation
                )

    def sync_to_current_state(self):
        self.current_joint_positions = list(self.actual_joint_positions)
        self.target_joint_positions = list(self.actual_joint_positions)

        self.current_cartesian_position = list(self.actual_cartesian_position)
        self.target_cartesian_position = list(self.actual_cartesian_position)
        self.current_cartesian_orientation = list(self.actual_cartesian_orientation)
        self.target_cartesian_orientation = list(self.actual_cartesian_orientation)

    # ==========================================================================
    # Motion Control
    # ==========================================================================

    def is_robot_moving(self):
        if self.mode == 0:
            if self.control_mode == "joint":
                return self._is_joint_moving()
            if self.control_mode == "cartesian":
                return self._is_cartesian_moving()
            return False

        if self.control_mode == "joint":
            for i in range(7):
                if (
                    abs(self.target_joint_positions[i] - self.actual_joint_positions[i])
                    > JOINT_MOVEMENT_THRESHOLD
                ):
                    return True
            return False

        if self.control_mode == "cartesian":
            pos_diff = [
                self.target_cartesian_position[i] - self.actual_cartesian_position[i]
                for i in range(3)
            ]
            ori_diff = GeometryUtils.calculate_orientation_difference(
                self.actual_cartesian_orientation, self.target_cartesian_orientation
            )
            pos_distance = math.sqrt(sum([diff**2 for diff in pos_diff]))
            ori_distance = math.sqrt(sum([diff**2 for diff in ori_diff]))
            return (
                pos_distance > CARTESIAN_POSITION_THRESHOLD
                or ori_distance > CARTESIAN_ORIENTATION_THRESHOLD
            )

        if self.control_mode == "trajectory":
            return self.trajectory_status == "executing"

        return False

    def _is_joint_moving(self):
        if self.control_mode != "joint":
            return False

        for i in range(7):
            diff = abs(self.target_joint_positions[i] - self.current_joint_positions[i])
            if diff > JOINT_MOVEMENT_THRESHOLD:
                return True

        return False

    def _is_cartesian_moving(self):
        if self.control_mode != "cartesian":
            return False

        pos_diff = [
            self.target_cartesian_position[i] - self.current_cartesian_position[i]
            for i in range(3)
        ]

        ori_diff = GeometryUtils.calculate_orientation_difference(
            self.current_cartesian_orientation, self.target_cartesian_orientation
        )
        pos_distance = math.sqrt(sum([diff**2 for diff in pos_diff]))
        ori_distance = math.sqrt(sum([diff**2 for diff in ori_diff]))

        return (
            pos_distance > CARTESIAN_POSITION_THRESHOLD
            or ori_distance > CARTESIAN_ORIENTATION_THRESHOLD
        )

    # ==========================================================================
    # Trajectory Control
    # ==========================================================================

    def start_trajectory(self, position_waypoints, orientation_waypoints):
        if self.mode != 1:
            self._log_message(
                "Trajectory execution only available in real robot mode",
                logging.WARNING,
            )
            return False

        waypoints = self.trajectory_planner.build_tip_waypoints(
            position_waypoints,
            orientation_waypoints,
            self.actual_cartesian_orientation,
        )
        if not waypoints:
            return False

        self.ros2_interface.publish_trajectory_command("clear")

        for waypoint_index, (position, rotation_matrix) in enumerate(waypoints):
            vtk_matrix = vtk.vtkMatrix4x4()
            vtk_matrix.Identity()
            for i in range(3):
                for j in range(3):
                    vtk_matrix.SetElement(i, j, rotation_matrix[i][j])

            self._log_message(
                f"Trajectory waypoint {waypoint_index}: "
                f"pos=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})"
            )
            self.ros2_interface.publish_trajectory_waypoint(position, vtk_matrix)

        self.ros2_interface.publish_trajectory_command("start")
        self.trajectory_status = "executing"
        return True

    def cancel_trajectory(self):
        if self.mode != 1:
            return
        self.ros2_interface.publish_trajectory_command("cancel")

    def set_hold_current_orientation(self, hold):
        self.trajectory_planner.set_hold_current_orientation(hold)

    # ==========================================================================
    # Timer-based Motion Updates
    # ==========================================================================

    def _start_joint_update_timer(self):
        self.joint_update_timer = qt.QTimer()
        self.joint_update_timer.setInterval(TIMER_INTERVAL_MS)
        self.joint_update_timer.timeout.connect(self._update_joint_positions)
        self.joint_update_timer.start()
        self._log_message(f"Started joint update timer at {UPDATE_RATE_HZ}Hz")

    def _stop_joint_update_timer(self):
        if self.joint_update_timer:
            self.joint_update_timer.stop()
            self.joint_update_timer = None
            self._log_message("Stopped joint update timer")

    def _start_cartesian_update_timer(self):
        self.cartesian_update_timer = qt.QTimer()
        self.cartesian_update_timer.setInterval(TIMER_INTERVAL_MS)
        self.cartesian_update_timer.timeout.connect(self._update_cartesian_pose)
        self.cartesian_update_timer.start()
        self._log_message(f"Started cartesian update timer at {UPDATE_RATE_HZ}Hz")

    def _stop_cartesian_update_timer(self):
        if self.cartesian_update_timer:
            self.cartesian_update_timer.stop()
            self.cartesian_update_timer = None
            self._log_message("Stopped cartesian update timer")

    def _update_joint_positions(self):
        if self.mode != 0:
            return

        max_delta = self.max_joint_velocity / UPDATE_RATE_HZ

        new_positions = list(self.current_joint_positions)
        need_update = False

        for i in range(7):
            diff = self.target_joint_positions[i] - self.current_joint_positions[i]
            if abs(diff) > JOINT_MOVEMENT_THRESHOLD:
                move_amount = max(-max_delta, min(max_delta, diff))
                new_positions[i] += move_amount
                need_update = True

        if need_update:
            self.current_joint_positions = new_positions
            self._send_joint_angles(new_positions)

        if self.widget:
            self.widget.update_robot_status_indicator()

    def _update_cartesian_pose(self):
        if self.mode != 0:
            return

        max_pos_delta = self.max_cartesian_velocity / UPDATE_RATE_HZ
        max_ang_delta = self.max_angular_velocity / UPDATE_RATE_HZ

        pos_diff = [
            self.target_cartesian_position[i] - self.current_cartesian_position[i]
            for i in range(3)
        ]

        ori_diff = GeometryUtils.calculate_orientation_difference(
            self.current_cartesian_orientation, self.target_cartesian_orientation
        )

        pos_distance = math.sqrt(sum([diff**2 for diff in pos_diff]))
        ori_distance = math.sqrt(sum([diff**2 for diff in ori_diff]))

        need_update = (
            pos_distance > CARTESIAN_POSITION_THRESHOLD
            or ori_distance > CARTESIAN_ORIENTATION_THRESHOLD
        )

        if need_update:
            new_position = list(self.current_cartesian_position)
            new_orientation = list(self.current_cartesian_orientation)

            if pos_distance > 0.0:
                pos_scale = min(1.0, max_pos_delta / pos_distance)
                for i in range(3):
                    new_position[i] += pos_diff[i] * pos_scale

            # Update orientation with velocity limiting
            if ori_distance > 0.0:
                ori_scale = min(1.0, max_ang_delta / ori_distance)
                for i in range(3):
                    new_orientation[i] += ori_diff[i] * ori_scale

            self.current_cartesian_position = new_position
            self.current_cartesian_orientation = new_orientation
            self._send_tip_pose(new_position, new_orientation)

        if self.widget:
            self.widget.update_robot_status_indicator()

    # ==========================================================================
    # ROS2 Communication
    # ==========================================================================

    def _send_joint_angles(self, joint_angles):
        if self.mode == 0:  # simulation
            self._publish_joint_state_for_simulation(joint_angles)
        else:  # real robot
            self._publish_joint_state_for_robot(joint_angles)

    def _publish_joint_state_for_simulation(self, joint_angles):
        self.ros2_interface.publish_joint_state(
            self.ros2_interface.sim_joint_publisher, joint_angles
        )

    def _publish_joint_state_for_robot(self, joint_angles):
        self.ros2_interface.publish_joint_state(
            self.ros2_interface.cmd_joint_publisher, joint_angles
        )

    def _send_cartesian_pose(self, position, orientation):
        self._send_tip_pose(position, orientation)

    def _send_tip_pose(self, position, orientation):
        rotation_matrix = GeometryUtils.euler_to_matrix(*orientation)

        vtk_matrix = vtk.vtkMatrix4x4()
        vtk_matrix.Identity()
        for i in range(3):
            for j in range(3):
                vtk_matrix.SetElement(i, j, rotation_matrix[i][j])

        self.ros2_interface.publish_tip_pose(position, vtk_matrix)

    # ==========================================================================
    # ROS2 Callback Handlers
    # ==========================================================================

    def _joint_state_callback(self, caller, event):
        message = caller.GetLastMessage()
        positions = list(message.GetPosition())
        self.actual_joint_positions = positions

        if self.mode == 0:
            return

        self._publish_joint_state_for_simulation(positions)

        if self.widget:
            slicer.app.processEvents()  # Process events to update UI in real-time
            self.widget.update_current_joint_display(positions)

    def _tip_pose_callback(self, caller, event):
        message = caller.GetLastMessage()
        pose_matrix = message.GetPose()

        position = [
            pose_matrix.GetElement(0, 3),
            pose_matrix.GetElement(1, 3),
            pose_matrix.GetElement(2, 3),
        ]

        rotation_matrix = [[pose_matrix.GetElement(i, j) for j in range(3)] for i in range(3)]

        pitch = math.asin(-rotation_matrix[2][0])

        if abs(rotation_matrix[2][0]) < 0.99999:  # Not at singularity
            roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
            yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
        else:  # At singularity
            roll = 0
            yaw = math.atan2(-rotation_matrix[0][1], rotation_matrix[1][1])

        if roll < 0:
            roll += 2 * math.pi
        if yaw < 0:
            yaw += 2 * math.pi

        orientation = [roll, pitch, yaw]

        self.actual_cartesian_position = position
        self.actual_cartesian_orientation = orientation

        if self.mode == 0:
            return

        self.visualization_manager.update_end_effector_position(position, visible=True)

        if self.widget:
            self.widget.update_cartesian_display(position, orientation)

    def _trajectory_status_callback(self, caller, event):
        self.trajectory_status = caller.GetLastMessage()
