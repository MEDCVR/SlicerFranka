import math

import numpy as np
from scipy.spatial.transform import Rotation


class TrajectoryPlanner:
    def __init__(self, logger_callback=None):
        self.log_callback = logger_callback or self._default_log

        self.hold_current_orientation = False
        self.preserve_roll = True

    def _default_log(self, message):
        print(message)

    def set_hold_current_orientation(self, hold):
        self.hold_current_orientation = hold
        status = "enabled" if hold else "disabled"
        self.log_callback(f"Orientation hold during trajectory {status}")

    def set_preserve_roll(self, preserve):
        self.preserve_roll = preserve
        status = "enabled" if preserve else "disabled"
        self.log_callback(f"Roll preservation during trajectory {status}")

    def build_tip_waypoints(
        self, position_waypoints, orientation_waypoints, current_orientation_euler
    ):
        """
        Builds a list of (position, rotation_matrix_3x3) waypoints for robot-side execution.

        - `position_waypoints`: list of [x,y,z] in Slicer units (SlicerROS2 handles conversion).
        - `orientation_waypoints`: list of [x,y,z] points used to define tip +Z direction at each waypoint.
        - If `hold_current_orientation` is enabled, all waypoints use `current_orientation_euler`.
        """
        if not position_waypoints:
            return []

        if self.hold_current_orientation:
            rot = Rotation.from_euler("xyz", current_orientation_euler, degrees=False)
            rot_mat = rot.as_matrix().tolist()
            return [(pos, rot_mat) for pos in position_waypoints]

        reference_rot = None
        if self.preserve_roll:
            reference_rot = Rotation.from_euler(
                "xyz", current_orientation_euler, degrees=False
            )

        if not orientation_waypoints or len(orientation_waypoints) < len(
            position_waypoints
        ):
            self.log_callback("Orientation waypoint list is missing or too short")
            return []

        waypoints = []
        for i in range(len(position_waypoints)):
            pos = position_waypoints[i]
            ori_pt = orientation_waypoints[i]
            direction = [pos[0] - ori_pt[0], pos[1] - ori_pt[1], pos[2] - ori_pt[2]]

            length = math.sqrt(sum([d**2 for d in direction]))
            if length < 0.001:
                self.log_callback("Direction length too small")
                return []

            direction = [d / length for d in direction]
            quat_xyzw = self._direction_to_orientation(
                direction, reference_rotation=reference_rot
            )
            rot_mat = Rotation.from_quat(quat_xyzw).as_matrix().tolist()
            waypoints.append((pos, rot_mat))
            if self.preserve_roll:
                reference_rot = Rotation.from_quat(quat_xyzw)

        return waypoints

    def _direction_to_orientation(self, direction, reference_rotation=None):
        z_new = np.asarray(direction, float) / np.linalg.norm(direction)

        if reference_rotation is not None:
            ref_mat = reference_rotation.as_matrix()
            x_ref = ref_mat[:, 0]
            y_ref = ref_mat[:, 1]

            x_new = x_ref - np.dot(x_ref, z_new) * z_new
            if np.linalg.norm(x_new) < 1e-8:
                x_new = y_ref - np.dot(y_ref, z_new) * z_new

            if np.linalg.norm(x_new) < 1e-8:
                up = np.array([0, 0, 1.0])
                if abs(np.dot(z_new, up)) > 0.999:
                    up = np.array([0, 1.0, 0])
                x_new = np.cross(up, z_new)

            x_new /= np.linalg.norm(x_new)
            y_new = np.cross(z_new, x_new)
            rot = Rotation.from_matrix(np.column_stack((x_new, y_new, z_new)))
            return rot.as_quat()

        up = np.array([0, 0, 1.0])
        if abs(np.dot(z_new, up)) > 0.999:
            up = np.array([0, 1.0, 0])

        x_new = np.cross(up, z_new)
        x_new /= np.linalg.norm(x_new)
        y_new = np.cross(z_new, x_new)

        rot = Rotation.from_matrix(np.column_stack((x_new, y_new, z_new)))
        return rot.as_quat()


class GeometryUtils:
    @staticmethod
    def euler_to_matrix(roll, pitch, yaw):
        # Roll - rotation around X axis
        c_roll = math.cos(roll)
        s_roll = math.sin(roll)
        R_x = [[1, 0, 0], [0, c_roll, -s_roll], [0, s_roll, c_roll]]

        # Pitch - rotation around Y axis
        c_pitch = math.cos(pitch)
        s_pitch = math.sin(pitch)
        R_y = [[c_pitch, 0, s_pitch], [0, 1, 0], [-s_pitch, 0, c_pitch]]

        # Yaw - rotation around Z axis
        c_yaw = math.cos(yaw)
        s_yaw = math.sin(yaw)
        R_z = [[c_yaw, -s_yaw, 0], [s_yaw, c_yaw, 0], [0, 0, 1]]

        # Combine rotations: R = R_z * R_y * R_x
        R_zy = GeometryUtils.multiply_matrices_3x3(R_z, R_y)
        rotation_matrix = GeometryUtils.multiply_matrices_3x3(R_zy, R_x)

        return rotation_matrix

    @staticmethod
    def multiply_matrices_3x3(matrix_a, matrix_b):
        result = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        for i in range(3):  # Iterate over rows of A
            for j in range(3):  # Iterate over columns of B
                for k in range(3):  # Iterate over rows of B (or columns of A)
                    result[i][j] += matrix_a[i][k] * matrix_b[k][j]

        return result

    @staticmethod
    def calculate_orientation_difference(current_orientation, target_orientation):
        orientation_diff = [0, 0, 0]

        for i in range(3):
            current = current_orientation[i]
            target = target_orientation[i]
            diff = target - current

            # Wrap angle difference to [-π, π]
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            orientation_diff[i] = diff

        return orientation_diff
