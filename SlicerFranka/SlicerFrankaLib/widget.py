
import logging
import slicer
import qt
from slicer.ScriptedLoadableModule import ScriptedLoadableModuleWidget

from .ui_helpers import UIControlBuilder, UISync, UIConnections
from .visualization import VisualizationManager, MarkupUtils
from .constants import MAX_JOINT_VELOCITY


class SlicerFrankaWidget(ScriptedLoadableModuleWidget):
    """
    Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None):
        """Called when the user opens the module the first time and the widget is initialized"""
        ScriptedLoadableModuleWidget.__init__(self, parent)

        self.ui = None
        self.logic = None

        self.joint_sliders = []
        self.joint_spinboxes = []
        self.position_controls = None
        self.orientation_controls = None
        self.current_joint_labels = []
        self.current_cartesian_labels = []

        self.ui_builder = UIControlBuilder()
        self.visualization_manager = VisualizationManager(self.log_message)

    def setup(self):
        """Called when the user opens the module the first time and the widget is initialized"""
        ScriptedLoadableModuleWidget.setup(self)

        ui_widget = slicer.util.loadUI(self.resourcePath('UI/SlicerFranka.ui'))
        self.layout.addWidget(ui_widget)
        self.ui = slicer.util.childWidgetVariables(ui_widget)

        from .logic import SlicerFrankaLogic
        self.logic = SlicerFrankaLogic()
        self.logic.set_widget(self)

        self._setup_initial_state()

        self._setup_ui_controls()
        self._setup_connections()

        self.log_message("SlicerFranka module widget initialized")

    def set_logic(self, logic_instance):
        self.logic = logic_instance
        self.logic.set_widget(self)

    def _setup_initial_state(self):
        self.ui.simRealFrame.enabled = False
        self.ui.velocityFrame.enabled = False
        self.ui.controlTabWidget.enabled = False

        self.ui.maxVelocitySpinBox.setValue(MAX_JOINT_VELOCITY)

    def _setup_ui_controls(self):
        self.joint_sliders, self.joint_spinboxes = self.ui_builder.setup_joint_controls(
            self.ui, self._on_joint_position_changed
        )

        self.position_controls, self.orientation_controls = self.ui_builder.setup_cartesian_controls(
            self.ui, self._on_cartesian_position_changed, self._on_cartesian_orientation_changed
        )

        self.current_joint_labels = self.ui_builder.setup_current_joint_display(self.ui)
        self.current_cartesian_labels = self.ui_builder.setup_current_cartesian_display(self.ui)

    def _setup_connections(self):
        callback_methods = {
            'initialize': self._on_initialize_button_clicked,
            'mode_changed': self._on_mode_changed,
            'set_max_velocity': self._on_set_max_velocity_clicked,
            'home_position': self._on_home_button_clicked,
            'cartesian_home': self._on_cartesian_home_button_clicked,
            'refresh_markups': self._on_refresh_markups_clicked,
            'publish_markups': self._on_publish_markups_clicked,
            'cancel_trajectory': self._on_cancel_trajectory_clicked,
            'visualize_orientations': self._on_visualize_orientations_toggled,
            'hold_orientation': self._on_hold_orientation_toggled,
            'add_control_point_p': lambda: self._on_add_control_point_clicked("p_frame"),
            'clear_control_points_p': lambda: self._on_clear_control_points_clicked("p_frame"),
            'add_control_point_q': lambda: self._on_add_control_point_clicked("q_frame"),
            'clear_control_points_q': lambda: self._on_clear_control_points_clicked("q_frame"),
            'tab_changed': self._on_tab_changed
        }

        UIConnections.connect_button_signals(self.ui, callback_methods)

    # ==========================================================================
    # UI Event Handlers
    # ==========================================================================

    def _on_initialize_button_clicked(self):
        self.log_message("Initializing robot connection...")

        self.ui.statusLabel.text = "Status: Connecting..."
        self.ui.statusLabel.setStyleSheet("color: orange;")
        self.ui.initializeButton.enabled = False
        slicer.app.processEvents()  # Process events to update UI immediately

        success = self.logic.initialize_robot_connection()

        if success:
            self.ui.statusLabel.text = "Status: Connected"
            self.ui.statusLabel.setStyleSheet("color: green;")
            self.ui.simRealFrame.enabled = True
            self.ui.velocityFrame.enabled = True
            self.ui.controlTabWidget.enabled = True

            self._on_refresh_markups_clicked()
            self.log_message("Robot connection initialized successfully")
        else:
            self.ui.statusLabel.text = "Status: Connection Failed"
            self.ui.statusLabel.setStyleSheet("color: red;")
            self.ui.initializeButton.enabled = True  # Re-enable button on failure
            self.log_message("Failed to initialize robot connection", logging.ERROR)

    def _on_mode_changed(self, index):
        mode_name = "Simulation Only" if index == 0 else "Real Robot"
        self.log_message(f"Switching to {mode_name} mode")

        self.logic.set_mode(index)

        workspace_node = slicer.util.getNode("PandaWorkspace")
        if workspace_node:
            display_node = workspace_node.GetDisplayNode()
            display_node.SetVisibility(index == 1)

        if index == 1: # real robot
            self.log_message("Synchronizing UI with current robot position...")
            self._sync_ui_to_current_position()
            self._sync_ui_to_current_cartesian_position()

    def _on_set_max_velocity_clicked(self):
        new_velocity = self.ui.maxVelocitySpinBox.value
        self.log_message(f"Setting maximum joint velocity to {new_velocity:.3f} rad/s")
        self.logic.set_max_joint_velocity(new_velocity)

    def _on_home_button_clicked(self):
        home_position = self.logic.get_home_joint_position()
        UISync.sync_joint_controls_to_positions(
            self.joint_spinboxes, self.joint_sliders, home_position
        )
        self.log_message("Home position values set - robot moving to home position")

    def _on_cartesian_home_button_clicked(self):
        home_position, home_orientation = self.logic.get_home_cartesian_pose()
        UISync.sync_cartesian_controls_to_pose(
            self.position_controls, self.orientation_controls,
            home_position, home_orientation
        )
        self.log_message("Home position values set - robot moving to home position")

    def _on_refresh_markups_clicked(self):
        available_markups = MarkupUtils.get_available_markups()

        UIConnections.populate_markup_selector(self.ui.markupsSelector, available_markups)
        UIConnections.populate_markup_selector(self.ui.orientationMarkupsSelector, available_markups)

        fiducial_nodes = slicer.util.getNodesByClass('vtkMRMLMarkupsFiducialNode')
        curve_nodes = slicer.util.getNodesByClass('vtkMRMLMarkupsCurveNode')
        self.log_message(f"Found {len(fiducial_nodes)} fiducial nodes and {len(curve_nodes)} curve nodes")

    def _on_visualize_orientations_toggled(self, checked):
        if checked:
            pos_markup_name = self.ui.markupsSelector.currentText
            ori_markup_name = self.ui.orientationMarkupsSelector.currentText

            if not pos_markup_name or not ori_markup_name:
                self.log_message("Both position and orientation markups must be selected for visualization", logging.WARNING)
                self.ui.visualizeOrientationsCheckBox.setChecked(False)
                return

            self.visualization_manager.create_orientation_vectors(pos_markup_name, ori_markup_name)
        else:
            self.visualization_manager.hide_orientation_vectors()

        status = "enabled" if checked else "disabled"
        self.log_message(f"Orientation visualization {status}")

    def _on_publish_markups_clicked(self):
        pos_markup_name = self.ui.markupsSelector.currentText
        ori_markup_name = self.ui.orientationMarkupsSelector.currentText

        if not pos_markup_name:
            self.log_message("No position markup selected", logging.WARNING)
            return

        if not ori_markup_name:
            self.log_message("No orientation markup selected", logging.WARNING)
            return

        pos_node = slicer.util.getNode(pos_markup_name)
        ori_node = slicer.util.getNode(ori_markup_name)

        if not pos_node or not ori_node:
            self.log_message("Could not find markup nodes", logging.ERROR)
            return

        pos_waypoints = self.visualization_manager.extract_points_from_markup(pos_node)
        ori_waypoints = self.visualization_manager.extract_points_from_markup(ori_node, len(pos_waypoints))

        success = self.logic.start_trajectory(pos_waypoints, ori_waypoints)
        if success:
            self.log_message(f"Starting trajectory with {len(pos_waypoints)} waypoints and orientations")
        else:
            self.log_message("Failed to start trajectory", logging.ERROR)

    def _on_cancel_trajectory_clicked(self):
        self.logic.cancel_trajectory()
        self.log_message("Trajectory cancel requested")

    def _on_hold_orientation_toggled(self, checked):
        self.logic.set_hold_current_orientation(checked)
        status = "enabled" if checked else "disabled"
        self.log_message(f"Orientation hold during trajectory {status}")

    def _on_tab_changed(self, index):
        tab_name = self.ui.controlTabWidget.tabText(index)
        self.log_message(f"Tab changed to: {tab_name}")

        self.logic.sync_to_current_state()
        self._sync_ui_to_current_position()
        self._sync_ui_to_current_cartesian_position()

        control_modes = ["joint", "cartesian", "trajectory", "compliant"]
        self.logic.set_control_mode(control_modes[index])

        if index == 3:  # Registration tab
            self._update_control_points_display("p_frame")
            self._update_control_points_display("q_frame")

    def _on_joint_position_changed(self, joint_index, value):
        self.logic.set_target_joint_position(joint_index, value)

    def _on_cartesian_position_changed(self, axis_index, value):
        self.logic.set_target_cartesian_position(axis_index, value)

    def _on_cartesian_orientation_changed(self, axis_index, value):
        self.logic.set_target_cartesian_orientation(axis_index, value)

    def _on_add_control_point_clicked(self, frame_name):
        current_position = self.logic.get_actual_cartesian_position()

        point_index = MarkupUtils.add_control_point_to_markup(frame_name, current_position)
        if point_index >= 0:
            point_name = f"{frame_name[0].upper()}{point_index}"
            base_frame_node = slicer.util.getNode(frame_name)
            base_frame_node.SetNthControlPointLabel(point_index, point_name)

            self._update_control_points_display(frame_name)
            self.log_message(f"Added control point {point_name} at position: ({current_position[0]:.2f}, {current_position[1]:.2f}, {current_position[2]:.2f})")
        else:
            self.log_message(f"Failed to add control point to {frame_name}", logging.ERROR)

    def _on_clear_control_points_clicked(self, frame_name):
        MarkupUtils.clear_markup_points(frame_name)
        self._update_control_points_display(frame_name)
        self.log_message(f"Cleared all control points from {frame_name}")

    # ==========================================================================
    # UI Update Methods
    # ==========================================================================

    def _sync_ui_to_current_position(self):
        current_positions = self.logic.get_actual_joint_positions()
        UISync.sync_joint_controls_to_positions(
            self.joint_spinboxes, self.joint_sliders, current_positions
        )
        self.log_message("UI synchronized with current robot joint position")

    def _sync_ui_to_current_cartesian_position(self):
        current_position = self.logic.get_actual_cartesian_position()
        current_orientation = self.logic.get_actual_cartesian_orientation()

        UISync.sync_cartesian_controls_to_pose(
            self.position_controls, self.orientation_controls,
            current_position, current_orientation
        )

        self.log_message("UI synchronized with current robot cartesian position")

    def update_current_joint_display(self, joint_positions):
        UISync.update_joint_display(self.current_joint_labels, joint_positions)

    def update_cartesian_display(self, position, orientation):
        UISync.update_cartesian_display(self.current_cartesian_labels, position, orientation)

    def update_robot_status_indicator(self):
        is_moving = self.logic.is_robot_moving()

        if is_moving:
            self.ui.statusLabel.setText("Status: Robot Moving")
            self.ui.statusLabel.setStyleSheet("color: orange;")

            current_tab = self.ui.controlTabWidget.currentIndex
            for i in range(self.ui.controlTabWidget.count):
                if i != current_tab:
                    self.ui.controlTabWidget.setTabEnabled(i, False)

            self.ui.setMaxVelocityButton.enabled = False  # Disable changing max velocity
        else:
            self.ui.statusLabel.setText("Status: Connected")
            self.ui.statusLabel.setStyleSheet("color: green;")

            for i in range(self.ui.controlTabWidget.count):
                self.ui.controlTabWidget.setTabEnabled(i, True)
            self.ui.setMaxVelocityButton.enabled = True  # Re-enable changing max velocity

    def _update_control_points_display(self, frame_name):
        if frame_name == "p_frame":
            text_widget = self.ui.pFrameControlPointsListText
        else:
            text_widget = self.ui.qFrameControlPointsListText

        text_widget.clear()

        points_info = MarkupUtils.get_markup_points_info(frame_name)
        if not points_info:
            text_widget.append(f"No {frame_name[0].upper()} points added yet.")
            return

        for point_info in points_info:
            pos = point_info['position']
            label = point_info['label']
            point_text = f"{label}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
            text_widget.append(point_text)

    def log_message(self, message, level=logging.INFO):
        if level == logging.ERROR:
            logging.error(message)
            self.ui.logTextEdit.append(f"ERROR: {message}")
        elif level == logging.WARNING:
            logging.warning(message)
            self.ui.logTextEdit.append(f"WARNING: {message}")
        else:
            logging.info(message)
            self.ui.logTextEdit.append(message)

    def cleanup(self):
        if self.logic:
            self.logic.cleanup()
        self.log_message("Widget cleanup complete")
