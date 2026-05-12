
import qt
import slicer

from .constants import (
    JOINT_LIMITS, CARTESIAN_POSITION_RANGES,
    CARTESIAN_ORIENTATION_RANGES, HOME_JOINT_POSITION, HOME_CARTESIAN_POSITION,
    HOME_CARTESIAN_ORIENTATION
)


class UIControlBuilder:
    def _default_log(self, message):
        print(message)

    def setup_joint_controls(self, widget_ui, joint_position_callback):
        joint_sliders = [
            widget_ui.joint1Slider, widget_ui.joint2Slider, widget_ui.joint3Slider,
            widget_ui.joint4Slider, widget_ui.joint5Slider, widget_ui.joint6Slider,
            widget_ui.joint7Slider
        ]

        joint_spinboxes = [
            widget_ui.joint1SpinBox, widget_ui.joint2SpinBox, widget_ui.joint3SpinBox,
            widget_ui.joint4SpinBox, widget_ui.joint5SpinBox, widget_ui.joint6SpinBox,
            widget_ui.joint7SpinBox
        ]

        for i in range(7):
            slider = joint_sliders[i]
            spinbox = joint_spinboxes[i]
            joint_min, joint_max = JOINT_LIMITS[i]

            min_milli = int(joint_min * 1000)
            max_milli = int(joint_max * 1000)

            slider.setMinimum(min_milli)
            slider.setMaximum(max_milli)
            slider.setSingleStep(1)

            spinbox.setMinimum(min_milli)
            spinbox.setMaximum(max_milli)
            spinbox.setSingleStep(10)

            slider.connect('valueChanged(int)', spinbox.setValue)
            spinbox.connect('valueChanged(int)', slider.setValue)

            spinbox.connect('valueChanged(int)', 
                          lambda value, idx=i: joint_position_callback(idx, value / 1000))

            home_pos_milli = int(HOME_JOINT_POSITION[i] * 1000)
            spinbox.setValue(home_pos_milli)
            slider.setValue(home_pos_milli)

        return joint_sliders, joint_spinboxes

    def setup_cartesian_controls(self, widget_ui, position_callback, orientation_callback):
        position_sliders = [widget_ui.xPosSlider, widget_ui.yPosSlider, widget_ui.zPosSlider]
        position_spinboxes = [widget_ui.xPosSpinBox, widget_ui.yPosSpinBox, widget_ui.zPosSpinBox]

        orientation_sliders = [widget_ui.rollSlider, widget_ui.pitchSlider, widget_ui.yawSlider]
        orientation_spinboxes = [widget_ui.rollSpinBox, widget_ui.pitchSpinBox, widget_ui.yawSpinBox]

        for i in range(3):
            slider = position_sliders[i]
            spinbox = position_spinboxes[i]
            min_value, max_value = CARTESIAN_POSITION_RANGES[i]

            slider.setMinimum(min_value)
            slider.setMaximum(max_value)
            spinbox.setMinimum(min_value)
            spinbox.setMaximum(max_value)

            slider.connect('valueChanged(int)', spinbox.setValue)
            spinbox.connect('valueChanged(int)', slider.setValue)

            spinbox.connect('valueChanged(int)', 
                          lambda value, idx=i: position_callback(idx, value))

            default_value = HOME_CARTESIAN_POSITION[i]
            spinbox.setValue(default_value)
            slider.setValue(default_value)

        for i in range(3):
            slider = orientation_sliders[i]
            spinbox = orientation_spinboxes[i]
            min_value, max_value = CARTESIAN_ORIENTATION_RANGES[i]

            min_milli = int(min_value * 1000)
            max_milli = int(max_value * 1000)

            slider.setMinimum(min_milli)
            slider.setMaximum(max_milli)
            spinbox.setMinimum(min_milli)
            spinbox.setMaximum(max_milli)

            slider.connect('valueChanged(int)', spinbox.setValue)
            spinbox.connect('valueChanged(int)', slider.setValue)

            spinbox.connect('valueChanged(int)', 
                          lambda value, idx=i: orientation_callback(idx, value / 1000))

            default_value = int(HOME_CARTESIAN_ORIENTATION[i] * 1000)
            spinbox.setValue(default_value)
            slider.setValue(default_value)

        position_controls = (position_sliders, position_spinboxes)
        orientation_controls = (orientation_sliders, orientation_spinboxes)

        return position_controls, orientation_controls

    def setup_current_joint_display(self, widget_ui):
        current_joint_labels = [
            widget_ui.currentJoint1Value, widget_ui.currentJoint2Value,
            widget_ui.currentJoint3Value, widget_ui.currentJoint4Value,
            widget_ui.currentJoint5Value, widget_ui.currentJoint6Value,
            widget_ui.currentJoint7Value
        ]

        for label in current_joint_labels:
            self._apply_monospace_font(label)

        return current_joint_labels

    def setup_current_cartesian_display(self, widget_ui):
        current_cartesian_labels = [
            widget_ui.currentXPos, widget_ui.currentYPos, widget_ui.currentZPos,
            widget_ui.currentRoll, widget_ui.currentPitch, widget_ui.currentYaw
        ]

        for label in current_cartesian_labels:
            self._apply_monospace_font(label)

        return current_cartesian_labels

    def _apply_monospace_font(self, label_widget):
        font = qt.QFont("Courier")
        font.setStyleHint(qt.QFont.Monospace)
        label_widget.setFont(font)


class UISync:
    @staticmethod
    def sync_joint_controls_to_positions(joint_spinboxes, joint_sliders, joint_positions):
        for i, (spinbox, slider) in enumerate(zip(joint_spinboxes, joint_sliders)):
            pos_milli = int(joint_positions[i] * 1000)
            spinbox.setValue(pos_milli)
            slider.setValue(pos_milli)

    @staticmethod
    def sync_cartesian_controls_to_pose(position_controls, orientation_controls, 
                                      cartesian_position, cartesian_orientation):
        position_sliders, position_spinboxes = position_controls
        orientation_sliders, orientation_spinboxes = orientation_controls

        for i in range(3):
            pos_value = int(cartesian_position[i])
            position_spinboxes[i].setValue(pos_value)
            position_sliders[i].setValue(pos_value)

        for i in range(3):
            ori_value = int(cartesian_orientation[i] * 1000)
            orientation_spinboxes[i].setValue(ori_value)
            orientation_sliders[i].setValue(ori_value)

    @staticmethod
    def update_joint_display(joint_labels, joint_positions):
        for i, label in enumerate(joint_labels):
            label.setText(f"{joint_positions[i]:.4f}")

    @staticmethod
    def update_cartesian_display(cartesian_labels, position, orientation):
        for i in range(3):
            cartesian_labels[i].setText(f"{position[i]:.4f}")

        for i in range(3):
            cartesian_labels[i + 3].setText(f"{orientation[i]:.4f}")


class UIConnections:
    @staticmethod
    def connect_button_signals(widget_ui, callback_methods):
        widget_ui.initializeButton.connect('clicked(bool)', callback_methods['initialize'])
        widget_ui.modeComboBox.connect('currentIndexChanged(int)', callback_methods['mode_changed'])
        widget_ui.setMaxVelocityButton.connect('clicked(bool)', callback_methods['set_max_velocity'])
        widget_ui.homePositionButton.connect('clicked(bool)', callback_methods['home_position'])
        widget_ui.cartesianHomeButton.connect('clicked(bool)', callback_methods['cartesian_home'])
        widget_ui.refreshMarkupsButton.connect('clicked(bool)', callback_methods['refresh_markups'])
        widget_ui.publishMarkupsButton.connect('clicked(bool)', callback_methods['publish_markups'])
        widget_ui.cancelTrajectoryButton.connect('clicked(bool)', callback_methods['cancel_trajectory'])
        widget_ui.visualizeOrientationsCheckBox.connect('toggled(bool)', 
                                                           callback_methods['visualize_orientations'])
        widget_ui.holdOrientationCheckBox.connect('toggled(bool)', 
                                                     callback_methods['hold_orientation'])
        widget_ui.addControlPointButtonPFrame.connect('clicked(bool)', 
                                                         callback_methods['add_control_point_p'])
        widget_ui.clearControlPointsButtonPFrame.connect('clicked(bool)', 
                                                            callback_methods['clear_control_points_p'])
        widget_ui.addControlPointButtonQFrame.connect('clicked(bool)', 
                                                         callback_methods['add_control_point_q'])
        widget_ui.clearControlPointsButtonQFrame.connect('clicked(bool)', 
                                                            callback_methods['clear_control_points_q'])
        widget_ui.controlTabWidget.connect('currentChanged(int)', callback_methods['tab_changed'])

    @staticmethod
    def populate_markup_selector(selector_widget, markup_names):
        selector_widget.clear()
        for name in markup_names:
            selector_widget.addItem(name)
