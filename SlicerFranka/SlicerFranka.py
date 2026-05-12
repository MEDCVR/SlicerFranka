from slicer.ScriptedLoadableModule import *
from SlicerFrankaLib.logic import SlicerFrankaLogic
from SlicerFrankaLib.widget import SlicerFrankaWidget


class SlicerFranka(ScriptedLoadableModule):
    """
    Main module class for SlicerFranka

    Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)

        self.parent.title = "SlicerFranka"
        self.parent.categories = ["IGT"]
        self.parent.dependencies = ["ROS2"]
        self.parent.contributors = ["Iseoluwa (MedCVR)"]
        self.parent.helpText = """
        A module for bidirectional control and monitoring of Franka robots through 3D Slicer.

        Features:
        - Real-time robot visualization and state feedback
        - Joint space and cartesian space motion control
        - Trajectory planning and execution
        - Registration workflows

        This module requires SlicerROS2 to be installed.
        """
        self.parent.acknowledgementText = """
        This module integrates 3D Slicer with Franka Robots via ROS2.
        Developed for medical robotics research and applications.
        """


__all__ = ["SlicerFranka", "SlicerFrankaWidget", "SlicerFrankaLogic"]
