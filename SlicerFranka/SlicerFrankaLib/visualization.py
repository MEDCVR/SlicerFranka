
import vtk
import numpy as np
from scipy.interpolate import CubicSpline

import slicer


class VisualizationManager:
    def __init__(self, logger_callback=None):
        self.log_callback = logger_callback or self._default_log

    def _default_log(self, message):
        print(message)

    def create_workspace_cube(self, visible=False):
        bounds = [50, 750, -750, 750, 50, 750]
        coordinates = [
            (bounds[0], bounds[2], bounds[4]), (bounds[1], bounds[2], bounds[4]),
            (bounds[1], bounds[3], bounds[4]), (bounds[0], bounds[3], bounds[4]),  # Bottom face indices 0-3
            (bounds[0], bounds[2], bounds[5]), (bounds[1], bounds[2], bounds[5]),
            (bounds[1], bounds[3], bounds[5]), (bounds[0], bounds[3], bounds[5])   # Top face indices 4-7
        ]

        points = vtk.vtkPoints()
        point_ids = [points.InsertNextPoint(coord) for coord in coordinates]

        edge_indices = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face edges
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top face edges
            (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical connecting edges
        ]

        lines = vtk.vtkCellArray()
        for i, j in edge_indices:
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, point_ids[i])
            line.GetPointIds().SetId(1, point_ids[j])
            lines.InsertNextCell(line)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)

        model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", "PandaWorkspace")
        model_node.SetAndObservePolyData(polydata)

        display_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode")
        display_node.SetColor(0.2, 0.6, 1.0)  # blue
        display_node.SetOpacity(0.7)
        display_node.SetLineWidth(3.0)
        display_node.SetRepresentation(1)  # Wireframe display
        display_node.SetVisibility(visible)

        model_node.SetAndObserveDisplayNodeID(display_node.GetID())

    def create_orientation_vectors(self, position_markup_name, orientation_markup_name):
        position_node = slicer.util.getNode(position_markup_name)
        orientation_node = slicer.util.getNode(orientation_markup_name)

        if not position_node:
            self.log_callback("Could not find markup nodes for position vectors")
            return

        if not orientation_node:
            self.log_callback("Could not find markup nodes for orientation vectors")
            return

        position_points = self.extract_points_from_markup(position_node)
        orientation_points = self.extract_points_from_markup(orientation_node, len(position_points))

        model_node = slicer.mrmlScene.GetFirstNodeByName("OrientationVectors")
        if not model_node:
            model_node = slicer.vtkMRMLModelNode()
            model_node.SetName("OrientationVectors")
            slicer.mrmlScene.AddNode(model_node)

            display_node = slicer.vtkMRMLModelDisplayNode()
            display_node.SetColor(1.0, 0.0, 0.0)  # red
            display_node.SetLineWidth(1.0)
            slicer.mrmlScene.AddNode(display_node)
            model_node.SetAndObserveDisplayNodeID(display_node.GetID())

        self._generate_vector_lines(position_points, orientation_points, model_node)
        model_node.GetDisplayNode().SetVisibility(True)

        self.log_callback(f"Created {len(position_points)} orientation vectors for visualization")

    def hide_orientation_vectors(self):
        model_node = slicer.mrmlScene.GetFirstNodeByName("OrientationVectors")
        if model_node:
            model_node.GetDisplayNode().SetVisibility(False)

    def _generate_vector_lines(self, position_points, orientation_points, model_node):
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()

        for i in range(len(position_points)):
            point_id1 = points.InsertNextPoint(position_points[i])
            point_id2 = points.InsertNextPoint(orientation_points[i])

            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, point_id1)
            line.GetPointIds().SetId(1, point_id2)
            lines.InsertNextCell(line)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)
        model_node.SetAndObservePolyData(polydata)

    def extract_points_from_markup(self, markup_node, num_points=None):
        points = []

        if markup_node.IsA("vtkMRMLMarkupsCurveNode"):
            curve_points = markup_node.GetCurvePointsWorld()
            number_of_points = curve_points.GetNumberOfPoints()

            for i in range(number_of_points):
                point_coords = curve_points.GetPoint(i)
                points.append(list(point_coords))

            if num_points:
                points = self._resample_curve_points(points, num_points)

        else:
            # fiducial nodes
            for i in range(markup_node.GetNumberOfControlPoints()):
                pos = [0, 0, 0]
                markup_node.GetNthControlPointPosition(i, pos)
                points.append(pos)

        return points

    def _resample_curve_points(self, points, num_points):
        points_array = np.array(points)
        original_count = len(points)

        t_old = np.linspace(0, 1, original_count)
        t_new = np.linspace(0, 1, num_points)

        spline = CubicSpline(t_old, points_array, axis=0)
        resampled_points = spline(t_new).tolist()

        return resampled_points

    def setup_end_effector_markup(self):
        markups_logic = slicer.modules.markups.logic()
        markups_logic.AddNewFiducialNode("franka_ee_pos")
        markups_logic.AddControlPoint(0, 0, 0)

        ee_node = slicer.util.getNode("franka_ee_pos")
        ee_node.SetDisplayVisibility(False)
        ee_node.GetDisplayNode().SetGlyphScale(2)
        ee_node.GetDisplayNode().SetTextScale(0)

    def update_end_effector_position(self, position, visible=True):
        ee_node = slicer.util.getNode("franka_ee_pos")
        ee_node.SetDisplayVisibility(visible)
        ee_node.SetNthControlPointPosition(0, position[0], position[1], position[2])

    def create_registration_markup_nodes(self):
        markups_logic = slicer.modules.markups.logic()
        markups_logic.AddNewFiducialNode("p_frame")
        markups_logic.AddNewFiducialNode("q_frame")


class MarkupUtils:
    @staticmethod
    def get_available_markups(exclude_names=None):
        if exclude_names is None:
            exclude_names = ["franka_ee_pos", "p_frame", "q_frame"]

        fiducial_nodes = slicer.util.getNodesByClass('vtkMRMLMarkupsFiducialNode')
        curve_nodes = slicer.util.getNodesByClass('vtkMRMLMarkupsCurveNode')

        available_names = []
        all_nodes = fiducial_nodes + curve_nodes

        for node in all_nodes:
            node_name = node.GetName()
            if node_name not in exclude_names:
                available_names.append(node_name)

        return available_names

    @staticmethod
    def add_control_point_to_markup(markup_name, position, point_label=None):
        markup_node = slicer.util.getNode(markup_name)
        if not markup_node:
            return -1

        markup_node.AddControlPoint(position[0], position[1], position[2])
        point_index = markup_node.GetNumberOfControlPoints() - 1

        if point_label:
            markup_node.SetNthControlPointLabel(point_index, point_label)

        return point_index

    @staticmethod
    def clear_markup_points(markup_name):
        markup_node = slicer.util.getNode(markup_name)
        if markup_node:
            markup_node.RemoveAllControlPoints()

    @staticmethod
    def get_markup_points_info(markup_name):
        markup_node = slicer.util.getNode(markup_name)
        if not markup_node:
            return []

        points_info = []
        num_points = markup_node.GetNumberOfControlPoints()

        for i in range(num_points):
            pos = [0, 0, 0]
            markup_node.GetNthControlPointPosition(i, pos)
            label = markup_node.GetNthControlPointLabel(i)

            points_info.append({
                'position': pos,
                'label': label,
                'index': i
            })

        return points_info
