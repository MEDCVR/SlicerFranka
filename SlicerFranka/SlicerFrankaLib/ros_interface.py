
import os
import logging
import subprocess
import signal

from .constants import (
    SIM_JOINT_STATE_TOPIC, CMD_JOINT_STATE_TOPIC, CURRENT_JOINT_STATE_TOPIC,
    CMD_TIP_POSE_TOPIC, CURRENT_TIP_POSE_TOPIC, CONTROL_MODE_TOPIC,
    TRAJECTORY_WAYPOINT_TOPIC, TRAJECTORY_COMMAND_TOPIC, TRAJECTORY_STATUS_TOPIC,
    JOINT_NAMES
)


class ROS2Interface:
    def __init__(self, logger_callback=None):
        self.log_callback = logger_callback or self._default_log

        self.ros_distro = None
        self.ros_path = None
        self.ros2_env = None
        self.ros2_exec = None

        self.joint_state_publisher_process = None
        self.robot_state_publisher_process = None

        self.ros2_logic = None
        self.ros2_node = None

        self.sim_joint_publisher = None
        self.cmd_joint_publisher = None
        self.tip_pose_publisher = None
        self.mode_publisher = None
        self.trajectory_waypoint_publisher = None
        self.trajectory_command_publisher = None

        self.joint_subscriber = None
        self.tip_pose_subscriber = None
        self.trajectory_status_subscriber = None

        self._setup_ros2_environment()

    def _default_log(self, message, level=logging.INFO):
        if level == logging.ERROR:
            logging.error(message)
        elif level == logging.WARNING:
            logging.warning(message)
        else:
            logging.info(message)

    def _setup_ros2_environment(self):
        try:
            self.ros_distro = os.environ['ROS_DISTRO']
            self.ros_path = '/opt/ros/' + self.ros_distro
            self.ros2_env = 'unset PYTHONHOME ; unset PYTHONPATH ; . ' + \
                self.ros_path + '/setup.sh ; '
            self.ros2_exec = self.ros2_env + '/usr/bin/python3 /opt/ros/' + \
                self.ros_distro + '/bin/ros2 '
            self.log_callback("ROS2 environment setup completed", logging.INFO)
        except KeyError:
            self.log_callback(
                "ROS_DISTRO environment variable not found", logging.ERROR)

    def initialize_node(self, slicer_ros2_logic):
        self.ros2_logic = slicer_ros2_logic
        self.ros2_node = self.ros2_logic.GetDefaultROS2Node()

        self._launch_robot_nodes()

        self.ros2_node.CreateAndAddRobotNode(
            'robot1', '/robot_state_publisher', 'robot_description', 'base', ''
        )

        self.log_callback("ROS2 node initialization complete", logging.INFO)

    def _launch_robot_nodes(self):
        self.log_callback(
            "Launching robot_state_publisher and joint_state_publisher", logging.INFO)

        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            __file__)), 'Resources', 'URDF', 'fr3_pointer.urdf')
        self.log_callback(f"Using URDF file: {urdf_path}", logging.INFO)

        self.log_callback("Launching joint_state_publisher...", logging.INFO)
        self.joint_state_publisher_process = self._run_ros2_command(
            "run joint_state_publisher joint_state_publisher " +
            f"--ros-args -p source_list:='[\"{SIM_JOINT_STATE_TOPIC}\"]' -p rate:=50.0"
        )

        self.log_callback("Launching robot_state_publisher...", logging.INFO)
        self.robot_state_publisher_process = self._run_ros2_command(
            f"run robot_state_publisher robot_state_publisher {urdf_path}"
        )

        self.log_callback("Robot nodes launched", logging.INFO)

    def _run_ros2_command(self, command):
        full_command = self.ros2_exec + command
        self.log_callback(f"Executing command: {command}", logging.INFO)

        process = subprocess.Popen(
            full_command,
            shell=True,
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return process

    def setup_publishers(self):
        if not self.ros2_node:
            raise RuntimeError("ROS2 node not initialized")

        joint_state_type = 'vtkMRMLROS2PublisherJointStateNode'

        self.sim_joint_publisher = self.ros2_node.CreateAndAddPublisherNode(
            joint_state_type, SIM_JOINT_STATE_TOPIC
        )
        self.log_callback(
            f"Created simulation joint publisher on {SIM_JOINT_STATE_TOPIC}",
            logging.INFO,
        )

        self.cmd_joint_publisher = self.ros2_node.CreateAndAddPublisherNode(
            joint_state_type, CMD_JOINT_STATE_TOPIC
        )
        self.log_callback(
            f"Created command joint publisher on {CMD_JOINT_STATE_TOPIC}",
            logging.INFO,
        )

        pose_type = 'vtkMRMLROS2PublisherPoseStampedNode'
        self.tip_pose_publisher = self.ros2_node.CreateAndAddPublisherNode(
            pose_type, CMD_TIP_POSE_TOPIC
        )
        self.log_callback(
            f"Created tip pose publisher on {CMD_TIP_POSE_TOPIC}", logging.INFO
        )

        self.trajectory_waypoint_publisher = self.ros2_node.CreateAndAddPublisherNode(
            pose_type, TRAJECTORY_WAYPOINT_TOPIC
        )
        self.log_callback(
            f"Created trajectory waypoint publisher on {TRAJECTORY_WAYPOINT_TOPIC}",
            logging.INFO,
        )

        string_type = 'vtkMRMLROS2PublisherStringNode'
        self.mode_publisher = self.ros2_node.CreateAndAddPublisherNode(
            string_type, CONTROL_MODE_TOPIC
        )
        self.log_callback(
            f"Created control mode publisher on {CONTROL_MODE_TOPIC}", logging.INFO
        )

        self.trajectory_command_publisher = self.ros2_node.CreateAndAddPublisherNode(
            string_type, TRAJECTORY_COMMAND_TOPIC
        )
        self.log_callback(
            f"Created trajectory command publisher on {TRAJECTORY_COMMAND_TOPIC}",
            logging.INFO,
        )

        self.log_callback("Publishers set up successfully", logging.INFO)

    def setup_subscribers(
        self,
        joint_callback,
        tip_pose_callback,
        trajectory_status_callback,
    ):
        if not self.ros2_node:
            raise RuntimeError(
                "ROS2 node not initialized. Call initialize_node() first.")

        joint_state_type = 'vtkMRMLROS2SubscriberJointStateNode'
        self.joint_subscriber = self.ros2_node.CreateAndAddSubscriberNode(
            joint_state_type, CURRENT_JOINT_STATE_TOPIC
        )
        self.joint_subscriber.AddObserver('ModifiedEvent', joint_callback)
        self.log_callback(
            f"Created joint state subscriber on {CURRENT_JOINT_STATE_TOPIC}",
            logging.INFO,
        )

        pose_type = 'vtkMRMLROS2SubscriberPoseStampedNode'
        self.tip_pose_subscriber = self.ros2_node.CreateAndAddSubscriberNode(
            pose_type, CURRENT_TIP_POSE_TOPIC
        )
        self.tip_pose_subscriber.AddObserver('ModifiedEvent', tip_pose_callback)
        self.log_callback(
            f"Created tip pose subscriber on {CURRENT_TIP_POSE_TOPIC}",
            logging.INFO,
        )

        string_type = 'vtkMRMLROS2SubscriberStringNode'
        self.trajectory_status_subscriber = self.ros2_node.CreateAndAddSubscriberNode(
            string_type, TRAJECTORY_STATUS_TOPIC
        )
        self.trajectory_status_subscriber.AddObserver(
            'ModifiedEvent', trajectory_status_callback
        )
        self.log_callback(
            f"Created trajectory status subscriber on {TRAJECTORY_STATUS_TOPIC}",
            logging.INFO,
        )

        self.log_callback("Subscribers set up successfully", logging.INFO)

    def publish_joint_state(self, publisher, joint_angles):
        if not publisher:
            raise RuntimeError("Publisher not initialized")

        message = publisher.GetBlankMessage()
        message.SetName(tuple(JOINT_NAMES))
        message.SetPosition(tuple(joint_angles))
        publisher.Publish(message)

    def publish_pose(self, publisher, position, orientation_matrix):
        if not publisher:
            raise RuntimeError("Pose publisher not initialized")

        for i in range(3):
            orientation_matrix.SetElement(i, 3, position[i])

        message = publisher.GetBlankMessage()
        message.SetPose(orientation_matrix)
        publisher.Publish(message)

    def publish_tip_pose(self, position, orientation_matrix):
        self.publish_pose(self.tip_pose_publisher, position, orientation_matrix)

    def publish_trajectory_waypoint(self, position, orientation_matrix):
        self.publish_pose(
            self.trajectory_waypoint_publisher, position, orientation_matrix
        )

    def publish_trajectory_command(self, command):
        if not self.trajectory_command_publisher:
            raise RuntimeError("Trajectory command publisher not initialized")
        self.trajectory_command_publisher.Publish(command)
        self.log_callback(f"Published trajectory command: {command}", logging.INFO)

    def publish_control_mode(self, mode):
        if not self.mode_publisher:
            raise RuntimeError("Mode publisher not initialized")

        self.mode_publisher.Publish(mode)
        self.log_callback(f"Published control mode: {mode}", logging.INFO)

    def cleanup(self):
        self.log_callback("Cleaning up ROS2 processes...", logging.INFO)

        if self.joint_state_publisher_process:
            try:
                os.killpg(os.getpgid(
                    self.joint_state_publisher_process.pid), signal.SIGTERM)
                self.log_callback(
                    "Terminated joint_state_publisher process", logging.INFO)
            except Exception as e:
                self.log_callback(
                    f"Error terminating joint_state_publisher: {str(e)}",
                    logging.WARNING,
                )

        if self.robot_state_publisher_process:
            try:
                os.killpg(os.getpgid(
                    self.robot_state_publisher_process.pid), signal.SIGTERM)
                self.log_callback(
                    "Terminated robot_state_publisher process", logging.INFO)
            except Exception as e:
                self.log_callback(
                    f"Error terminating robot_state_publisher: {str(e)}",
                    logging.WARNING,
                )

        self.log_callback("ROS2 processes cleanup complete", logging.INFO)
