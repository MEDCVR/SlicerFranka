# robot physical limits and constraints
JOINT_LIMITS = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]

MAX_JOINT_VELOCITY = 0.5  # radians per second
MAX_CARTESIAN_VELOCITY = 75  # mm per second
MAX_ANGULAR_VELOCITY = 0.5  # radians per second

MAX_ALLOWED_JOINT_VELOCITY = 0.75

# default robot positions
HOME_JOINT_POSITION = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
HOME_CARTESIAN_POSITION = [315, 0.0, 450]  # [x, y, z] in mm
HOME_CARTESIAN_ORIENTATION = [3.14, 0.0, 5.4]  # [roll, pitch, yaw] in radians

UPDATE_RATE_HZ = 50
TIMER_INTERVAL_MS = 20

CARTESIAN_POSITION_RANGES = [
    [0, 1000],  # x range in mm
    [-1000, 1000],  # y range in mm
    [0, 1000],  # z range in mm
]

CARTESIAN_ORIENTATION_RANGES = [
    [0, 6.28],  # roll range in radians (0 to 2π)
    [-3.14, 3.14],  # pitch range in radians (-π to π)
    [0, 6.28],  # yaw range in radians (0 to 2π)
]

JOINT_MOVEMENT_THRESHOLD = 0.01
CARTESIAN_POSITION_THRESHOLD = 0.01
CARTESIAN_ORIENTATION_THRESHOLD = 0.01

# ROS2 communication topics
SIM_JOINT_STATE_TOPIC = "/franka/slicer/joint_state"
CMD_JOINT_STATE_TOPIC = "/franka/command/joint_state"
CURRENT_JOINT_STATE_TOPIC = "/franka/current/joint_state"

CMD_TIP_POSE_TOPIC = "/franka/command/tip_pose"
CURRENT_TIP_POSE_TOPIC = "/franka/current/tip_pose"

CONTROL_MODE_TOPIC = "/franka/control_mode"

# trajectory execution (robot-side)
TRAJECTORY_WAYPOINT_TOPIC = "/franka/trajectory/waypoint"
TRAJECTORY_COMMAND_TOPIC = "/franka/trajectory/command"
TRAJECTORY_STATUS_TOPIC = "/franka/trajectory/status"


# robot configuration
JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]
