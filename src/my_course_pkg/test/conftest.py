import sys
import types

import numpy as np


def _module_exists(module_name):
    try:
        __import__(module_name)
        return True
    except ImportError:
        return False


def _install_ros_test_stubs():
    if not _module_exists("rclpy"):
        rclpy = types.ModuleType("rclpy")
        rclpy.duration = types.SimpleNamespace(
            Duration=lambda seconds: types.SimpleNamespace(seconds=seconds)
        )
        rclpy.time = types.SimpleNamespace(Time=lambda: object())

        rclpy_action = types.ModuleType("rclpy.action")
        rclpy_action.ActionClient = object
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = object
        rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")
        rclpy_callback_groups.MutuallyExclusiveCallbackGroup = object
        rclpy_callback_groups.ReentrantCallbackGroup = object
        rclpy_qos = types.ModuleType("rclpy.qos")

        class QoSProfile:
            def __init__(self, depth=10, durability=None):
                self.depth = depth
                self.durability = durability

        rclpy_qos.QoSProfile = QoSProfile
        rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
            TRANSIENT_LOCAL=object()
        )

        sys.modules.setdefault("rclpy", rclpy)
        sys.modules.setdefault("rclpy.action", rclpy_action)
        sys.modules.setdefault("rclpy.node", rclpy_node)
        sys.modules.setdefault("rclpy.callback_groups", rclpy_callback_groups)
        sys.modules.setdefault("rclpy.qos", rclpy_qos)

    if not _module_exists("geometry_msgs.msg"):
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

        class Point:
            def __init__(self, x=0.0, y=0.0, z=0.0):
                self.x = x
                self.y = y
                self.z = z

        geometry_msgs_msg.Point = Point
        sys.modules.setdefault("geometry_msgs", geometry_msgs)
        sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    if not _module_exists("visualization_msgs.msg"):
        visualization_msgs = types.ModuleType("visualization_msgs")
        visualization_msgs_msg = types.ModuleType("visualization_msgs.msg")

        class Marker:
            SPHERE = 2
            ARROW = 0
            ADD = 0

            def __init__(self):
                self.header = types.SimpleNamespace(frame_id="", stamp=None)
                self.ns = ""
                self.id = 0
                self.type = 0
                self.action = 0
                self.pose = types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(w=1.0),
                )
                self.scale = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.color = types.SimpleNamespace(
                    r=0.0,
                    g=0.0,
                    b=0.0,
                    a=0.0,
                )
                self.points = []

        class MarkerArray:
            def __init__(self):
                self.markers = []

        visualization_msgs_msg.Marker = Marker
        visualization_msgs_msg.MarkerArray = MarkerArray
        sys.modules.setdefault("visualization_msgs", visualization_msgs)
        sys.modules.setdefault("visualization_msgs.msg", visualization_msgs_msg)

    if not _module_exists("action_msgs.msg"):
        action_msgs = types.ModuleType("action_msgs")
        action_msgs_msg = types.ModuleType("action_msgs.msg")
        action_msgs_msg.GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)
        sys.modules.setdefault("action_msgs", action_msgs)
        sys.modules.setdefault("action_msgs.msg", action_msgs_msg)

    if not _module_exists("control_msgs.action"):
        control_msgs = types.ModuleType("control_msgs")
        control_msgs_action = types.ModuleType("control_msgs.action")
        control_msgs_action.FollowJointTrajectory = object
        control_msgs_action.GripperCommand = object
        control_msgs_msg = types.ModuleType("control_msgs.msg")
        control_msgs_msg.GripperCommand = object
        sys.modules.setdefault("control_msgs", control_msgs)
        sys.modules.setdefault("control_msgs.action", control_msgs_action)
        sys.modules.setdefault("control_msgs.msg", control_msgs_msg)

    if not _module_exists("sensor_msgs.msg"):
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

        class JointState:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id="")
                self.name = []
                self.position = []

        sensor_msgs_msg.JointState = JointState
        sys.modules.setdefault("sensor_msgs", sensor_msgs)
        sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    if not _module_exists("trajectory_msgs.msg"):
        trajectory_msgs = types.ModuleType("trajectory_msgs")
        trajectory_msgs_msg = types.ModuleType("trajectory_msgs.msg")

        class JointTrajectory:
            def __init__(self):
                self.joint_names = []
                self.points = []

        class JointTrajectoryPoint:
            def __init__(self):
                self.positions = []
                self.time_from_start = types.SimpleNamespace(sec=0)

        trajectory_msgs_msg.JointTrajectory = JointTrajectory
        trajectory_msgs_msg.JointTrajectoryPoint = JointTrajectoryPoint
        sys.modules.setdefault("trajectory_msgs", trajectory_msgs)
        sys.modules.setdefault("trajectory_msgs.msg", trajectory_msgs_msg)

    if not _module_exists("arm_api2_py.arm_api2_client"):
        arm_api2_py = types.ModuleType("arm_api2_py")
        arm_api2_client = types.ModuleType("arm_api2_py.arm_api2_client")
        arm_api2_client.ArmApi2Client = object
        sys.modules.setdefault("arm_api2_py", arm_api2_py)
        sys.modules.setdefault("arm_api2_py.arm_api2_client", arm_api2_client)

    if not _module_exists("tf2_ros"):
        tf2_ros = types.ModuleType("tf2_ros")
        tf2_ros.TransformException = Exception
        sys.modules.setdefault("tf2_ros", tf2_ros)

    if not _module_exists("tf_transformations"):
        tf_transformations = types.ModuleType("tf_transformations")
        tf_transformations.quaternion_matrix = lambda quat: np.eye(4)
        sys.modules.setdefault("tf_transformations", tf_transformations)


_install_ros_test_stubs()
