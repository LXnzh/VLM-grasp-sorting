import importlib.util
import sys
import types
from pathlib import Path


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node, executor=None: None

    rclpy_action = types.ModuleType("rclpy.action")
    rclpy_action.ActionServer = object
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = object
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy_node.ParameterDescriptor = object
    rclpy_node.ParameterType = types.SimpleNamespace(
        PARAMETER_STRING=1,
        PARAMETER_INTEGER=2,
        PARAMETER_DOUBLE=3,
        PARAMETER_BOOL=4,
    )
    rclpy_node.ParameterNotDeclaredException = Exception
    rclpy_node.ParameterUninitializedException = Exception
    rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")
    rclpy_callback_groups.ReentrantCallbackGroup = object
    rclpy_callback_groups.MutuallyExclusiveCallbackGroup = object

    control_msgs = types.ModuleType("control_msgs")
    control_msgs_action = types.ModuleType("control_msgs.action")

    class GripperCommandAction:
        class Result:
            def __init__(self, position=0.0, effort=0.0, stalled=False, reached_goal=False):
                self.position = position
                self.effort = effort
                self.stalled = stalled
                self.reached_goal = reached_goal

        class Feedback:
            def __init__(self, position=0.0, effort=0.0, stalled=False, reached_goal=False):
                self.position = position
                self.effort = effort
                self.stalled = stalled
                self.reached_goal = reached_goal

    control_msgs_action.GripperCommand = GripperCommandAction
    control_msgs_msg = types.ModuleType("control_msgs.msg")
    control_msgs_msg.GripperCommand = object

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = object

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.action"] = rclpy_action
    sys.modules["rclpy.executors"] = rclpy_executors
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.callback_groups"] = rclpy_callback_groups
    sys.modules["control_msgs"] = control_msgs
    sys.modules["control_msgs.action"] = control_msgs_action
    sys.modules["control_msgs.msg"] = control_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg


def _load_adapter_node_module():
    _install_ros_stubs()
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "robotiq_2f_adapter_node.py"
    )
    spec = importlib.util.spec_from_file_location(
        "robotiq_2f_adapter_node", script_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeLogger:
    def info(self, message):
        pass

    def error(self, message):
        pass


class FakeGoalHandle:
    def __init__(self):
        self.aborted = False
        self.succeeded = False

    def abort(self):
        self.aborted = True

    def succeed(self):
        self.succeeded = True

    def publish_feedback(self, feedback):
        pass


class ReconnectableAdapter:
    min_position = 0
    max_position = 255

    def __init__(self):
        self.connect_calls = 0
        self.activate_calls = 0

    def connect(self, hostname, port):
        self.connect_calls += 1

    def activate(self, auto_calibrate=True):
        self.activate_calls += 1

    @property
    def position(self):
        return 0


def test_gripper_command_reconnects_when_socket_becomes_available():
    module = _load_adapter_node_module()
    node = object.__new__(module.Robotiq2fAdapterNode)
    adapter = ReconnectableAdapter()
    goal_handle = FakeGoalHandle()

    node.get_logger = lambda: FakeLogger()
    node.gripper_adapter = adapter
    node._gripper_ready = False
    node._robot_ip = "127.0.0.1"
    node._robot_port = 63352
    node._auto_calibrate = False
    node._normalized_effort_factor = (235.0 - 20.0) / 255
    node._normalized_effort_baseline = 20.0
    node._normalized_speed_factor = (0.15 - 0.02) / 255
    node._normalized_speed_baseline = 0.02

    result = node._Robotiq2fAdapterNode__move_gripper_to_joint_position(
        goal_handle=goal_handle,
        joint_position_rad=0.0,
        max_effort_N=140.0,
    )

    assert adapter.connect_calls == 1
    assert adapter.activate_calls == 1
    assert node._gripper_ready is True
    assert goal_handle.succeeded is True
    assert goal_handle.aborted is False
    assert result.reached_goal is True
