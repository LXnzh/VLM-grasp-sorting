import time
from collections import namedtuple

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionClient

from my_course_pkg.grasp.config import (
    GRIPPER_ACTION_NAMES,
    GRIPPER_ACTION_WAIT_SEC,
    GRIPPER_CLOSE_MIN_POSITION,
    GRIPPER_COMMAND_MODE,
    GRIPPER_EFFORT,
    GRIPPER_OPEN_ATTEMPTS,
    GRIPPER_OPEN_MAX_POSITION,
    GRIPPER_OPEN_POSITION,
    GRIPPER_OPEN_RETRY_DELAY_SEC,
)


class GripperCommandResult(
    namedtuple(
        "GripperCommandResult",
        [
            "accepted",
            "target_position",
            "actual_position",
            "effort",
            "stalled",
            "reached_goal",
            "action_status",
            "action_succeeded",
            "command_mode",
            "action_name",
            "attempts",
        ],
    )
):
    __slots__ = ()

    def __bool__(self):
        return bool(self.accepted)


class GripperController:
    def __init__(self, node, arm_api2_client, callback_group):
        self.node = node
        self.arm_api2_client = arm_api2_client
        self.gripper_action_clients = {
            action_name: ActionClient(
                node,
                GripperCommand,
                action_name,
                callback_group=callback_group,
            )
            for action_name in GRIPPER_ACTION_NAMES
        }
        self.gripper_action_name = None
        self.gripper_action_client = None

    def send_gripper_command(self, position):
        print(f"Setting gripper to: {position:.2f}")
        if GRIPPER_COMMAND_MODE in ("arm_api_topic", "topic", "native"):
            return self._send_gripper_topic(position)

        self.ensure_gripper_ready()
        attempts = (
            GRIPPER_OPEN_ATTEMPTS
            if self._is_open_command(position)
            else 1
        )
        result = None
        for attempt_index in range(1, attempts + 1):
            result = self._send_gripper_action_once(position, attempt_index)
            if result.accepted:
                return result
            if attempt_index < attempts:
                self.node.get_logger().warn(
                    "Gripper open command was not accepted; retrying once "
                    f"before failing closed. attempt={attempt_index}/"
                    f"{attempts}"
                )
                time.sleep(GRIPPER_OPEN_RETRY_DELAY_SEC)

        return result

    def _send_gripper_action_once(self, position, attempt_index=1):
        command = GripperCommandMsg()
        command.position = float(position)
        command.max_effort = GRIPPER_EFFORT
        goal_msg = GripperCommand.Goal()
        goal_msg.command = command

        print(f"Sending gripper command through: {self.gripper_action_name}")
        result = self.gripper_action_client.send_goal(goal_msg)
        reached_position = result.result.position
        reached_effort = result.result.effort
        stalled = result.result.stalled
        reached_goal = result.result.reached_goal
        action_succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
        command_accepted = self._gripper_result_accepted(
            commanded_position=position,
            reached_position=reached_position,
            stalled=stalled,
            reached_goal=reached_goal,
            action_succeeded=action_succeeded,
        )
        print(
            "Gripper result: "
            f"status={result.status}, "
            f"accepted={command_accepted}, position={reached_position:.3f}, "
            f"effort={reached_effort:.1f}, stalled={stalled}, "
            f"reached_goal={reached_goal}"
        )
        if reached_goal and not action_succeeded:
            self.node.get_logger().warn(
                "Gripper action reported reached_goal=True but did not finish with "
                f"SUCCEEDED status. Treating the command as accepted. "
                f"Action server: {self.gripper_action_name}, status={result.status}."
            )
        if not command_accepted:
            self.node.get_logger().warn(
                "Gripper command result is not acceptable for the requested target. "
                f"Action server: {self.gripper_action_name}. "
                "Check the gripper adapter/mocked gripper action server."
            )
        return self._build_result(
            accepted=command_accepted,
            target_position=position,
            actual_position=reached_position,
            effort=reached_effort,
            stalled=stalled,
            reached_goal=reached_goal,
            action_status=result.status,
            action_succeeded=action_succeeded,
            command_mode="action",
            action_name=self.gripper_action_name,
            attempts=attempt_index,
        )

    def _send_gripper_topic(self, position):
        ok, reached_position, reached_effort, stalled, reached_goal = (
            self.arm_api2_client.send_gripper_command(
                float(position),
                GRIPPER_EFFORT,
                wait_for_result=False,
            )
        )
        print(
            "Gripper command published through ArmApi2Client topic: "
            f"ok={ok}, position={reached_position:.3f}, "
            f"effort={reached_effort:.1f}, stalled={stalled}, "
            f"reached_goal={reached_goal}"
        )
        action_succeeded = bool(ok)
        accepted = self._gripper_result_accepted(
            commanded_position=position,
            reached_position=reached_position,
            stalled=stalled,
            reached_goal=reached_goal,
            action_succeeded=action_succeeded,
        )
        return self._build_result(
            accepted=accepted,
            target_position=position,
            actual_position=reached_position,
            effort=reached_effort,
            stalled=stalled,
            reached_goal=reached_goal,
            action_status=None,
            action_succeeded=action_succeeded,
            command_mode="topic",
            action_name="ArmApi2Client",
            attempts=1,
        )

    @staticmethod
    def _is_open_command(position):
        return float(position) <= float(GRIPPER_OPEN_POSITION + 1e-9)

    def _gripper_result_accepted(
        self,
        commanded_position,
        reached_position,
        stalled,
        reached_goal,
        action_succeeded,
    ):
        commanded_position = float(commanded_position)
        reached_position = float(reached_position)
        stalled = bool(stalled)
        reached_goal = bool(reached_goal)
        action_succeeded = bool(action_succeeded)

        if self._is_open_command(commanded_position):
            opened = reached_position <= GRIPPER_OPEN_MAX_POSITION
            if not opened:
                self.node.get_logger().warn(
                    "Gripper open command did not reach the open position: "
                    f"target={commanded_position:.3f}, "
                    f"actual={reached_position:.3f}, "
                    f"max_open={GRIPPER_OPEN_MAX_POSITION:.3f}, "
                    f"stalled={stalled}, reached_goal={reached_goal}."
                )
            return opened

        if stalled and reached_position < GRIPPER_CLOSE_MIN_POSITION:
            self.node.get_logger().warn(
                "Gripper close command stalled below the minimum useful "
                f"position: actual={reached_position:.3f}, "
                f"minimum={GRIPPER_CLOSE_MIN_POSITION:.3f}."
            )
            return False

        return action_succeeded or reached_goal or stalled

    def _build_result(
        self,
        *,
        accepted,
        target_position,
        actual_position,
        effort,
        stalled,
        reached_goal,
        action_status,
        action_succeeded,
        command_mode,
        action_name,
        attempts,
    ):
        result = GripperCommandResult(
            accepted=bool(accepted),
            target_position=float(target_position),
            actual_position=float(actual_position),
            effort=float(effort),
            stalled=bool(stalled),
            reached_goal=bool(reached_goal),
            action_status=action_status,
            action_succeeded=bool(action_succeeded),
            command_mode=str(command_mode),
            action_name=action_name,
            attempts=int(attempts),
        )
        self.last_commanded_position = result.target_position
        self.last_reached_position = result.actual_position
        self.last_effort = result.effort
        self.last_stalled = result.stalled
        self.last_reached_goal = result.reached_goal
        self.last_action_status = result.action_status
        self.last_command_result = result
        return result

    def ensure_gripper_ready(self):
        if self.gripper_action_client is not None:
            return

        deadline = time.monotonic() + GRIPPER_ACTION_WAIT_SEC
        while time.monotonic() < deadline:
            for action_name, action_client in self.gripper_action_clients.items():
                if action_client.wait_for_server(timeout_sec=0.2):
                    self.gripper_action_name = action_name
                    self.gripper_action_client = action_client
                    print(f"Using gripper action server: {action_name}")
                    return

        raise RuntimeError(
            "No gripper action server is available. Tried: "
            f"{GRIPPER_ACTION_NAMES}. Check `ros2 action list | grep gripper`, "
            "or set GRASP_GRIPPER_ACTIONS to the correct action name."
        )
