import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp import plan_only as plan_only_module


class DummyLogger:
    def __init__(self):
        self.errors = []
        self.infos = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)

    def warn(self, message):
        self.warnings.append(message)


class DummyNode:
    def __init__(self):
        self.logger = DummyLogger()

    def get_logger(self):
        return self.logger


class DummyArmClient:
    def __init__(
        self,
        *,
        planonly_results=(True, True),
        state_result=True,
        move_results=(True,),
    ):
        self.events = []
        self.planonly_results = list(planonly_results)
        self.state_result = state_result
        self.move_results = list(move_results)

    def set_planonly(self, enabled):
        self.events.append(("set_planonly", enabled))
        result = self.planonly_results.pop(0)
        if isinstance(result, BaseException):
            raise result
        return result

    def change_state_to(self, state):
        self.events.append(("change_state_to", state))
        if isinstance(self.state_result, BaseException):
            raise self.state_result
        return self.state_result

    def move_to_pose(self, pose):
        self.events.append(("move_to_pose", pose))
        result = self.move_results.pop(0)
        if isinstance(result, BaseException):
            raise result
        return result


def planning_result(index, pregrasp_xyz):
    pose_6d = np.array([*pregrasp_xyz, 0.1, 0.2, 0.3], dtype=float)
    return SimpleNamespace(
        candidate_index=index,
        candidate_count=2,
        plan=SimpleNamespace(pre_grasp_pose_6d=pose_6d),
    )


def run_session(client, candidates, *, planner_error=None):
    node = DummyNode()

    def planner(_node, start_pose_6d):
        client.events.append(("planner", np.asarray(start_pose_6d)))
        if planner_error is not None:
            raise planner_error
        return candidates

    result = plan_only_module.run_plan_only_session(
        node,
        client,
        planner=planner,
        current_pose_reader=lambda _client: np.zeros(6, dtype=float),
    )
    return node, result


def test_plan_only_is_enabled_before_planning_goal_and_restored_after_success():
    first = planning_result(1, [0.1, 0.2, 0.3])
    client = DummyArmClient()

    _node, result = run_session(client, [first])

    assert result.planning_result is first
    assert result.attempted_candidates == 1
    assert [event[0] for event in client.events] == [
        "set_planonly",
        "planner",
        "change_state_to",
        "move_to_pose",
        "set_planonly",
    ]
    assert client.events[0] == ("set_planonly", True)
    assert client.events[-1] == ("set_planonly", False)
    pose = client.events[3][1]
    assert pose.header.frame_id == "world"
    assert np.allclose(
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
        [0.1, 0.2, 0.3],
    )


def test_plan_only_tries_the_next_candidate_after_moveit_rejects_the_first():
    first = planning_result(1, [0.1, 0.2, 0.3])
    second = planning_result(2, [0.4, 0.5, 0.6])
    client = DummyArmClient(move_results=(False, True))

    node, result = run_session(client, [first, second])

    assert result.planning_result is second
    assert result.attempted_candidates == 2
    assert len([event for event in client.events if event[0] == "move_to_pose"]) == 2
    assert len(node.logger.warnings) == 1


def test_plan_only_fails_clearly_when_moveit_rejects_every_candidate():
    first = planning_result(1, [0.1, 0.2, 0.3])
    second = planning_result(2, [0.4, 0.5, 0.6])
    client = DummyArmClient(move_results=(False, False))

    with pytest.raises(
        plan_only_module.PlanOnlyVerificationError,
        match="No grasp candidate has a reachable pre-grasp",
    ):
        run_session(client, [first, second])

    assert client.events[-1] == ("set_planonly", False)


def test_plan_only_restores_server_state_after_planner_exception():
    client = DummyArmClient()

    with pytest.raises(ValueError, match="planner failed"):
        run_session(client, [], planner_error=ValueError("planner failed"))

    assert client.events[0] == ("set_planonly", True)
    assert client.events[1][0] == "planner"
    assert np.allclose(client.events[1][1], np.zeros(6, dtype=float))
    assert client.events[2] == ("set_planonly", False)


def test_plan_only_restores_server_state_after_moveit_exception():
    candidate = planning_result(1, [0.1, 0.2, 0.3])
    client = DummyArmClient(move_results=(ValueError("action failed"),))

    with pytest.raises(ValueError, match="action failed"):
        run_session(client, [candidate])

    assert client.events[-1] == ("set_planonly", False)


def test_plan_only_enable_failure_sends_no_planner_or_action_goal():
    candidate = planning_result(1, [0.1, 0.2, 0.3])
    client = DummyArmClient(planonly_results=(False, True))

    with pytest.raises(
        plan_only_module.PlanOnlyVerificationError,
        match="Could not enable",
    ):
        run_session(client, [candidate])

    assert client.events == [
        ("set_planonly", True),
        ("set_planonly", False),
    ]


def test_plan_only_state_change_failure_sends_no_action_goal():
    candidate = planning_result(1, [0.1, 0.2, 0.3])
    client = DummyArmClient(state_result=False)

    with pytest.raises(
        plan_only_module.PlanOnlyVerificationError,
        match="CART_TRAJ_CTL",
    ):
        run_session(client, [candidate])

    assert not any(event[0] == "move_to_pose" for event in client.events)
    assert client.events[-1] == ("set_planonly", False)


def test_plan_only_cleanup_failure_blocks_successful_result():
    candidate = planning_result(1, [0.1, 0.2, 0.3])
    client = DummyArmClient(planonly_results=(True, False))

    with pytest.raises(
        plan_only_module.PlanOnlyCleanupError,
        match="Could not disable",
    ):
        run_session(client, [candidate])


def test_plan_only_module_has_no_motion_executor_cartesian_or_gripper_path():
    source_path = Path(plan_only_module.__file__)
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    imported_names = set()
    called_attributes = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported_names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            called_attributes.add(node.func.attr)

    assert "ArmMotionExecutor" not in imported_names
    assert "GripperController" not in imported_names
    assert called_attributes.isdisjoint(
        {
            "execute_plan",
            "execute_first_reachable_plan",
            "interpolate_to_pose",
            "send_pose_cmd",
            "move_to_pose_path",
            "return_to_initial_pose",
            "send_gripper_command",
        }
    )


def test_setup_registers_grasp_plan_only_console_entrypoint():
    setup_path = Path(__file__).parents[1] / "setup.py"
    setup_text = setup_path.read_text(encoding="utf-8")

    assert (
        "grasp_plan_only = my_course_pkg.grasp.plan_only:run"
        in setup_text
    )
