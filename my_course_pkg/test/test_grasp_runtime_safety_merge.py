from types import SimpleNamespace
import sys
import types

import numpy as np
import pytest

# Keep this unit test independent from generated arm_api2 message packages.
# The executor only needs the joint ordering for the pure safety checks below.
verify_init_pose = types.ModuleType("my_course_pkg.verify_init_pose")


class _VerifyInitPoseNode:
    arm_joint_order = []


verify_init_pose.VerifyInitPoseNode = _VerifyInitPoseNode
sys.modules.setdefault("my_course_pkg.verify_init_pose", verify_init_pose)

from my_course_pkg.grasp import config
from my_course_pkg.grasp import pick_place_planner as planner_module
from my_course_pkg.grasp.executor import (
    ArmMotionExecutor,
    MoveItMotionError,
    VerticalFinalZBounds,
)
from my_course_pkg.grasp.gripper_control import GripperController
from my_course_pkg.grasp.pick_place_planner import (
    ApproachClearanceConfig,
    SceneClearanceObstacle,
    _approach_clearance_violation,
)
from my_course_pkg.grasp.trajectory_planner import plan_safe_pick_place_steps


class _Logger:
    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


def _gripper_controller_without_ros_init():
    controller = GripperController.__new__(GripperController)
    controller.node = _Node()
    return controller


def test_close_stall_below_minimum_is_rejected():
    controller = _gripper_controller_without_ros_init()

    assert not controller._gripper_result_accepted(
        commanded_position=config.GRIPPER_CLOSED_POSITION,
        reached_position=config.GRIPPER_CLOSE_MIN_POSITION / 2.0,
        stalled=True,
        reached_goal=False,
        action_succeeded=False,
    )


def test_open_requires_measured_open_position():
    controller = _gripper_controller_without_ros_init()

    assert not controller._gripper_result_accepted(
        commanded_position=config.GRIPPER_OPEN_POSITION,
        reached_position=config.GRIPPER_OPEN_MAX_POSITION + 0.01,
        stalled=True,
        reached_goal=False,
        action_succeeded=False,
    )


def test_sorting_drop_xyz_is_preserved_and_release_has_pre_open_hold():
    grasp = np.array([0.25, -0.10, 0.92, 0.0, np.pi, 0.0])
    drop = np.array([-0.41, -0.63, 1.04, 0.0, np.pi, 0.0])
    plan = plan_safe_pick_place_steps(
        start_pose_6d=np.array([0.0, 0.0, 1.1, 0.0, np.pi, 0.0]),
        grasp_pose_6d=grasp,
        drop_pose_6d=drop,
    )

    assert np.allclose(plan.drop_pose_6d, drop)
    names = [step.name for step in plan.steps]
    assert names.index("hold_before_release") + 1 == names.index(
        "open_gripper_to_release"
    )


def test_exact_approach_corridor_rejects_nearby_box():
    half_extents = np.array([0.02, 0.02, 0.04])
    transform = np.eye(4)
    transform[:3, 3] = [0.0, 0.0, 0.94]
    obstacle = SceneClearanceObstacle(
        name="nearby_object",
        center=transform[:3, 3].copy(),
        radius_xy=float(np.hypot(*half_extents[:2])),
        half_height=float(half_extents[2]),
        T_world_bounds=transform,
        half_extents=half_extents,
    )
    plan = SimpleNamespace(
        pre_grasp_pose_6d=np.array([-0.10, 0.0, 0.94, 0.0, 0.0, 0.0]),
        grasp_pose_6d=np.array([0.10, 0.0, 0.94, 0.0, 0.0, 0.0]),
    )
    clearance = ApproachClearanceConfig(
        profile="side",
        enabled=True,
        require_scene=False,
        corridor_radius_m=0.01,
        clearance_margin_m=0.0,
        vertical_margin_m=0.01,
    )

    violation = _approach_clearance_violation(plan, [obstacle], clearance)

    assert violation is not None
    assert violation.obstacle.name == "nearby_object"


def test_vertical_final_z_gate_uses_live_object_bounds():
    bounds = VerticalFinalZBounds(
        bottom_z=0.89,
        center_z=0.905,
        top_z=0.92,
        valid=True,
        reason="valid",
    )
    target = np.array([0.0, 0.0, 0.898, 0.0, 0.0, 0.0])

    accepted = ArmMotionExecutor._vertical_final_z_decision(
        target,
        np.array([0.0, 0.0, 0.904, 0.0, 0.0, 0.0]),
        bounds,
    )
    rejected = ArmMotionExecutor._vertical_final_z_decision(
        target,
        np.array([0.0, 0.0, 0.906, 0.0, 0.0, 0.0]),
        bounds,
    )

    assert accepted.passed
    assert accepted.mode == "controlled_center"
    assert not rejected.passed
    assert rejected.reason == "actual Z is above live center"


def test_scene_safety_defaults_to_existing_simulator_topic():
    assert config.GRASP_SCENE_CLEARANCE_TOPIC == "/scene_description"


def test_candidate_retry_only_follows_confirmed_pregrasp_planning_failure():
    executor = ArmMotionExecutor.__new__(ArmMotionExecutor)
    executor.node = _Node()
    plans = [object(), object()]
    calls = []

    def execute_plan(plan):
        calls.append(plan)
        if plan is plans[0]:
            raise MoveItMotionError("move_to_pre_grasp", "planning_failed")

    executor.execute_plan = execute_plan

    assert executor.execute_first_reachable_plan(plans) is plans[1]
    assert calls == plans


def test_unknown_pregrasp_motion_status_does_not_retry():
    executor = ArmMotionExecutor.__new__(ArmMotionExecutor)
    executor.node = _Node()
    plans = [object(), object()]
    executor.execute_plan = lambda _plan: (_ for _ in ()).throw(
        MoveItMotionError("move_to_pre_grasp")
    )

    with pytest.raises(RuntimeError, match="controller may still be moving"):
        executor.execute_first_reachable_plan(plans)


def test_planner_keeps_current_sorting_drop_target(monkeypatch):
    object_pose = np.eye(4)
    grasp_transform = np.eye(4)
    grasp_transform[:3, 3] = [0.10, 0.0, 0.20]
    grasp_pose = np.array([0.10, 0.0, 0.20, 0.0, np.pi, 0.0])
    sorting_position = np.array([-0.42, -0.61, 1.03])

    monkeypatch.setattr(
        planner_module,
        "get_selected_object_info",
        lambda _path: "tomato_soup_can",
    )
    monkeypatch.setattr(
        planner_module,
        "estimate_object_world_pose",
        lambda *_args, **_kwargs: object_pose.copy(),
    )
    monkeypatch.setattr(
        planner_module,
        "select_grasp_pose_candidates_6d",
        lambda *_args, **_kwargs: [(grasp_pose.copy(), grasp_transform.copy())],
    )
    monkeypatch.setattr(
        planner_module,
        "_load_scene_clearance_obstacles",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        planner_module,
        "resolve_drop_target",
        lambda *_args, **_kwargs: SimpleNamespace(
            source="test_sorting_target",
            category="food",
            bin_name="food_bin",
            position=sorting_position.copy(),
        ),
    )

    result = planner_module.plan_pick_place_candidates_from_perception(
        node=SimpleNamespace(),
        start_pose_6d=np.array([0.0, 0.0, 0.40, 0.0, np.pi, 0.0]),
        object_cam_pose_path="unused.json",
        selected_object_path="unused.json",
        canonicalize_tabletop=False,
    )[0]

    np.testing.assert_allclose(result.plan.drop_pose_6d[:3], sorting_position)
    assert result.plan.debug_info["drop_target"].bin_name == "food_bin"
