from dataclasses import dataclass, field

import numpy as np
from sim_pick_place.utils.helpers import _ensure_pose_array
from sim_pick_place.utils.pick_place_utils import get_pregrasp_pose

from my_course_pkg.grasp.config import (
    APPROACH_DIST,
    DROP_POSITION,
    DROP_RELEASE_Z_OFFSET,
    DROP_HIGH_HOLD_SEC,
    GRASP_EXECUTION_MODE,
    GRASP_EXECUTION_MODES,
    GRASP_LIFT_HEIGHT,
    GRASP_LIFT_HOLD_SEC,
    GRASP_RETURN_RELEASE_CLEARANCE_M,
    GRIPPER_CLOSED_POSITION,
    GRIPPER_CLOSE_SETTLE_SEC,
    GRIPPER_OPEN_POSITION,
    GRIPPER_SETTLE_SEC,
    RELEASE_PRE_OPEN_HOLD_SEC,
)
from my_course_pkg.grasp.transforms import pose_text


@dataclass
class PlanStep:
    name: str
    action: str
    mode: str = ""
    start_pose_6d: np.ndarray | None = None
    end_pose_6d: np.ndarray | None = None
    pose_6d: np.ndarray | None = None
    gripper_position: float | None = None
    duration_sec: float = 0.0


@dataclass
class PickPlacePlan:
    start_pose_6d: np.ndarray
    pre_grasp_pose_6d: np.ndarray
    grasp_pose_6d: np.ndarray
    drop_high_pose_6d: np.ndarray
    drop_pose_6d: np.ndarray
    steps: list[PlanStep]
    debug_info: dict = field(default_factory=dict)


def build_drop_pose_6d(grasp_pose_6d, drop_position=DROP_POSITION, release_z=None):
    grasp_pose_6d = _ensure_pose_array(grasp_pose_6d)
    drop_pose_6d = grasp_pose_6d.copy()
    drop_position = np.asarray(drop_position, dtype=float)
    drop_pose_6d[:2] = drop_position[:2]
    drop_pose_6d[2] = (
        float(release_z)
        if release_z is not None
        else float(grasp_pose_6d[2] + DROP_RELEASE_Z_OFFSET)
    )
    return drop_pose_6d


def build_drop_high_pose_6d(drop_pose_6d, minimum_z=None):
    drop_high_pose_6d = drop_pose_6d.copy()
    drop_high_pose_6d[2] += GRASP_LIFT_HEIGHT
    if minimum_z is not None:
        drop_high_pose_6d[2] = max(
            drop_high_pose_6d[2],
            float(minimum_z),
        )
    return drop_high_pose_6d


def build_pre_grasp_pose_6d(grasp_pose_6d):
    return get_pregrasp_pose(grasp_pose_6d, -APPROACH_DIST)


def build_lift_pose_6d(grasp_pose_6d):
    grasp_pose_6d = _ensure_pose_array(grasp_pose_6d)
    lift_pose_6d = grasp_pose_6d.copy()
    lift_pose_6d[2] += GRASP_LIFT_HEIGHT
    return lift_pose_6d


def moveit_step(name, start_pose_6d, end_pose_6d):
    return PlanStep(
        name=name,
        action="move",
        mode="moveit",
        start_pose_6d=start_pose_6d,
        end_pose_6d=end_pose_6d,
    )


def cartesian_step(name, start_pose_6d, end_pose_6d):
    return PlanStep(
        name=name,
        action="move",
        mode="cartesian",
        start_pose_6d=start_pose_6d,
        end_pose_6d=end_pose_6d,
    )


def gripper_step(name, gripper_position):
    return PlanStep(
        name=name,
        action="gripper",
        gripper_position=float(gripper_position),
    )


def hold_step(name, pose_6d, duration_sec, mode=""):
    return PlanStep(
        name=name,
        action="hold",
        mode=mode,
        pose_6d=pose_6d,
        duration_sec=float(duration_sec),
    )


def plan_safe_pick_place_steps(
    start_pose_6d,
    grasp_pose_6d,
    drop_pose_6d=None,
    drop_high_pose_6d=None,
    direct_to_grasp=False,
    debug_info=None,
    execution_mode=GRASP_EXECUTION_MODE,
):
    execution_mode = str(execution_mode).strip().lower()
    if execution_mode not in GRASP_EXECUTION_MODES:
        valid_modes = ", ".join(sorted(GRASP_EXECUTION_MODES))
        raise ValueError(
            "execution_mode must be one of "
            f"{valid_modes}; got {execution_mode!r}."
        )
    grasp_pose_6d = _ensure_pose_array(grasp_pose_6d)
    # ``direct_to_grasp`` is a legacy side-profile flag. It still preserves
    # the existing side transfer/drop behavior below, but it must never skip
    # the collision-aware pregrasp and final Cartesian approach.
    pre_grasp_pose_6d = build_pre_grasp_pose_6d(grasp_pose_6d)
    pregrasp_distance = float(
        np.linalg.norm(
            _ensure_pose_array(grasp_pose_6d)[:3]
            - pre_grasp_pose_6d[:3]
        )
    )
    if APPROACH_DIST > 0.0 and pregrasp_distance < 0.05:
        raise ValueError(
            "Side pre-grasp generation failed: "
            f"configured approach offset is {APPROACH_DIST:.4f} m, but "
            f"pregrasp distance is only {pregrasp_distance:.4f} m."
        )
    lift_pose_6d = build_lift_pose_6d(grasp_pose_6d)

    if execution_mode == "lift_return":
        return_pose_6d = grasp_pose_6d.copy()
        return_pose_6d[2] += GRASP_RETURN_RELEASE_CLEARANCE_M
        steps = [
            gripper_step("open_gripper_before_approach", GRIPPER_OPEN_POSITION),
            moveit_step("move_to_pre_grasp", start_pose_6d, pre_grasp_pose_6d),
            cartesian_step("approach_grasp", pre_grasp_pose_6d, grasp_pose_6d),
            gripper_step("close_gripper_at_grasp", GRIPPER_CLOSED_POSITION),
            hold_step(
                "hold_after_close",
                grasp_pose_6d,
                GRIPPER_CLOSE_SETTLE_SEC,
                mode="passive" if direct_to_grasp else "",
            ),
            cartesian_step("lift_after_grasp", grasp_pose_6d, lift_pose_6d),
            hold_step(
                "hold_after_lift",
                lift_pose_6d,
                GRASP_LIFT_HOLD_SEC,
            ),
            cartesian_step("return_to_grasp", lift_pose_6d, return_pose_6d),
            hold_step(
                "hold_before_release",
                return_pose_6d,
                RELEASE_PRE_OPEN_HOLD_SEC,
            ),
            gripper_step("open_gripper_to_release", GRIPPER_OPEN_POSITION),
            hold_step(
                "hold_after_release",
                return_pose_6d,
                GRIPPER_SETTLE_SEC,
                mode="passive" if direct_to_grasp else "",
            ),
            cartesian_step(
                "retreat_after_release",
                return_pose_6d,
                pre_grasp_pose_6d,
            ),
        ]
        return PickPlacePlan(
            start_pose_6d=start_pose_6d,
            pre_grasp_pose_6d=pre_grasp_pose_6d,
            grasp_pose_6d=grasp_pose_6d,
            drop_high_pose_6d=lift_pose_6d,
            drop_pose_6d=return_pose_6d,
            steps=steps,
            debug_info={} if debug_info is None else dict(debug_info),
        )

    drop_pose_6d = (
        build_drop_pose_6d(grasp_pose_6d)
        if drop_pose_6d is None
        else drop_pose_6d
    )
    drop_high_pose_6d = (
        build_drop_high_pose_6d(
            drop_pose_6d,
            minimum_z=lift_pose_6d[2],
        )
        if drop_high_pose_6d is None
        else np.array(drop_high_pose_6d, dtype=float, copy=True)
    )
    drop_high_pose_6d[2] = max(drop_high_pose_6d[2], lift_pose_6d[2])

    if direct_to_grasp:
        steps = [
            gripper_step("open_gripper_before_approach", GRIPPER_OPEN_POSITION),
            moveit_step("move_to_pre_grasp", start_pose_6d, pre_grasp_pose_6d),
            cartesian_step("approach_grasp", pre_grasp_pose_6d, grasp_pose_6d),
            gripper_step("close_gripper_at_grasp", GRIPPER_CLOSED_POSITION),
            hold_step(
                "hold_after_close",
                grasp_pose_6d,
                GRIPPER_CLOSE_SETTLE_SEC,
                mode="passive",
            ),
            cartesian_step("lift_after_grasp", grasp_pose_6d, lift_pose_6d),
            hold_step(
                "hold_after_lift",
                lift_pose_6d,
                GRASP_LIFT_HOLD_SEC,
            ),
            moveit_step("transfer_to_drop_high", lift_pose_6d, drop_high_pose_6d),
            hold_step(
                "hold_before_descend",
                drop_high_pose_6d,
                DROP_HIGH_HOLD_SEC,
                mode="passive",
            ),
            cartesian_step("descend_to_drop", drop_high_pose_6d, drop_pose_6d),
            hold_step("hold_before_release", drop_pose_6d, RELEASE_PRE_OPEN_HOLD_SEC),
            gripper_step("open_gripper_to_release", GRIPPER_OPEN_POSITION),
            hold_step(
                "hold_after_release",
                drop_pose_6d,
                GRIPPER_SETTLE_SEC,
                mode="passive",
            ),
            cartesian_step("retreat_from_drop", drop_pose_6d, drop_high_pose_6d),
        ]
    else:
        steps = [
            gripper_step("open_gripper_before_approach", GRIPPER_OPEN_POSITION),
            moveit_step("move_to_pre_grasp", start_pose_6d, pre_grasp_pose_6d),
            cartesian_step("approach_grasp", pre_grasp_pose_6d, grasp_pose_6d),
            gripper_step("close_gripper_at_grasp", GRIPPER_CLOSED_POSITION),
            hold_step("hold_after_close", grasp_pose_6d, GRIPPER_CLOSE_SETTLE_SEC),
            cartesian_step("lift_after_grasp", grasp_pose_6d, lift_pose_6d),
            hold_step("hold_after_lift", lift_pose_6d, GRASP_LIFT_HOLD_SEC),
            moveit_step("transfer_to_drop_high", lift_pose_6d, drop_high_pose_6d),
            hold_step("hold_before_descend", drop_high_pose_6d, DROP_HIGH_HOLD_SEC),
            cartesian_step("descend_to_drop", drop_high_pose_6d, drop_pose_6d),
            hold_step("hold_before_release", drop_pose_6d, RELEASE_PRE_OPEN_HOLD_SEC),
            gripper_step("open_gripper_to_release", GRIPPER_OPEN_POSITION),
            hold_step("hold_after_release", drop_pose_6d, GRIPPER_SETTLE_SEC),
            cartesian_step("retreat_from_drop", drop_pose_6d, drop_high_pose_6d),
        ]

    return PickPlacePlan(
        start_pose_6d=start_pose_6d,
        pre_grasp_pose_6d=pre_grasp_pose_6d,
        grasp_pose_6d=grasp_pose_6d,
        drop_high_pose_6d=drop_high_pose_6d,
        drop_pose_6d=drop_pose_6d,
        steps=steps,
        debug_info={} if debug_info is None else dict(debug_info),
    )


def print_plan_summary(plan):
    for idx, step in enumerate(plan.steps):
        if step.action == "move":
            print(
                f"step[{idx}] {step.name}: move "
                f"{pose_text(step.start_pose_6d)} -> {pose_text(step.end_pose_6d)}"
            )
        elif step.action == "gripper":
            print(f"step[{idx}] {step.name}: gripper {step.gripper_position:.3f}")
        elif step.action == "hold":
            print(
                f"step[{idx}] {step.name}: hold {step.duration_sec:.2f}s "
                f"at {pose_text(step.pose_6d)}"
            )
        else:
            print(f"step[{idx}] {step.name}: unknown action {step.action}")
