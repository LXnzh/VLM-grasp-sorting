from dataclasses import dataclass, field, replace
import math
from types import MappingProxyType

import numpy as np
from sim_pick_place.utils.helpers import _ensure_pose_array, _transform_matrix_to_pose6d
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
    TUNA_LIFT_MICROSEGMENT_MAX_TRANSLATION_M,
    TUNA_LIFT_OBSERVATION_SPACING_M,
    TUNA_MAX_PHYSICAL_COMMANDS,
    TUNA_POST_CLOSE_HOLD_SEC,
    TUNA_ROLL_CHECKPOINT_ANGLES_DEG,
    TUNA_ROLL_MICROSEGMENT_MAX_ANGLE_DEG,
    TUNA_TEST_LIFT_M,
)
from my_course_pkg.grasp.tuna_roll_grasp import (
    FinalizationCommitMetadata,
    FrozenPivotContract,
    generate_lift_waypoints,
    generate_roll_waypoints,
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


@dataclass(frozen=True)
class DeferredTunaSuffix:
    object_name: str
    base_plan_revision: int
    prefix_step_count: int
    final_roll_angle_deg: float


@dataclass(frozen=True)
class TunaPreparedStep:
    name: str
    action: str
    mode: str = ""
    start_pose_6d: np.ndarray | None = None
    end_pose_6d: np.ndarray | None = None
    pose_6d: np.ndarray | None = None
    gripper_position: float | None = None
    duration_sec: float = 0.0

    def __post_init__(self):
        for field_name in ("start_pose_6d", "end_pose_6d", "pose_6d"):
            value = getattr(self, field_name)
            if value is None:
                continue
            array = np.array(value, dtype=float, copy=True)
            if array.shape != (6,) or not np.isfinite(array).all():
                raise ValueError(f"{field_name} must be a finite six-vector.")
            array.setflags(write=False)
            object.__setattr__(self, field_name, array)


@dataclass(frozen=True)
class PreparedTunaSuffix:
    object_name: str
    base_plan_revision: int
    steps: tuple[TunaPreparedStep, ...]
    debug_updates: object
    physical_command_count: int


@dataclass
class PickPlacePlan:
    start_pose_6d: np.ndarray
    pre_grasp_pose_6d: np.ndarray
    grasp_pose_6d: np.ndarray
    drop_high_pose_6d: np.ndarray
    drop_pose_6d: np.ndarray
    steps: list[PlanStep]
    debug_info: dict = field(default_factory=dict)
    deferred_tuna_suffix: DeferredTunaSuffix | None = None
    plan_revision: int = 0


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
        mode = "cartesian",
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


def observe_tuna_step(name):
    return PlanStep(name=name, action="observe_tuna_bounds")


def _freeze_tuna_debug_value(value):
    if isinstance(value, np.ndarray):
        array = np.array(value, dtype=float, copy=True)
        if not np.isfinite(array).all():
            raise ValueError("Tuna debug arrays must contain only finite values.")
        array.setflags(write=False)
        return array
    if isinstance(value, dict):
        return MappingProxyType(
            {key: _freeze_tuna_debug_value(item) for key, item in value.items()}
        )
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_tuna_debug_value(item) for item in value)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not np.isfinite(value):
            raise ValueError("Tuna debug floats must be finite.")
        return value
    raise TypeError(f"unsupported Tuna debug value: {type(value).__name__}")


def _thaw_tuna_debug_value(value):
    if isinstance(value, np.ndarray):
        return np.array(value, dtype=float, copy=True)
    if isinstance(value, MappingProxyType):
        return {key: _thaw_tuna_debug_value(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw_tuna_debug_value(item) for item in value]
    return value


def build_tuna_preclamp_plan(
    *,
    start_pose_6d,
    pregrasp_pose_6d,
    contact_support_pose_6d,
    preclamp_position,
    final_roll_angle_deg=30.0,
    debug_info=None,
):
    """Build the only executable prefix allowed before pivot finalization."""
    start_pose = _ensure_pose_array(start_pose_6d)
    pregrasp_pose = _ensure_pose_array(pregrasp_pose_6d)
    contact_pose = _ensure_pose_array(contact_support_pose_6d)
    preclamp = float(preclamp_position)
    final_angle = float(final_roll_angle_deg)
    if (
        not np.isfinite(preclamp)
        or not GRIPPER_OPEN_POSITION < preclamp < GRIPPER_CLOSED_POSITION
    ):
        raise ValueError(
            "Tuna preclamp position must be explicitly calibrated and strictly "
            "between the shared open and closed commands."
        )
    if not np.isfinite(final_angle) or not 20.0 < final_angle <= 35.0:
        raise ValueError("Tuna final roll angle must be within (20, 35] degrees.")
    debug = {} if debug_info is None else dict(debug_info)
    debug.update(
        {
            "object_name": "tuna_fish_can",
            "grasp_profile": "tuna_roll_up",
            "tuna_policy": "roll_up",
            "tuna_preclamp_position": preclamp,
            "tuna_final_roll_angle_deg": final_angle,
            "tuna_observations": {
                "observe_tuna_after_preclamp": {
                    "stage": "preclamp",
                    "reference": "initial_bounds",
                }
            },
            "tuna_hold_contract_timeline": [
                {
                    "target": "preclamp",
                    "starts_after": "preclamp_tuna",
                    "ends_before": "close_tuna_after_roll",
                },
                {
                    "target": "full_close",
                    "starts_after": "close_tuna_after_roll",
                    "ends_after": "normal_lift",
                },
            ],
        }
    )
    steps = [
        gripper_step("open_gripper_before_tuna_approach", GRIPPER_OPEN_POSITION),
        moveit_step("move_to_tuna_pregrasp", start_pose, pregrasp_pose),
        cartesian_step(
            "approach_tuna_contact_support",
            pregrasp_pose,
            contact_pose,
        ),
        gripper_step("preclamp_tuna", preclamp),
        observe_tuna_step("observe_tuna_after_preclamp"),
    ]
    descriptor = DeferredTunaSuffix(
        object_name="tuna_fish_can",
        base_plan_revision=0,
        prefix_step_count=len(steps),
        final_roll_angle_deg=final_angle,
    )
    return PickPlacePlan(
        start_pose_6d=start_pose,
        pre_grasp_pose_6d=pregrasp_pose,
        grasp_pose_6d=contact_pose,
        drop_high_pose_6d=contact_pose.copy(),
        drop_pose_6d=contact_pose.copy(),
        steps=steps,
        debug_info=debug,
        deferred_tuna_suffix=descriptor,
        plan_revision=0,
    )


def _prepared_step_from_plan_step(step):
    return TunaPreparedStep(
        name=step.name,
        action=step.action,
        mode=step.mode,
        start_pose_6d=step.start_pose_6d,
        end_pose_6d=step.end_pose_6d,
        pose_6d=step.pose_6d,
        gripper_position=step.gripper_position,
        duration_sec=step.duration_sec,
    )


def _plan_step_from_prepared(step):
    return PlanStep(
        name=step.name,
        action=step.action,
        mode=step.mode,
        start_pose_6d=(
            None if step.start_pose_6d is None else np.array(step.start_pose_6d, copy=True)
        ),
        end_pose_6d=(
            None if step.end_pose_6d is None else np.array(step.end_pose_6d, copy=True)
        ),
        pose_6d=None if step.pose_6d is None else np.array(step.pose_6d, copy=True),
        gripper_position=step.gripper_position,
        duration_sec=step.duration_sec,
    )


def _roll_stage_end_angles(final_angle_deg):
    endpoints = [
        angle
        for angle in TUNA_ROLL_CHECKPOINT_ANGLES_DEG[:-1]
        if angle < final_angle_deg
    ]
    endpoints.append(float(final_angle_deg))
    if len(endpoints) != 3:
        raise ValueError(
            "The first Tuna patch requires three visible roll stages ending "
            "above 20 and at or below 35 degrees."
        )
    return tuple(endpoints)


def _tuna_pose6d_from_transform(transform):
    # scipy's Rotation wrapper rejects read-only memoryviews; Tuna geometry
    # intentionally freezes its matrices, so convert through a private copy.
    return _transform_matrix_to_pose6d(np.array(transform, dtype=float, copy=True))


def _pose_list_from_waypoints(waypoints):
    return tuple(
        _tuna_pose6d_from_transform(waypoint.T_world_tcp)
        for waypoint in waypoints
    )


def _split_normal_lift_observation_distances(total_lift_m, test_lift_m, spacing_m):
    remaining = float(total_lift_m) - float(test_lift_m)
    if remaining < -1e-12:
        raise ValueError("normal lift height cannot be below the Tuna test lift.")
    if remaining <= 1e-12:
        return ()
    count = int(math.ceil(remaining / float(spacing_m)))
    increment = remaining / count
    return tuple(float(test_lift_m) + increment * index for index in range(1, count + 1))


def build_prepared_tuna_suffix(
    *,
    plan,
    pivot_contract,
    roll_axis_world,
    total_lift_m=GRASP_LIFT_HEIGHT,
    execution_mode=GRASP_EXECUTION_MODE,
    drop_pose_6d=None,
    max_physical_commands=TUNA_MAX_PHYSICAL_COMMANDS,
):
    """Purely prepare a validated-shape suffix; this never mutates ``plan``."""
    if not isinstance(plan, PickPlacePlan):
        raise TypeError("plan must be a PickPlacePlan.")
    descriptor = plan.deferred_tuna_suffix
    if not isinstance(descriptor, DeferredTunaSuffix):
        raise ValueError("Tuna plan has no uncommitted deferred suffix.")
    if descriptor.object_name != "tuna_fish_can" or plan.debug_info.get("object_name") != "tuna_fish_can":
        raise ValueError("Tuna suffix preparation requires exact tuna_fish_can identity.")
    if plan.plan_revision != descriptor.base_plan_revision:
        raise ValueError("Tuna plan revision changed before suffix preparation.")
    if not isinstance(pivot_contract, FrozenPivotContract):
        raise TypeError("pivot_contract must be a FrozenPivotContract.")

    prepared_steps = []
    micro_waypoints = {}
    observations = {}
    physical_commands = descriptor.prefix_step_count - 1  # observation is not motion
    previous_angle = 0.0
    previous_pose = _tuna_pose6d_from_transform(
        pivot_contract.T_world_tcp_preclamp
    )
    for stage_index, end_angle in enumerate(
        _roll_stage_end_angles(descriptor.final_roll_angle_deg),
        start=1,
    ):
        waypoints = generate_roll_waypoints(
            pivot_contract,
            roll_axis_world,
            end_angle,
            TUNA_ROLL_MICROSEGMENT_MAX_ANGLE_DEG,
            start_angle_deg=previous_angle,
        )
        waypoint_poses = _pose_list_from_waypoints(waypoints)
        step_name = f"roll_tuna_segment_{stage_index:02d}_of_03"
        observe_name = f"observe_tuna_after_roll_segment_{stage_index:02d}_of_03"
        end_pose = waypoint_poses[-1]
        prepared_steps.append(
            TunaPreparedStep(
                name=step_name,
                action="move",
                mode="cartesian",
                start_pose_6d=previous_pose,
                end_pose_6d=end_pose,
            )
        )
        prepared_steps.append(
            TunaPreparedStep(name=observe_name, action="observe_tuna_bounds")
        )
        micro_waypoints[step_name] = waypoint_poses
        observations[observe_name] = {
            "stage": f"roll_segment_{stage_index}",
            "start_angle_deg": previous_angle,
            "end_angle_deg": end_angle,
        }
        physical_commands += len(waypoints)
        previous_pose = end_pose
        previous_angle = end_angle

    rolled_close_pose = np.array(previous_pose, dtype=float, copy=True)
    close_step = TunaPreparedStep(
        name="close_tuna_after_roll",
        action="gripper",
        gripper_position=GRIPPER_CLOSED_POSITION,
    )
    prepared_steps.extend(
        [
            close_step,
            TunaPreparedStep(
                name="hold_tuna_after_close",
                action="hold",
                mode="passive",
                pose_6d=rolled_close_pose,
                duration_sec=TUNA_POST_CLOSE_HOLD_SEC,
            ),
            TunaPreparedStep(
                name="observe_tuna_after_close",
                action="observe_tuna_bounds",
            ),
        ]
    )
    observations["observe_tuna_after_close"] = {
        "stage": "close",
        "expected_angle_deg": descriptor.final_roll_angle_deg,
    }
    physical_commands += 2

    rolled_transform = np.array(
        generate_roll_waypoints(
            pivot_contract,
            roll_axis_world,
            descriptor.final_roll_angle_deg,
            descriptor.final_roll_angle_deg,
        )[-1].T_world_tcp,
        copy=True,
    )
    test_waypoints = generate_lift_waypoints(
        rolled_transform,
        TUNA_TEST_LIFT_M,
        TUNA_LIFT_MICROSEGMENT_MAX_TRANSLATION_M,
    )
    test_poses = _pose_list_from_waypoints(test_waypoints)
    test_step_name = "test_lift_tuna_30mm"
    prepared_steps.extend(
        [
            TunaPreparedStep(
                name=test_step_name,
                action="move",
                mode="cartesian",
                start_pose_6d=rolled_close_pose,
                end_pose_6d=test_poses[-1],
            ),
            TunaPreparedStep(
                name="observe_tuna_after_test_lift",
                action="observe_tuna_bounds",
            ),
        ]
    )
    micro_waypoints[test_step_name] = test_poses
    observations["observe_tuna_after_test_lift"] = {
        "stage": "test_lift",
        "expected_world_z_delta_m": TUNA_TEST_LIFT_M,
    }
    physical_commands += len(test_waypoints)

    normal_distances = _split_normal_lift_observation_distances(
        total_lift_m,
        TUNA_TEST_LIFT_M,
        TUNA_LIFT_OBSERVATION_SPACING_M,
    )
    previous_distance = TUNA_TEST_LIFT_M
    previous_lift_pose = test_poses[-1]
    for segment_index, end_distance in enumerate(normal_distances, start=1):
        segment_distance = end_distance - previous_distance
        segment_start_transform = np.array(rolled_transform, copy=True)
        segment_start_transform[2, 3] += previous_distance
        waypoints = generate_lift_waypoints(
            segment_start_transform,
            segment_distance,
            TUNA_LIFT_MICROSEGMENT_MAX_TRANSLATION_M,
        )
        waypoint_poses = _pose_list_from_waypoints(waypoints)
        total_segments = len(normal_distances)
        step_name = (
            f"normal_lift_tuna_segment_{segment_index:02d}_of_{total_segments:02d}"
        )
        observe_name = (
            "observe_tuna_after_normal_lift_segment_"
            f"{segment_index:02d}_of_{total_segments:02d}"
        )
        prepared_steps.extend(
            [
                TunaPreparedStep(
                    name=step_name,
                    action="move",
                    mode="cartesian",
                    start_pose_6d=previous_lift_pose,
                    end_pose_6d=waypoint_poses[-1],
                ),
                TunaPreparedStep(
                    name=observe_name,
                    action="observe_tuna_bounds",
                ),
            ]
        )
        micro_waypoints[step_name] = waypoint_poses
        observations[observe_name] = {
            "stage": "normal_lift",
            "expected_world_z_delta_m": end_distance,
        }
        physical_commands += len(waypoints)
        previous_distance = end_distance
        previous_lift_pose = waypoint_poses[-1]

    execution_mode = str(execution_mode).strip().lower()
    if execution_mode == "lift_return":
        release_pose = np.array(rolled_close_pose, copy=True)
        release_pose[2] += GRASP_RETURN_RELEASE_CLEARANCE_M
        rolled_retreat = np.array(release_pose, copy=True)
        release_transform = np.array(rolled_transform, copy=True)
        release_transform[:3, 3] = release_pose[:3]
        rolled_retreat[:3] -= APPROACH_DIST * release_transform[:3, 2]
        tail = [
            TunaPreparedStep(
                name="hold_after_tuna_normal_lift",
                action="hold",
                pose_6d=previous_lift_pose,
                duration_sec=GRASP_LIFT_HOLD_SEC,
            ),
            TunaPreparedStep(
                name="return_tuna_to_rolled_release",
                action="move",
                mode="cartesian",
                start_pose_6d=previous_lift_pose,
                end_pose_6d=release_pose,
            ),
            TunaPreparedStep(
                name="hold_before_tuna_release",
                action="hold",
                pose_6d=release_pose,
                duration_sec=RELEASE_PRE_OPEN_HOLD_SEC,
            ),
            TunaPreparedStep(
                name="open_gripper_to_release_tuna",
                action="gripper",
                gripper_position=GRIPPER_OPEN_POSITION,
            ),
            TunaPreparedStep(
                name="hold_after_tuna_release",
                action="hold",
                mode="passive",
                pose_6d=release_pose,
                duration_sec=GRIPPER_SETTLE_SEC,
            ),
            TunaPreparedStep(
                name="retreat_from_released_tuna",
                action="move",
                mode="cartesian",
                start_pose_6d=release_pose,
                end_pose_6d=rolled_retreat,
            ),
        ]
    elif execution_mode == "safe_place":
        if drop_pose_6d is None:
            raise ValueError("safe_place Tuna suffix requires an explicit drop pose.")
        drop_pose = _ensure_pose_array(drop_pose_6d)
        drop_pose[3:] = rolled_close_pose[3:]
        drop_high = np.array(drop_pose, copy=True)
        drop_high[2] = max(drop_high[2] + GRASP_LIFT_HEIGHT, previous_lift_pose[2])
        tail = [
            TunaPreparedStep(
                name="transfer_rolled_tuna_to_drop_high",
                action="move",
                mode="moveit",
                start_pose_6d=previous_lift_pose,
                end_pose_6d=drop_high,
            ),
            TunaPreparedStep(
                name="descend_rolled_tuna_to_drop",
                action="move",
                mode="cartesian",
                start_pose_6d=drop_high,
                end_pose_6d=drop_pose,
            ),
            TunaPreparedStep(
                name="open_gripper_to_release_tuna",
                action="gripper",
                gripper_position=GRIPPER_OPEN_POSITION,
            ),
            TunaPreparedStep(
                name="retreat_after_rolled_tuna_drop",
                action="move",
                mode="cartesian",
                start_pose_6d=drop_pose,
                end_pose_6d=drop_high,
            ),
        ]
    else:
        raise ValueError(f"unsupported Tuna execution mode: {execution_mode!r}")
    prepared_steps.extend(tail)
    physical_commands += len(tail)

    maximum_commands = int(max_physical_commands)
    if physical_commands > maximum_commands:
        raise ValueError(
            "Tuna physical command budget exceeds the fail-closed cap: "
            f"planned={physical_commands}, cap={maximum_commands}."
        )
    debug_updates = _freeze_tuna_debug_value(
        {
            "tuna_micro_waypoints": micro_waypoints,
            "tuna_observations": observations,
            "tuna_roll_segment_count": 3,
            "tuna_normal_lift_segment_count": len(normal_distances),
            "tuna_physical_command_budget": {
                "planned": physical_commands,
                "cap": maximum_commands,
            },
        }
    )
    return PreparedTunaSuffix(
        object_name="tuna_fish_can",
        base_plan_revision=plan.plan_revision,
        steps=tuple(prepared_steps),
        debug_updates=debug_updates,
        physical_command_count=physical_commands,
    )


def commit_prepared_tuna_suffix(plan, prepared_suffix, commit_metadata):
    """Return a new plan with one all-or-nothing Tuna suffix commit."""
    if not isinstance(plan, PickPlacePlan):
        raise TypeError("plan must be a PickPlacePlan.")
    if not isinstance(prepared_suffix, PreparedTunaSuffix):
        raise TypeError("prepared_suffix must be a PreparedTunaSuffix.")
    if not isinstance(commit_metadata, FinalizationCommitMetadata):
        raise TypeError("commit_metadata must be FinalizationCommitMetadata.")
    descriptor = plan.deferred_tuna_suffix
    if not isinstance(descriptor, DeferredTunaSuffix):
        raise ValueError("Tuna suffix is absent or was already committed.")
    if (
        prepared_suffix.object_name != "tuna_fish_can"
        or descriptor.object_name != "tuna_fish_can"
        or plan.debug_info.get("object_name") != "tuna_fish_can"
    ):
        raise ValueError("Tuna suffix commit requires exact tuna_fish_can identity.")
    if (
        plan.plan_revision != prepared_suffix.base_plan_revision
        or plan.plan_revision != descriptor.base_plan_revision
        or commit_metadata.plan_revision != plan.plan_revision
    ):
        raise ValueError("Tuna suffix commit revision mismatch.")
    if len(plan.steps) != descriptor.prefix_step_count:
        raise ValueError("Tuna executable prefix changed before suffix commit.")
    if "tuna_suffix_commit" in plan.debug_info:
        raise ValueError("Tuna suffix already has commit metadata.")

    debug_info = dict(plan.debug_info)
    debug_updates = _thaw_tuna_debug_value(prepared_suffix.debug_updates)
    existing_observations = dict(debug_info.get("tuna_observations", {}))
    existing_observations.update(debug_updates.pop("tuna_observations"))
    debug_info.update(debug_updates)
    debug_info["tuna_observations"] = existing_observations
    debug_info["tuna_suffix_commit"] = {
        "suffix_generation_id": commit_metadata.suffix_generation_id,
        "attempt_count": commit_metadata.attempt_count,
        "pivot_contract_sha256": commit_metadata.pivot_contract_sha256,
        "base_plan_revision": commit_metadata.plan_revision,
        "committed_plan_revision": plan.plan_revision + 1,
    }
    committed_steps = list(plan.steps) + [
        _plan_step_from_prepared(step) for step in prepared_suffix.steps
    ]
    return replace(
        plan,
        steps=committed_steps,
        debug_info=debug_info,
        deferred_tuna_suffix=None,
        plan_revision=plan.plan_revision + 1,
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
