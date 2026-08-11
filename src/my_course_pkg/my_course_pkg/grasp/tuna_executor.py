from dataclasses import dataclass
import math
import time
import uuid

import numpy as np
from sim_pick_place.utils.helpers import _pose6d_to_transform_matrix

from my_course_pkg.grasp.config import (
    GRASP_EXECUTION_MODE,
    GRIPPER_CLOSED_POSITION,
    GRIPPER_OPEN_POSITION,
    read_tuna_config,
)
from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.tuna_finalizer import (
    TunaFinalizer,
    finalize_tuna_post_preclamp,
)
from my_course_pkg.grasp.trajectory_planner import commit_prepared_tuna_suffix
from my_course_pkg.grasp.tuna_roll_grasp import (
    FinalizationCommitMetadata,
    aperture_gate,
    classify_roll_residual,
    cylinder_aabb_height,
    evaluate_contact_limited_close,
    evaluate_initial_straddle,
    evaluate_lower_finger_access,
    freeze_pivot_contract,
    freeze_retention_contract,
    interpolate_gripper_geometry,
    required_width,
    support_hull_clearance_lower_bound,
    validate_qpos_samples,
    validate_tuna_observation_samples,
)


TUNA_FINALIZATION_RETRY_DELAY_SEC = 0.5


@dataclass(frozen=True)
class TunaExecutionOutcome:
    plan: object
    stopped_after: str
    physical_command_count: int


def _rotation_about_axis(axis, angle_rad):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3)
        + math.sin(angle_rad) * skew
        + (1.0 - math.cos(angle_rad)) * (skew @ skew)
    )


class TunaExecutionCoordinator:
    """Exact-name Tuna protocol layered over the existing arm executor.

    ``runtime`` is deliberately a narrow duck-typed adapter.  Non-Tuna plans
    never construct this class and continue through the legacy executor path.
    """

    def __init__(self, runtime, calibration, finalizer=None, sleep=time.sleep):
        self.runtime = runtime
        self.calibration = calibration
        self.finalizer = (
            TunaFinalizer(runtime.node) if finalizer is None else finalizer
        )
        self.sleep = sleep
        self.config = read_tuna_config()
        self.command_count = 0
        self.current_gripper_target = None
        self.last_bounds_source_timestamp = 0.0
        self.initial_center = None
        self.initial_size = None
        self.latest_observation = None
        self.preclamp_observation = None
        self.rolled_observation = None
        self.pivot_contract = None
        self.retention_contract = None

    @staticmethod
    def _exact_tuna(plan):
        return getattr(plan, "debug_info", {}).get("object_name") == "tuna_fish_can"

    def _error(self, code, stage, reason, **diagnostics):
        content = {
            "object_name": "tuna_fish_can",
            "reason": str(reason),
            "physical_command_count": int(self.command_count),
        }
        content.update(diagnostics)
        return TunaGraspError(code, stage, content)

    def _log(self, event, **details):
        logger = getattr(self.runtime, "log_tuna_event", None)
        if callable(logger):
            logger(event, details)

    def _confirm(self, stage):
        callback = getattr(self.runtime, "confirm_tuna_stage", None)
        if not callable(callback) or not bool(callback(stage)):
            raise self._error(
                TunaErrorCode.MOTION,
                stage,
                "operator_confirmation_missing_or_rejected",
            )

    def _consume_command_budget(self, step_name):
        if self.command_count >= self.config.max_physical_commands:
            raise self._error(
                TunaErrorCode.COMMAND_BUDGET,
                step_name,
                "physical_command_cap_reached",
                cap=int(self.config.max_physical_commands),
            )
        self.command_count += 1

    def _execute_step(self, step):
        if step.action in {"move", "gripper", "hold"}:
            self._consume_command_budget(step.name)
        result = self.runtime.execute_tuna_step(step)
        if step.action == "gripper":
            self.current_gripper_target = float(step.gripper_position)
        return result

    def _execute_micro_pose(self, pose_6d, step_name):
        self._consume_command_budget(step_name)
        if not bool(self.runtime.execute_tuna_micro_pose(pose_6d, step_name)):
            raise self._error(
                TunaErrorCode.MOTION,
                step_name,
                "micro_waypoint_failed_or_did_not_converge",
            )

    def _collect_qpos(self, boundary, stage):
        samples = self.runtime.collect_tuna_qpos_samples(
            required_count=3,
            command_boundary=float(boundary),
        )
        try:
            return validate_qpos_samples(
                samples,
                boundary,
                self.config.qpos_stability_tolerance_rad,
            )
        except (TypeError, ValueError) as exc:
            raise self._error(
                TunaErrorCode.PIVOT_INVALIDATED,
                stage,
                str(exc),
            ) from exc

    def _collect_observation(self, stage, required_count=2):
        reception_boundary = time.monotonic()
        samples = self.runtime.collect_tuna_bounds_samples(
            required_count=required_count,
            reception_boundary=reception_boundary,
        )
        try:
            observation = validate_tuna_observation_samples(
                samples,
                required_count=required_count,
                source_boundary=self.last_bounds_source_timestamp,
                reception_boundary=reception_boundary,
                stability_tolerance_m=self.config.bounds_tolerance_m,
            )
        except (TypeError, ValueError) as exc:
            raise self._error(TunaErrorCode.BOUNDS, stage, str(exc)) from exc
        self.last_bounds_source_timestamp = observation.source_timestamps[-1]
        self.latest_observation = observation
        return observation

    def _check_preclamp_observation(self, observation):
        center_residual = observation.center - self.initial_center
        size_residual = observation.size - self.initial_size
        residual = float(
            max(np.max(np.abs(center_residual)), np.max(np.abs(size_residual)))
        )
        if residual > self.config.preclamp_bounds_tolerance_m:
            raise self._error(
                TunaErrorCode.BOUNDS,
                "preclamp",
                "preclamp_displacement",
                residual_m=residual,
                tolerance_m=self.config.preclamp_bounds_tolerance_m,
            )
        self.preclamp_observation = observation

    def _check_roll_observation(self, observation, end_angle_deg, roll_axis):
        pivot = self.pivot_contract.T_world_pivot[:3, 3]
        rotation = _rotation_about_axis(roll_axis, math.radians(end_angle_deg))
        expected_center = pivot + rotation @ (self.initial_center - pivot)
        center_residual = observation.center - expected_center
        radial = np.asarray(
            self.plan.debug_info["tuna_radial_direction_world"], dtype=float
        )
        tangent = np.asarray(roll_axis, dtype=float)
        radial_residual = float(center_residual @ radial)
        tangential_residual = float(center_residual @ tangent)
        vertical_residual = float(center_residual[2])
        expected_height = cylinder_aabb_height(
            self.initial_size[2],
            0.5 * (self.initial_size[0] + self.initial_size[1]),
            end_angle_deg,
        )
        expected_increase = expected_height - self.initial_size[2]
        actual_increase = float(observation.size[2] - self.initial_size[2])
        classification = classify_roll_residual(
            radial_residual,
            tangential_residual,
            vertical_residual,
            expected_increase,
            actual_increase,
            self.config.bounds_tolerance_m,
        )
        height_residual = abs(float(observation.size[2]) - expected_height)
        if classification != "success" or height_residual > self.config.bounds_tolerance_m:
            raise self._error(
                TunaErrorCode.BOUNDS,
                "roll",
                classification if classification != "success" else "height_residual",
                end_angle_deg=float(end_angle_deg),
                radial_residual_m=radial_residual,
                tangential_residual_m=tangential_residual,
                vertical_residual_m=vertical_residual,
                height_residual_m=height_residual,
                tolerance_m=self.config.bounds_tolerance_m,
            )

    def _check_close_observation(self, observation):
        reference = self.latest_roll_observation
        residual = float(
            max(
                np.max(np.abs(observation.center - reference.center)),
                np.max(np.abs(observation.size - reference.size)),
            )
        )
        if residual > self.config.bounds_tolerance_m:
            raise self._error(
                TunaErrorCode.RETENTION,
                "close",
                "bounds_changed_during_close",
                residual_m=residual,
                tolerance_m=self.config.bounds_tolerance_m,
            )
        self.rolled_observation = observation

    def _check_lift_observation(self, observation, expected_delta_m, stage):
        expected_center = self.rolled_observation.center.copy()
        expected_center[2] += float(expected_delta_m)
        center_residual = observation.center - expected_center
        size_residual = observation.size - self.rolled_observation.size
        residual = float(
            max(np.max(np.abs(center_residual)), np.max(np.abs(size_residual)))
        )
        if residual > self.config.lift_follow_tolerance_m:
            raise self._error(
                TunaErrorCode.RETENTION,
                stage,
                "object_did_not_follow_lift",
                residual_m=residual,
                tolerance_m=self.config.lift_follow_tolerance_m,
            )

    def _interpolated_gap(self, qpos):
        return interpolate_gripper_geometry(self.calibration, qpos)

    def _check_pivot_aperture(self, boundary, stage):
        batch = self._collect_qpos(boundary, stage)
        geometry = self._interpolated_gap(batch.median_qpos)
        gate = aperture_gate(
            geometry.pad_gap_m,
            self.pivot_contract.measured_pad_gap_m,
            self.config.pivot_aperture_drift_tolerance_m,
        )
        if not gate.passed:
            raise self._error(
                TunaErrorCode.PIVOT_INVALIDATED,
                stage,
                "pivot_aperture_drift",
                residual_m=gate.residual,
                tolerance_m=gate.limit,
            )

    def _check_retention_aperture(self, boundary, stage):
        batch = self._collect_qpos(boundary, stage)
        geometry = self._interpolated_gap(batch.median_qpos)
        gate = aperture_gate(
            geometry.pad_gap_m,
            self.retention_contract.measured_post_close_pad_gap_m,
            self.config.retention_aperture_drift_tolerance_m,
        )
        if not gate.passed:
            raise self._error(
                TunaErrorCode.RETENTION_APERTURE_DRIFT,
                stage,
                "retention_aperture_drift",
                residual_m=gate.residual,
                tolerance_m=gate.limit,
            )

    @staticmethod
    def _contact_result_compatible(result):
        if result is None or isinstance(result, bool):
            return False
        return bool(getattr(result, "accepted", False)) and bool(
            getattr(result, "stalled", False)
            or getattr(result, "reached_goal", False)
        )

    def _freeze_preclamp(self, command_result, command_boundary):
        samples = self.runtime.collect_tuna_qpos_samples(
            required_count=3,
            command_boundary=command_boundary,
        )
        T_world_tcp = self.runtime.current_tuna_tcp_transform()
        try:
            contract = freeze_pivot_contract(
                T_world_tcp,
                samples,
                command_boundary,
                self.calibration,
                self.config.qpos_stability_tolerance_rad,
                self.config.calibration_max_interpolation_error_m,
            )
        except (TypeError, ValueError) as exc:
            raise self._error(
                TunaErrorCode.PIVOT_INVALIDATED,
                "preclamp",
                str(exc),
            ) from exc
        free_geometry = self._interpolated_gap(self.config.preclamp_position)
        deflection = contract.measured_pad_gap_m - free_geometry.pad_gap_m
        if (
            deflection < self.config.min_contact_deflection_m
            or not self._contact_result_compatible(command_result)
        ):
            raise self._error(
                TunaErrorCode.CONTACT,
                "preclamp",
                "contact_evidence_failed",
                measured_deflection_m=float(deflection),
                required_deflection_m=self.config.min_contact_deflection_m,
            )
        geometry = self._interpolated_gap(contract.measured_qpos)
        clearance = support_hull_clearance_lower_bound(
            contract.T_world_tcp_preclamp,
            geometry.support_hull_tcp,
            float(self.plan.debug_info["tuna_table_z_m"]),
            geometry.interpolation_error_m,
        )
        if clearance < self.config.min_table_clearance_m:
            raise self._error(
                TunaErrorCode.COLLISION,
                "preclamp",
                "measured_aperture_table_clearance",
                clearance_m=clearance,
                required_clearance_m=self.config.min_table_clearance_m,
            )
        self.pivot_contract = contract

    def _fresh_finalization_evidence(self, attempt):
        observation = self._collect_observation(
            f"finalization_evidence_{attempt}", required_count=2
        )
        residual = float(
            max(
                np.max(np.abs(observation.center - self.preclamp_observation.center)),
                np.max(np.abs(observation.size - self.preclamp_observation.size)),
            )
        )
        if residual > self.config.preclamp_bounds_tolerance_m:
            raise self._error(
                TunaErrorCode.PIVOT_INVALIDATED,
                "finalize",
                "fresh_bounds_changed_after_preclamp",
                residual_m=residual,
                attempt=int(attempt),
            )
        boundary = time.monotonic()
        self._check_pivot_aperture(boundary, f"finalization_aperture_{attempt}")
        return observation.received_monotonic[-1]

    def _finalize_once_or_retry(self, plan, roll_axis):
        joint_names, joint_positions = self.runtime.current_tuna_joint_state()
        last_error = None
        for attempt in (1, 2):
            evidence_timestamp = self._fresh_finalization_evidence(attempt)
            try:
                preparation = finalize_tuna_post_preclamp(
                    self.finalizer,
                    plan=plan,
                    pivot_contract=self.pivot_contract,
                    current_joint_names=joint_names,
                    current_joint_positions=joint_positions,
                    roll_axis_world=roll_axis,
                    attempt=attempt,
                    evidence_after_timestamp=evidence_timestamp,
                    execution_mode=self.plan.debug_info.get(
                        "execution_mode", GRASP_EXECUTION_MODE
                    ),
                    drop_pose_6d=plan.drop_pose_6d,
                    max_physical_commands=self.config.max_physical_commands,
                )
                metadata = FinalizationCommitMetadata(
                    suffix_generation_id=str(uuid.uuid4()),
                    attempt_count=attempt,
                    pivot_contract_sha256=preparation.pivot_contract_sha256,
                    plan_revision=plan.plan_revision,
                )
                return commit_prepared_tuna_suffix(
                    plan,
                    preparation.prepared_suffix,
                    metadata,
                )
            except TunaGraspError as exc:
                last_error = exc
                if not exc.retryable or attempt == 2:
                    raise
                self._log(
                    "finalization_transient_retry",
                    attempt=attempt,
                    delay_sec=TUNA_FINALIZATION_RETRY_DELAY_SEC,
                    error=exc.to_dict(),
                )
                self.sleep(TUNA_FINALIZATION_RETRY_DELAY_SEC)
        raise last_error

    def _validate_close_preconditions(self):
        height = float(self.initial_size[2])
        diameter = float(0.5 * (self.initial_size[0] + self.initial_size[1]))
        delta = float(self.plan.debug_info["tuna_pitch_deg"])
        projected = required_width(height, diameter, delta)
        open_geometry = self._interpolated_gap(GRIPPER_OPEN_POSITION)
        straddle = evaluate_initial_straddle(
            open_geometry.pad_gap_m,
            projected,
            self.config.min_straddle_margin_m,
        )
        lower_access = evaluate_lower_finger_access(
            float(self.plan.debug_info["tuna_clearance_lower_bound_m"]),
            self.config.min_table_clearance_m,
        )
        if not straddle.passed or not lower_access.passed:
            raise self._error(
                TunaErrorCode.COLLISION,
                "close",
                "closing_geometry_gate_failed",
                straddle_residual_m=straddle.residual,
                lower_access_residual_m=lower_access.residual,
            )
        return projected

    def _freeze_close(self, result, command_boundary, projected_width):
        if not self._contact_result_compatible(result):
            raise self._error(
                TunaErrorCode.CONTACT,
                "close",
                "full_close_result_not_contact_compatible",
            )
        samples = self.runtime.collect_tuna_qpos_samples(
            required_count=3,
            command_boundary=command_boundary,
        )
        try:
            contract = freeze_retention_contract(
                GRIPPER_CLOSED_POSITION,
                samples,
                command_boundary,
                self.calibration,
                self.config.qpos_stability_tolerance_rad,
            )
        except (TypeError, ValueError) as exc:
            raise self._error(TunaErrorCode.RETENTION, "close", str(exc)) from exc
        if contract.measured_post_close_qpos <= self.pivot_contract.measured_qpos:
            raise self._error(
                TunaErrorCode.CONTACT,
                "close",
                "full_close_did_not_request_additional_travel",
            )
        gate = evaluate_contact_limited_close(
            contract.measured_post_close_pad_gap_m,
            projected_width,
            self.config.bounds_tolerance_m,
        )
        if not gate.passed:
            raise self._error(
                TunaErrorCode.CONTACT,
                "close",
                "contact_limited_aperture_below_predicted_width",
                residual_m=gate.residual,
                tolerance_m=gate.limit,
            )
        self.retention_contract = contract

    def _dispatch_micro_step(self, step, micro_poses):
        is_roll = step.name.startswith("roll_tuna_segment_")
        is_lift = step.name.startswith("test_lift_tuna_") or step.name.startswith(
            "normal_lift_tuna_segment_"
        )
        if is_roll:
            boundary = time.monotonic()
            self._check_pivot_aperture(boundary, f"{step.name}:before")
        elif is_lift:
            boundary = time.monotonic()
            self._check_retention_aperture(boundary, f"{step.name}:before")
        for micro_index, pose in enumerate(micro_poses, start=1):
            self._execute_micro_pose(pose, step.name)
            boundary = time.monotonic()
            if is_roll:
                self._check_pivot_aperture(
                    boundary, f"{step.name}:micro_{micro_index}:after"
                )
            elif is_lift:
                self._check_retention_aperture(
                    boundary, f"{step.name}:micro_{micro_index}:after"
                )

    def _debug_stop_for_observation(self, step_name):
        selector = self.config.debug_stop_after
        mapping = {
            "observe_tuna_after_preclamp": "preclamp",
            "observe_tuna_after_roll_segment_01_of_03": "roll_segment_1",
            "observe_tuna_after_roll_segment_02_of_03": "roll_segment_2",
            "observe_tuna_after_roll_segment_03_of_03": "roll_segment_3",
            "observe_tuna_after_close": "close",
            "observe_tuna_after_test_lift": "test_lift",
        }
        if selector == mapping.get(step_name):
            return selector
        if selector == "normal_lift" and step_name.startswith(
            "observe_tuna_after_normal_lift_segment_"
        ):
            expected_count = int(
                self.plan.debug_info.get("tuna_normal_lift_segment_count", 0)
            )
            if step_name.endswith(f"_of_{expected_count:02d}"):
                return selector
        return None

    def _execute_observation_step(self, step):
        metadata = self.plan.debug_info["tuna_observations"][step.name]
        observation = self._collect_observation(step.name)
        stage = metadata["stage"]
        if stage == "preclamp":
            self._check_preclamp_observation(observation)
            self._confirm("preclamp")
        elif stage.startswith("roll_segment_"):
            self._check_roll_observation(
                observation,
                float(metadata["end_angle_deg"]),
                self.roll_axis,
            )
            self.latest_roll_observation = observation
            self._confirm(stage)
        elif stage == "close":
            self._check_close_observation(observation)
            self._check_retention_aperture(time.monotonic(), "close:observation")
            self._confirm("close")
        elif stage in {"test_lift", "normal_lift"}:
            self._check_retention_aperture(
                time.monotonic(), f"{stage}:observation"
            )
            self._check_lift_observation(
                observation,
                float(metadata["expected_world_z_delta_m"]),
                stage,
            )
            self._confirm(stage)
        else:
            raise self._error(
                TunaErrorCode.CONFIGURATION,
                step.name,
                "unknown_observation_stage",
            )

    def execute(self, plan):
        if not self._exact_tuna(plan):
            raise self._error(
                TunaErrorCode.CONFIGURATION,
                "execute",
                "exact_tuna_required",
            )
        self.plan = plan
        if self.config.debug_stop_after == "generation":
            if plan.steps:
                raise self._error(
                    TunaErrorCode.CONFIGURATION,
                    "generation",
                    "generation_plan_contains_executable_steps",
                )
            return TunaExecutionOutcome(plan, "generation", 0)
        if plan.debug_info.get("tuna_calibration_sha256") != self.calibration.source_sha256:
            raise self._error(
                TunaErrorCode.CALIBRATION_SHA,
                "execute",
                "plan_runtime_calibration_sha_mismatch",
            )
        self.initial_center = np.asarray(
            plan.debug_info["tuna_bounds_center"], dtype=float
        )
        self.initial_size = np.asarray(plan.debug_info["tuna_bounds_size"], dtype=float)
        source_timestamps = plan.debug_info.get("tuna_bounds_source_timestamps", [0.0])
        self.last_bounds_source_timestamp = float(source_timestamps[-1])
        radial = np.asarray(plan.debug_info["tuna_radial_direction_world"], dtype=float)
        self.roll_axis = np.cross(np.array([0.0, 0.0, 1.0]), radial)

        try:
            prefix = list(plan.steps)
            if len(prefix) != 5 or prefix[-1].name != "observe_tuna_after_preclamp":
                raise self._error(
                    TunaErrorCode.CONFIGURATION,
                    "prefix",
                    "unexpected_unfinalized_prefix_shape",
                )
            self._execute_step(prefix[0])
            self._execute_step(prefix[1])
            if self.config.debug_stop_after == "pregrasp":
                return TunaExecutionOutcome(plan, "pregrasp", self.command_count)
            self._execute_step(prefix[2])
            self._confirm("contact_support")
            if self.config.debug_stop_after == "contact_support":
                return TunaExecutionOutcome(
                    plan, "contact_support", self.command_count
                )
            preclamp_result = self._execute_step(prefix[3])
            preclamp_boundary = time.monotonic()
            self._freeze_preclamp(preclamp_result, preclamp_boundary)
            self._execute_observation_step(prefix[4])
            if self.config.debug_stop_after == "preclamp":
                return TunaExecutionOutcome(plan, "preclamp", self.command_count)

            committed = self._finalize_once_or_retry(plan, self.roll_axis)
            self.plan = committed
            micro_waypoints = committed.debug_info["tuna_micro_waypoints"]
            for step in committed.steps[len(prefix) :]:
                if step.action == "observe_tuna_bounds":
                    self._execute_observation_step(step)
                    stop = self._debug_stop_for_observation(step.name)
                    if stop is not None:
                        return TunaExecutionOutcome(
                            committed, stop, self.command_count
                        )
                    continue
                if step.name in micro_waypoints:
                    self._dispatch_micro_step(step, micro_waypoints[step.name])
                    continue
                if step.name == "close_tuna_after_roll":
                    projected_width = self._validate_close_preconditions()
                    close_result = self._execute_step(step)
                    close_boundary = time.monotonic()
                    self._freeze_close(
                        close_result,
                        close_boundary,
                        projected_width,
                    )
                    continue
                self._execute_step(step)
            return TunaExecutionOutcome(committed, "none", self.command_count)
        except TunaGraspError:
            hold = getattr(self.runtime, "hold_tuna_failure", None)
            if callable(hold):
                hold(self.current_gripper_target)
            raise
        except (RuntimeError, TypeError, ValueError) as exc:
            hold = getattr(self.runtime, "hold_tuna_failure", None)
            if callable(hold):
                hold(self.current_gripper_target)
            raise self._error(
                TunaErrorCode.MOTION,
                "runtime_adapter",
                type(exc).__name__,
            ) from exc
