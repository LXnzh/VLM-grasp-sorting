from dataclasses import dataclass
import hashlib
import json
import threading
import time

import numpy as np

from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.trajectory_planner import (
    PickPlacePlan,
    PreparedTunaSuffix,
    build_prepared_tuna_suffix,
)
from my_course_pkg.grasp.tuna_roll_grasp import (
    FinalizationAttemptMetadata,
    FrozenPivotContract,
)


TUNA_ALLOWED_CONTACT_PAD_LINKS = frozenset(
    {
        "robotiq_85_left_finger_tip_link",
        "robotiq_85_right_finger_tip_link",
    }
)


class TunaDependencyUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class MoveItPoseValidation:
    success: bool
    joint_names: tuple[str, ...] = ()
    joint_positions: tuple[float, ...] = ()
    collision_links: tuple[str, ...] = ()
    semantic_response_received: bool = True
    reason: str = ""

    def __post_init__(self):
        if len(self.joint_names) != len(self.joint_positions):
            raise ValueError("joint names and positions must have equal length.")
        if not np.isfinite(self.joint_positions).all():
            raise ValueError("MoveIt joint positions must be finite.")


@dataclass(frozen=True)
class FinalizedTunaPreparation:
    prepared_suffix: PreparedTunaSuffix
    attempt_metadata: FinalizationAttemptMetadata
    validated_pose_count: int
    final_joint_names: tuple[str, ...]
    final_joint_positions: tuple[float, ...]
    pivot_contract_sha256: str


def _canonical_json_bytes(value):
    return json.dumps(
        value,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def pivot_contract_sha256(contract):
    if not isinstance(contract, FrozenPivotContract):
        raise TypeError("contract must be a FrozenPivotContract.")
    content = {
        "calibration_sha256": contract.calibration_sha256,
        "sample_timestamps": list(contract.sample_timestamps),
        "qpos_peak_to_peak_rad": contract.qpos_peak_to_peak_rad,
        "measured_qpos": contract.measured_qpos,
        "measured_pad_gap_m": contract.measured_pad_gap_m,
        "T_tcp_pivot_frozen": contract.T_tcp_pivot_frozen.tolist(),
        "T_world_pivot": contract.T_world_pivot.tolist(),
    }
    return hashlib.sha256(_canonical_json_bytes(content)).hexdigest()


def _pose_sequence(prepared_suffix):
    micro = prepared_suffix.debug_updates["tuna_micro_waypoints"]
    for step in prepared_suffix.steps:
        if step.action != "move":
            continue
        if step.name in micro:
            for micro_pose in micro[step.name]:
                yield step.name, np.array(micro_pose, dtype=float, copy=True)
        elif step.end_pose_6d is not None:
            yield step.name, np.array(step.end_pose_6d, dtype=float, copy=True)


class RosMoveItValidationBackend:
    """Lazy exact-Tuna MoveIt service adapter.

    Construction imports MoveIt messages and creates clients.  The class is
    never constructed on a non-Tuna route.
    """

    def __init__(self, node, service_timeout_sec=2.0):
        from moveit_msgs.srv import GetPositionIK, GetStateValidity

        self.node = node
        self.service_timeout_sec = float(service_timeout_sec)
        self.GetPositionIK = GetPositionIK
        self.GetStateValidity = GetStateValidity
        self.ik_client = node.create_client(GetPositionIK, "/compute_ik")
        self.state_validity_client = node.create_client(
            GetStateValidity,
            "/check_state_validity",
        )
        self.group_name = getattr(node, "tuna_move_group_name", "ur_manipulator")

    def _call(self, client, request, dependency):
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise TunaDependencyUnavailable(f"{dependency} service unavailable")
        future = client.call_async(request)
        ready = threading.Event()
        future.add_done_callback(lambda _future: ready.set())
        if not ready.wait(self.service_timeout_sec):
            raise TimeoutError(f"{dependency} timed out before a response")
        exception = future.exception()
        if exception is not None:
            raise TunaDependencyUnavailable(
                f"{dependency} failed before a semantic response: {exception}"
            )
        return future.result()

    @staticmethod
    def _joint_state_message(joint_names, joint_positions):
        from sensor_msgs.msg import JointState

        message = JointState()
        message.name = list(joint_names)
        message.position = list(joint_positions)
        return message

    @staticmethod
    def _pose_stamped_message(pose_6d):
        from geometry_msgs.msg import PoseStamped
        from scipy.spatial.transform import Rotation

        pose = np.asarray(pose_6d, dtype=float)
        result = PoseStamped()
        result.header.frame_id = "world"
        result.pose.position.x = float(pose[0])
        result.pose.position.y = float(pose[1])
        result.pose.position.z = float(pose[2])
        quaternion = Rotation.from_euler("xyz", pose[3:]).as_quat()
        result.pose.orientation.x = float(quaternion[0])
        result.pose.orientation.y = float(quaternion[1])
        result.pose.orientation.z = float(quaternion[2])
        result.pose.orientation.w = float(quaternion[3])
        return result

    def validate_pose(self, pose_6d, joint_names, joint_positions, stage):
        ik_request = self.GetPositionIK.Request()
        ik_request.ik_request.group_name = self.group_name
        ik_request.ik_request.pose_stamped = self._pose_stamped_message(pose_6d)
        ik_request.ik_request.robot_state.joint_state = self._joint_state_message(
            joint_names,
            joint_positions,
        )
        ik_request.ik_request.avoid_collisions = True
        response = self._call(self.ik_client, ik_request, "compute_ik")
        error_value = getattr(getattr(response, "error_code", None), "val", None)
        if error_value != 1:
            return MoveItPoseValidation(
                success=False,
                semantic_response_received=True,
                reason=f"no_ik:error_code={error_value!r}",
            )
        solution = response.solution
        names = tuple(solution.joint_state.name)
        positions = tuple(float(value) for value in solution.joint_state.position)
        if not names or len(names) != len(positions) or not np.isfinite(positions).all():
            return MoveItPoseValidation(
                success=False,
                semantic_response_received=True,
                reason="malformed_ik_solution",
            )

        validity_request = self.GetStateValidity.Request()
        validity_request.group_name = self.group_name
        validity_request.robot_state = solution
        validity_response = self._call(
            self.state_validity_client,
            validity_request,
            "check_state_validity",
        )
        collision_links = []
        for contact in getattr(validity_response, "contacts", []):
            collision_links.extend(
                [
                    str(getattr(contact, "contact_body_1", "")),
                    str(getattr(contact, "contact_body_2", "")),
                ]
            )
        collision_links = tuple(sorted(set(link for link in collision_links if link)))
        allowed_target_contact = bool(collision_links) and all(
            link in TUNA_ALLOWED_CONTACT_PAD_LINKS or "tuna_fish_can" in link
            for link in collision_links
        )
        valid = bool(getattr(validity_response, "valid", False)) or allowed_target_contact
        return MoveItPoseValidation(
            success=valid,
            joint_names=names,
            joint_positions=positions,
            collision_links=collision_links,
            semantic_response_received=True,
            reason="" if valid else "state_invalid",
        )


class TunaFinalizer:
    def __init__(self, node, backend_factory=None):
        self.node = node
        self._backend_factory = (
            (lambda: RosMoveItValidationBackend(node))
            if backend_factory is None
            else backend_factory
        )
        self._backend = None

    def _get_backend(self):
        if self._backend is None:
            self._backend = self._backend_factory()
        return self._backend

    def prepare(
        self,
        *,
        plan,
        pivot_contract,
        current_joint_names,
        current_joint_positions,
        roll_axis_world,
        attempt,
        evidence_after_timestamp,
        execution_mode,
        drop_pose_6d=None,
        max_physical_commands=96,
    ):
        if not isinstance(plan, PickPlacePlan) or plan.debug_info.get("object_name") != "tuna_fish_can":
            raise TunaGraspError(
                TunaErrorCode.CONFIGURATION,
                "finalize",
                {
                    "object_name": str(
                        getattr(plan, "debug_info", {}).get("object_name", "")
                    ),
                    "reason": "exact_tuna_required",
                },
            )
        joint_names = tuple(str(name) for name in current_joint_names)
        joint_positions = tuple(float(value) for value in current_joint_positions)
        if (
            not joint_names
            or len(joint_names) != len(joint_positions)
            or not np.isfinite(joint_positions).all()
        ):
            raise TunaGraspError(
                TunaErrorCode.PLANNING,
                "finalize",
                {
                    "object_name": "tuna_fish_can",
                    "reason": "invalid_current_joint_state",
                },
            )

        started = time.monotonic()
        prepared = build_prepared_tuna_suffix(
            plan=plan,
            pivot_contract=pivot_contract,
            roll_axis_world=roll_axis_world,
            execution_mode=execution_mode,
            drop_pose_6d=drop_pose_6d,
            max_physical_commands=max_physical_commands,
        )
        validated_count = 0
        try:
            backend = self._get_backend()
            for stage, pose in _pose_sequence(prepared):
                result = backend.validate_pose(
                    pose,
                    joint_names,
                    joint_positions,
                    stage,
                )
                if not isinstance(result, MoveItPoseValidation):
                    raise TunaGraspError(
                        TunaErrorCode.PLANNING,
                        "finalize",
                        {
                            "object_name": "tuna_fish_can",
                            "reason": "malformed_validation_result",
                            "plan_step": stage,
                        },
                    )
                if not result.success:
                    code = (
                        TunaErrorCode.COLLISION
                        if result.collision_links
                        else TunaErrorCode.PLANNING
                    )
                    raise TunaGraspError(
                        code,
                        "finalize",
                        {
                            "object_name": "tuna_fish_can",
                            "reason": result.reason or "semantic_validation_failure",
                            "plan_step": stage,
                            "collision_links": list(result.collision_links),
                            "semantic_response_received": result.semantic_response_received,
                        },
                    )
                joint_names = result.joint_names
                joint_positions = result.joint_positions
                validated_count += 1
        except TunaGraspError:
            raise
        except (TimeoutError, TunaDependencyUnavailable) as exc:
            raise TunaGraspError(
                TunaErrorCode.DEPENDENCY_TRANSIENT,
                "finalize",
                {
                    "object_name": "tuna_fish_can",
                    "reason": type(exc).__name__,
                    "dependency": "moveit",
                    "result_received": False,
                    "robot_command_issued": False,
                    "attempt": int(attempt),
                },
                retryable=True,
            ) from exc

        elapsed = time.monotonic() - started
        attempt_metadata = FinalizationAttemptMetadata(
            attempt=int(attempt),
            evidence_after_timestamp=float(evidence_after_timestamp),
            dependency="moveit",
            elapsed_sec=float(elapsed),
            result_received=True,
        )
        return FinalizedTunaPreparation(
            prepared_suffix=prepared,
            attempt_metadata=attempt_metadata,
            validated_pose_count=validated_count,
            final_joint_names=joint_names,
            final_joint_positions=joint_positions,
            pivot_contract_sha256=pivot_contract_sha256(pivot_contract),
        )


def finalize_tuna_post_preclamp(finalizer, **kwargs):
    if not isinstance(finalizer, TunaFinalizer):
        raise TypeError("finalizer must be a TunaFinalizer.")
    return finalizer.prepare(**kwargs)
