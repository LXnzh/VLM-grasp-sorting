from dataclasses import dataclass
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from sim_pick_place.utils.helpers import (
    _pose6d_to_posestamped_msg,
    _posestamped_msg_to_pose6d,
    _transform_matrix_to_pose6d,
)
from sim_pick_place.utils.pick_place_utils import interpolate_lin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from my_course_pkg.grasp.config import (
    APPROACH_DIST,
    CARTESIAN_END_MAX_ERROR_M,
    CARTESIAN_END_WAIT_SEC,
    CARTESIAN_START_MAX_ERROR_M,
    FINAL_APPROACH_COMMAND_PERIOD_SEC,
    FINAL_APPROACH_POSITION_TOLERANCE_M,
    FINAL_APPROACH_SETTLE_TIMEOUT_SEC,
    GRASP_DEBUG_STOP_AFTER_CLOSE,
    GRASP_DEBUG_STOP_AFTER_LIFT,
    GRASP_DEBUG_STOP_AT_GRASP,
    GRASP_DEBUG_STOP_AT_PREGRASP,
    GRASP_DEBUG_STOP_BEFORE_RELEASE,
    GRIPPER_OPEN_MAX_POSITION,
    INTERPOLATE_AVG_SPEED,
    INTERPOLATE_FILTER_ANGLE_DEG,
    INTERPOLATE_FILTER_DISTANCE,
    RETURN_TO_INITIAL_STABLE_DELTA_RAD,
    RETURN_TO_INITIAL_STABLE_MIN_DURATION_SEC,
    RETURN_TO_INITIAL_STABLE_POLL_SEC,
    RETURN_TO_INITIAL_STABLE_SAMPLES,
    RETURN_TO_INITIAL_STABLE_TIMEOUT_SEC,
    SERVO_MODE_SETTLE_SEC,
    VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M,
    VERTICAL_APPROACH_XY_TOLERANCE_M,
    VERTICAL_APPROACH_Z_TOLERANCE_M,
    VERTICAL_DESCENT_XY_TOLERANCE_M,
    VERTICAL_REANCHOR_MAX_ITERATIONS,
    VERTICAL_REANCHOR_MAX_OFFSET_M,
    VERTICAL_REANCHOR_SAMPLE_COUNT,
    VERTICAL_REANCHOR_SETTLE_SEC,
    WAYPOINT_MAX_DIST,
)
from my_course_pkg.grasp.execution import debug as grasp_debug
from my_course_pkg.grasp.execution import return_home
from my_course_pkg.grasp.transforms import pose_text
from my_course_pkg.verify_init_pose import VerifyInitPoseNode


@dataclass(frozen=True)
class VerticalApproachCalibration:
    observed_median_xyz: np.ndarray
    command_offset_xyz: np.ndarray
    command_count: int


@dataclass(frozen=True)
class VerticalFinalZBounds:
    bottom_z: float | None
    center_z: float | None
    top_z: float | None
    valid: bool
    reason: str


@dataclass(frozen=True)
class VerticalFinalZDecision:
    passed: bool
    mode: str
    signed_z_error_m: float
    z_error_m: float
    negative_tolerance_m: float
    positive_tolerance_m: float
    center_positive_limit_m: float | None
    bounds: VerticalFinalZBounds | None
    reason: str


class MoveItMotionError(RuntimeError):
    def __init__(self, step_name, action_status=None):
        self.step_name = step_name
        self.action_status = action_status
        detail = (
            f" (action status: {action_status})"
            if action_status
            else ""
        )
        super().__init__(
            f"MoveIt did not complete the action for step {step_name!r}{detail}."
        )


class FinalApproachConvergenceError(RuntimeError):
    def __init__(self, target_pose_6d, actual_pose_6d, position_error_m):
        self.target_pose_6d = np.asarray(target_pose_6d, dtype=float).copy()
        self.actual_pose_6d = np.asarray(actual_pose_6d, dtype=float).copy()
        self.position_error_m = float(position_error_m)
        super().__init__(
            "Final grasp approach did not converge within the configured "
            f"position tolerance: error={self.position_error_m:.4f} m."
        )


class VerticalApproachConvergenceError(RuntimeError):
    def __init__(
        self,
        stage,
        target_pose_6d,
        actual_pose_6d,
        xy_error_m,
        z_error_m,
        reason,
        xy_tolerance_m,
        command_count=0,
        z_decision=None,
    ):
        self.stage = str(stage)
        self.target_pose_6d = np.asarray(target_pose_6d, dtype=float).copy()
        self.actual_pose_6d = np.asarray(actual_pose_6d, dtype=float).copy()
        self.xy_error_m = float(xy_error_m)
        self.z_error_m = float(z_error_m)
        self.xy_tolerance_m = float(xy_tolerance_m)
        self.z_tolerance_m = float(VERTICAL_APPROACH_Z_TOLERANCE_M)
        if z_decision is None:
            self.signed_z_error_m = float(
                self.actual_pose_6d[2] - self.target_pose_6d[2]
            )
            self.z_negative_tolerance_m = self.z_tolerance_m
            self.z_positive_tolerance_m = self.z_tolerance_m
            self.z_center_positive_limit_m = None
            self.z_acceptance_mode = "ordinary"
            self.z_policy_reason = "ordinary symmetric Z gate"
            self.z_bounds = None
        else:
            self.signed_z_error_m = float(z_decision.signed_z_error_m)
            self.z_negative_tolerance_m = float(z_decision.negative_tolerance_m)
            self.z_positive_tolerance_m = float(z_decision.positive_tolerance_m)
            self.z_center_positive_limit_m = (
                None
                if z_decision.center_positive_limit_m is None
                else float(z_decision.center_positive_limit_m)
            )
            self.z_acceptance_mode = str(z_decision.mode)
            self.z_policy_reason = str(z_decision.reason)
            self.z_bounds = z_decision.bounds
        self.command_count = int(command_count)
        self.reason = str(reason)
        if self.z_bounds is None:
            bounds_text = "bounds=unavailable"
        else:
            bounds_text = (
                f"bounds_bottom_z={self.z_bounds.bottom_z!r}, "
                f"bounds_center_z={self.z_bounds.center_z!r}, "
                f"bounds_top_z={self.z_bounds.top_z!r}, "
                f"bounds_valid={str(self.z_bounds.valid).lower()}"
            )
        if self.z_center_positive_limit_m is None:
            center_limit_text = "z_center_positive_limit=unavailable"
        else:
            center_limit_text = (
                "z_center_positive_limit="
                f"{self.z_center_positive_limit_m:.4f} m"
            )
        super().__init__(
            f"Vertical approach stage {self.stage!r} failed: {self.reason}; "
            f"xy_error={self.xy_error_m:.4f} m, "
            f"z_error={self.z_error_m:.4f} m, "
            f"signed_z_error={self.signed_z_error_m:+.4f} m, "
            f"xy_tolerance={self.xy_tolerance_m:.4f} m, "
            f"z_tolerance={self.z_tolerance_m:.4f} m, "
            f"z_negative_tolerance={self.z_negative_tolerance_m:.4f} m, "
            f"z_positive_tolerance={self.z_positive_tolerance_m:.4f} m, "
            f"{center_limit_text}, "
            f"z_acceptance_mode={self.z_acceptance_mode}, "
            f"z_policy_reason={self.z_policy_reason!r}, {bounds_text}, "
            f"commands={self.command_count}."
        )


class GripperCommandError(RuntimeError):
    def __init__(self, step_name, message):
        self.step_name = step_name
        super().__init__(message)


class ArmMotionExecutor:
    def __init__(self, node, arm_api2_client, gripper_controller, callback_group):
        self.node = node
        self.arm_api2_client = arm_api2_client
        self.gripper_controller = gripper_controller
        self.latest_arm_joint_names = []
        self.latest_arm_position_by_name = {}
        # Increment only after a complete arm-state sample arrives.  The
        # return-to-initial handoff uses this to avoid treating a pre-switch
        # /joint_states message as evidence that Servo has stopped.
        self._arm_joint_state_sequence = 0
        self.trajectory_action_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            callback_group=callback_group,
        )
        self.joint_state_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=callback_group,
        )
        self.grasp_debug_marker_publisher = node.create_publisher(
            MarkerArray,
            "/my_course_pkg/grasp_debug_markers",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.debug_stop_reached = False

    def joint_state_callback(self, msg):
        arm_position_by_name = VerifyInitPoseNode.arm_joint_position_map(msg)
        if len(arm_position_by_name) == len(VerifyInitPoseNode.arm_joint_order):
            self.latest_arm_position_by_name = arm_position_by_name
            self.latest_arm_joint_names = list(VerifyInitPoseNode.arm_joint_order)
            self._arm_joint_state_sequence += 1

    def enter_servo_pos_mode(self):
        controller_services_ready = (
            self.arm_api2_client._service_client_list_controllers.service_is_ready()
            and self.arm_api2_client._service_client_switch_controller.service_is_ready()
        )
        if controller_services_ready:
            return self.arm_api2_client.change_state_to_servo_pos_ctl()

        self.node.get_logger().warn(
            "controller_manager services are not available; skipping controller "
            "switch and requesting SERVO_POS_CTL state only."
        )
        return self.arm_api2_client.change_state_to("SERVO_POS_CTL")

    def enter_moveit_mode(self):
        controller_services_ready = (
            self.arm_api2_client._service_client_list_controllers.service_is_ready()
            and self.arm_api2_client._service_client_switch_controller.service_is_ready()
        )
        if controller_services_ready:
            return self.arm_api2_client.change_state_to_cartesian_ctl()

        self.node.get_logger().warn(
            "controller_manager services are not available; skipping controller "
            "switch and requesting CART_TRAJ_CTL state only."
        )
        return self.arm_api2_client.change_state_to("CART_TRAJ_CTL")

    def get_current_ee_pose_stamped(self, frame_id=None):
        """Return the measured EE pose, optionally transformed to a frame."""
        deadline = time.monotonic() + 5.0
        ee_pose = self.arm_api2_client.get_current_ee_pose()
        while ee_pose.header.frame_id == "" and time.monotonic() < deadline:
            time.sleep(0.1)
            ee_pose = self.arm_api2_client.get_current_ee_pose()

        if ee_pose.header.frame_id == "":
            raise RuntimeError("Current EE pose was not received on /arm/state/current_pose.")

        if frame_id is not None and ee_pose.header.frame_id != frame_id:
            source_frame = ee_pose.header.frame_id
            ee_pose = self.arm_api2_client.transform_to_frame(ee_pose, frame_id)
            if ee_pose is None:
                raise RuntimeError(
                    "Could not transform current EE pose from "
                    f"{source_frame!r} to {frame_id!r}."
                )

        return ee_pose

    def get_current_ee_pose_6d(self, frame_id=None):
        ee_pose = self.get_current_ee_pose_stamped(frame_id=frame_id)
        return _posestamped_msg_to_pose6d(ee_pose)

    def hold_pose(self, pose_msg, duration_sec):
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            self.arm_api2_client.interpolate_to_pose(
                pose_msg,
                avg_speed=INTERPOLATE_AVG_SPEED,
                filter_distance=INTERPOLATE_FILTER_DISTANCE,
                filter_angle_deg=INTERPOLATE_FILTER_ANGLE_DEG,
            )
            time.sleep(0.05)

    def move_to_pose(self, pose_msg, avg_speed=None):
        self.arm_api2_client.interpolate_to_pose(
            pose_msg,
            avg_speed=(
                INTERPOLATE_AVG_SPEED
                if avg_speed is None
                else float(avg_speed)
            ),
            filter_distance=INTERPOLATE_FILTER_DISTANCE,
            filter_angle_deg=INTERPOLATE_FILTER_ANGLE_DEG,
        )

    def log_target_error(self, label, target_pose_6d):
        try:
            current_pose_6d = self.get_current_ee_pose_6d()
        except Exception as exc:
            self.node.get_logger().warn(f"Could not read EE pose after {label}: {exc}")
            return

        delta_xyz = current_pose_6d[:3] - target_pose_6d[:3]
        self.node.get_logger().info(
            f"{label} target error xyz={delta_xyz} "
            f"norm={sum(x * x for x in delta_xyz) ** 0.5:.4f} m"
        )

    def require_cartesian_target_reached(self, label, target_pose_6d):
        """Wait briefly for Servo tracking and reject an unfinished motion."""
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        deadline = time.monotonic() + CARTESIAN_END_WAIT_SEC
        last_pose_6d = None
        error_m = float("inf")
        while True:
            last_pose_6d = self.get_current_ee_pose_6d()
            error_m = float(
                np.linalg.norm(last_pose_6d[:3] - target_pose_6d[:3])
            )
            if error_m <= CARTESIAN_END_MAX_ERROR_M:
                self.node.get_logger().info(
                    f"{label} Cartesian target reached: error={error_m:.4f} m."
                )
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)

        raise RuntimeError(
            f"Cartesian step {label!r} did not reach its target: "
            f"TCP error {error_m:.4f} m exceeds "
            f"{CARTESIAN_END_MAX_ERROR_M:.4f} m after "
            f"{CARTESIAN_END_WAIT_SEC:.2f} s. Refusing to continue."
        )

    def move_linear(self, start_pose_6d, end_pose_6d, avg_speed=None):
        for pose_6d in interpolate_lin(start_pose_6d, end_pose_6d, WAYPOINT_MAX_DIST):
            pose_msg = _pose6d_to_posestamped_msg(pose_6d[:6], frame_id="world")
            self.move_to_pose(pose_msg, avg_speed=avg_speed)
    
    @staticmethod
    def _vertical_pose_errors(target_pose_6d, actual_pose_6d):
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        actual_pose_6d = np.asarray(actual_pose_6d, dtype=float)
        xy_error_m = float(
            np.linalg.norm(actual_pose_6d[:2] - target_pose_6d[:2])
        )
        z_error_m = float(abs(actual_pose_6d[2] - target_pose_6d[2]))
        return xy_error_m, z_error_m

    @classmethod
    def _vertical_final_z_bounds(cls, plan):
        if cls._grasp_profile(plan) != "vertical":
            return VerticalFinalZBounds(
                bottom_z=None,
                center_z=None,
                top_z=None,
                valid=False,
                reason="non-vertical grasp profile",
            )

        debug_info = getattr(plan, "debug_info", None) or {}
        center = debug_info.get("vertical_target_bounds_center")
        bottom_z = debug_info.get("vertical_bounds_bottom_z")
        if center is None or bottom_z is None:
            return VerticalFinalZBounds(
                bottom_z=None,
                center_z=None,
                top_z=None,
                valid=False,
                reason="missing live vertical bounds",
            )

        try:
            center = np.asarray(center, dtype=float)
            bottom_z = float(bottom_z)
        except (TypeError, ValueError):
            return VerticalFinalZBounds(
                bottom_z=None,
                center_z=None,
                top_z=None,
                valid=False,
                reason="live vertical bounds have invalid values",
            )
        if center.shape != (3,):
            return VerticalFinalZBounds(
                bottom_z=bottom_z,
                center_z=None,
                top_z=None,
                valid=False,
                reason="live vertical bounds center must contain three values",
            )

        center_z = float(center[2])
        top_z = float(2.0 * center_z - bottom_z)
        if not np.all(np.isfinite([bottom_z, center_z, top_z])):
            return VerticalFinalZBounds(
                bottom_z=bottom_z,
                center_z=center_z,
                top_z=top_z,
                valid=False,
                reason="live vertical bounds are non-finite",
            )
        if not bottom_z < center_z < top_z:
            return VerticalFinalZBounds(
                bottom_z=bottom_z,
                center_z=center_z,
                top_z=top_z,
                valid=False,
                reason="live vertical bounds are not strictly ordered",
            )
        return VerticalFinalZBounds(
            bottom_z=bottom_z,
            center_z=center_z,
            top_z=top_z,
            valid=True,
            reason="valid",
        )

    @staticmethod
    def _vertical_final_z_decision(
        target_pose_6d,
        actual_pose_6d,
        final_z_bounds,
    ):
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        actual_pose_6d = np.asarray(actual_pose_6d, dtype=float)
        target_z = float(target_pose_6d[2])
        actual_z = float(actual_pose_6d[2])
        signed_z_error_m = float(actual_z - target_z)
        z_error_m = float(abs(signed_z_error_m))
        ordinary_tolerance_m = float(VERTICAL_APPROACH_Z_TOLERANCE_M)

        if not np.all(np.isfinite([target_z, actual_z, signed_z_error_m])):
            return VerticalFinalZDecision(
                passed=False,
                mode="rejected",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=ordinary_tolerance_m,
                center_positive_limit_m=None,
                bounds=final_z_bounds,
                reason="target or actual Z is non-finite",
            )
        if z_error_m <= ordinary_tolerance_m:
            return VerticalFinalZDecision(
                passed=True,
                mode="ordinary",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=ordinary_tolerance_m,
                center_positive_limit_m=None,
                bounds=final_z_bounds,
                reason="ordinary symmetric Z gate",
            )
        if signed_z_error_m < -ordinary_tolerance_m:
            return VerticalFinalZDecision(
                passed=False,
                mode="rejected",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=ordinary_tolerance_m,
                center_positive_limit_m=None,
                bounds=final_z_bounds,
                reason="downward residual exceeds ordinary tolerance",
            )
        if final_z_bounds is None:
            final_z_bounds = VerticalFinalZBounds(
                bottom_z=None,
                center_z=None,
                top_z=None,
                valid=False,
                reason="missing live vertical bounds",
            )
        if not final_z_bounds.valid:
            return VerticalFinalZDecision(
                passed=False,
                mode="rejected",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=ordinary_tolerance_m,
                center_positive_limit_m=None,
                bounds=final_z_bounds,
                reason=final_z_bounds.reason,
            )
        if not final_z_bounds.bottom_z <= target_z <= final_z_bounds.top_z:
            return VerticalFinalZDecision(
                passed=False,
                mode="rejected",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=ordinary_tolerance_m,
                center_positive_limit_m=None,
                bounds=final_z_bounds,
                reason="target Z is outside live bounds",
            )
        center_positive_limit_m = float(final_z_bounds.center_z - target_z)
        positive_tolerance_m = max(
            ordinary_tolerance_m,
            center_positive_limit_m,
        )
        if actual_z < final_z_bounds.bottom_z:
            return VerticalFinalZDecision(
                passed=False,
                mode="rejected",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=positive_tolerance_m,
                center_positive_limit_m=center_positive_limit_m,
                bounds=final_z_bounds,
                reason="actual Z is below live bounds",
            )
        if actual_z > final_z_bounds.center_z:
            return VerticalFinalZDecision(
                passed=False,
                mode="rejected",
                signed_z_error_m=signed_z_error_m,
                z_error_m=z_error_m,
                negative_tolerance_m=ordinary_tolerance_m,
                positive_tolerance_m=positive_tolerance_m,
                center_positive_limit_m=center_positive_limit_m,
                bounds=final_z_bounds,
                reason="actual Z is above live center",
            )
        return VerticalFinalZDecision(
            passed=True,
            mode="controlled_center",
            signed_z_error_m=signed_z_error_m,
            z_error_m=z_error_m,
            negative_tolerance_m=ordinary_tolerance_m,
            positive_tolerance_m=positive_tolerance_m,
            center_positive_limit_m=center_positive_limit_m,
            bounds=final_z_bounds,
            reason="controlled positive residual at or below live center",
        )

    def _log_vertical_gate_success(
        self,
        *,
        stage,
        target_pose_6d,
        actual_pose_6d,
        xy_error_m,
        z_error_m,
        xy_tolerance_m,
        command_count,
        z_decision=None,
    ):
        if z_decision is None:
            signed_z_error_m = float(
                np.asarray(actual_pose_6d, dtype=float)[2]
                - np.asarray(target_pose_6d, dtype=float)[2]
            )
            negative_tolerance_m = float(VERTICAL_APPROACH_Z_TOLERANCE_M)
            positive_tolerance_m = negative_tolerance_m
            center_positive_limit_m = None
            acceptance_mode = "ordinary"
            bounds_text = "bounds=unavailable"
            no_additional_lower_command = False
        else:
            signed_z_error_m = float(z_decision.signed_z_error_m)
            negative_tolerance_m = float(z_decision.negative_tolerance_m)
            positive_tolerance_m = float(z_decision.positive_tolerance_m)
            center_positive_limit_m = z_decision.center_positive_limit_m
            acceptance_mode = str(z_decision.mode)
            no_additional_lower_command = acceptance_mode == "controlled_center"
            if z_decision.bounds is None:
                bounds_text = "bounds=unavailable"
            elif not z_decision.bounds.valid:
                bounds_text = (
                    f"bounds_bottom_z={z_decision.bounds.bottom_z!r}, "
                    f"bounds_center_z={z_decision.bounds.center_z!r}, "
                    f"bounds_top_z={z_decision.bounds.top_z!r}, "
                    "bounds_valid=false, "
                    f"bounds_reason={z_decision.bounds.reason!r}"
                )
            else:
                bounds_text = (
                    f"bounds_bottom_z={z_decision.bounds.bottom_z:.4f}, "
                    f"bounds_center_z={z_decision.bounds.center_z:.4f}, "
                    f"bounds_top_z={z_decision.bounds.top_z:.4f}, "
                    f"bounds_valid={str(z_decision.bounds.valid).lower()}"
                )
        if center_positive_limit_m is None:
            center_limit_text = "z_center_positive_limit_m=unavailable"
        else:
            center_limit_text = (
                "z_center_positive_limit_m="
                f"{float(center_positive_limit_m):.4f}"
            )
        self.node.get_logger().info(
            "Vertical approach gate passed: "
            f"stage={stage}, target_xyz="
            f"{self._array_text(np.asarray(target_pose_6d)[:3])}, "
            f"actual_xyz={self._array_text(np.asarray(actual_pose_6d)[:3])}, "
            f"xy_error_m={xy_error_m:.4f}, z_error_m={z_error_m:.4f}, "
            f"signed_z_error_m={signed_z_error_m:+.4f}, "
            f"xy_tolerance_m={xy_tolerance_m:.4f}, "
            f"z_negative_tolerance_m={negative_tolerance_m:.4f}, "
            f"z_positive_tolerance_m={positive_tolerance_m:.4f}, "
            f"{center_limit_text}, "
            f"z_acceptance_mode={acceptance_mode}, {bounds_text}, "
            "no_additional_lower_command="
            f"{str(no_additional_lower_command).lower()}, "
            f"commands={command_count}"
        )

    def _hold_and_raise_vertical_approach_error(
        self,
        *,
        stage,
        target_pose_6d,
        actual_pose_6d,
        xy_error_m,
        z_error_m,
        reason,
        xy_tolerance_m,
        command_count=0,
        z_decision=None,
    ):
        actual_pose_6d = np.asarray(actual_pose_6d, dtype=float)
        hold_msg = _pose6d_to_posestamped_msg(actual_pose_6d[:6], frame_id="world")
        self.arm_api2_client.send_pose_cmd(hold_msg)
        error = VerticalApproachConvergenceError(
            stage,
            target_pose_6d,
            actual_pose_6d,
            xy_error_m,
            z_error_m,
            reason,
            xy_tolerance_m,
            command_count=command_count,
            z_decision=z_decision,
        )
        self.node.get_logger().error(
            f"{error} target={pose_text(target_pose_6d)} "
            f"actual={pose_text(actual_pose_6d)}; holding observed pose."
        )
        raise error

    def _sample_vertical_pregrasp_pose(self):
        command_period_sec = max(FINAL_APPROACH_COMMAND_PERIOD_SEC, 0.001)
        samples = []
        for sample_index in range(VERTICAL_REANCHOR_SAMPLE_COUNT):
            samples.append(
                np.asarray(self.get_current_ee_pose_6d(), dtype=float).copy()
            )
            if sample_index + 1 < VERTICAL_REANCHOR_SAMPLE_COUNT:
                time.sleep(command_period_sec)

        sample_xyz = np.vstack([sample[:3] for sample in samples])
        median_xyz = np.median(sample_xyz, axis=0)
        latest_pose_6d = samples[-1]
        return sample_xyz, median_xyz, latest_pose_6d

    @staticmethod
    def _vertical_feedback_stationarity(sample_xyz):
        sample_xyz = np.asarray(sample_xyz, dtype=float)
        if sample_xyz.ndim != 2 or sample_xyz.shape[1] != 3:
            raise ValueError("Vertical feedback samples must have shape (N, 3).")

        pairwise_xy_delta = (
            sample_xyz[:, np.newaxis, :2] - sample_xyz[np.newaxis, :, :2]
        )
        xy_spread_m = float(
            np.max(np.linalg.norm(pairwise_xy_delta, axis=2))
        )
        z_spread_m = float(np.max(sample_xyz[:, 2]) - np.min(sample_xyz[:, 2]))
        xy_limit_m = 0.5 * VERTICAL_APPROACH_XY_TOLERANCE_M
        z_limit_m = 0.5 * VERTICAL_APPROACH_Z_TOLERANCE_M
        stationary = bool(
            np.isfinite(xy_spread_m)
            and np.isfinite(z_spread_m)
            and xy_spread_m <= xy_limit_m
            and z_spread_m <= z_limit_m
        )
        return stationary, xy_spread_m, z_spread_m, xy_limit_m, z_limit_m

    def converge_at_vertical_pregrasp(self, target_pose_6d):
        """Learn a bounded per-run command offset at the safe hover pose."""
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        command_offset_xyz = np.zeros(3, dtype=float)
        command_count = 0
        previous_residual_norm_m = None
        stationary_deadline = None
        command_period_sec = max(FINAL_APPROACH_COMMAND_PERIOD_SEC, 0.001)
        divergence_margin_m = max(
            VERTICAL_APPROACH_XY_TOLERANCE_M,
            VERTICAL_APPROACH_Z_TOLERANCE_M,
        )

        while True:
            sample_xyz, median_xyz, latest_pose_6d = (
                self._sample_vertical_pregrasp_pose()
            )
            (
                stationary,
                xy_spread_m,
                z_spread_m,
                stationary_xy_limit_m,
                stationary_z_limit_m,
            ) = self._vertical_feedback_stationarity(sample_xyz)
            self.node.get_logger().info(
                "Vertical pregrasp feedback window: "
                f"count={VERTICAL_REANCHOR_SAMPLE_COUNT}, "
                f"sample_xyz={self._array_text(sample_xyz)}, "
                f"median_xyz={self._array_text(median_xyz)}, "
                f"latest_xyz={self._array_text(latest_pose_6d[:3])}, "
                f"xy_spread_m={xy_spread_m:.4f}, "
                f"xy_stationary_limit_m={stationary_xy_limit_m:.4f}, "
                f"z_spread_m={z_spread_m:.4f}, "
                f"z_stationary_limit_m={stationary_z_limit_m:.4f}, "
                f"stationary={stationary}"
            )

            if not stationary:
                now = time.monotonic()
                if stationary_deadline is None:
                    stationary_deadline = now + max(
                        FINAL_APPROACH_SETTLE_TIMEOUT_SEC,
                        0.0,
                    )
                latest_xy_error_m, latest_z_error_m = self._vertical_pose_errors(
                    target_pose_6d,
                    latest_pose_6d,
                )
                if now >= stationary_deadline:
                    self._hold_and_raise_vertical_approach_error(
                        stage="pregrasp",
                        target_pose_6d=target_pose_6d,
                        actual_pose_6d=latest_pose_6d,
                        xy_error_m=latest_xy_error_m,
                        z_error_m=latest_z_error_m,
                        reason=(
                            "feedback did not become stationary within settle "
                            "timeout"
                        ),
                        xy_tolerance_m=VERTICAL_APPROACH_XY_TOLERANCE_M,
                        command_count=command_count,
                    )
                self.node.get_logger().info(
                    "Vertical pregrasp feedback is still moving; waiting without "
                    "issuing a compensated command: "
                    f"remaining_sec={max(stationary_deadline - now, 0.0):.3f}, "
                    f"commands={command_count}"
                )
                time.sleep(command_period_sec)
                continue

            stationary_deadline = None
            observed_pose_6d = latest_pose_6d.copy()
            observed_pose_6d[:3] = median_xyz
            median_xy_error_m, median_z_error_m = self._vertical_pose_errors(
                target_pose_6d,
                observed_pose_6d,
            )
            latest_xy_error_m, latest_z_error_m = self._vertical_pose_errors(
                target_pose_6d,
                latest_pose_6d,
            )
            median_passed = bool(
                median_xy_error_m <= VERTICAL_APPROACH_XY_TOLERANCE_M
                and median_z_error_m <= VERTICAL_APPROACH_Z_TOLERANCE_M
            )
            latest_passed = bool(
                latest_xy_error_m <= VERTICAL_APPROACH_XY_TOLERANCE_M
                and latest_z_error_m <= VERTICAL_APPROACH_Z_TOLERANCE_M
            )
            self.node.get_logger().info(
                "Vertical pregrasp convergence check: "
                f"median_xy_error_m={median_xy_error_m:.4f}, "
                f"median_z_error_m={median_z_error_m:.4f}, "
                f"median_passed={median_passed}, "
                f"latest_xy_error_m={latest_xy_error_m:.4f}, "
                f"latest_z_error_m={latest_z_error_m:.4f}, "
                f"latest_passed={latest_passed}"
            )
            if median_passed and latest_passed:
                self._log_vertical_gate_success(
                    stage="pregrasp",
                    target_pose_6d=target_pose_6d,
                    actual_pose_6d=observed_pose_6d,
                    xy_error_m=median_xy_error_m,
                    z_error_m=median_z_error_m,
                    xy_tolerance_m=VERTICAL_APPROACH_XY_TOLERANCE_M,
                    command_count=command_count,
                )
                self.node.get_logger().info(
                    "Vertical pregrasp calibration frozen for descent: "
                    f"offset_xyz={self._array_text(command_offset_xyz)}, "
                    f"commands={command_count}"
                )
                return VerticalApproachCalibration(
                    observed_median_xyz=median_xyz.copy(),
                    command_offset_xyz=command_offset_xyz.copy(),
                    command_count=command_count,
                )

            xy_error_m = median_xy_error_m
            z_error_m = median_z_error_m
            residual_xyz = target_pose_6d[:3] - median_xyz
            residual_norm_m = float(np.linalg.norm(residual_xyz))
            if (
                previous_residual_norm_m is not None
                and residual_norm_m
                > previous_residual_norm_m + divergence_margin_m
            ):
                self._hold_and_raise_vertical_approach_error(
                    stage="pregrasp",
                    target_pose_6d=target_pose_6d,
                    actual_pose_6d=latest_pose_6d,
                    xy_error_m=xy_error_m,
                    z_error_m=z_error_m,
                    reason=(
                        "calibration residual diverged: "
                        f"previous_norm={previous_residual_norm_m:.4f} m, "
                        f"current_norm={residual_norm_m:.4f} m"
                    ),
                    xy_tolerance_m=VERTICAL_APPROACH_XY_TOLERANCE_M,
                    command_count=command_count,
                )

            if command_count >= VERTICAL_REANCHOR_MAX_ITERATIONS:
                self._hold_and_raise_vertical_approach_error(
                    stage="pregrasp",
                    target_pose_6d=target_pose_6d,
                    actual_pose_6d=latest_pose_6d,
                    xy_error_m=xy_error_m,
                    z_error_m=z_error_m,
                    reason=(
                        "calibration iterations exhausted before strict gate passed"
                    ),
                    xy_tolerance_m=VERTICAL_APPROACH_XY_TOLERANCE_M,
                    command_count=command_count,
                )

            proposed_offset_xyz = command_offset_xyz + residual_xyz
            proposed_offset_norm_m = float(np.linalg.norm(proposed_offset_xyz))
            if (
                not np.all(np.isfinite(proposed_offset_xyz))
                or proposed_offset_norm_m > VERTICAL_REANCHOR_MAX_OFFSET_M
            ):
                self._hold_and_raise_vertical_approach_error(
                    stage="pregrasp",
                    target_pose_6d=target_pose_6d,
                    actual_pose_6d=latest_pose_6d,
                    xy_error_m=xy_error_m,
                    z_error_m=z_error_m,
                    reason=(
                        "proposed calibration exceeds maximum offset: "
                        f"norm={proposed_offset_norm_m:.4f} m, "
                        f"maximum={VERTICAL_REANCHOR_MAX_OFFSET_M:.4f} m"
                    ),
                    xy_tolerance_m=VERTICAL_APPROACH_XY_TOLERANCE_M,
                    command_count=command_count,
                )

            command_pose_6d = target_pose_6d.copy()
            command_pose_6d[:3] += proposed_offset_xyz
            self.node.get_logger().info(
                "Vertical pregrasp calibration command: "
                f"command={command_count + 1}, "
                f"nominal_xyz={self._array_text(target_pose_6d[:3])}, "
                f"command_xyz={self._array_text(command_pose_6d[:3])}, "
                f"residual_xyz={self._array_text(residual_xyz)}, "
                f"offset_xyz={self._array_text(proposed_offset_xyz)}, "
                f"residual_norm_m={residual_norm_m:.4f}"
            )
            command_pose_msg = _pose6d_to_posestamped_msg(
                command_pose_6d[:6], frame_id="world"
            )
            self.arm_api2_client.send_pose_cmd(command_pose_msg)
            command_offset_xyz = proposed_offset_xyz
            command_count += 1
            previous_residual_norm_m = residual_norm_m
            time.sleep(VERTICAL_REANCHOR_SETTLE_SEC)

    def _execute_vertical_approach_waypoint(
        self,
        target_pose_6d,
        waypoint_index,
        waypoint_count,
        command_offset_xyz,
        final_z_bounds=None,
    ):
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        command_offset_xyz = np.asarray(command_offset_xyz, dtype=float)
        if command_offset_xyz.shape != (3,) or not np.all(
            np.isfinite(command_offset_xyz)
        ):
            raise ValueError(
                "Vertical command offset must contain three finite XYZ values."
            )
        offset_norm_m = float(np.linalg.norm(command_offset_xyz))
        if offset_norm_m > VERTICAL_REANCHOR_MAX_OFFSET_M:
            raise ValueError(
                "Vertical command offset exceeds configured maximum: "
                f"norm={offset_norm_m:.4f} m, "
                f"maximum={VERTICAL_REANCHOR_MAX_OFFSET_M:.4f} m."
            )
        stage = f"waypoint_{waypoint_index}_of_{waypoint_count}"
        is_final_waypoint = waypoint_index == waypoint_count
        actual_pose_6d = np.asarray(self.get_current_ee_pose_6d(), dtype=float)
        xy_error_m, z_error_m = self._vertical_pose_errors(
            target_pose_6d,
            actual_pose_6d,
        )
        command_period_sec = max(FINAL_APPROACH_COMMAND_PERIOD_SEC, 0.001)
        if xy_error_m > VERTICAL_DESCENT_XY_TOLERANCE_M:
            precommand_deadline = time.monotonic() + max(
                FINAL_APPROACH_SETTLE_TIMEOUT_SEC,
                0.0,
            )
            self.node.get_logger().info(
                "Vertical approach pre-command lateral settling: "
                f"stage={stage}, "
                f"target_xy={self._array_text(target_pose_6d[:2])}, "
                f"actual_xy={self._array_text(actual_pose_6d[:2])}, "
                f"xy_error_m={xy_error_m:.4f}, "
                f"xy_tolerance_m={VERTICAL_DESCENT_XY_TOLERANCE_M:.4f}"
            )
            while xy_error_m > VERTICAL_DESCENT_XY_TOLERANCE_M:
                if time.monotonic() >= precommand_deadline:
                    self._hold_and_raise_vertical_approach_error(
                        stage=stage,
                        target_pose_6d=target_pose_6d,
                        actual_pose_6d=actual_pose_6d,
                        xy_error_m=xy_error_m,
                        z_error_m=z_error_m,
                        reason=(
                            "timeout waiting for lateral convergence before "
                            "lower waypoint"
                        ),
                        xy_tolerance_m=VERTICAL_DESCENT_XY_TOLERANCE_M,
                    )
                time.sleep(command_period_sec)
                actual_pose_6d = np.asarray(
                    self.get_current_ee_pose_6d(),
                    dtype=float,
                )
                xy_error_m, z_error_m = self._vertical_pose_errors(
                    target_pose_6d,
                    actual_pose_6d,
                )
            self.node.get_logger().info(
                "Vertical approach pre-command lateral convergence passed: "
                f"stage={stage}, "
                f"actual_xy={self._array_text(actual_pose_6d[:2])}, "
                f"xy_error_m={xy_error_m:.4f}, "
                f"xy_tolerance_m={VERTICAL_DESCENT_XY_TOLERANCE_M:.4f}"
            )

        command_pose_6d = target_pose_6d.copy()
        command_pose_6d[:3] += command_offset_xyz
        self.node.get_logger().info(
            "Vertical approach waypoint command: "
            f"stage={stage}, "
            f"nominal_xyz={self._array_text(target_pose_6d[:3])}, "
            f"command_xyz={self._array_text(command_pose_6d[:3])}, "
            f"offset_xyz={self._array_text(command_offset_xyz)}"
        )
        target_pose_msg = _pose6d_to_posestamped_msg(
            command_pose_6d[:6], frame_id="world"
        )
        self.arm_api2_client.send_pose_cmd(target_pose_msg)
        deadline = time.monotonic() + max(FINAL_APPROACH_SETTLE_TIMEOUT_SEC, 0.0)

        while True:
            actual_pose_6d = np.asarray(self.get_current_ee_pose_6d(), dtype=float)
            xy_error_m, z_error_m = self._vertical_pose_errors(
                target_pose_6d,
                actual_pose_6d,
            )
            if (
                xy_error_m <= VERTICAL_DESCENT_XY_TOLERANCE_M
                and z_error_m <= VERTICAL_APPROACH_Z_TOLERANCE_M
            ):
                z_decision = None
                if is_final_waypoint:
                    z_decision = self._vertical_final_z_decision(
                        target_pose_6d,
                        actual_pose_6d,
                        final_z_bounds,
                    )
                self._log_vertical_gate_success(
                    stage=stage,
                    target_pose_6d=target_pose_6d,
                    actual_pose_6d=actual_pose_6d,
                    xy_error_m=xy_error_m,
                    z_error_m=z_error_m,
                    xy_tolerance_m=VERTICAL_DESCENT_XY_TOLERANCE_M,
                    command_count=1,
                    z_decision=z_decision,
                )
                return actual_pose_6d
            if time.monotonic() >= deadline:
                z_decision = None
                reason = "timeout waiting for vertical waypoint convergence"
                if is_final_waypoint:
                    z_decision = self._vertical_final_z_decision(
                        target_pose_6d,
                        actual_pose_6d,
                        final_z_bounds,
                    )
                    if (
                        xy_error_m <= VERTICAL_DESCENT_XY_TOLERANCE_M
                        and z_decision.passed
                        and z_decision.mode == "controlled_center"
                    ):
                        self._log_vertical_gate_success(
                            stage=stage,
                            target_pose_6d=target_pose_6d,
                            actual_pose_6d=actual_pose_6d,
                            xy_error_m=xy_error_m,
                            z_error_m=z_error_m,
                            xy_tolerance_m=VERTICAL_DESCENT_XY_TOLERANCE_M,
                            command_count=1,
                            z_decision=z_decision,
                        )
                        return actual_pose_6d
                    reason = (
                        f"{reason}; final Z policy: {z_decision.reason}"
                    )
                self._hold_and_raise_vertical_approach_error(
                    stage=stage,
                    target_pose_6d=target_pose_6d,
                    actual_pose_6d=actual_pose_6d,
                    xy_error_m=xy_error_m,
                    z_error_m=z_error_m,
                    reason=reason,
                    xy_tolerance_m=VERTICAL_DESCENT_XY_TOLERANCE_M,
                    command_count=1,
                    z_decision=z_decision,
                )
            time.sleep(command_period_sec)

    def move_vertical_approach(
        self,
        start_pose_6d,
        end_pose_6d,
        command_offset_xyz,
        final_z_bounds=None,
    ):
        """Execute the initial vertical descent one feedback-gated pose at a time."""
        waypoints = interpolate_lin(
            start_pose_6d,
            end_pose_6d,
            VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M,
        )[1:]
        waypoint_count = len(waypoints)
        for waypoint_index, pose_6d in enumerate(waypoints, start=1):
            self._execute_vertical_approach_waypoint(
                pose_6d,
                waypoint_index,
                waypoint_count,
                command_offset_xyz,
                final_z_bounds=(
                    final_z_bounds
                    if waypoint_index == waypoint_count
                    else None
                ),
            )

    def verify_vertical_final_grasp(
        self,
        target_pose_6d,
        final_z_bounds=None,
    ):
        """Verify the final vertical pose without correcting near the object."""
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        actual_pose_6d = np.asarray(self.get_current_ee_pose_6d(), dtype=float)
        xy_error_m, z_error_m = self._vertical_pose_errors(
            target_pose_6d,
            actual_pose_6d,
        )
        z_decision = self._vertical_final_z_decision(
            target_pose_6d,
            actual_pose_6d,
            final_z_bounds,
        )
        if (
            xy_error_m <= VERTICAL_DESCENT_XY_TOLERANCE_M
            and z_decision.passed
        ):
            self._log_vertical_gate_success(
                stage="final_verify",
                target_pose_6d=target_pose_6d,
                actual_pose_6d=actual_pose_6d,
                xy_error_m=xy_error_m,
                z_error_m=z_error_m,
                xy_tolerance_m=VERTICAL_DESCENT_XY_TOLERANCE_M,
                command_count=0,
                z_decision=z_decision,
            )
            return actual_pose_6d
        self._hold_and_raise_vertical_approach_error(
            stage="final_verify",
            target_pose_6d=target_pose_6d,
            actual_pose_6d=actual_pose_6d,
            xy_error_m=xy_error_m,
            z_error_m=z_error_m,
            reason=f"final verification failed: {z_decision.reason}",
            xy_tolerance_m=VERTICAL_DESCENT_XY_TOLERANCE_M,
            z_decision=z_decision,
        )

    @staticmethod
    def _grasp_profile(plan):
        debug_info = getattr(plan, "debug_info", None) or {}
        return str(debug_info.get("grasp_profile", "")).strip().lower()

    @staticmethod
    def _final_approach_position_tolerance_m(_plan):
        return FINAL_APPROACH_POSITION_TOLERANCE_M

    def _converge_plan_at_final_grasp(self, plan, final_z_bounds=None):
        profile = self._grasp_profile(plan)
        if profile == "vertical":
            if final_z_bounds is None:
                final_z_bounds = self._vertical_final_z_bounds(plan)
            self.node.get_logger().info(
                "Final vertical approach tolerances: "
                f"xy_tolerance_m={VERTICAL_DESCENT_XY_TOLERANCE_M:.4f}, "
                "z_negative_tolerance_m="
                f"{VERTICAL_APPROACH_Z_TOLERANCE_M:.4f}, "
                "z_controlled_upper_bound=live_center, "
                f"bounds_valid={str(final_z_bounds.valid).lower()}, "
                f"bounds_reason={final_z_bounds.reason!r}"
            )
            return self.verify_vertical_final_grasp(
                plan.grasp_pose_6d,
                final_z_bounds=final_z_bounds,
            )
        tolerance_m = self._final_approach_position_tolerance_m(plan)
        self.node.get_logger().info(
            "Final approach tolerance: "
            f"profile={profile or 'global'}, "
            f"position_tolerance_m={tolerance_m:.4f}"
        )
        return self.converge_at_final_grasp(
            plan.grasp_pose_6d,
            position_tolerance_m=tolerance_m,
        )

    def converge_at_final_grasp(
        self,
        target_pose_6d,
        position_tolerance_m=None,
    ):
        """Keep commanding the final TCP until state feedback confirms arrival."""
        if position_tolerance_m is None:
            position_tolerance_m = FINAL_APPROACH_POSITION_TOLERANCE_M
        position_tolerance_m = float(position_tolerance_m)
        target_pose_6d = np.asarray(target_pose_6d, dtype=float)
        target_pose_msg = _pose6d_to_posestamped_msg(
            target_pose_6d[:6], frame_id="world"
        )
        deadline = time.monotonic() + max(FINAL_APPROACH_SETTLE_TIMEOUT_SEC, 0.0)
        command_period_sec = max(FINAL_APPROACH_COMMAND_PERIOD_SEC, 0.001)
        command_count = 0

        while True:
            actual_pose_6d = np.asarray(self.get_current_ee_pose_6d(), dtype=float)
            position_error_m = float(
                np.linalg.norm(actual_pose_6d[:3] - target_pose_6d[:3])
            )
            if position_error_m <= position_tolerance_m:
                self.node.get_logger().info(
                    "Final approach converged: "
                    f"commands={command_count}, position_error_m="
                    f"{position_error_m:.4f}, position_tolerance_m="
                    f"{position_tolerance_m:.4f}"
                )
                return actual_pose_6d

            if time.monotonic() >= deadline:
                error = FinalApproachConvergenceError(
                    target_pose_6d,
                    actual_pose_6d,
                    position_error_m,
                )
                self.node.get_logger().error(
                    f"{error} target={pose_text(target_pose_6d)} "
                    f"actual={pose_text(actual_pose_6d)}"
                )
                raise error

            self.node.get_logger().info(
                "Final approach convergence: "
                f"command={command_count + 1}, position_error_m="
                f"{position_error_m:.4f}"
            )
            self.arm_api2_client.send_pose_cmd(target_pose_msg)
            command_count += 1
            time.sleep(command_period_sec)

    def move_with_moveit(self, target_pose_6d):
        pose_msg = _pose6d_to_posestamped_msg(target_pose_6d[:6], frame_id="world")
        return self.arm_api2_client.move_to_pose(pose_msg)

    def execute_step(self, step):
        print(f"Executing step: {step.name} ({step.action})")
        if step.action == "move":
            if step.mode == "moveit":
                self.enter_moveit_mode()
                if not self.move_with_moveit(step.end_pose_6d):
                    action_status = getattr(
                        getattr(self, "arm_api2_client", None),
                        "last_move_to_pose_status",
                        None,
                    )
                    message = (
                        f"MoveIt did not complete step {step.name!r}"
                        f" ({action_status or 'unknown action status'}); "
                        "stopping before any Cartesian fallback."
                    )
                    self.node.get_logger().error(message)
                    raise MoveItMotionError(step.name, action_status)
                self.log_target_error(step.name, step.end_pose_6d)
            elif step.mode == "cartesian":
                if self.enter_servo_pos_mode() is False:
                    raise RuntimeError(
                        f"Could not enter Servo position mode for {step.name!r}."
                    )
                if SERVO_MODE_SETTLE_SEC > 0.0:
                    time.sleep(SERVO_MODE_SETTLE_SEC)

                actual_start_pose_6d = self.get_current_ee_pose_6d()
                planned_start_pose_6d = np.asarray(
                    step.start_pose_6d,
                    dtype=float,
                )
                start_error_m = float(
                    np.linalg.norm(
                        actual_start_pose_6d[:3]
                        - planned_start_pose_6d[:3]
                    )
                )
                self.node.get_logger().info(
                    f"{step.name} Cartesian start error={start_error_m:.4f} m; "
                    "interpolating from measured TCP pose."
                )
                if start_error_m > CARTESIAN_START_MAX_ERROR_M:
                    raise RuntimeError(
                        f"Refusing Cartesian step {step.name!r}: measured TCP start "
                        f"differs from the planned start by {start_error_m:.4f} m "
                        f"(limit {CARTESIAN_START_MAX_ERROR_M:.4f} m)."
                    )

                self.move_linear(
                    actual_start_pose_6d,
                    step.end_pose_6d,
                    avg_speed=step.avg_speed,
                )
                self.require_cartesian_target_reached(
                    step.name,
                    step.end_pose_6d,
                )
            else:
                raise RuntimeError(f"Unsupported move mode: {step.mode}")
            return

        if step.action == "gripper":
            result = self.gripper_controller.send_gripper_command(
                step.gripper_position
            )
            self._validate_gripper_result(step, result)
            return result

        if step.action == "hold":
            if step.duration_sec <= 0.0:
                return
            if step.mode == "passive":
                time.sleep(step.duration_sec)
                return
            pose_msg = _pose6d_to_posestamped_msg(step.pose_6d[:6], frame_id="world")
            self.hold_pose(pose_msg, step.duration_sec)
            return

        raise RuntimeError(f"Unsupported plan step action: {step.action!r}")

    @staticmethod
    def _gripper_result_field(result, name, default=None):
        if result is None:
            return default
        return getattr(result, name, default)

    @staticmethod
    def _legacy_gripper_bool(result):
        if isinstance(result, bool):
            return result
        return None

    @classmethod
    def _gripper_result_accepted(cls, result):
        legacy = cls._legacy_gripper_bool(result)
        if legacy is not None:
            return bool(legacy)
        return bool(cls._gripper_result_field(result, "accepted", False))

    @classmethod
    def _gripper_result_is_open(cls, result, target_position):
        legacy = cls._legacy_gripper_bool(result)
        if legacy is not None:
            return bool(legacy)

        actual_position = cls._gripper_result_field(result, "actual_position", None)
        if actual_position is None:
            return bool(cls._gripper_result_field(result, "reached_goal", False))
        try:
            return float(actual_position) <= GRIPPER_OPEN_MAX_POSITION
        except (TypeError, ValueError):
            return bool(cls._gripper_result_field(result, "reached_goal", False))

    def _raise_gripper_error(self, step, result, reason):
        message = (
            f"Gripper command failed during step {step.name!r} "
            f"at target position {step.gripper_position}: {reason}. "
            f"result={result!r}"
        )
        self.node.get_logger().error(message)
        raise GripperCommandError(step.name, message)

    def _validate_gripper_result(self, step, result):
        if not self._gripper_result_accepted(result):
            self._raise_gripper_error(step, result, "command result was not accepted")

        if step.name in {
            "open_gripper_before_approach",
            "open_gripper_to_release",
        } and not self._gripper_result_is_open(result, step.gripper_position):
            self._raise_gripper_error(
                step,
                result,
                "open command did not reach the configured open position",
            )

    def _validate_pear_close_result(self, plan, step, result):
        debug_info = getattr(plan, "debug_info", {}) or {}
        object_name = str(debug_info.get("object_name", "")).strip().lower()
        object_name = object_name.replace(" ", "_")
        if object_name != "pear" or step.name != "close_gripper_at_grasp":
            return

        minimum_position = debug_info.get("pear_minimum_close_position_rad")
        expected_position = debug_info.get("pear_expected_close_position_rad")
        projected_width = debug_info.get("pear_projected_width_m")
        actual_position = self._gripper_result_field(result, "actual_position", None)
        try:
            minimum_position = float(minimum_position)
            expected_position = float(expected_position)
            projected_width = float(projected_width)
            actual_position = float(actual_position)
        except (TypeError, ValueError):
            self._raise_gripper_error(
                step,
                result,
                "pear close readiness data is missing or invalid",
            )

        if not np.isfinite(
            [minimum_position, expected_position, projected_width, actual_position]
        ).all():
            self._raise_gripper_error(
                step,
                result,
                "pear close readiness data is non-finite",
            )
        if actual_position + 1e-9 < minimum_position:
            self._raise_gripper_error(
                step,
                result,
                "pear close stopped before the bilateral-contact minimum "
                f"(actual={actual_position:.3f}, minimum={minimum_position:.3f}, "
                f"expected={expected_position:.3f}, "
                f"projected_width_m={projected_width:.5f})",
            )

        self.node.get_logger().info(
            "Pear close readiness passed: "
            f"actual_position_rad={actual_position:.3f}, "
            f"minimum_position_rad={minimum_position:.3f}, "
            f"expected_position_rad={expected_position:.3f}, "
            f"projected_width_m={projected_width:.5f}"
        )

    @staticmethod
    def _step_reaches_final_grasp(plan, step):
        """Return whether a movement step ends at the selected final TCP."""
        if step.action != "move" or step.end_pose_6d is None:
            return False
        return bool(
            np.allclose(
                np.asarray(step.end_pose_6d, dtype=float)[:6],
                np.asarray(plan.grasp_pose_6d, dtype=float)[:6],
            )
        )

    @staticmethod
    def _gripper_targets(plan):
        return grasp_debug.gripper_targets(plan)

    @staticmethod
    def _array_text(value, precision=4):
        return grasp_debug.array_text(value, precision=precision)

    def _publish_grasp_debug_markers(self, plan):
        """Publish planned object/TCP geometry for one focused RViz inspection."""
        return grasp_debug.publish_grasp_debug_markers(
            self,
            plan,
            point_type=Point,
            marker_type=Marker,
            marker_array_type=MarkerArray,
        )

    def _log_grasp_debug(self, plan, stage):
        """Log the selected side-grasp geometry without changing motion policy."""
        return grasp_debug.log_grasp_debug(
            self,
            plan,
            stage,
            approach_distance=APPROACH_DIST,
            pose_formatter=pose_text,
            transform_to_pose6d=_transform_matrix_to_pose6d,
        )

    def execute_plan(self, plan):
        self.debug_stop_reached = False
        stop_after_close_pending = False
        profile = self._grasp_profile(plan)
        vertical_command_offset_xyz = np.zeros(3, dtype=float)
        vertical_final_z_bounds = None
        if profile == "vertical":
            vertical_final_z_bounds = self._vertical_final_z_bounds(plan)
        for step in plan.steps:
            if (
                (GRASP_DEBUG_STOP_AT_PREGRASP or GRASP_DEBUG_STOP_AT_GRASP)
                and step.name == "open_gripper_before_approach"
            ):
                print(
                    "[GraspDebug] debug stop enabled: skipping "
                    "pre-approach gripper command."
                )
                continue
            if step.name == "open_gripper_to_release" and GRASP_DEBUG_STOP_BEFORE_RELEASE:
                self._log_grasp_debug(plan, "before_release")
                self.debug_stop_reached = True
                print(
                    "[GraspDebug] GRASP_DEBUG_STOP_BEFORE_RELEASE=1: "
                    "stopping at drop pose before opening the gripper."
                )
                return

            if stop_after_close_pending:
                if step.name == "hold_after_close":
                    self.execute_step(step)
                else:
                    self.node.get_logger().warn(
                        "GRASP_DEBUG_STOP_AFTER_CLOSE=1 expected "
                        "hold_after_close; stopping before the next motion step."
                    )
                self._log_grasp_debug(plan, "after_close")
                self.debug_stop_reached = True
                print(
                    "[GraspDebug] GRASP_DEBUG_STOP_AFTER_CLOSE=1: "
                    "hold completed; lift/drop skipped."
                )
                return

            reaches_final_grasp = self._step_reaches_final_grasp(plan, step)
            is_vertical_initial_approach = (
                profile == "vertical"
                and step.name == "approach_grasp"
                and step.action == "move"
                and step.mode == "cartesian"
                and reaches_final_grasp
            )
            if reaches_final_grasp and (
                GRASP_DEBUG_STOP_AT_GRASP or GRASP_DEBUG_STOP_AFTER_CLOSE
            ):
                self._log_grasp_debug(plan, "before_final_grasp")
            if is_vertical_initial_approach:
                self.move_vertical_approach(
                    step.start_pose_6d,
                    step.end_pose_6d,
                    vertical_command_offset_xyz,
                    final_z_bounds=vertical_final_z_bounds,
                )
                step_result = None
            else:
                step_result = self.execute_step(step)
                self._validate_pear_close_result(plan, step, step_result)
            if step.name == "move_to_pre_grasp" and profile == "vertical":
                if self.enter_servo_pos_mode() is False:
                    raise RuntimeError(
                        "Could not enter Servo position mode for vertical pre-grasp "
                        "feedback convergence."
                    )
                calibration = self.converge_at_vertical_pregrasp(
                    plan.pre_grasp_pose_6d
                )
                vertical_command_offset_xyz = calibration.command_offset_xyz.copy()
            if step.name == "move_to_pre_grasp" and GRASP_DEBUG_STOP_AT_PREGRASP:
                self._log_grasp_debug(plan, "at_pregrasp")
                self.debug_stop_reached = True
                print(
                    "[GraspDebug] GRASP_DEBUG_STOP_AT_PREGRASP=1: "
                    "holding at pregrasp; approach/gripper/lift/drop skipped."
                )
                return
            if reaches_final_grasp and (
                GRASP_DEBUG_STOP_AT_GRASP or GRASP_DEBUG_STOP_AFTER_CLOSE
            ):
                self._converge_plan_at_final_grasp(
                    plan,
                    final_z_bounds=vertical_final_z_bounds,
                )
                self._log_grasp_debug(plan, "after_final_grasp")
                if GRASP_DEBUG_STOP_AT_GRASP:
                    self.debug_stop_reached = True
                    print(
                        "[GraspDebug] GRASP_DEBUG_STOP_AT_GRASP=1: "
                        "holding at final grasp pose; gripper/lift/drop skipped."
                    )
                    return
            elif reaches_final_grasp:
                self._converge_plan_at_final_grasp(
                    plan,
                    final_z_bounds=vertical_final_z_bounds,
                )
            if step.name == "close_gripper_at_grasp" and GRASP_DEBUG_STOP_AFTER_CLOSE:
                stop_after_close_pending = True
            if step.name == "hold_after_lift" and GRASP_DEBUG_STOP_AFTER_LIFT:
                self._log_grasp_debug(plan, "after_lift")
                self.debug_stop_reached = True
                print(
                    "[GraspDebug] GRASP_DEBUG_STOP_AFTER_LIFT=1: "
                    "lift hold completed; transfer/drop skipped."
                )
                return

    def execute_first_reachable_plan(self, plans):
        for candidate_index, plan in enumerate(plans, start=1):
            print(f"Trying grasp candidate {candidate_index}/{len(plans)}")
            try:
                self.execute_plan(plan)
                return plan
            except MoveItMotionError as exc:
                if exc.step_name != "move_to_pre_grasp":
                    raise
                if exc.action_status != "planning_failed":
                    raise RuntimeError(
                        "Pre-grasp MoveIt action did not complete "
                        f"({exc.action_status or 'unknown status'}); refusing "
                        "to try another candidate while the controller may "
                        "still be moving."
                    ) from exc
                self.node.get_logger().warn(
                    f"Grasp candidate {candidate_index}/{len(plans)} could "
                    "not be planned for pre-grasp; trying the next candidate."
                )

        raise RuntimeError(
            "No grasp candidate could be planned for its pre-grasp pose."
        )

    def switch_to_joint_control(self):
        return return_home.switch_to_joint_control(self)

    def send_initial_pose_trajectory(self, goal_joint_state):
        return return_home.send_initial_pose_trajectory(
            self,
            goal_joint_state,
            goal_status=GoalStatus,
            follow_joint_trajectory=FollowJointTrajectory,
            joint_trajectory_type=JointTrajectory,
            joint_trajectory_point_type=JointTrajectoryPoint,
        )

    def wait_for_arm_joint_state_stable(self):
        """Wait for new, consecutive arm-state samples to become stationary.

        This deliberately does not reuse the most recent sample: after a
        Servo-to-joint handoff, that sample may predate the handoff and the
        arm can still be moving.  Joint deltas are wrapped so a crossing at
        ``-pi/pi`` is not mistaken for a large move.
        """
        return return_home.wait_for_arm_joint_state_stable(
            self,
            expected_names=VerifyInitPoseNode.arm_joint_order,
            stable_delta_rad=RETURN_TO_INITIAL_STABLE_DELTA_RAD,
            stable_min_duration_sec=RETURN_TO_INITIAL_STABLE_MIN_DURATION_SEC,
            stable_poll_sec=RETURN_TO_INITIAL_STABLE_POLL_SEC,
            stable_samples_required=RETURN_TO_INITIAL_STABLE_SAMPLES,
            stable_timeout_sec=RETURN_TO_INITIAL_STABLE_TIMEOUT_SEC,
            clock=time,
        )

    def return_to_initial_pose(self):
        return return_home.return_to_initial_pose(
            self,
            verify_init_pose_node=VerifyInitPoseNode,
            clock=time,
        )
