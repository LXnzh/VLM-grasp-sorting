"""ROS orchestration for follow-until-stop PBVS grasping."""

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_course_pkg.grasp.config import CAMERA_FRAME
from my_course_pkg.grasp.transforms import (
    camera_pose_convention_transform,
    get_transform_checked,
)
from my_course_pkg.rgbd_file_save import RGBDPerceptionNode
from my_course_pkg.tasks.tracking.node import FoundationPoseGraspNode
from my_course_pkg.tasks.tracking.motion_gate import LOST, STABLE
from my_course_pkg.tasks.voice_input import (
    add_instruction_arguments,
    instruction_from_args,
)
from sim_pick_place.utils.helpers import _pose6d_to_posestamped_msg

from .control import (
    PBVS_MOTION_OBSERVATION_S,
    PBVSError,
    PositionWindowStabilityGate,
    bounded_camera_pbvs_target,
    continuous_stop_ready,
    masked_depth_centroid,
    should_run_pbvs,
    target_observation_jump_m,
)


class FollowStopPBVSGraspNode(FoundationPoseGraspNode):
    """PBVS-follow while moving, then reuse stable-frame grasp execution."""

    def __init__(self, rgbd_node, instruction="", selection_frame=None):
        # PBVS in this workspace intentionally tracks one physical target.
        # Low-rate camera jumps must not turn duplicate SAM2 proposals into a
        # false target-loss safety stop.
        super().__init__(
            rgbd_node,
            instruction,
            selection_frame,
            single_instance_reanchor=True,
        )
        self.declare_parameter("pbvs_rate", 10.0)
        self.declare_parameter(
            "pbvs_motion_observation_s",
            PBVS_MOTION_OBSERVATION_S,
        )
        self.declare_parameter("pbvs_gain", 0.45)
        self.declare_parameter("pbvs_deadband_m", 0.008)
        self.declare_parameter("pbvs_max_step_m", 0.020)
        self.declare_parameter("pbvs_reference_offset_x", 0.0)
        self.declare_parameter("pbvs_reference_offset_y", 0.0)
        self.declare_parameter("pbvs_reference_offset_z", 0.0)
        self.declare_parameter("pbvs_workspace_frame", "base_link")
        self.declare_parameter("pbvs_workspace_x_min", -0.90)
        # The tabletop object placement spans roughly base-X +/-0.28 m. Keep
        # a safety margin while allowing PBVS to follow targets across it.
        self.declare_parameter("pbvs_workspace_x_max", 0.35)
        self.declare_parameter("pbvs_workspace_y_min", -0.55)
        self.declare_parameter("pbvs_workspace_y_max", 0.65)
        self.declare_parameter("pbvs_workspace_z_min", 0.10)
        self.declare_parameter("pbvs_workspace_z_max", 0.85)
        self.declare_parameter("pbvs_min_depth_pixels", 30)
        self.declare_parameter("pbvs_stop_target_span_m", 0.005)
        self.declare_parameter("pbvs_stop_tcp_span_m", 0.003)
        self.declare_parameter("pbvs_stop_duration_s", 1.5)
        # The simulated RGB-D stream can fall to about 2 Hz while rendering
        # two 1280x720 cameras. Duration and position-span checks provide the
        # primary stability guarantee; three fresh samples prevent a single
        # stale frame from satisfying the gate at that rate.
        self.declare_parameter("pbvs_stop_min_frames", 3)
        self._pbvs_period = 1.0 / max(
            1.0,
            float(self.get_parameter("pbvs_rate").value),
        )
        self._last_pbvs_command = 0.0
        self._last_pbvs_frame_stamp = -float("inf")
        self._last_stop_frame_stamp = -float("inf")
        self._pbvs_mode_entered = False
        self._pbvs_orientation = None
        self._pbvs_reference_camera = None
        self._pbvs_observation_tracker_identity = None
        self._last_pbvs_target_camera = None
        stop_duration_s = float(
            self.get_parameter("pbvs_stop_duration_s").value
        )
        stop_min_frames = int(
            self.get_parameter("pbvs_stop_min_frames").value
        )
        self._target_stability = PositionWindowStabilityGate(
            max_span_m=float(
                self.get_parameter("pbvs_stop_target_span_m").value
            ),
            duration_s=stop_duration_s,
            minimum_samples=stop_min_frames,
        )
        self._tcp_stability = PositionWindowStabilityGate(
            max_span_m=float(
                self.get_parameter("pbvs_stop_tcp_span_m").value
            ),
            duration_s=stop_duration_s,
            minimum_samples=stop_min_frames,
        )

    def _camera_reference_offset(self):
        return np.array(
            [
                self.get_parameter("pbvs_reference_offset_x").value,
                self.get_parameter("pbvs_reference_offset_y").value,
                self.get_parameter("pbvs_reference_offset_z").value,
            ],
            dtype=float,
        )

    def _initialize_target(self):
        """Lock the instance and initial camera-relative servo reference."""
        super()._initialize_target()
        with self.tracking.lock:
            initial_camera = (
                None
                if self.tracking.initial_target_camera_point is None
                else self.tracking.initial_target_camera_point.copy()
            )
        if initial_camera is None:
            raise PBVSError(
                "The locked target has no valid initial camera-space "
                "depth center."
            )
        self._pbvs_reference_camera = (
            initial_camera + self._camera_reference_offset()
        )
        self._status(
            "PBVS_CAMERA_REFERENCE_LOCKED: "
            f"initial_target_cv={initial_camera.round(4).tolist()}, "
            f"reference_cv={self._pbvs_reference_camera.round(4).tolist()}"
        )

    def _latest_target_camera_observation(self):
        """Return an atomic target center, frame stamp, and motion snapshot."""
        with self.tracking.lock:
            latest = self.tracking.latest_tracked
            if latest is None:
                raise PBVSError("No tracked RGB-D-mask frame is available.")
            _rgb, depth, mask, stamp, snapshot = latest
            tracker = self.tracking.tracker
            tracker_identity = id(tracker)
            if tracker_identity != self._pbvs_observation_tracker_identity:
                self._pbvs_observation_tracker_identity = tracker_identity
                self._last_pbvs_target_camera = None
            try:
                point_cv = masked_depth_centroid(
                    depth,
                    mask,
                    self.rgbd_node.K,
                    minimum_pixels=int(
                        self.get_parameter("pbvs_min_depth_pixels").value
                    ),
                )
            except PBVSError as exc:
                tracker.invalidate(
                    stamp,
                    f"full-resolution target depth invalid: {exc}",
                )
                raise
            if self._last_pbvs_target_camera is not None:
                jump_m = target_observation_jump_m(
                    self._last_pbvs_target_camera,
                    point_cv,
                )
                limit_m = float(self.get_parameter("lost_depth_jump_m").value)
                if jump_m > limit_m:
                    reason = (
                        "full-resolution masked depth jump "
                        f"{jump_m:.4f}m exceeds {limit_m:.4f}m"
                    )
                    tracker.invalidate(stamp, reason)
                    raise PBVSError(reason)
            self._last_pbvs_target_camera = point_cv.copy()
        return point_cv, float(stamp), snapshot

    def _pbvs_transforms(self):
        """Return camera-vector and workspace transforms for one PBVS tick."""
        T_tf_cam_cv_cam = camera_pose_convention_transform()
        T_world_cam = get_transform_checked(self, CAMERA_FRAME, "world")
        T_world_cv_cam = T_world_cam @ T_tf_cam_cv_cam
        workspace_frame = str(
            self.get_parameter("pbvs_workspace_frame").value
        ).strip()
        if not workspace_frame:
            raise PBVSError("pbvs_workspace_frame cannot be empty.")
        if workspace_frame == "world":
            T_workspace_world = np.eye(4, dtype=float)
        else:
            T_workspace_world = get_transform_checked(
                self,
                "world",
                workspace_frame,
            )
        return T_world_cv_cam, T_workspace_world, workspace_frame

    def _pbvs_tick(
        self,
        current_pose=None,
        target_camera=None,
        frame_stamp=None,
    ):
        """Run one camera-relative PBVS update and report command emission."""
        now = time.monotonic()
        if now - self._last_pbvs_command < self._pbvs_period:
            return False
        if target_camera is None or frame_stamp is None:
            (
                target_camera,
                frame_stamp,
                _snapshot,
            ) = self._latest_target_camera_observation()
        if frame_stamp <= self._last_pbvs_frame_stamp:
            return False
        self._last_pbvs_command = now
        self._last_pbvs_frame_stamp = float(frame_stamp)
        if current_pose is None:
            current_pose = self.motion.get_current_ee_pose_6d()
        current_pose = np.asarray(current_pose, dtype=float)
        if self._pbvs_orientation is None:
            self._pbvs_orientation = current_pose[3:6].copy()
        target_camera = np.asarray(target_camera, dtype=float).reshape(3)
        if self._pbvs_reference_camera is None:
            self._pbvs_reference_camera = (
                target_camera + self._camera_reference_offset()
            )
            self._status(
                "PBVS_CAMERA_REFERENCE_FALLBACK: "
                f"target_cv={target_camera.round(4).tolist()}, "
                f"reference_cv={self._pbvs_reference_camera.round(4).tolist()}"
            )
        (
            T_world_cv_cam,
            T_workspace_world,
            workspace_frame,
        ) = self._pbvs_transforms()
        command_xyz, error_camera, should_command = bounded_camera_pbvs_target(
            current_pose[:3],
            target_camera,
            self._pbvs_reference_camera,
            T_world_cv_camera=T_world_cv_cam,
            T_workspace_world=T_workspace_world,
            gain=float(self.get_parameter("pbvs_gain").value),
            deadband_m=float(self.get_parameter("pbvs_deadband_m").value),
            max_step_m=float(self.get_parameter("pbvs_max_step_m").value),
            workspace_min=[
                self.get_parameter("pbvs_workspace_x_min").value,
                self.get_parameter("pbvs_workspace_y_min").value,
                self.get_parameter("pbvs_workspace_z_min").value,
            ],
            workspace_max=[
                self.get_parameter("pbvs_workspace_x_max").value,
                self.get_parameter("pbvs_workspace_y_max").value,
                self.get_parameter("pbvs_workspace_z_max").value,
            ],
            workspace_frame=workspace_frame,
        )
        if not should_command:
            return False
        if not self._pbvs_mode_entered:
            if not self.motion.enter_servo_pos_mode():
                raise PBVSError("Could not enter SERVO_POS_CTL for PBVS.")
            self._pbvs_mode_entered = True
            self._status("PBVS_FOLLOW_ACTIVE")
        command_pose = np.concatenate([command_xyz, self._pbvs_orientation])
        self.motion.move_to_pose(
            _pose6d_to_posestamped_msg(command_pose, frame_id="world")
        )
        self._status(
            "PBVS_COMMAND: "
            f"target_cv={target_camera.round(4).tolist()}, "
            f"error_cv={error_camera.round(4).tolist()}, "
            f"command_world={command_xyz.round(4).tolist()}"
        )
        return True

    def _wait_for_stable_frame(self):
        last_state = None
        observation_started = time.monotonic()
        observation_duration = max(
            0.0,
            float(self.get_parameter("pbvs_motion_observation_s").value),
        )
        observation_announced = False
        stop_wait_announced = False
        while rclpy.ok():
            snapshot = self.tracking.snapshot()
            if snapshot is None:
                time.sleep(0.03)
                continue
            if snapshot.state == LOST:
                reason = snapshot.loss_reason or "tracking confidence lost"
                self._status(f"PBVS_TRACKING_LOST: {reason}")
                self._target_stability.reset()
                self._tcp_stability.reset()
                self._recover_lost_target()
                last_state = None
                continue
            try:
                (
                    target_camera,
                    frame_stamp,
                    snapshot,
                ) = self._latest_target_camera_observation()
            except PBVSError:
                time.sleep(0.03)
                continue
            if frame_stamp <= self._last_stop_frame_stamp:
                time.sleep(0.02)
                continue
            self._last_stop_frame_stamp = frame_stamp
            if snapshot.state != last_state:
                loss_reason = (
                    f", reason={snapshot.loss_reason}"
                    if snapshot.loss_reason
                    else ""
                )
                self._status(
                    "PBVS_MOTION_GATE: "
                    f"state={snapshot.state}, "
                    f"pixel={snapshot.pixel_motion:.2f}, "
                    f"depth={snapshot.depth_motion_m:.4f}m, "
                    f"iou={snapshot.mask_iou:.3f}"
                    f"{loss_reason}"
                )
                last_state = snapshot.state
            if snapshot.state == LOST:
                self._target_stability.reset()
                self._tcp_stability.reset()
                self._recover_lost_target()
                last_state = None
                continue
            current_pose = np.asarray(
                self.motion.get_current_ee_pose_6d(),
                dtype=float,
            )
            target_stable, target_span = self._target_stability.update(
                frame_stamp,
                target_camera,
            )
            tcp_stable, tcp_span = self._tcp_stability.update(
                frame_stamp,
                current_pose[:3],
            )
            command_sent = False
            if should_run_pbvs(snapshot.state):
                command_sent = self._pbvs_tick(
                    current_pose=current_pose,
                    target_camera=target_camera,
                    frame_stamp=frame_stamp,
                )
            if command_sent:
                self._target_stability.reset()
                self._tcp_stability.reset()
                target_stable = False
                tcp_stable = False
            if snapshot.state == STABLE:
                elapsed = time.monotonic() - observation_started
                if elapsed < observation_duration:
                    if not observation_announced:
                        self._status(
                            "PBVS_OBSERVING_FOR_MOTION: "
                            "target is currently stable; observing for "
                            f"{observation_duration:.2f}s before grasp"
                        )
                        observation_announced = True
                    time.sleep(0.02)
                    continue
                if not continuous_stop_ready(target_stable, tcp_stable):
                    if not stop_wait_announced:
                        self._status(
                            "PBVS_WAITING_FOR_CONTINUOUS_STOP: "
                            f"target_span={target_span:.4f}m/"
                            f"{self._target_stability.max_span_m:.4f}m, "
                            f"tcp_span={tcp_span:.4f}m/"
                            f"{self._tcp_stability.max_span_m:.4f}m, "
                            f"samples={len(self._target_stability.samples)}/"
                            f"{self._target_stability.minimum_samples}, "
                            f"covered="
                            f"{self._target_stability.covered_duration_s:.2f}/"
                            f"{self._target_stability.duration_s:.2f}s"
                        )
                        stop_wait_announced = True
                    time.sleep(0.02)
                    continue
                tracked = self.tracking.stable_frame()
                if tracked is not None:
                    self._status(
                        "PBVS_RELATIVE_POSE_STABLE: freezing follow control "
                        "before FoundationPose"
                    )
                    return tracked
            else:
                stop_wait_announced = False
            time.sleep(0.02)
        raise RuntimeError("ROS stopped before the target became stable.")


def _parse_main_args(args=None):
    parser = argparse.ArgumentParser(
        description=(
            "Follow a moving target with RGB-D PBVS and grasp after it stops."
        ),
    )
    add_instruction_arguments(parser)
    return parser.parse_known_args(args)


def main(args=None):
    cli_args, ros_args = _parse_main_args(args)
    rclpy.init(args=ros_args)
    rgbd = RGBDPerceptionNode("pbvs_follow_stop_rgbd", auto_save=False)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(rgbd)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    node = None
    try:
        deadline = time.monotonic() + 30.0
        while (
            rgbd.get_latest_rgbd()[0] is None
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        if rgbd.get_latest_rgbd()[0] is None:
            raise TimeoutError("Timed out waiting for synchronized RGB-D.")

        node = FollowStopPBVSGraspNode(rgbd)
        executor.add_node(node)
        node._status("RETURNING_TO_INITIAL_POSE")
        if not node.motion.return_to_initial_pose():
            raise RuntimeError("Could not verify the robot initial pose.")
        node._status("INITIAL_POSE_READY")

        instruction = instruction_from_args(
            cli_args,
            prompt=(
                "Instruction (PBVS follows while the target moves, then "
                "grasps after it stops): "
            ),
        )
        selection_frame = rgbd.get_latest_rgbd()
        if selection_frame[0] is None:
            raise TimeoutError("No instruction-time RGB-D frame is available.")
        node.set_instruction(instruction, selection_frame)
        node.run_task()
    except Exception as exc:
        failure = f"TASK_FAILED: {type(exc).__name__}: {exc}"
        if node is not None:
            node._status(failure)
        else:
            rgbd.get_logger().error(failure)
        raise
    finally:
        if node is not None:
            node.stop()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        executor.remove_node(rgbd)
        rgbd.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
