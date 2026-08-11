"""ROS node and command-line entry point for FoundationPose tracking grasps."""

import argparse
import json
import threading
import time

from arm_api2_py.arm_api2_client import ArmApi2Client
import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from my_course_pkg.grasp.config import CAMERA_FRAME, GRIPPER_COMMAND_MODE
from my_course_pkg.grasp.gripper_control import GripperController
from my_course_pkg.grasp.pick_place_planner import (
    plan_pick_place_candidates_from_perception,
)
from my_course_pkg.grasp.transforms import estimate_object_world_pose
from my_course_pkg.paths import OUTPUT_DIR
from my_course_pkg.perception.foundationpose import FoundationPoseEstimationNode
from my_course_pkg.perception.llm_sam2 import (
    build_sam2_text_prompt,
    run_sam2_api,
    vlm_classify_food,
    vlm_select_target,
)
from my_course_pkg.rgbd_file_save import RGBDPerceptionNode
from my_course_pkg.tasks.tracking.motion_gate import LOST, STABLE, MotionConfig
from my_course_pkg.tasks.sorting.drop_target import resolve_drop_target
from my_course_pkg.tasks.voice_input import (
    add_instruction_arguments,
    instruction_from_args,
)
from sim_pick_place.sim_client_node import SimClientNode
from sim_pick_place.utils.helpers import (
    _pose6d_to_posestamped_msg,
    _transform_matrix_to_pose6d,
)

from .guarded_executor import GuardedMotionExecutor
from .recovery import TargetLostError, TargetRecoveryMixin
from .worker import HighRateTrackingWorker, TargetMovedError


class _DirectMaskFoundationPose(FoundationPoseEstimationNode):
    """Send the synchronized tracker/SAM mask directly to FoundationPose."""

    def __init__(self, *args, target_mask, **kwargs):
        super().__init__(*args, **kwargs)
        self.target_mask = np.asarray(target_mask, dtype=bool)

    def _load_mask(self, target):
        self._validate_mask_geometry(self.target_mask, target)
        return self.target_mask.copy()


class FoundationPoseGraspNode(TargetRecoveryMixin, SimClientNode):
    """Use high-rate motion events and stable-frame FoundationPose."""

    def __init__(
        self,
        rgbd_node,
        instruction="",
        selection_frame=None,
        single_instance_reanchor=False,
    ):
        super().__init__("foundationpose_tracking_grasp")
        self.rgbd_node = rgbd_node
        self.instruction = instruction.strip()
        self.selection_frame = selection_frame

        # 0 means: keep tracker frames at most 640 pixels wide.
        self.declare_parameter("tracking_scale", 0.0)
        self.declare_parameter("tracking_rate", 30.0)
        self.declare_parameter("history_frames", 600)
        # Universal mode: a static target may proceed, while any observed
        # movement still invalidates perception/planning and must settle first.
        self.declare_parameter("require_disturbance", False)
        self.declare_parameter("move_pixel", 2.5)
        self.declare_parameter("move_depth_m", 0.008)
        self.declare_parameter("move_iou", 0.94)
        self.declare_parameter("stable_pixel", 0.8)
        self.declare_parameter("stable_depth_m", 0.003)
        self.declare_parameter("stable_iou", 0.98)
        self.declare_parameter("stable_duration", 1.0)
        self.declare_parameter("stable_frames", 8)
        self.declare_parameter("min_features", 8)
        self.declare_parameter("lost_depth_jump_m", 0.20)
        self.declare_parameter("reanchor_min_iou", 0.20)
        self.declare_parameter(
            "single_instance_reanchor",
            bool(single_instance_reanchor),
        )
        self.declare_parameter("lost_recovery_attempts", 3)
        config = MotionConfig(
            move_pixel=float(self.get_parameter("move_pixel").value),
            move_depth_m=float(self.get_parameter("move_depth_m").value),
            move_iou=float(self.get_parameter("move_iou").value),
            stable_pixel=float(self.get_parameter("stable_pixel").value),
            stable_depth_m=float(self.get_parameter("stable_depth_m").value),
            stable_iou=float(self.get_parameter("stable_iou").value),
            stable_duration_s=float(
                self.get_parameter("stable_duration").value
            ),
            stable_frames=int(self.get_parameter("stable_frames").value),
            min_features=int(self.get_parameter("min_features").value),
            lost_depth_jump_m=float(
                self.get_parameter("lost_depth_jump_m").value
            ),
        )
        requested_scale = float(self.get_parameter("tracking_scale").value)
        if requested_scale <= 0.0:
            current_rgb, _, _ = rgbd_node.get_latest_rgbd()
            source_width = (
                current_rgb.shape[1] if current_rgb is not None else 1280
            )
            requested_scale = min(1.0, 640.0 / float(source_width))
        self.tracking = HighRateTrackingWorker(
            rgbd_node,
            config=config,
            scale=requested_scale,
            poll_rate=float(self.get_parameter("tracking_rate").value),
            history_frames=int(self.get_parameter("history_frames").value),
            require_disturbance=bool(
                self.get_parameter("require_disturbance").value
            ),
        )

        self.cbg = ReentrantCallbackGroup()
        self.arm_api = ArmApi2Client(self, callback_group=self.cbg)
        self.gripper = GripperController(self, self.arm_api, self.cbg)
        self.motion = GuardedMotionExecutor(
            self,
            self.arm_api,
            self.gripper,
            self.cbg,
            motion_guard=self.tracking.assert_unchanged,
        )

        self.status_pub = self.create_publisher(
            String,
            "/my_course_pkg/foundationpose_tracking/status",
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/my_course_pkg/foundationpose_tracking/object_pose",
            10,
        )
        self.session_dir = (
            OUTPUT_DIR / "foundationpose_tracking" / str(int(time.time()))
        )
        self.selected_path = self.session_dir / "selected_object.json"
        self.target_name = ""
        self.grounding_prompt = ""
        self.accepted_sam2_class_names = ()
        self.drop_target = None
        self.recovery_attempts = 0
        self.execution_started = False

    def set_instruction(self, instruction, selection_frame):
        instruction = str(instruction).strip()
        if not instruction:
            raise ValueError("Instruction cannot be empty.")
        self.instruction = instruction
        self.selection_frame = selection_frame

    def _status(self, value):
        message = String()
        message.data = value
        self.status_pub.publish(message)
        self.get_logger().info(value)

    @staticmethod
    def pose_message(pose_6d):
        return _pose6d_to_posestamped_msg(pose_6d[:6], "world")

    def _wait_for_frame(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            frame = self.rgbd_node.get_latest_rgbd()
            if frame[0] is not None:
                return frame
            time.sleep(0.05)
        raise TimeoutError("Timed out waiting for a synchronized RGB-D frame.")

    @staticmethod
    def _save_frame(directory, rgb, depth):
        directory.mkdir(parents=True, exist_ok=True)
        rgb_path = directory / "rgb.png"
        depth_path = directory / "depth.npy"
        cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        np.save(depth_path, np.asarray(depth, dtype=np.float32))
        return rgb_path, depth_path

    def _write_selection(
        self,
        selection,
        classification=None,
        drop_target=None,
        overview=None,
    ):
        payload = {
            "user_instruction": self.instruction,
            "candidates": selection["candidates"],
            "selected_object_name": self.target_name,
            "target_region": selection["target_region"],
            "visual_attributes": selection["visual_attributes"],
            "grounding_prompt": selection["grounding_prompt"],
            "accepted_sam2_class_names": selection[
                "accepted_sam2_class_names"
            ],
        }
        if selection["matched_instruction_alias"] is not None:
            payload.update(
                {
                    "matched_instruction_alias": selection[
                        "matched_instruction_alias"
                    ],
                    "visual_target_description": selection[
                        "visual_target_description"
                    ],
                }
            )
        if classification is not None:
            payload.update(
                {
                    "target_category": classification["category"],
                    "category_confidence": classification["confidence"],
                    "category_reason": classification["reason"],
                }
            )
        if overview is not None:
            payload["overview_observation"] = overview
        if drop_target is not None:
            payload["drop_target"] = drop_target.as_dict()
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.selected_path.write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )

    def _initialize_target(self):
        if not self.instruction:
            raise ValueError("Instruction must be set before target selection.")
        if self.selection_frame is None:
            rgb, depth, stamp = self._wait_for_frame()
        else:
            rgb, depth, stamp = self.selection_frame
        initial_dir = self.session_dir / "initial"
        rgb_path, depth_path = self._save_frame(initial_dir, rgb, depth)
        selection = vlm_select_target(str(rgb_path), self.instruction)
        self.target_name = selection["selected_object_name"]
        self.grounding_prompt = selection["grounding_prompt"]
        self.accepted_sam2_class_names = tuple(
            selection["accepted_sam2_class_names"]
        )

        classification = vlm_classify_food(
            str(rgb_path),
            self.target_name,
        )
        overview = {
            "stamp": float(stamp),
            "camera_frame": CAMERA_FRAME,
            "camera_intrinsic": np.asarray(self.rgbd_node.K, dtype=float).tolist(),
            "robot_pose_world": np.asarray(
                self.motion.get_current_ee_pose_6d(frame_id="world"),
                dtype=float,
            ).tolist(),
            "rgb_path": str(rgb_path),
            "depth_path": str(depth_path),
        }
        self.drop_target = resolve_drop_target(
            classification["category"],
            observation_stamp=stamp,
            rgb_path=rgb_path,
            depth_path=depth_path,
            node=self,
            camera_frame=CAMERA_FRAME,
            diagnostics_dir=initial_dir / "sorting",
        )
        self._write_selection(
            selection,
            classification,
            self.drop_target,
            overview,
        )

        sam2_dir = initial_dir / "sam2"
        run_sam2_api(
            str(rgb_path),
            build_sam2_text_prompt([self.grounding_prompt]),
            output_dir=sam2_dir,
        )
        selector = FoundationPoseEstimationNode(
            selected_json=self.selected_path,
            sam2_response_json=sam2_dir / "response.json",
            rgb_path=rgb_path,
            depth_path=depth_path,
            output_dir=initial_dir / "mask_selection",
        )
        try:
            initial_mask = selector._load_mask(self.target_name)
        except RuntimeError as exc:
            raise RuntimeError(
                f"{exc} Selection image: {rgb_path}. "
                f"SAM2 result: {sam2_dir / 'response.json'}."
            ) from exc
        self.tracking.activate(rgb, depth, initial_mask, stamp)
        self._status(
            f"TARGET_LOCKED: {self.target_name}, "
            f"category={classification['category']}, "
            f"drop={self.drop_target.bin_name}; "
            "high-rate instance tracking active"
        )

    def _wait_for_stable_frame(self):
        last_state = None
        while rclpy.ok():
            snapshot = self.tracking.snapshot()
            if snapshot is None:
                time.sleep(0.05)
                continue
            if snapshot.state != last_state:
                loss_reason = (
                    f", reason={snapshot.loss_reason}"
                    if snapshot.loss_reason
                    else ""
                )
                self._status(
                    "MOTION_GATE: "
                    f"state={snapshot.state}, "
                    f"pixel={snapshot.pixel_motion:.2f}, "
                    f"depth={snapshot.depth_motion_m:.4f}m, "
                    f"iou={snapshot.mask_iou:.3f}"
                    f"{loss_reason}"
                )
                last_state = snapshot.state
            if snapshot.state == LOST:
                self._recover_lost_target()
                last_state = None
                continue
            if snapshot.state == STABLE:
                tracked = self.tracking.stable_frame()
                if tracked is not None:
                    return tracked
            time.sleep(0.03)
        raise RuntimeError("ROS stopped before the target became stable.")

    def _foundationpose_on_stable_frame(self, tracked):
        observation_dir = (
            self.session_dir
            / "stable_observations"
            / f"{int(tracked.stamp * 1e9)}"
        )
        rgb_path, depth_path = self._save_frame(
            observation_dir,
            tracked.rgb,
            tracked.depth,
        )
        sam2_dir = observation_dir / "sam2"
        run_sam2_api(
            str(rgb_path),
            build_sam2_text_prompt([self.grounding_prompt]),
            output_dir=sam2_dir,
        )
        self.tracking.assert_unchanged(tracked.movement_version)
        mask = self._select_reanchored_mask(
            sam2_dir / "response.json",
            tracked.mask,
        )

        fp_dir = observation_dir / "foundationpose"
        result = _DirectMaskFoundationPose(
            selected_json=self.selected_path,
            sam2_response_json=sam2_dir / "response.json",
            rgb_path=rgb_path,
            depth_path=depth_path,
            output_dir=fp_dir,
            target_mask=mask,
        ).run()
        self.tracking.assert_unchanged(tracked.movement_version)
        if not result.get("success"):
            raise RuntimeError(
                f"FoundationPose failed: {result.get('error', 'unknown error')}"
            )
        pose_path = fp_dir / "pose_result.json"
        world_pose = estimate_object_world_pose(
            self,
            pose_path,
            CAMERA_FRAME,
        )
        pose = self.pose_message(_transform_matrix_to_pose6d(world_pose))
        pose.header.stamp.sec = int(tracked.stamp)
        pose.header.stamp.nanosec = int(
            (tracked.stamp - int(tracked.stamp)) * 1e9
        )
        self.pose_pub.publish(pose)
        return pose_path, rgb_path, depth_path

    def run_task(self):
        self._initialize_target()
        while rclpy.ok():
            try:
                tracked = self._wait_for_stable_frame()
                self._status(
                    "FOUNDATIONPOSE_REQUEST: synchronized stable RGB-D-mask"
                )
                pose_path, _stable_rgb_path, _stable_depth_path = (
                    self._foundationpose_on_stable_frame(
                        tracked
                    )
                )
                self._status("PLANNING_FROM_FOUNDATIONPOSE")
                start_pose = self.motion.get_current_ee_pose_6d()
                results = plan_pick_place_candidates_from_perception(
                    self,
                    start_pose,
                    object_cam_pose_path=pose_path,
                    selected_object_path=self.selected_path,
                    camera_frame=CAMERA_FRAME,
                    drop_target=self.drop_target,
                    execution_mode="safe_place",
                )
                self.tracking.assert_unchanged(tracked.movement_version)
                self.motion.arm_guard(tracked.movement_version)
                if GRIPPER_COMMAND_MODE == "action":
                    self.gripper.ensure_gripper_ready()
                self._status("EXECUTING_GRASP_WITH_PREAPPROACH_GUARD")
                self.execution_started = True
                self.motion.execute_first_reachable_plan(
                    [result.plan for result in results]
                )
                self.execution_started = False
                self.motion.disarm_guard()
                self._status(
                    "RETURNING_TO_INITIAL_POSE: pick/place completed"
                )
                if not self.motion.return_to_initial_pose():
                    raise RuntimeError(
                        "Pick/place completed, but the robot could not return "
                        "to its initial joint pose."
                    )
                self._status("DONE: returned to initial pose")
                return
            except TargetLostError as exc:
                self._status(f"SAFE_STOP_LOST_TARGET: {exc}")
                if self.execution_started:
                    self.motion.disarm_guard()
                    self._status(
                        "ABORT_RETREAT: returning robot to the initial pose"
                    )
                    self.motion.return_to_initial_pose()
                    self.execution_started = False
                return
            except TargetMovedError as exc:
                self._status(f"RESULT_DISCARDED: {exc}")

    def stop(self):
        self.tracking.stop()


def _parse_main_args(args=None):
    parser = argparse.ArgumentParser(
        description="FoundationPose dynamic grasp from text or voice.",
    )
    add_instruction_arguments(parser)
    return parser.parse_known_args(args)


def main(args=None):
    cli_args, ros_args = _parse_main_args(args)
    rclpy.init(args=ros_args)
    rgbd = RGBDPerceptionNode(
        "foundationpose_tracking_rgbd",
        auto_save=False,
    )
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
            raise TimeoutError(
                "Timed out waiting for the first synchronized RGB-D frame."
            )

        node = FoundationPoseGraspNode(rgbd)
        executor.add_node(node)
        node._status(
            "RETURNING_TO_INITIAL_POSE: mandatory first grasp stage"
        )
        if not node.motion.return_to_initial_pose():
            raise RuntimeError(
                "Could not verify the robot initial pose; target selection "
                "has not started."
            )
        return_complete_stamp = rgbd.get_latest_rgbd()[2]
        deadline = time.monotonic() + 10.0
        while (
            rgbd.get_latest_rgbd()[2] <= return_complete_stamp
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        if rgbd.get_latest_rgbd()[2] <= return_complete_stamp:
            raise TimeoutError(
                "No fresh RGB-D frame arrived after returning to initial pose."
            )
        node._status("INITIAL_POSE_READY: target selection may start")

        print("\nRGB-D ready. Acquire one grasp instruction below.")
        instruction = instruction_from_args(
            cli_args,
            prompt=(
                "Instruction "
                "(the object may move after you press Enter): "
            ),
        )
        deadline = time.monotonic() + 30.0
        selection_frame = rgbd.get_latest_rgbd()
        while selection_frame[0] is None and time.monotonic() < deadline:
            time.sleep(0.05)
            selection_frame = rgbd.get_latest_rgbd()
        if selection_frame[0] is None:
            raise TimeoutError(
                "Timed out waiting for the instruction-time RGB-D frame."
            )
        node.set_instruction(instruction, selection_frame)
        node.run_task()
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
