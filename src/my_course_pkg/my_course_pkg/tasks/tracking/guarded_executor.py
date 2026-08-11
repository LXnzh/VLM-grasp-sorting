"""Motion executor that consults the high-rate target movement gate."""

import time

from arm_api2_msgs.action import MoveCartesian

from my_course_pkg.grasp.executor import ArmMotionExecutor

from .worker import TargetMovedError


class GuardedMotionExecutor(ArmMotionExecutor):
    """Cancel robot motion when the high-rate gate reports target movement."""

    def __init__(self, *args, motion_guard, **kwargs):
        super().__init__(*args, **kwargs)
        self.motion_guard = motion_guard
        self.movement_version = None
        self.guard_enabled = False

    def arm_guard(self, movement_version):
        self.movement_version = int(movement_version)
        self.guard_enabled = True

    def disarm_guard(self):
        self.guard_enabled = False

    def _check_target(self, require_stable=False):
        if self.guard_enabled:
            self.motion_guard(
                self.movement_version,
                require_stable=require_stable,
            )

    def move_to_pose(self, pose_msg):
        self._check_target()
        return super().move_to_pose(pose_msg)

    def move_with_moveit(self, target_pose_6d):
        self._check_target()
        pose_msg = self.node.pose_message(target_pose_6d)
        client = self.arm_api2_client._action_client_cartesian
        if not client.wait_for_server(timeout_sec=2.0):
            return False

        goal = MoveCartesian.Goal()
        goal.goal = pose_msg
        send_future = client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.02)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        try:
            while not result_future.done():
                self._check_target()
                time.sleep(0.05)
        except TargetMovedError:
            cancel_future = goal_handle.cancel_goal_async()
            deadline = time.monotonic() + 2.0
            while not cancel_future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            raise
        return bool(result_future.result().result.success)

    def execute_step(self, step):
        if step.name == "move_to_pre_grasp":
            # The fixed camera sees the robot enter and eventually occlude the
            # target ROI. Optical flow/depth in that ROI then describe robot
            # self-motion, not object motion. Perform one final stable check
            # immediately before approach, then suspend the visual guard.
            self._check_target(require_stable=True)
            self.node._status(
                "TARGET_CONFIRMED_BEFORE_APPROACH: suspending visual motion "
                "gate while the robot occludes the target"
            )
            self.disarm_guard()
        elif step.name == "close_gripper_at_grasp":
            self._check_target(require_stable=True)
        result = super().execute_step(step)
        if step.name == "close_gripper_at_grasp":
            # Once grasped, object motion is expected and no longer invalidates
            # the already completed target acquisition.
            self.guard_enabled = False
        return result
