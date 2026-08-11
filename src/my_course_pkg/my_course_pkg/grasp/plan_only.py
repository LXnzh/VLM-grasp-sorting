"""Safe Apple A3 entrypoint that plans pre-grasp poses without executing them."""

from dataclasses import dataclass
import threading
import time

import rclpy
from arm_api2_py.arm_api2_client import ArmApi2Client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sim_pick_place.sim_client_node import SimClientNode
from sim_pick_place.utils.helpers import (
    _pose6d_to_posestamped_msg,
    _posestamped_msg_to_pose6d,
)

from my_course_pkg.grasp.pick_place_planner import (
    plan_pick_place_candidates_from_perception,
)


class PlanOnlyVerificationError(RuntimeError):
    """Raised when A3 cannot produce a reachable pre-grasp plan."""


class PlanOnlyCleanupError(RuntimeError):
    """Raised when the MoveIt server cannot be returned to execution mode."""


@dataclass(frozen=True)
class PlanOnlySessionResult:
    planning_result: object
    attempted_candidates: int


def get_current_ee_pose_6d(arm_api2_client, timeout_sec=5.0, poll_period_sec=0.1):
    """Wait for one usable current end-effector pose and return it as 6D."""
    deadline = time.monotonic() + timeout_sec
    while True:
        ee_pose = arm_api2_client.get_current_ee_pose()
        if ee_pose.header.frame_id:
            return _posestamped_msg_to_pose6d(ee_pose)
        if time.monotonic() >= deadline:
            raise PlanOnlyVerificationError(
                "Current EE pose was not received on /arm/state/current_pose."
            )
        time.sleep(poll_period_sec)


def _verify_pregrasp_candidates(node, arm_api2_client, planning_results):
    if not planning_results:
        raise PlanOnlyVerificationError(
            "Perception and corridor filtering produced no grasp candidates."
        )

    if not arm_api2_client.change_state_to("CART_TRAJ_CTL"):
        raise PlanOnlyVerificationError(
            "Could not switch arm_api2 to CART_TRAJ_CTL for MoveIt plan-only goals."
        )

    candidate_count = len(planning_results)
    for attempt, planning_result in enumerate(planning_results, start=1):
        candidate_index = getattr(planning_result, "candidate_index", attempt)
        node.get_logger().info(
            "A3 MoveIt plan-only attempt "
            f"{attempt}/{candidate_count} (candidate {candidate_index})."
        )
        pose_msg = _pose6d_to_posestamped_msg(
            planning_result.plan.pre_grasp_pose_6d[:6],
            frame_id="world",
        )
        if arm_api2_client.move_to_pose(pose_msg):
            node.get_logger().info(
                "A3 MoveIt plan-only accepted pre-grasp candidate "
                f"{candidate_index} after {attempt} attempt(s)."
            )
            return PlanOnlySessionResult(
                planning_result=planning_result,
                attempted_candidates=attempt,
            )

        node.get_logger().warn(
            "A3 MoveIt plan-only rejected pre-grasp candidate "
            f"{candidate_index}; trying the next corridor-safe candidate."
        )

    raise PlanOnlyVerificationError(
        "No grasp candidate has a reachable pre-grasp pose according to MoveIt "
        "plan-only verification."
    )


def run_plan_only_session(
    node,
    arm_api2_client,
    *,
    planner=None,
    current_pose_reader=None,
):
    """Run one fail-safe A3 session and always restore ``planonly=False``.

    Plan-only is enabled before perception/planning so a future change cannot
    accidentally introduce an executable MoveIt goal before the guard is set.
    The only action goal sent here is ``move_to_pose`` while that guard is
    active.  Cartesian servo and gripper interfaces are deliberately absent.
    """
    planner = planner or plan_pick_place_candidates_from_perception
    current_pose_reader = current_pose_reader or get_current_ee_pose_6d
    primary_error = None
    primary_traceback = None
    session_result = None

    try:
        if not arm_api2_client.set_planonly(True):
            raise PlanOnlyVerificationError(
                "Could not enable arm_api2 plan-only mode; no planning goal was sent."
            )
        node.get_logger().info(
            "A3 plan-only guard enabled; MoveIt may plan but must not execute."
        )
        start_pose_6d = current_pose_reader(arm_api2_client)
        planning_results = planner(node, start_pose_6d)
        session_result = _verify_pregrasp_candidates(
            node,
            arm_api2_client,
            planning_results,
        )
    except BaseException as exc:  # cleanup also applies to Ctrl-C/system exits
        primary_error = exc
        primary_traceback = exc.__traceback__

    cleanup_error = None
    try:
        if not arm_api2_client.set_planonly(False):
            cleanup_error = PlanOnlyCleanupError(
                "Could not disable arm_api2 plan-only mode; block all subsequent "
                "motion until /arm/set_planonly is explicitly restored to false."
            )
    except BaseException as exc:
        cleanup_error = PlanOnlyCleanupError(
            "Exception while disabling arm_api2 plan-only mode; block all "
            f"subsequent motion until recovery: {exc}"
        )

    if cleanup_error is not None:
        node.get_logger().error(str(cleanup_error))
        if primary_error is not None:
            raise cleanup_error from primary_error
        raise cleanup_error

    node.get_logger().info("A3 plan-only guard restored to false.")
    if primary_error is not None:
        raise primary_error.with_traceback(primary_traceback)
    return session_result


class GraspPlanOnlyNode(SimClientNode):
    """ROS adapter for the non-executing A3 planning session."""

    def __init__(self):
        super().__init__("grasp_plan_only_node")
        self.callback_group = ReentrantCallbackGroup()
        self.arm_api2_client = ArmApi2Client(
            self,
            callback_group=self.callback_group,
        )

    def run_plan_only(self):
        result = run_plan_only_session(self, self.arm_api2_client)
        selected = result.planning_result
        print(
            "A3 PLAN-ONLY PASS: pre-grasp candidate "
            f"{selected.candidate_index} is corridor-safe and MoveIt-reachable; "
            f"attempts={result.attempted_candidates}. No trajectory or gripper "
            "command was executed."
        )
        return result


def run():
    """Console entrypoint for one Apple A3 plan-only trial."""
    rclpy.init()
    node = GraspPlanOnlyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    worker_error = []

    def _run_worker():
        try:
            node.run_plan_only()
        except BaseException as exc:
            worker_error.append(exc)

    worker = threading.Thread(target=_run_worker, daemon=True)
    worker.start()
    try:
        while rclpy.ok() and worker.is_alive():
            executor.spin_once(timeout_sec=0.1)
        worker.join()
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if worker_error:
        raise worker_error[0]


if __name__ == "__main__":
    run()
