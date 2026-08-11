import threading
import time

import rclpy
from arm_api2_py.arm_api2_client import ArmApi2Client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sim_pick_place.sim_client_node import SimClientNode

from my_course_pkg.grasp.config import (
    CAMERA_FRAME,
    GRIPPER_COMMAND_MODE,
)
from my_course_pkg.grasp.executor import ArmMotionExecutor
from my_course_pkg.grasp.grasp_selector import tool_z_down_angle_deg
from my_course_pkg.grasp.gripper_control import GripperController
from my_course_pkg.grasp.pick_place_planner import (
    plan_pick_place_candidates_from_perception,
)
from my_course_pkg.grasp.trajectory_planner import print_plan_summary
from my_course_pkg.grasp.transforms import print_pose_summary


class GraspDemoNode(SimClientNode):
    def __init__(self):
        super().__init__("grasp_demo_node")
        self.cbg = ReentrantCallbackGroup()
        self.arm_api2_client = ArmApi2Client(self, callback_group=self.cbg)
        self.gripper_controller = GripperController(
            self,
            self.arm_api2_client,
            self.cbg,
        )
        self.motion_executor = ArmMotionExecutor(
            self,
            self.arm_api2_client,
            self.gripper_controller,
            self.cbg,
        )

    def run(self):
        self.motion_executor.enter_servo_pos_mode()
        time.sleep(1.0)

        print(f"Using camera frame: {CAMERA_FRAME}")

        ee_pose_6d = self.motion_executor.get_current_ee_pose_6d()
        planning_results = plan_pick_place_candidates_from_perception(
            self,
            ee_pose_6d,
        )
        print_pose_summary("object world", planning_results[0].T_world_obj)

        if GRIPPER_COMMAND_MODE == "action":
            self.gripper_controller.ensure_gripper_ready()
        else:
            print(f"Using gripper command mode: {GRIPPER_COMMAND_MODE}")

        for planning_result in planning_results:
            print(
                "Candidate "
                f"{planning_result.candidate_index}/"
                f"{planning_result.candidate_count}"
            )
            print_pose_summary("grasp world", planning_result.T_world_grasp)
            print(
                f"grasp tool +Z angle to global -Z: "
                f"{tool_z_down_angle_deg(planning_result.T_world_grasp):.2f} deg"
            )
            print_plan_summary(planning_result.plan)

        self.motion_executor.execute_first_reachable_plan(
            [planning_result.plan for planning_result in planning_results]
        )
        if self.motion_executor.debug_stop_reached:
            print(
                "Grasp debug stop reached; staying at the final grasp pose "
                "without returning home. Press Ctrl-C when inspection is complete."
            )
            while rclpy.ok():
                time.sleep(0.25)
            return
        returned_home = self.motion_executor.return_to_initial_pose()
        if not returned_home:
            self.get_logger().warn("Continuing shutdown after return-to-initial failure.")
        print("Pick and place completed.")
        time.sleep(0.5)


def run():
    rclpy.init()
    node = GraspDemoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    worker_error = []

    def _run_worker():
        try:
            node.run()
        except Exception as exc:
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
