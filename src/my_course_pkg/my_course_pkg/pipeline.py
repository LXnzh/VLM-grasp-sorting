#!/usr/bin/env python3
"""High-level pipeline: init pose -> RGB-D -> LLM/SAM2 -> FoundationPose."""

import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_course_pkg.verify_init_pose import VerifyInitPoseNode
from my_course_pkg.rgbd_file_save import RgbdFileSaveNode
from my_course_pkg.perception.llm_sam2 import LlmSam2Node
from my_course_pkg.perception.foundationpose import FoundationPoseEstimationNode


def spin_until(executor, predicate, timeout_sec: float, label: str) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(f"Timed out while waiting for {label}.")


def wait_for_initial_pose(executor, timeout_sec: float) -> None:
    node = VerifyInitPoseNode()
    executor.add_node(node)
    spin_until(
        executor,
        lambda: node.pose_verified,
        timeout_sec,
        "initial pose verification",
    )
    node.get_logger().info("Pipeline: initial pose is ready.")
    return node


def capture_rgbd_once(executor, timeout_sec: float) -> None:
    node = RgbdFileSaveNode()
    executor.add_node(node)
    spin_until(
        executor,
        lambda: node.get_latest_rgbd()[0] is not None,
        timeout_sec,
        "first RGB-D frame",
    )
    if not node.save_current_frame():
        raise RuntimeError("RGB-D frame was received but saving failed.")
    node.get_logger().info("Pipeline: RGB-D frame saved.")
    return node


def run_pipeline(instruction: str, pose_timeout: float, rgbd_timeout: float) -> None:
    instruction = instruction.strip()
    if not instruction:
        raise ValueError("Instruction cannot be empty.")

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    ros_nodes = []
    try:
        print("Step 1: verify initial pose")
        ros_nodes.append(wait_for_initial_pose(executor, pose_timeout))

        print("Step 2: capture one RGB-D frame")
        ros_nodes.append(capture_rgbd_once(executor, rgbd_timeout))
    finally:
        executor.shutdown()
        for node in ros_nodes:
            try:
                executor.remove_node(node)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("Step 3: run LLM + SAM2")
    llm_sam2 = LlmSam2Node()
    llm_sam2.run(instruction)

    print("Step 4: run FoundationPose")
    fp = FoundationPoseEstimationNode()
    fp.run()

    print("Pipeline finished.")


def main() -> None:
    instruction = input("Instruction: ")
    pose_timeout = 60.0  # seconds
    rgbd_timeout = 30.0  # seconds
    run_pipeline(instruction, pose_timeout, rgbd_timeout)


if __name__ == "__main__":
    main()
