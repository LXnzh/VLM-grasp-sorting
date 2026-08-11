#!/usr/bin/env python3

import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class GripperActionClient(Node):

    def __init__(self):
        super().__init__("gripper_action_client")
        self._action_client = ActionClient(
            self, GripperCommand, "robotiq_2f_urcap_adapter/gripper_command"
        )
        self.running = False

        self.thread = threading.Thread(target=self.doing_thread)
        self.thread.start()

    def send_gripper_command(self, command: GripperCommandMsg):
        self.running = True
        goal_msg = GripperCommand.Goal()
        goal_msg.command = command

        self.get_logger().info("Waiting for action server...")

        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.running = False
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        self.running = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")

    def open_gripper(self):
        self.get_logger().info("Opening gripper...")
        command = GripperCommandMsg()
        command.position = 0.0  # rad
        command.max_effort = 20.0  # N
        self.send_gripper_command(command)

    def close_gripper(self):
        self.get_logger().info("Closing gripper...")
        command = GripperCommandMsg()
        command.position = 0.8  # rad
        command.max_effort = 20.0  # N
        self.send_gripper_command(command)

    def doing_thread(self):

        # Close the gripper
        self.close_gripper()

        while self.running:
            pass

        # wait 2 seconds
        time.sleep(2.0)

        # Open the gripper
        self.open_gripper()

        while self.running:
            pass

        # wait 2 seconds
        time.sleep(2.0)

        # Close the gripper halfway
        command = GripperCommandMsg()
        command.position = 0.4  # rad
        command.max_effort = 20.0  # N
        self.send_gripper_command(command)

        while self.running:
            pass

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    gripper_action_client = GripperActionClient()

    executor = MultiThreadedExecutor()
    executor.add_node(gripper_action_client)

    try:
        executor.spin()

    finally:
        gripper_action_client.destroy_node()


if __name__ == "__main__":
    main()
