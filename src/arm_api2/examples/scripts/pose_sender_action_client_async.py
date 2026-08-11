#!/usr/bin/env python3

import rclpy
from arm_api2_msgs.action import MoveCartesian
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node


class PoseSenderActionClientAsync(Node):

    def __init__(self):
        super().__init__("pose_sender_action_client_async")
        self._action_client = ActionClient(self, MoveCartesian, "arm/move_to_pose")

    def send_goal(self, goal):
        goal_msg = MoveCartesian.Goal()
        goal_msg.goal = goal

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
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Feedback: {0}".format(feedback.status))


def main(args=None):
    rclpy.init(args=args)

    action_client = PoseSenderActionClientAsync()

    goal = PoseStamped()
    goal.header.frame_id = "world"
    goal.pose.position.x = 0.1
    goal.pose.position.y = -0.7
    goal.pose.position.z = 1.4
    goal.pose.orientation.x = 1.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0

    action_client.send_goal(goal)

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
