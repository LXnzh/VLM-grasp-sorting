#!/usr/bin/env python3

import rclpy
from arm_api2_msgs.action import MoveJoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.node import Node


class JointSenderActionClientAsync(Node):

    def __init__(self):
        super().__init__("joint_sender_action_client_async")
        self._action_client = ActionClient(self, MoveJoint, "arm/move_to_joint")

    def send_goal(self, goal: JointState):
        goal_msg = MoveJoint.Goal()
        goal_msg.joint_state = goal

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

    action_client = JointSenderActionClientAsync()
    names =["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    positions = [-4.0,-2.067,1.849,-1.428,-1.395,2.045]
    goal = JointState()
    goal.name = names
    goal.position = positions

    action_client.send_goal(goal)

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
