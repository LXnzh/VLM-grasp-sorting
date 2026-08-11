#!/usr/bin/env python3

import threading
import rclpy
from arm_api2_msgs.action import MoveCartesian
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node


class PoseSenderActionClient(Node):

    def __init__(self):
        super().__init__("pose_sender_action_client")
        self._action_client = ActionClient(self, MoveCartesian, "arm/move_to_pose")
        
        # new thread to handle the actual task without blocking the main thread
        self._execution_thread = threading.Thread(target=self._execute)
        self._execution_thread.start()

    def _send_goal(self, goal):
        goal_msg = MoveCartesian.Goal()
        goal_msg.goal = goal

        self.get_logger().info("Waiting for action server...")

        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")

        result = self._action_client.send_goal(goal_msg)
        
        self.get_logger().info("Result: {0}".format(result))
        rclpy.shutdown()
        
    def _execute(self):
        
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x = 0.1
        goal.pose.position.y = -0.7
        goal.pose.position.z = 1.4
        goal.pose.orientation.x = 1.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0
        
        self._send_goal(goal)

        
        

def main(args=None):
    rclpy.init(args=args)

    action_client = PoseSenderActionClient()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
