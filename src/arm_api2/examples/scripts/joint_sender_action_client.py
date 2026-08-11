#!/usr/bin/env python3

import threading
import rclpy
from arm_api2_msgs.action import MoveJoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.node import Node


class JointSenderActionClient(Node):

    def __init__(self):
        super().__init__("joint_sender_action_client")
        self._action_client = ActionClient(self, MoveJoint, "arm/move_to_joint")
        
        # new thread to handle the actual task without blocking the main thread
        self._execution_thread = threading.Thread(target=self._execute)
        self._execution_thread.start()

    def _send_goal(self, goal: JointState):
        goal_msg = MoveJoint.Goal()
        goal_msg.joint_state = goal

        self.get_logger().info("Waiting for action server...")

        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")

        result = self._action_client.send_goal(goal_msg)
        
        self.get_logger().info("Result: {0}".format(result))
        rclpy.shutdown()
    
    def _execute(self):
        
        names =["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
        positions = [-4.0,-2.067,1.849,-1.428,-1.395,2.045]
        goal = JointState()
        goal.name = names
        goal.position = positions

        self._send_goal(goal)
        

def main(args=None):
    rclpy.init(args=args)

    action_client = JointSenderActionClient()
    
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
