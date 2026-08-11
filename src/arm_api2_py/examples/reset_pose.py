#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from arm_api2_py.arm_api2_client import ArmApi2Client
from geometry_msgs.msg import PoseStamped
import threading

class ClientNode(Node):
    def __init__(self):
        super().__init__('arm_api2_client')
        self.get_logger().info('ArmApi2Client Node has been started')
        self.arm_api2_client = ArmApi2Client(self)
        # RECORDING Jan 22
        self.default_reset_pose = [-0.788, 0.115, 0.196, 0.006, 0.9998, -0.018, 0.0027]
        #self.default_reset_pose = [-0.958, 0.107, 0.19, -0.715, 0.699, 0.015, 0.011]

        self.declare_parameter("reset_pose_array", self.default_reset_pose)
        reset_pose_array = self.get_parameter("reset_pose_array").get_parameter_value().double_array_value
        
        if len(reset_pose_array) != 7:
            self.get_logger().warning(f"reset_pose_array must have 7 elements [x,y,z,qx,qy,qz,qw] but received {reset_pose_array}. Continue using default pose instead...")
            reset_pose_array = self.default_reset_pose
        
        self.reset_pose = PoseStamped()
        self.reset_pose.header.frame_id = "base_link"
        self.reset_pose.pose.position.x = reset_pose_array[0]
        self.reset_pose.pose.position.y = reset_pose_array[1]
        self.reset_pose.pose.position.z = reset_pose_array[2]
        self.reset_pose.pose.orientation.x = reset_pose_array[3]
        self.reset_pose.pose.orientation.y = reset_pose_array[4]
        self.reset_pose.pose.orientation.z = reset_pose_array[5]
        self.reset_pose.pose.orientation.w = reset_pose_array[6]
        
        self.reset_thread = threading.Thread(target=self.reset_thread_func, daemon=True)
        self.reset_thread.start()
        self.done_event = threading.Event()

    def reset_thread_func(self):
        time.sleep(3.0)

        current_ee_pose = self.arm_api2_client.get_current_ee_pose()
        self.get_logger().debug(f"Current EE pose: {current_ee_pose}")
        self.get_logger().info(f"Sending goal pose")#
        try:
            self.arm_api2_client.change_state_to_servo_pos_ctl()
            self.get_logger().warn('Note: robot needs to be in SERVO_POS_CTL mode to interpolate')
            self.reset_pose.header.stamp = self.get_clock().now().to_msg() # Add a timestamp
            self.arm_api2_client.interpolate_to_pose(self.reset_pose, avg_speed=0.02)
            self.done_event.set()
        except Exception as e:
            self.get_logger().error(f"Failed to interpolate to pose: {e}")
        
def main(args=None):
    rclpy.init(args=args) 
    node = ClientNode()
    
    try:
        while rclpy.ok() and not node.done_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()