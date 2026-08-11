#!/usr/bin/env python3

import threading
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from rclpy.time import Time
from controller_manager_msgs.srv import SwitchController
from rclpy.qos import QoSProfile

from arm_api2_msgs.srv import ChangeState

class ServoWatchdog(Node):
    def __init__(self):
        super().__init__('servo_watchdog_node')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.avoid_ros_namespace_conventions = True

        # Create a publisher for the twist topic with zero velocity
        self.pub_twist = self.create_publisher(TwistStamped, '/arm/delta_twist_cmds', qos_profile)
        self.pub_jog = self.create_publisher(JointJog, '/arm/delta_joint_cmds', qos_profile)

        # Create a subscriber for the twist topic to check if messages are being published
        self.sub_twist = self.create_subscription(
            TwistStamped,
            '/arm/delta_twist_cmds',
            self.twist_callback,
            10
        )
        
        self.sub_jog = self.create_subscription(
            JointJog,
            '/arm/delta_joint_cmds',
            self.jog_callback,
            10
        )
        self.sub_arm_api2_state = self.create_subscription(
            String,
            '/arm/state/current_state',
            self.arm_api2_state_callback,
            10
        )
        
        self._service_client_switch_controller = self.create_client(
            SwitchController, "controller_manager/switch_controller")
        self._service_client_state_change = self.create_client(
            ChangeState, "arm/change_state")

        # Set a timer to publish a zero velocity twist message every second
        self.timer = self.create_timer(1.0, self.publish_zero_twist)

        # Flag to track whether we received a message
        self.received_message = False
        self.joint_names = []
        self.watchdog_active = False
        
        self.watchdog_counter = 0
        
        self.get_logger().info('Servo watchdog node started')

    def twist_callback(self, msg):
        """Callback that gets triggered when a new Twist message is received."""
        #self.get_logger().info('Received Twist message!')
        self.received_message = True  # Message received, reset the flag
    
    def jog_callback(self, msg):
        """Callback that gets triggered when a new Twist message is received."""
        #self.get_logger().info('Received JointJog message!')
        self.joint_names = msg.joint_names
        self.received_message = True  # Message received, reset the flag
    
    def arm_api2_state_callback(self, msg):
        """Callback that gets triggered when a new Twist message is received."""
        #self.get_logger().info('Received Twist message!')
        if msg.data == "SERVO_CTL":
            if self.watchdog_active == False:
                self.get_logger().info('arm_api2 is in SERVO_CTL mode, starting watchdog')
            self.watchdog_active = True
        else:
            if self.watchdog_active == True:
                self.get_logger().info('arm_api2 is not in SERVO_CTL mode anymore, stopping watchdog')
            self.watchdog_active = False
            self.watchdog_counter = 0

    def publish_zero_twist(self):
        """Publish a zero velocity twist message if no message was received in the last second."""
        if self.watchdog_active:
            
            if not self.received_message and self.watchdog_active:
                self.get_logger().info('No Twist/JointJog message received, publishing zero velocity message. Counter: ' + str(self.watchdog_counter))
                zero_twist = TwistStamped()
                zero_twist.header.stamp = Time().to_msg()
                zero_twist.header.frame_id = 'world'
                zero_twist.twist.linear.x = 0.0
                zero_twist.twist.linear.y = 0.0
                zero_twist.twist.linear.z = 0.0
                zero_twist.twist.angular.x = 0.0
                zero_twist.twist.angular.y = 0.0    
                zero_twist.twist.angular.z = 0.0
                self.pub_twist.publish(zero_twist)  # Publish zero velocity
                
                zero_jog = JointJog()
                zero_jog.header.stamp = Time().to_msg()
                zero_jog.joint_names = self.joint_names
                zero_jog.velocities = [0.0] * len(self.joint_names)
                self.pub_jog.publish(zero_jog)
                
                self.watchdog_counter += 1
                if self.watchdog_counter > 5:   # 5 seconds of no messages
                    self.get_logger().info('No Twist/JointJog message received for 5 seconds, switching to CART_TRAJ_CTL' + str(self.watchdog_counter))
                    thread = threading.Thread(target=self.change_state_to_cartesian_ctl)   
                    thread.start()
            else:
                #self.get_logger().info('Twist/JointJog message received in the last second')
                # Reset the if a message was received in the last second
                self.watchdog_counter = 0
                self.received_message = False
        else:
            #self.get_logger().info('Watchdog is not active')
            pass
            
    def switch_controller(self, start_controllers: List[str], stop_controllers: List[str]):
        """
        Sends a request to the service server to switch controllers.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        reference about available controllers in UR ROS2 driver: 
        https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html#commanding-controllers

        Args:
            start_controllers (List[str]): The controllers to start.
            stop_controllers (List[str]): The controllers to stop.

        Returns:
            bool: True if the controllers were switched, False otherwise.
        """
    
        
        if len(start_controllers) == 0 and len(stop_controllers) == 0:
            self.get_logger().info("No controllers to switch")
            return True
        
        request = SwitchController.Request()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        request.strictness = 2  # STRICT=2, BEST_EFFORT=1

        self.get_logger().info("Waiting for service server...")

        self._service_client_switch_controller.wait_for_service()

        self.get_logger().info(
            "switch_controller request sent, waiting for response...")

        response = self._service_client_switch_controller.call(request)

        if response.ok:
            self.get_logger().info(f"Controllers switched")
        else:
            self.get_logger().info(f"Switching controllers failed")

        return response.ok

    def change_state_to(self, state: str):
        """
        Sends a request to the service server to change the arm state to the specified state.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            state (str): The state to change the arm to. One of "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL"

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = ChangeState.Request()
        request.state = state

        self.get_logger().info("Waiting for service server...")

        self._service_client_state_change.wait_for_service()

        self.get_logger().info(
            "change_state request sent, waiting for response...")

        response = self._service_client_state_change.call(request)

        if response.success:
            self.get_logger().info(f"State changed to {state}")
        else:
            self.get_logger().info(f"State change to {state} failed")

        return response.success
    
    def change_state_to_cartesian_ctl(self):
        """
        Sends a request to the service server to change the arm state to cartesian control and switch controllers.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        stop_controllers = ["forward_position_controller"]
        start_controllers = ["scaled_joint_trajectory_controller"]
        res1 = self.switch_controller(start_controllers, stop_controllers)
        res2 = self.change_state_to("CART_TRAJ_CTL")
        return res1 and res2


def main(args=None):
    rclpy.init(args=args)
    node = ServoWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
