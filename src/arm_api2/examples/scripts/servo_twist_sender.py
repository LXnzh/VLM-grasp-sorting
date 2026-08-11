#!/usr/bin/env python3


import numpy as np
import transforms3d.quaternions as quat
import copy

import time
import rclpy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, TransformStamped
import math
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster



class ServoTwistSender(Node):
    def __init__(self):
        super().__init__('servo_pose_sender')
        self.twist_pub = self.create_publisher(TwistStamped, '/arm/delta_twist_cmds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/arm/state/current_pose', self.pose_callback, 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.current_pose = None
        self.start_pose = None
        
        timer_period = 0.1  # seconds
        self.rate = self.create_timer(timer_period, self.run)
        

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        
    def send_transform(self, name, pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = name

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        
        
        self.tf_static_broadcaster.sendTransform(t)

    def run(self):
        if self.current_pose is None:
            return
        if self.start_pose is None:
            self.start_time = time.time()
            self.start_pose = copy.deepcopy(self.current_pose)
            
            q_delta = [0.9848078, 0, 0, 0.1736482]
            q_target = [self.start_pose.orientation.w, self.start_pose.orientation.x, self.start_pose.orientation.y, self.start_pose.orientation.z]
            self.get_logger().info('q_target 1: ' + str(q_target))
            q_target = quat.qmult(q_target, q_delta)
            self.get_logger().info('q_target 2: ' + str(q_target))
            q_target = quat.qmult(q_target, q_delta)
            self.get_logger().info('q_target 3: ' + str(q_target))
            
            self.start_pose.orientation.w = q_target[0]
            self.start_pose.orientation.x = q_target[1]
            self.start_pose.orientation.y = q_target[2]
            self.start_pose.orientation.z = q_target[3]
            
            
        else:
            elapsed_time = (time.time() - self.start_time)
            angle = elapsed_time * 2 * math.pi / 20  # Complete a circle in 20 seconds
            offset_x = 0.1 * math.cos(angle) - 0.1 # 10 cm radius
            offset_y = 0.1 * math.sin(angle)
            
            gain = 15.0
            
            
            #self.get_logger().info('Elapsed time: %f, angle: %f, offset_x: %f, offset_y: %f' % (elapsed_time, angle, offset_x, offset_y))
            
            target_pose = Pose()
            target_pose.position.x = self.start_pose.position.x + offset_x
            target_pose.position.y = self.start_pose.position.y + offset_y
            target_pose.position.z = self.start_pose.position.z
            target_pose.orientation = self.start_pose.orientation
            
            self.send_transform('target_pose', target_pose)
            
            
            
            # create twist from delta between current and target pose
            twist = TwistStamped()
            twist.header.stamp = Time().to_msg()
            twist.header.frame_id = 'world'
            twist.twist.linear.x = (target_pose.position.x - self.current_pose.position.x) * gain
            twist.twist.linear.y = (target_pose.position.y - self.current_pose.position.y) * gain
            twist.twist.linear.z = (target_pose.position.z - self.current_pose.position.z) * gain
            
            
            dt = 0.1
            q_current = [self.current_pose.orientation.w, self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z]
            q_target = [target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z]
            angular_velocity = quaternion_to_angular_velocity(q_current, q_target, dt)
            self.get_logger().info('Angular velocity: ' + str(angular_velocity))
            twist.twist.angular.x = angular_velocity[0]
            twist.twist.angular.y = angular_velocity[1]
            twist.twist.angular.z = angular_velocity[2]
            
            
            self.twist_pub.publish(twist)
            #self.get_logger().info('Published twist message')
                  
            
def quaternion_to_angular_velocity(q_current, q_target, dt):
    """
    Computes the required angular velocity in global frame to rotate from q_current to q_target.

    Args:
        q_current (list or np.array): Current quaternion [w, x, y, z]
        q_target (list or np.array): Target quaternion [w, x, y, z]
        dt (float): Time step over which rotation is applied

    Returns:
        np.array: Angular velocity (wx, wy, wz) in global frame
    """
    # Ensure numpy arrays
    q_current = np.array(q_current)
    q_target = np.array(q_target)

    # Compute relative quaternion: q_relative = q_target * q_current⁻¹
    q_current_inv = quat.qinverse(q_current)
    q_relative = quat.qmult(q_target, q_current_inv)
    
    if q_relative[0] > 1.0:
        print('q_relative:' + str(q_relative))
        return np.array([0.0, 0.0, 0.0])

    # Extract angle and axis from q_relative
    theta = 2 * np.arccos(q_relative[0])  # q_relative[0] is the w-component

    # Avoid division by zero when theta is very small
    sin_half_theta = np.sqrt(1 - q_relative[0]**2)
    if np.isclose(sin_half_theta, 0):
        angular_velocity = np.array([0.0, 0.0, 0.0])  # No rotation needed
    else:
        axis = q_relative[-3:] / sin_half_theta  # Normalize rotation axis
        angular_velocity = (theta / dt) * axis  # ω = (θ/Δt) * axis

    return angular_velocity



def main(args=None):
    rclpy.init(args=args)

    action_client = ServoTwistSender()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
