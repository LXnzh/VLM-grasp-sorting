#!/usr/bin/env python3

import time
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from arm_api2_py.arm_api2_client import ArmApi2Client

from scipy.spatial.transform import Rotation as R

class ArmApi2ClientExample(Node):
    
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('MyNode has been started')
        self.arm_api2_client = ArmApi2Client(self)

        # start the user input thread
        self.user_input_thread = threading.Thread(target=self.user_input)  
        self.user_input_thread.start()

    def user_input(self):
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "world"
        goal_pose.pose.position.x = -0.4
        goal_pose.pose.position.y = -0.9
        goal_pose.pose.position.z = 1.4
        goal_pose.pose.orientation.x = 1.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        names =["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
        positions = [0.0,-np.pi/2,-np.pi/2,-np.pi/2,np.pi/2,np.pi]
        goal_joint = JointState()
        goal_joint.name = names
        goal_joint.position = positions

        twist_stamped = TwistStamped()
        twist_stamped.header.frame_id = "tcp"
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = 0.0

        time.sleep(1) # wait for current pose to be published

        while True:

            current_ee_pose = self.arm_api2_client.get_current_ee_pose()
            print("Current pose: ", current_ee_pose)

            print("Select wheter to \n\
                  - send [c]artesian, [j]oint_state or [p]ath goal, \n\
                  - [go] open gripper, [gc] close gripper,\n\
                  - [sws] switch controller to servo, \n\
                  - [swj] switch controller to joint control, \n\
                  - [swc] switch controller to cartesian control, \n\
                  - [l]eft twist movement, [r]ight twist movement, \n\
                  - [1] base rotation ccw, [2] base rotation cw, \n \
                  - [s]top movement, \n\
                  - [pot] plan only true, [pof] plan only false, \n\
                  - or anything else to end:")
            
            cmd = input()

            if cmd == 'c':
                #self.arm_api2_client.set_eelink("orbbec_base")
                self.arm_api2_client.set_vel_acc(0.1, 0.1)
                self.arm_api2_client.change_state_to_cartesian_ctl()
                self.arm_api2_client.move_to_pose(goal_pose)

            elif cmd == 'j':
                self.arm_api2_client.set_vel_acc(0.2, 0.2)
                self.arm_api2_client.change_state_to_joint_ctl()
                self.arm_api2_client.move_to_joint(goal_joint)
            
            elif cmd == 'p':
                self.arm_api2_client.set_eelink("tcp")
                current_pose = self.arm_api2_client.get_current_ee_pose()
                path = create_sample_trajectory(self, current_pose)
                self.arm_api2_client.change_state_to_cartesian_ctl()
                self.arm_api2_client.move_to_pose_path(path)
            
            elif cmd == 'i':
                self.arm_api2_client.change_state_to_servo_pos_ctl()
                self.arm_api2_client.interpolate_to_pose(goal_pose, avg_speed=0.05, acceleration=0.1)
            
            elif cmd == 'go':
                #self.arm_api2_client.send_gripper_command(0.0, 140.0) # open gripper
                self.arm_api2_client.gripper_open()

            elif cmd == 'gc':
                #self.arm_api2_client.send_gripper_command(0.8, 140.0) # close gripper
                self.arm_api2_client.gripper_close()
            
            elif cmd == 'sws':
                self.arm_api2_client.change_state_to_servo_ctl()
            
            elif cmd == 'swj':
                self.arm_api2_client.change_state_to_joint_ctl()
            
            elif cmd == 'swc':
                self.arm_api2_client.change_state_to_cartesian_ctl()
            
            elif cmd == 'l':
                twist_stamped.twist.linear.y = 0.05
                self.arm_api2_client.send_twist_cmd(twist_stamped)
            
            elif cmd == 'r':
                twist_stamped.twist.linear.y = -0.05
                self.arm_api2_client.send_twist_cmd(twist_stamped)
            
            elif cmd == 's':
                twist_stamped.twist.linear.y = 0.0
                self.arm_api2_client.send_twist_cmd(twist_stamped)
                vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.arm_api2_client.send_joint_vel_cmd(joint_names=names, joint_velocities=vels)
            
            elif cmd == 'pot':
                self.arm_api2_client.set_planonly(True)
            
            elif cmd == 'pof':
                self.arm_api2_client.set_planonly(False)
            
            elif cmd == '1':
                vels = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.arm_api2_client.send_joint_vel_cmd(joint_names=names, joint_velocities=vels)
            
            elif cmd == '2':
                vels = [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.arm_api2_client.send_joint_vel_cmd(joint_names=names, joint_velocities=vels)
                
            elif cmd == 'pg':
                print("Publishing gripper command without waiting for result")
                self.arm_api2_client.send_gripper_command(0.5, 100.0, wait_for_result=False)
            

            else:
                break;
        rclpy.shutdown()

def to_T(pose: PoseStamped):
    T = np.eye(4)
    T[:3, :3] = R.from_quat([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]).as_matrix()
    T[:3, 3] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    return T


def create_sample_trajectory(self, ref_pose: PoseStamped):

    data = [ # x, y, z
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.1, 0.1, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.2, 0.0],
            [0.1, 0.2, 0.0],
            [0.0, 0.0, 0.0],]
    path = []
    for p in data:
        ref_T = to_T(ref_pose)
        p = np.array([p[0], p[1], p[2], 1])
        p_ = np.matmul(ref_T, p)
        r_ = R.from_matrix(ref_T[:3, :3])
        quat = r_.as_quat()
        rosp = PoseStamped()
        rosp.header.frame_id = ref_pose.header.frame_id
        rosp.pose.position.x = p_[0]
        rosp.pose.position.y = p_[1]
        rosp.pose.position.z = p_[2]
        rosp.pose.orientation.x = quat[0]
        rosp.pose.orientation.y = quat[1]
        rosp.pose.orientation.z = quat[2]
        rosp.pose.orientation.w = quat[3]
        path.append(rosp)
    return path

def main(args=None):
    rclpy.init(args=args)
    
    node = ArmApi2ClientExample()
    
    rclpy.spin(node)

if __name__ == "__main__":
    main()
