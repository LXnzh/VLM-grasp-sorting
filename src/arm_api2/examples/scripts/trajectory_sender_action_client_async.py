#!/usr/bin/env python3

import csv
import threading

import numpy as np
import rclpy
import rclpy.duration
from ament_index_python.packages import get_package_share_directory
from arm_api2_msgs.action import MoveCartesianPath
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class CreateAndPublishTrajectoryAsync(Node):

    def __init__(self):
        super().__init__("create_publish_trajectory_async")

        # Create subscribers
        self.curr_p_sub = self.create_subscription(
            PoseStamped, "/arm/state/current_pose", self.curr_p_cb, 1
        )

        # Create action client
        self._action_client = ActionClient(
            self, MoveCartesianPath, "arm/move_to_pose_path"
        )

        # Path to predefined trajectories
        c_csv_pth = get_package_share_directory("arm_api2") + "/utils/CS_C_easy.csv"
        s_csv_pth = get_package_share_directory("arm_api2") + "/utils/CS_S_easy.csv"

        self.get_logger().info(f"C trajectory path is: {c_csv_pth}")
        self.get_logger().info(f"S trajectory path is: {s_csv_pth}")
        # Load positions from YAML file

        self.c_data = self.load_positions(c_csv_pth)
        self.s_data = self.load_positions(s_csv_pth)

        # Flags
        self.send_traj_flag = threading.Event()
        self.receive_pose_flag = threading.Event()

        # Create a separate thread to handle user input
        self.user_input_thread = threading.Thread(target=self.handle_user_input)
        self.user_input_thread.start()

    def send_goal(self, goal_path):
        goal_msg = MoveCartesianPath.Goal()
        goal_msg.poses = goal_path

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
        if result.success:
            self.get_logger().info("Result: Trajectory executed successfully!")
        else:
            self.get_logger().info("Result: Trajectory execution failed!")
        self.send_traj_flag.clear()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Feedback: {0}".format(feedback.status))

    def load_positions(self, csv_pth):
        p_data = []
        with open(csv_pth, "r") as file:
            reader = csv.DictReader(file)
            for row in reader:
                x = row["x"]
                y = row["y"]
                z = row["z"]
                p_data.append(np.array([float(x), float(y), float(z), 1]).T)
        self.get_logger().info("Loaded positions.")
        return p_data

    def curr_p_cb(self, msg):
        self.curr_p = PoseStamped()
        self.curr_p.header = msg.header
        self.curr_p.pose.position = msg.pose.position
        self.curr_p.pose.orientation = msg.pose.orientation
        p, q = msg.pose.position, msg.pose.orientation
        x, y, z = p.x, p.y, p.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # self.get_logger().info(f"x: {x}\t {y}\t {z}\t")
        # x, y, z, w
        R_ = R.from_quat([qx, qy, qz, qw])
        r = R_.as_matrix()
        p_ = np.array([x, y, z]).T.reshape(3, 1)
        self.get_logger().debug(f"R shape is: {r.shape}")
        self.get_logger().debug(f"p shape is: {p_.shape}")
        T_ = np.hstack((r, p_))
        self.T = np.vstack((T_, np.array([0, 0, 0, 1])))
        self.get_logger().debug(f"Reciv T matrix is: {self.T}")
        self.receive_pose_flag.set()

    def create_trajectory(self, letter):
        self.get_logger().info("Create trajectory!")

        if letter == "S":
            data = self.s_data
        if letter == "C":
            data = self.c_data
        path = []
        for p in data:
            self.get_logger().info(f"p is: {p}")
            self.get_logger().info(f"T is: {self.T}")
            p_ = np.matmul(self.T, p)
            self.get_logger().info(f"p_ is: {p_}")
            r_ = R.from_matrix(self.T[:3, :3])
            quat = r_.as_quat()
            rosp = PoseStamped()
            rosp.header.frame_id = self.curr_p.header.frame_id
            rosp.pose.position.x = p_[0]
            rosp.pose.position.y = p_[1]
            rosp.pose.position.z = p_[2]
            rosp.pose.orientation.x = quat[0]
            rosp.pose.orientation.y = quat[1]
            rosp.pose.orientation.z = quat[2]
            rosp.pose.orientation.w = quat[3]
            path.append(rosp)
        return path

    def handle_user_input(self):
        while rclpy.ok():

            while self.send_traj_flag.is_set() or not self.receive_pose_flag.is_set():
                pass

            user_input = input(
                "Enter 's' for S trajectory, 'c' for C trajectory, enter 'q' to quit: "
            )

            if user_input.lower() == "s":
                self.get_logger().info("Starting S trajectory...")
                traj = self.create_trajectory("S")
                self.send_goal(traj)
                self.send_traj_flag.set()

            elif user_input.lower() == "c":
                self.get_logger().info("Starting C trajectory...")
                traj = self.create_trajectory("C")
                self.send_goal(traj)
                self.send_traj_flag.set()

            if user_input.lower() == "q":
                self.get_logger().info("Shutting down.")
                rclpy.shutdown()
                break


def main(args=None):
    rclpy.init(args=args)
    CPT = CreateAndPublishTrajectoryAsync()

    executor = MultiThreadedExecutor()
    executor.add_node(CPT)

    try:
        executor.spin()

    finally:
        CPT.destroy_node()


if __name__ == "__main__":
    main()
