# circle_trajectory_publisher.py
#
# Publishes a circular trajectory (in XY plane) around the current EE pose
# at 30Hz to the topic `/arm/absolute_pose_cmds`

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped

class CircularTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("circular_trajectory_publisher")

        self.pose_pub = self.create_publisher(PoseStamped, "/arm/absolute_pose_cmds", 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/arm/state/current_pose", self._pose_cb, 1)
        
        self._start_pose = None
        self._received_pose = False

        self.get_logger().info("Waiting for initial pose on /current_pose...")

    def _pose_cb(self, msg: PoseStamped):
        # Convert msg.header.stamp to rclpy.time.Time
        msg_time = Time.from_msg(msg.header.stamp)
        time_diff = self.get_clock().now() - msg_time
        self.get_logger().info(f"Received pose at time diff: {time_diff.nanoseconds / 1e9:.3f} seconds")

        if not self._received_pose:
            self._start_pose = msg
            self._received_pose = True
            self.get_logger().info("Initial pose received. Starting circular trajectory...")
            self.get_logger().info(f"Start pose: {self._start_pose.pose.position.x}, {self._start_pose.pose.position.y}, {self._start_pose.pose.position.z}")

    def run(self, radius=0.05, duration=10.0, frequency=30.0):
        """Run the circular trajectory for a specified duration"""
        while not self._received_pose and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            return

        print("Wait 2 seconds before starting trajectory...")
        time.sleep(2) # Wait a bit so subscription is ready -> important!

        start_time = time.time()
        rate = 1.0 / frequency

        while rclpy.ok() and (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            angle = 2 * math.pi * (elapsed / duration)  # full circle over duration

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self._start_pose.header.frame_id
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            pose_msg.pose.position.x = self._start_pose.pose.position.x + radius * (math.cos(angle)-1)
            pose_msg.pose.position.y = self._start_pose.pose.position.y + radius * math.sin(angle)
            pose_msg.pose.position.z = self._start_pose.pose.position.z

            pose_msg.pose.orientation = self._start_pose.pose.orientation  # keep same orientation
            
            print(f"Publishing pose: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z}")

            self.pose_pub.publish(pose_msg)
            time.sleep(rate)


def main():
    rclpy.init()
    node = CircularTrajectoryPublisher()
    try:
        node.run(radius=0.1, duration=3.0, frequency=30.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
