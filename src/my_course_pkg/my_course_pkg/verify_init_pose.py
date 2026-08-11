import rclpy
import threading
import time
import math
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_api2_py.arm_api2_client import ArmApi2Client


class VerifyInitPoseNode(Node):
    arm_joint_order = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    expected_joint_positions = [-0.2345, -1.0715, -1.8688, -1.5812, 1.6339, 2.8947]
    expected_joint_by_name = dict(zip(arm_joint_order, expected_joint_positions))
    gripper_joint_keywords = ("robotiq",)

    def __init__(self, node_name: str = 'verify_init_pose_node'):
        super().__init__(node_name)
        self.get_logger().info(
            "Node initialized. Please verify the robot's initial pose "
            "in the simulation."
        )

        # Avoid overlapping move commands and joint-state callbacks.
        self.cbg = MutuallyExclusiveCallbackGroup()
        self.arm_cbg = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cbg
        )
        self.arm_api2_client = ArmApi2Client(self, callback_group=self.arm_cbg)
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            callback_group=self.arm_cbg,
        )

        # Flag to prevent spamming move commands
        self.is_moving = False
        self.motion_done = False
        self.pose_verified = False
        self.initial_pose_stable_count = 0
        self.required_stable_frames = 3
        self.latest_arm_position_by_name = {}
        self.last_move_attempt_time = 0.0
        self.move_retry_cooldown_sec = 2.0
        self._worker_threads = []

    def _ensure_joint_control_mode(self):
        try:
            controller_services_ready = (
                self.arm_api2_client._service_client_list_controllers.service_is_ready()
                and self.arm_api2_client._service_client_switch_controller.service_is_ready()
            )
            if controller_services_ready:
                success = self.arm_api2_client.change_state_to_joint_ctl()
            else:
                success = self.arm_api2_client.change_state_to("JOINT_TRAJ_CTL")

            if not success:
                self.get_logger().warn(
                    "change_state_to_joint_ctl failed; retrying direct "
                    "JOINT_TRAJ_CTL state request."
                )
                success = self.arm_api2_client.change_state_to("JOINT_TRAJ_CTL")

            if success:
                self.get_logger().info("Changed arm state to JOINT_TRAJ_CTL.")
            else:
                self.get_logger().warn(
                    "Failed to change arm state to JOINT_TRAJ_CTL. "
                    "move_to_joint may be rejected by action server."
                )
            return success
        except Exception as e:
            self.get_logger().warn(
                f"Exception while changing arm state to JOINT_TRAJ_CTL: {e}"
            )
            return False

    def _start_worker(self, target, *args):
        thread = threading.Thread(target=target, args=args, daemon=True)
        self._worker_threads.append(thread)
        thread.start()
        return thread

    @classmethod
    def arm_joint_names_from_msg(cls, msg: JointState):
        return [
            name
            for name in msg.name
            if name in cls.expected_joint_by_name
        ]

    @classmethod
    def build_initial_joint_goal(cls, joint_names=None, stamp=None, frame_id="world"):
        names = [
            name
            for name in cls.arm_joint_order
            if joint_names is None or name in joint_names
        ]
        goal_joint_state = JointState()
        if stamp is not None:
            goal_joint_state.header.stamp = stamp
        goal_joint_state.header.frame_id = frame_id
        goal_joint_state.name = names
        goal_joint_state.position = [cls.expected_joint_by_name[name] for name in names]
        return goal_joint_state

    @classmethod
    def _angle_error(cls, current, expected):
        return math.atan2(math.sin(current - expected), math.cos(current - expected))

    @classmethod
    def arm_joint_position_map(cls, msg: JointState):
        return {
            name: position
            for name, position in zip(msg.name, msg.position)
            if name in cls.expected_joint_by_name
        }

    @classmethod
    def ordered_positions(cls, position_by_name):
        return [position_by_name[name] for name in cls.arm_joint_order if name in position_by_name]

    def joint_state_callback(self, msg: JointState):
        if self.pose_verified:
            return

        arm_position_by_name = self.arm_joint_position_map(msg)
        # Ensure we got exactly 6 arm joints
        if len(arm_position_by_name) != len(self.arm_joint_order):
            self.get_logger().debug(
                f"Expected {len(self.arm_joint_order)} arm joints, "
                f"got {len(arm_position_by_name)}. Ignoring this message."
            )
            return

        self.latest_arm_position_by_name = arm_position_by_name

        # Check if current pose matches expected pose
        errors = [
            abs(self._angle_error(arm_position_by_name[name], self.expected_joint_by_name[name]))
            for name in self.arm_joint_order
        ]
        if all(error < 0.02 for error in errors):
            self.initial_pose_stable_count += 1
            if self.initial_pose_stable_count >= self.required_stable_frames:
                self.get_logger().info("Initial pose verified successfully!")
                self.pose_verified = True
                self.is_moving = False
        else:
            self.initial_pose_stable_count = 0
            if self.is_moving:
                return

            # Rate-limit retries when the previous move attempt fails.
            now = time.monotonic()
            if now - self.last_move_attempt_time < self.move_retry_cooldown_sec:
                return

            self.get_logger().warn("Current joint positions do not match expected initial pose.")
            self.get_logger().info(f"Joint order: {self.arm_joint_order}")
            expected_text = [
                f"{self.expected_joint_by_name[name]:.4f}"
                for name in self.arm_joint_order
            ]
            current_text = [
                f"{arm_position_by_name[name]:.4f}"
                for name in self.arm_joint_order
            ]
            self.get_logger().info(
                f"Expected: {expected_text}"
            )
            self.get_logger().info(
                f"Current:  {current_text}"
            )
            self.get_logger().info(f"Error:    {[f'{error:.4f}' for error in errors]}")
            self.get_logger().info("Sending move_to_joint command...")

            # Use correct API: move_to_joint with JointState object
            self.is_moving = True
            self.motion_done = False
            self.initial_pose_stable_count = 0
            self.last_move_attempt_time = now
            goal_joint_state = self.build_initial_joint_goal(
                self.arm_joint_order,
                self.get_clock().now().to_msg(),
            )

            # Run in a separate thread to avoid blocking callbacks
            self._start_worker(self._move_to_joint_thread, goal_joint_state)

    def _move_to_joint_thread(self, goal_joint_state: JointState):
        """Execute move command in separate thread to avoid callback blocking"""
        try:
            try:
                self.arm_api2_client.set_vel_acc(0.2, 0.2)
            except Exception as e:
                self.get_logger().warn(f"Could not set joint move velocity/acceleration: {e}")

            success = False
            for attempt in range(1, 3):
                self._ensure_joint_control_mode()
                time.sleep(0.5)
                self.get_logger().info(f"move_to_joint attempt {attempt}/2")
                success = self.arm_api2_client.move_to_joint(goal_joint_state)
                if success:
                    break
                time.sleep(0.5)

            if not success:
                self.get_logger().warn(
                    "MoveIt move_to_joint failed; falling back to direct trajectory controller."
                )
                success = self._send_initial_pose_trajectory(goal_joint_state)

            if not success:
                self.get_logger().error("Move to initial pose failed!")
                self.is_moving = False
                return

            self.get_logger().info("Move to initial pose completed!")
            self.motion_done = True
            self.get_logger().info(
                "Waiting for /joint_states to confirm initial pose "
                "before continuing."
            )
        except Exception as e:
            self.get_logger().error(f"Exception in move_to_joint: {e}")
            self.is_moving = False

    def _send_initial_pose_trajectory(self, goal_joint_state: JointState):
        if not self.trajectory_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "Trajectory action server is not available: "
                "/scaled_joint_trajectory_controller/follow_joint_trajectory"
            )
            return False

        traj = JointTrajectory()
        traj.joint_names = list(goal_joint_state.name)

        if self.latest_arm_position_by_name:
            start_point = JointTrajectoryPoint()
            start_point.positions = [
                self.latest_arm_position_by_name[name]
                for name in traj.joint_names
            ]
            start_point.time_from_start.sec = 0
            traj.points.append(start_point)

        goal_point = JointTrajectoryPoint()
        goal_point.positions = list(goal_joint_state.position)
        goal_point.time_from_start.sec = 4
        traj.points.append(goal_point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info(
            "Sending direct trajectory to scaled_joint_trajectory_controller."
        )
        result = self.trajectory_action_client.send_goal(goal_msg)
        success = result.status == GoalStatus.STATUS_SUCCEEDED
        if success:
            self.get_logger().info("Direct trajectory completed.")
        else:
            self.get_logger().error(
                f"Direct trajectory failed with action status {result.status}."
            )
        return success

    def destroy_node(self):
        for thread in list(self._worker_threads):
            if thread.is_alive():
                thread.join(timeout=0.2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VerifyInitPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
