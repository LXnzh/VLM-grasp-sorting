#!/usr/bin/env python3

import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from typing import List, Optional, TypeVar
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController, ListControllers
from controller_manager_msgs.msg import ControllerState
from geometry_msgs.msg import TwistStamped
from arm_api2_msgs.action import MoveCartesian, MoveJoint, MoveCartesianPath
from arm_api2_msgs.srv import ChangeState, SetVelAcc, SetStringParam
from std_srvs.srv import SetBool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from arm_api2_py.transformation_utils import extract_pos_quat_from_pose, posestamped_to_pose, pos_quat_to_pose, pose_to_posestamped

ResponseT = TypeVar("ResponseT")

class ArmApi2Client:

    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup | None = None):
        
        self._cbg = callback_group or ReentrantCallbackGroup()
        
        # Action Clients
        self._action_client_cartesian = ActionClient(
            node, MoveCartesian, "arm/move_to_pose", callback_group=self._cbg)
        self._action_client_joint = ActionClient(
            node, MoveJoint, "arm/move_to_joint", callback_group=self._cbg)
        self._action_client_cartesian_path = ActionClient(
            node, MoveCartesianPath, "arm/move_to_pose_path", callback_group=self._cbg)
        self._action_client_gripper = ActionClient(
            node, GripperCommand, "arm/gripper_control", callback_group=self._cbg)

        # Service Clients
        self._service_client_state_change = node.create_client(
            ChangeState, "arm/change_state", callback_group=self._cbg)
        self._service_client_setvelacc = node.create_client(
            SetVelAcc, "arm/set_vel_acc", callback_group=self._cbg)
        self._service_client_seteelink = node.create_client(
            SetStringParam, "arm/set_eelink", callback_group=self._cbg)
        self._service_client_setplanonly = node.create_client(
            SetBool, "arm/set_planonly", callback_group=self._cbg)
        self._service_client_switch_controller = node.create_client(
            SwitchController, "controller_manager/switch_controller", callback_group=self._cbg)
        self._service_client_list_controllers = node.create_client(
            ListControllers, "controller_manager/list_controllers", callback_group=self._cbg)

        # Publishers
        self._publisher_twist_cmd = node.create_publisher(
            TwistStamped, "/arm/delta_twist_cmds", 10)
        self._publisher_pose_cmd = node.create_publisher(
            PoseStamped, "/arm/absolute_pose_cmds", 10)
        self._publisher_joint_vel_cmd = node.create_publisher(
            JointJog, "/arm/delta_joint_cmds", 10)
        self._publisher_gripper_cmd = node.create_publisher(
            GripperCommandMsg, "robotiq_2f_urcap_adapter/gripper_command_topic", 10)

        # Subscribers
        self._subscriber_current_ee_pose = node.create_subscription(
            PoseStamped, "/arm/state/current_pose", self.current_pose_callback, 1)

        self.logger = node.get_logger()
        self._node = node
        self._current_ee_pose = PoseStamped()
        self.moveit_frame = "world"
        self.last_interpolate_run_time = 0.0
        self.last_interpolate_end_quat = None
        self.last_interpolate_end_pos = None
        
        # TF2 buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)
        
        self._log_info("Initializing...") 
                
        self._log_info("Waiting for action servers...")
        res = [False for _ in range(4)]
        res[0] = self._action_client_cartesian.wait_for_server(timeout_sec=2.0)
        res[1] = self._action_client_joint.wait_for_server(timeout_sec=2.0)
        res[2] = self._action_client_cartesian_path.wait_for_server(timeout_sec=2.0)
        res[3] = self._action_client_gripper.wait_for_server(timeout_sec=2.0)
        self._log_info("Action servers are available: " + str(res))
        
        
        self._log_info("Waiting for service servers...")
        res = [False for _ in range(6)]
        res[0] = self._service_client_state_change.wait_for_service(timeout_sec=2.0)
        res[1] = self._service_client_setvelacc.wait_for_service(timeout_sec=2.0)
        res[2] = self._service_client_seteelink.wait_for_service(timeout_sec=2.0)
        res[3] = self._service_client_setplanonly.wait_for_service(timeout_sec=2.0)
        res[4] = self._service_client_switch_controller.wait_for_service(timeout_sec=2.0)
        res[5] = self._service_client_list_controllers.wait_for_service(timeout_sec=2.0)
        self._log_info("Service servers are available: " + str(res))

    def _log_info(self, message: str):
        self.logger.info(f"[ArmApi2Client] {message}")
    
    def _log_warn(self, message: str):
        self.logger.warn(f"[ArmApi2Client] {message}")
    
    def _log_error(self, message: str):
        self.logger.error(f"[ArmApi2Client] {message}")
    
    def transform_to_frame(self, pose: PoseStamped, destination_frame: str) -> PoseStamped:
        timeout = rclpy.duration.Duration(seconds=2.0)
        pose_frame = pose.header.frame_id
        if self.tf_buffer.can_transform(destination_frame, pose_frame, rclpy.time.Time(), timeout):
            try:
                t = self.tf_buffer.lookup_transform(
                    destination_frame,
                    pose_frame,
                    rclpy.time.Time())
                msg = tf2_geometry_msgs.do_transform_pose_stamped(pose, t)
                return msg
            except TransformException as ex:
                self._log_error(
                    f'Could not transform {pose_frame} to {destination_frame}: {ex}')
                return None
        else:
            self._log_error(f"Transform from {pose_frame} to {destination_frame} not available within {timeout} s.")
            return None

    def current_pose_callback(self, msg):
        self._current_ee_pose = msg

    def get_current_ee_pose(self):
        """
        Returns the current end effector pose. EE link is set by the set_eelink service.

        Returns:
            PoseStamped: The current end effector pose.
        """
        return self._current_ee_pose

    def set_vel_acc(self, max_vel_scaling: float, max_acc_scaling: float):
        """
        Sends a request to the service server to set the velocity and acceleration scaling factors.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            max_vel_scaling (float): The maximum velocity scaling factor. In the range [0.0, 1.0].
            max_acc_scaling (float): The maximum acceleration scaling factor. In the range [0.0, 1.0].

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = SetVelAcc.Request()
        request.max_vel = max_vel_scaling
        request.max_acc = max_acc_scaling
        
        response = self._call_service_sync(
            self._service_client_setvelacc,
            request,
            service_name="set_vel_acc",
            wait_for_service_sec=2.0,
            timeout_sec=10.0,
        )
        
        if response is None:
            return False

        if response.success:
            self._log_info(
                f"Velocity and acceleration scaling \
                    factors set to {max_vel_scaling}, {max_acc_scaling}")
        else:
            self._log_error(
                f"Setting velocity and acceleration scaling \
                factors to {max_vel_scaling}, {max_acc_scaling} failed")

        return response.success

    def set_eelink(self, eelink_name: str):
        """
        Sends a request to the service server to set the end effector link name.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            eelink_name (str): The name of the end effector link.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = SetStringParam.Request()
        request.value = eelink_name
        
        response = self._call_service_sync(
            self._service_client_seteelink,
            request,
            service_name="set_eelink",
            wait_for_service_sec=2.0,
            timeout_sec=10.0,
        )

        if response is None:
            return False

        if response.success:
            self._log_info(f"End effector link set to {eelink_name}")
        else:
            self._log_error(f"Setting end effector link to {eelink_name} failed")

        return response.success

    def set_planonly(self, plan_only: bool):
        """
        Sends a request to the service server to set the plan only flag.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.
        
        Args:
            plan_only (bool): The plan only flag.
            
        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        request = SetBool.Request()
        request.data = plan_only
        
        response = self._call_service_sync(
            self._service_client_setplanonly,
            request,
            service_name="set_planonly",
            wait_for_service_sec=2.0,
            timeout_sec=10.0,
        )

        if response is None:
            return False

        if response.success:
            self._log_info(f"Plan only flag set to {plan_only}")
        else:
            self._log_error(f"Setting plan only flag to {plan_only} failed")

        return response.success

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

        response = self._call_service_sync(
            self._service_client_state_change,
            request,
            service_name="change_state",
            wait_for_service_sec=2.0,
            timeout_sec=10.0,
        )

        if response is None:
            return False

        if response.success:
            self._log_info(f"State changed to {state}")
        else:
            self._log_info(f"State change to {state} failed")

        return response.success

    def change_state_to_joint_ctl(self):
        """
        Sends a request to the service server to change the arm state to joint control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        res1 = self.switch_controller_for_joint_cartesian_ctl()
        res2 = self.change_state_to("JOINT_TRAJ_CTL")
        return res1 and res2

    def change_state_to_cartesian_ctl(self):
        """
        Sends a request to the service server to change the arm state to cartesian control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """
        res1 = self.switch_controller_for_joint_cartesian_ctl()
        res2 = self.change_state_to("CART_TRAJ_CTL")
        return res1 and res2

    def change_state_to_servo_ctl(self):
        """
        Sends a request to the service server to change the arm state to servo control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """

        res1 = self.switch_controller_for_servo_ctl()
        res2 = self.change_state_to("SERVO_CTL")
        return res1 and res2

    def change_state_to_servo_pos_ctl(self):
        """
        Sends a request to the service server to change the arm state to servo position control.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the service server response was received, False otherwise.
        """

        res1 = self.switch_controller_for_servo_ctl()
        res2 = self.change_state_to("SERVO_POS_CTL")
        return res1 and res2
    
    def list_controllers(self):
        """
        Lists the available controllers in the UR ROS2 driver.

        !!! IMPORTANT: This function is blocking until the service server response is received 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            List[str]: The list of available controllers.
        """
        request = ListControllers.Request()
        
        response = self._call_service_sync(
            self._service_client_list_controllers,
            request,
            service_name="list_controllers",
            wait_for_service_sec=2.0,
            timeout_sec=10.0,
        )

        if response is None:
            return []
        
        controllers = response.controller
        
        self._log_info(f"Available controllers: {len(controllers)}")
        
        return controllers
            

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
        
        current_controllers = self.list_controllers()
        for controller in current_controllers:
            if controller.name  in start_controllers and controller.state == "active":
                self._log_info(f"Controller {controller.name} is already active")
                start_controllers.remove(controller.name)
            if controller.name in stop_controllers and controller.state == "inactive":
                self._log_info(f"Controller {controller.name} is already inactive")
                stop_controllers.remove(controller.name)
    
        
        if len(start_controllers) == 0 and len(stop_controllers) == 0:
            self._log_info("No controllers to switch")
            return True
        
        request = SwitchController.Request()
        request.start_controllers = start_controllers
        request.stop_controllers = stop_controllers
        request.strictness = 2  # STRICT=2, BEST_EFFORT=1
        
        response = self._call_service_sync(
            self._service_client_switch_controller,
            request,
            service_name="switch_controller",
            wait_for_service_sec=2.0,
            timeout_sec=10.0,
        )

        if response is None:
            return False

        if response.ok:
            self._log_info(f"Controllers switched")
        else:
            self._log_info(f"Switching controllers failed")

        return response.ok

    def switch_controller_for_servo_ctl(self):
        """
        Sends a request to the service server to switch controllers to servo control.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the controllers were switched, False otherwise.
        """
        stop_controllers = ["scaled_joint_trajectory_controller"]
        start_controllers = ["forward_position_controller"]
        return self.switch_controller(start_controllers, stop_controllers)

    def switch_controller_for_joint_cartesian_ctl(self):
        """
        Sends a request to the service server to switch controllers to joint and cartesian control.

        !!! IMPORTANT: This function is blocking until the service server response is received
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the controllers were switched, False otherwise.
        """
        stop_controllers = ["forward_position_controller"]
        start_controllers = ["scaled_joint_trajectory_controller"]
        return self.switch_controller(start_controllers, stop_controllers)

    def gripper_open(self):
        """
        Sends a request to the gripper action server to open the gripper.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        return self.send_gripper_command(0.0, 140.0)

    def gripper_close(self):
        """
        Sends a request to the gripper action server to close the gripper.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        return self.send_gripper_command(0.79, 140.0)

    def send_gripper_command(self, position: float, effort: float, wait_for_result: bool = True):
        """
        Sends a goal to the gripper action server to move the gripper to the specified position and effort.

        !!! IMPORTANT: This function is blocking until the goal is reached
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            position (float): The position to move the gripper to. In the range [0.0, 0.8].
            effort (float): The effort to apply to the gripper. In the range [20.0, 140.0].

        Returns:
            tuple: A tuple containing the following values: 
                - bool: True if the request was successful, False otherwise.
                - float: The position the gripper reached.
                - float: The effort the gripper reached.
                - bool: True if the gripper is stalled, False otherwise.
                - bool: True if the gripper reached the goal, False otherwise
        """

        command = GripperCommandMsg()
        command.position = position
        command.max_effort = effort
        
        if not wait_for_result:
            self._publisher_gripper_cmd.publish(command)
            return True, position, effort, False, False

        goal_msg = GripperCommand.Goal()
        goal_msg.command = command

        self._log_info("Waiting for action server...")

        self._action_client_gripper.wait_for_server()

        self._log_info("send_gripper_command goal request sent, waiting for result...")

        result = self._action_client_gripper.send_goal(goal_msg)

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._log_info("Goal reached")
            return True, result.result.position, result.result.effort, result.result.stalled, result.result.reached_goal
        else:
            self._log_error("send_gripper_command failed")
            return False, 0.0, 0.0, False, False

    def send_joint_vel_cmd(self, joint_names: List[str], joint_velocities: List[float], base_frame: str = "base_link"):
        """
        Sends a joint velocity command to the joint velocity command topic.

        Args:
            joint_names (List[str]): The names of the joints to send the velocity command to.
            joint_velocities (List[float]): The velocities to send to the joints.
        """
        msg = JointJog()
        msg.joint_names = joint_names
        msg.velocities = joint_velocities
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        self._publisher_joint_vel_cmd.publish(msg)

    def send_twist_cmd(self, twist_stamped: TwistStamped):
        """
        Sends a twist command to the twist command topic. No planning is done, distance to goal should be small.

        Args:
            twist_stamped (TwistStamped): The twist command to send.
        """
        twist_stamped.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher_twist_cmd.publish(twist_stamped)

    def send_pose_cmd(self, pose_stamped: PoseStamped):
        """
        Sends a pose command to the pose command topic. No planning is done, distance to goal should be small (< 5 cm).

        Args:
            pose_stamped (PoseStamped): The pose command to send.
        """
        pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher_pose_cmd.publish(pose_stamped)
    
    def interpolate_to_pose(
        self,
        goal_pose: PoseStamped,
        avg_speed=0.01,
        acceleration=-1.0,
        filter_distance=0.5,
        filter_angle_deg=60,
    ):
        """Interpolate from current end-effector pose to the target pose. With collision checking. Without planning.

        Args:
            goal_pose (PoseStamped): Target pose, ideally in the moveit frame, but can be in any frame.
            avg_speed (float, optional): Max/cruise speed in m/s along the line. Defaults to 0.01 m/s.
            acceleration (float, optional): Acceleration magnitude in m/s^2 for speed ramp up/down.
                If <= 0, falls back to constant-speed interpolation. Defaults to -1.0 (good value is 0.05 m/s^2).
            filter_distance (float, optional): Maximum distance to consider for interpolation. Defaults to 0.5 m.
            filter_angle_deg (float, optional): Maximum orientation change in degrees to consider for interpolation.
                Defaults to 60 deg.
        """
        counter = 0
        max_retries = 20

        start_pos, start_quat = self._get_start_pose(counter, max_retries)
        end_pos, end_quat = self._transform_goal_pose_to_moveit_frame(goal_pose)

        # interpolation parameters
        start_pos = np.array(start_pos, dtype=float)
        end_pos = np.array(end_pos, dtype=float)
        delta = end_pos - start_pos

        distance = float(np.linalg.norm(delta))
        orientation_diff = R.from_quat(start_quat).inv() * R.from_quat(end_quat)
        angle = float(orientation_diff.magnitude())

        if distance > filter_distance:
            self._log_warn(
                f"Interpolation distance {distance:.3f} m exceeds filter distance {filter_distance:.3f} m. Skipping interpolation."
            )
            return

        if angle > np.deg2rad(filter_angle_deg):
            self._log_warn(
                f"Orientation change {np.rad2deg(angle):.2f} deg exceeds {filter_angle_deg} deg. Skipping interpolation."
            )
            return

        # Guard against zero motion
        if distance < 1e-9:
            self._log_info("Interpolation distance ~0; nothing to do.")
            self.last_interpolate_run_time = time.time()
            self.last_interpolate_end_pos = end_pos.tolist()
            self.last_interpolate_end_quat = end_quat
            return

        vmax = float(avg_speed)
        a = float(acceleration)

        # Build a trapezoidal/triangular timing model for 1D motion along the path length.
        # If acceleration <= 0 => constant speed profile.
        if a <= 0.0 or vmax <= 0.0:
            movement_time = distance / max(vmax, 1e-9)
            t_acc = 0.0
            t_flat = movement_time
            triangular = False
            v_peak = vmax
        else:
            t_to_vmax = vmax / a
            d_acc = 0.5 * a * t_to_vmax**2

            if distance < 2.0 * d_acc:
                # Triangular profile: never reaches vmax
                triangular = True
                v_peak = np.sqrt(distance * a)
                t_acc = v_peak / a
                t_flat = 0.0
                movement_time = 2.0 * t_acc
            else:
                # Trapezoidal profile
                triangular = False
                v_peak = vmax
                t_acc = t_to_vmax
                d_flat = distance - 2.0 * d_acc
                t_flat = d_flat / vmax
                movement_time = 2.0 * t_acc + t_flat

        # sample every ~0.1s, at least 15 steps
        dt = 0.1
        num_steps = max(int(movement_time / dt), 15)
        times_sec = np.linspace(0.0, movement_time, num_steps)

        def progress_along_path(t: float) -> float:
            """Return normalized path progress s in [0,1] at time t (seconds)."""
            if a <= 0.0 or vmax <= 0.0:
                # constant speed
                x = (distance / max(movement_time, 1e-9)) * t
                return float(np.clip(x / distance, 0.0, 1.0))

            if triangular:
                if t <= t_acc:
                    # accelerate
                    x = 0.5 * a * t**2
                else:
                    # decelerate
                    td = t - t_acc
                    # distance covered in accel phase:
                    x_acc = 0.5 * a * t_acc**2
                    # during decel: start at v_peak, decel at -a
                    x = x_acc + v_peak * td - 0.5 * a * td**2
            else:
                if t <= t_acc:
                    # accelerate
                    x = 0.5 * a * t**2
                elif t <= t_acc + t_flat:
                    # cruise
                    x_acc = 0.5 * a * t_acc**2
                    x = x_acc + v_peak * (t - t_acc)
                else:
                    # decelerate
                    x_acc = 0.5 * a * t_acc**2
                    x_flat = v_peak * t_flat
                    td = t - (t_acc + t_flat)
                    x = x_acc + x_flat + v_peak * td - 0.5 * a * td**2

            return float(np.clip(x / distance, 0.0, 1.0))

        s_vals = np.array([progress_along_path(t) for t in times_sec], dtype=float)

        # position interpolation using progress s(t)
        iterp_positions = start_pos[None, :] + s_vals[:, None] * delta[None, :]

        # quaternion interpolation (SLERP) using same progress values
        key_rots = R.concatenate([R.from_quat(start_quat), R.from_quat(end_quat)])
        slerp = Slerp([0.0, 1.0], key_rots)
        iterp_rots = slerp(s_vals)

        self.logger.info(
            f"Interpolating {distance:.3f}m over {movement_time:.2f}s "
            f"({num_steps} steps), vmax={v_peak:.3f} m/s, a={max(a,0.0):.3f} m/s^2 "
            f"({'triangular' if triangular else 'trapezoidal' if a>0 else 'constant'})."
        )

        # send points with time-consistent sleeps
        for i in range(num_steps):
            if i == 0:
                continue  # skip first point (current position)

            inter_pos = iterp_positions[i]
            inter_quat = iterp_rots[i].as_quat()  # x, y, z, w
            interp_pose = pos_quat_to_pose(inter_pos, inter_quat)
            interp_pose_stamped = pose_to_posestamped(interp_pose, frame_id=self.moveit_frame)

            self.send_pose_cmd(interp_pose_stamped)

            # sleep according to time grid spacing
            time.sleep(float(times_sec[i] - times_sec[i - 1]))

        self.last_interpolate_run_time = time.time()
        self.last_interpolate_end_pos = end_pos.tolist()
        self.last_interpolate_end_quat = end_quat

    
    def _get_start_pose(self, counter, max_retries):
        current_time = time.time()
        if current_time - self.last_interpolate_run_time > 1.0:
            # wait for current end-effector pose        
            while self._current_ee_pose is None and counter < max_retries:
                self._log_warn(f"No current end-effector pose available for interpolation. ({counter+1}/{max_retries})")
                time.sleep(0.5)
                counter += 1
            if counter == max_retries:
                raise RuntimeError("Max retries reached. Aborting interpolation.")

            start_pos, start_quat = extract_pos_quat_from_pose(posestamped_to_pose(self._current_ee_pose))
            start_ref_frame = self._current_ee_pose.header.frame_id
            

            # transform start_pose to moveit frame if needed
            if start_ref_frame != self.moveit_frame:
                start_pose = self.transform_to_frame(self._current_ee_pose, self.moveit_frame)
                if start_pose is None:
                    raise RuntimeError(f"Transform current pose to move it frame ({self.moveit_frame}) failed; aborting interpolation.")
                start_pos, start_quat = extract_pos_quat_from_pose(posestamped_to_pose(start_pose))
        else:
            # if interpolation was run recently, use last target as start
            start_pos = self.last_interpolate_end_pos
            start_quat = self.last_interpolate_end_quat
        return start_pos,start_quat
    
    def _transform_goal_pose_to_moveit_frame(self, goal_pose: PoseStamped):
        goal_ref_frame = goal_pose.header.frame_id
        
        # transform goal_pose to moveit frame if needed
        if goal_ref_frame != self.moveit_frame:
            
            goal_pose_transformed = self.transform_to_frame(goal_pose, self.moveit_frame)
            if goal_pose_transformed is None:
                self._log_error("Transform to moveit frame failed; aborting absolute move command.")
                return
            end_pos, end_quat = extract_pos_quat_from_pose(posestamped_to_pose(goal_pose_transformed))

        else:
            end_pos, end_quat = extract_pos_quat_from_pose(posestamped_to_pose(goal_pose))
        
        return end_pos, end_quat

    def move_to_pose(self, goal_pose: PoseStamped):
        """
        Sends a goal to the action server to move the arm to the specified pose.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            goal_pose (PoseStamped): The goal pose to move the arm to.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        goal_msg = MoveCartesian.Goal()
        goal_msg.goal = goal_pose

        self._log_info("Waiting for action server...")

        self._action_client_cartesian.wait_for_server()

        self._log_info("move_to_pose goal request sent, waiting for result...")

        result = self._action_client_cartesian.send_goal(goal_msg)

        if result.result.success:
            self._log_info("Goal reached")
        else:
            self._log_info("move_to_pose failed")

        return result.result.success

    def move_to_joint(self, joint_positions: JointState):
        """
        Sends a goal to the action server to move the arm to the specified joint positions.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            joint_positions (JointState): The joint positions to move the arm to.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        goal_msg = MoveJoint.Goal()
        goal_msg.joint_state = joint_positions

        self._log_info("Waiting for action server...")

        self._action_client_joint.wait_for_server()

        self._log_info("move_to_joint goal request sent, waiting for result...")

        result = self._action_client_joint.send_goal(goal_msg)

        if result.result.success:
            self._log_info("Goal reached")
        else:
            self._log_error("move_to_joint failed")

        return result.result.success

    def move_to_pose_path(self, goal_path: List[PoseStamped]):
        """
        Sends a goal to the action server to move the arm to the specified poses in sequence.

        !!! IMPORTANT: This function is blocking until the goal is reached 
        This function must be not called in the main thread. Otherwise will cause a deadlock.

        Args:
            goal_poses (List[PoseStamped]): The goal poses to move the arm to.

        Returns:
            bool: True if the goal was reached, False otherwise.
        """

        goal_msg = MoveCartesianPath.Goal()
        goal_msg.poses = goal_path

        self._log_info("Waiting for action server...")

        self._action_client_cartesian_path.wait_for_server()

        self._log_info("move_to_pose_path goal request sent, waiting for result...")

        result = self._action_client_cartesian_path.send_goal(goal_msg)

        if result.result.success:
            self._log_info("Goal reached")
        else:
            self._log_info("move_to_pose_path failed")

        return result
    
    def _call_service_sync(
        self,
        client,
        request,
        *,
        service_name: str = "service",
        wait_for_service_sec: float = 2.0,
        timeout_sec: Optional[float] = 10.0,
    ) -> Optional[ResponseT]:
        """
        Generic sync wrapper around rclpy client.call_async(request).

        Requirements:
        - An executor is spinning elsewhere (e.g. background thread).
        - The client should ideally be created in a ReentrantCallbackGroup
            (or at least not blocked by the caller's callback group).

        Args:
            client: rclpy service client (from node.create_client)
            request: the request object for that service
            service_name: name used only for logging
            wait_for_service_sec: how long to wait for service availability
            timeout_sec: timeout for the call to complete; None = wait forever

        Returns:
            Response object on success, or None on timeout / failure.
        """
        self._log_info(f"{service_name}: Waiting for service server...")
        if not client.wait_for_service(timeout_sec=wait_for_service_sec):
            self._log_error(f"{service_name}: service server not available")
            return None

        self._log_info(f"{service_name}: request sent, waiting for response...")
        future = client.call_async(request)

        done_evt = threading.Event()
        future.add_done_callback(lambda _: done_evt.set())

        if timeout_sec is None:
            done_evt.wait()
        else:
            if not done_evt.wait(timeout=timeout_sec):
                self._log_error(f"{service_name}: service call timed out after {timeout_sec}s")
                return None

        try:
            return future.result()
        except Exception as e:
            self._log_error(f" ArmApi2Client: {service_name}: service call raised: {e}")
            return None