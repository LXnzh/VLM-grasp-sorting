import time
import json
import threading
from typing import Callable
import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from visualization_msgs.msg import MarkerArray
import tf2_ros
from tf2_ros import Buffer, TransformListener

from arm_api2_py.arm_api2_client import ArmApi2Client

from sim_pick_place.utils.helpers import Pose6D, _posestamped_msg_to_pose6d, _to_pose6d, _pose6d_to_posestamped_msg, _pose6d_to_transform_matrix, _transform_matrix_to_pose6d, _randomize_orientation_z
from sim_pick_place.sim_client_node import SimClientNode
from sim_pick_place.utils.sim_reset_client import SimResetClient
from sim_pick_place.utils.plot_utils import plot_trajectory_with_frames
from sim_pick_place.utils.pick_place_utils import plan_pick_place_trajectory, generate_random_pose_here, interpolate_lin

class SimPickPlaceNode(SimClientNode):
    
    def __init__(self, node_name: str = 'sim_pick_place_node'):
        super().__init__(node_name)
        self.cbg = ReentrantCallbackGroup()
        
        self._stop_evt = threading.Event()
        self._worker_thread = None
        
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # late joiner gets last command
        )
        
        # One-shot timer to start the worker after spin begins
        self._starter = self.create_timer(
            0.5, self._start_worker, callback_group=self.cbg)
    
    def close(self):
        """Signal the worker thread to stop."""
        self._stop_evt.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)

    def _start_worker(self):
        self.get_logger().info("Starting worker thread...")
        self._starter.cancel()
        
        self.sim_reset_client = SimResetClient(self, callback_group=self.cbg)
        self.arm_api2_client = ArmApi2Client(self, callback_group=self.cbg)
        
        # Start the thread only once the executor is already spinning
        self._worker_thread = threading.Thread(target=self.run, daemon=True)
        self._worker_thread.start()
    
    def run(self):
        
        if not rclpy.ok() or self._stop_evt.is_set():
            return
        
        START_POSE_6D = np.array([-0.45, -0.6, 1.3, 0.0, np.pi, np.pi]) # world frame
        
        self.arm_api2_client.change_state_to_servo_pos_ctl()
        time.sleep(1.0)
        
        try:

            print(f"Starting...")
            reset_spec = SimPickPlaceNode.get_reset_spec()
            # Initialize all other objects in the scene to positive x position (to drop under table)
            all_objects = self._get_objects_info()
            used_objects = reset_spec["objects"].keys()
            dx = 0
            for obj_name in all_objects:
                if obj_name not in used_objects:
                    reset_spec["objects"][obj_name] = {
                        "position": [1.0 + dx, 0.0, -0.5],  # positive x, drop under table
                        "orientation": [0.0, 0.0, 0.0, 1.0]
                    }
                    dx += 0.2  # stagger positions for multiple objects
            
            self.sim_reset_client.custom_reset(reset_spec)
            time.sleep(0.5)
            
            random_start_pose_6d = generate_random_pose_here(START_POSE_6D, range_pose_6d=[0.03, 0.03, 0.0, 0.0, 0.0, 0.0]) # +/- 3cm in x,y
            random_start_pose_msg = _pose6d_to_posestamped_msg(random_start_pose_6d, frame_id="world")
            print("Moving to random start pose:", random_start_pose_6d)
            self.arm_api2_client.interpolate_to_pose(random_start_pose_msg, avg_speed=0.5, filter_distance=1.0, filter_angle_deg=180)
            time.sleep(0.5)
                
            ee_pose = self.arm_api2_client.get_current_ee_pose()
            print("Current EE pose:", ee_pose)
            ee_pose_6d = _posestamped_msg_to_pose6d(ee_pose)
            
            red_cube_pose_6d = self._world_pose_for('red_cube')
            print("Red cube pose 6D:", red_cube_pose_6d)
            
            yellow_plate_pose_6d = self._world_pose_for('yellow_plate')
            red_cube_drop_pose_6d = yellow_plate_pose_6d.copy()
            red_cube_drop_pose_6d[2] += 0.1 # slightly above the plate
            
            traj = plan_pick_place_trajectory(
                start_pose_6d=ee_pose_6d,
                object_pose_6d=red_cube_pose_6d,
                drop_pose_6d=red_cube_drop_pose_6d,
                approach_dist=0.1,
                max_dist=0.02,
                include_retreat=True,
            )
            #plot_trajectory_with_frames(traj)

            prev_gripper_command = traj[0][6]

            for pose_6d in traj:
                pose_msg = _pose6d_to_posestamped_msg(pose_6d[:6], frame_id="world")
                self.arm_api2_client.send_gripper_command(pose_6d[6], effort=50.0, wait_for_result=False)
                self.arm_api2_client.interpolate_to_pose(pose_msg, avg_speed=0.2)
                if pose_6d[6] != prev_gripper_command:
                    print("Setting gripper to:", pose_6d[6])
                    self.arm_api2_client.send_gripper_command(pose_6d[6], effort=50.0, wait_for_result=False)
                    time.sleep(1.0)
                prev_gripper_command = pose_6d[6]
                #time.sleep(0.1)
                
            print(f"Execution completed.")
            time.sleep(0.5)
            
            # check if red_cube is on yellow_plate
            success = SimPickPlaceNode.check_success(self._world_pose_for)
                
        except Exception as e:
            self.get_logger().error(f"Worker error: {e}")   
        
        rclpy.shutdown()

    def check_success(world_pose_for_fcn: Callable, verbose: bool = True) -> bool:
        yellow_plate_pose_6d = world_pose_for_fcn('yellow_plate')
        final_red_cube_pose_6d = world_pose_for_fcn('red_cube')
        dist_xy = np.linalg.norm(final_red_cube_pose_6d[:2] - yellow_plate_pose_6d[:2])
        height_diff = final_red_cube_pose_6d[2] - yellow_plate_pose_6d[2]
        if verbose:
            print(f"Distance XY to plate: {dist_xy:.4f}, Height difference: {height_diff:.4f}")
        success = False
        if dist_xy < 0.05 and height_diff < 0.05:
            success = True
            if verbose:
                print("Success: Red cube is on the yellow plate!")
        else:
            if verbose:
                print(f"Failure: Red cube is NOT on the yellow plate. (Distance XY: {dist_xy:.4f}, Height diff: {height_diff:.4f})")
        return success

    def get_reset_spec():
        random_x = -0.6 + np.random.uniform(-0.15, 0.15)
        random_y = 0.2 + np.random.uniform(-0.15, 0.15)
        random_drop_x = -0.6 + np.random.uniform(-0.15, 0.15)
        random_drop_y = 0.2 + np.random.uniform(-0.15, 0.15)
        while abs(random_drop_x - random_x) < 0.12 and abs(random_drop_y - random_y) < 0.12:
            random_drop_x = -0.6 + np.random.uniform(-0.15, 0.15)
            random_drop_y = 0.2 + np.random.uniform(-0.15, 0.15)

        reset_spec = {
            "robot": {
                "joint_positions": [-0.2345, -1.0715, -1.8688, -1.5812, 1.6339, 2.8947]
            },
            "objects": {
                "red_cube": {
                    "position": [random_x, random_y, 0.15],
                    "orientation": _randomize_orientation_z([0.0, 1.0, 0.0, 0.0], angle_range=((-np.pi/2)-0.3, (-np.pi/2)+0.3))
                },
                "yellow_plate": {
                    "position": [random_drop_x, random_drop_y, 0.15],
                    "orientation": _randomize_orientation_z([0.0, 1.0, 0.0, 0.0], angle_range=((-np.pi/2)-0.03, (-np.pi/2)+0.03))
                },
            }
        }
                
        return reset_spec



def main(args=None):
    rclpy.init(args=args)
    node = SimPickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
