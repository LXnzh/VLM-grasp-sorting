import os
from pathlib import Path
import tkinter as tk
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sim_pick_place.sim_pick_place_node import SimPickPlaceNode
from sim_pick_place.sim_stack_cubes_node import SimStackCubesNode
from visualization_msgs.msg import MarkerArray

from sim_pick_place.sim_client_node import SimClientNode
from sim_pick_place.utils.sim_reset_client import SimResetClient
from sim_pick_place.utils.helpers import _randomize_orientation_z
from ament_index_python.packages import get_package_share_directory


class SimResetGUINode(SimClientNode):
    def __init__(self):
        super().__init__('sim_reset_gui_node')
        self.cbg = ReentrantCallbackGroup()
        self.sim_reset_client = SimResetClient(self, callback_group=self.cbg)
        
        self.sim_feedback_pub = self.create_publisher(
            String,
            '/sim_feedback',
            10
        )
        
        self.create_subscription(String, '/policy_state', self._policy_state_callback, 10)
        
        self.create_timer(0.2, self.check_success, callback_group=self.cbg)
        self.current_task = 'pick_place'
        
        self.get_logger().info('SimResetGUINode initialized')
    
    def _policy_state_callback(self, msg: String):
        if "RESETTING" in msg.data:
            try:
                seed = int(msg.data.split("_")[-1])
            except ValueError:
                seed = None
            self.reset_with_randomization(task_name=self.current_task, seed=seed)
        
    def check_success(self):
        
        success = False
        
        if self.current_task == 'pick_place':
            success = SimPickPlaceNode.check_success(self._world_pose_for, verbose=False)
        elif self.current_task == 'stack_cubes':
            success = SimStackCubesNode.check_success(self._world_pose_for, verbose=False)
        
        if success:
            msg = String()
            msg.data = 'task_done'
            self.sim_feedback_pub.publish(msg)
            self.get_logger().info('Published success feedback')

    def reset_with_randomization(self, task_name: str = 'pick_place', seed: int = None):
        """Reset simulation with randomized object positions"""
        # Set seed if provided
        if seed is not None:
            np.random.seed(seed)
            self.get_logger().info(f'Using seed: {seed}')
        
        if task_name == 'pick_place':
            reset_spec = SimPickPlaceNode.get_reset_spec()
            self.current_task = 'pick_place'
        elif task_name == 'stack_cubes':
            reset_spec = SimStackCubesNode.get_reset_spec()
            self.current_task = 'stack_cubes'
        else:
            self.get_logger().error(f'Unknown task name: {task_name}')
            return False
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
        self.get_logger().info('Resetting simulation with randomization...')
        success = self.sim_reset_client.custom_reset(reset_spec)
        if success:
            self.get_logger().info('Reset successful!')
        else:
            self.get_logger().error('Reset failed!')
        return success

class SimResetGUI:
    def __init__(self, node: SimResetGUINode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Simulation Reset GUI')
        self.root.geometry('300x250')
        share = get_package_share_directory('sim_pick_place')
        icon_path = os.path.join(share, 'icons', 'sim_reset_icon.png')
        icon = tk.PhotoImage(file=str(icon_path))
        self.root.iconphoto(True, icon)
        
        # add dropdown to select task
        self.task_var = tk.StringVar(value='pick_place')
        self.task_dropdown = tk.OptionMenu(
            self.root,
            self.task_var,
            'pick_place',
            'stack_cubes'
        )
        self.task_dropdown.config(font=('Arial', 12))
        self.task_dropdown.pack(pady=10)
        
        # Add seed input
        seed_frame = tk.Frame(self.root)
        seed_frame.pack(pady=5)
        
        seed_label = tk.Label(
            seed_frame,
            text='Seed (optional):',
            font=('Arial', 10)
        )
        seed_label.pack(side=tk.LEFT, padx=5)
        
        self.seed_entry = tk.Entry(
            seed_frame,
            font=('Arial', 10),
            width=15
        )
        self.seed_entry.pack(side=tk.LEFT, padx=5)
        
        self.reset_button = tk.Button(
            self.root,
            text='Reset Simulation',
            command=self.handle_reset,
            font=('Arial', 14),
            bg='#4CAF50',
            fg='white',
            padx=20,
            pady=10
        )
        self.reset_button.pack(expand=True, pady=10)
        
        self.status_label = tk.Label(
            self.root,
            text='Ready',
            font=('Arial', 10)
        )
        self.status_label.pack(pady=10)

    def handle_reset(self):
        """Handle reset button click"""
        self.status_label.config(text='Resetting...')
        self.reset_button.config(state='disabled')
        
        # Parse seed from entry field
        seed = None
        seed_text = self.seed_entry.get().strip()
        if seed_text:
            try:
                seed = int(seed_text)
            except ValueError:
                self.status_label.config(text='Invalid seed! Use integer.')
                self.reset_button.config(state='normal')
                return
        
        # Run reset in a separate thread to avoid blocking GUI
        def reset_task():
            success = self.node.reset_with_randomization(self.task_var.get(), seed=seed)
            self.root.after(0, lambda: self.reset_complete(success))
        
        threading.Thread(target=reset_task, daemon=True).start()

    def reset_complete(self, success):
        """Called after reset completes"""
        self.reset_button.config(state='normal')
        if success:
            self.status_label.config(text='Reset successful!')
        else:
            self.status_label.config(text='Reset failed!')

    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()


def main():
    rclpy.init()
    node = SimResetGUINode()
    
    # Start ROS2 executor in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Run GUI in main thread
    gui = SimResetGUI(node)
    gui.run()
    
    # Cleanup
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
