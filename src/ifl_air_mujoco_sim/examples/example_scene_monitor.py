#!/usr/bin/env python3
"""
Example script showing how to monitor the scene description ROS topic.
This demonstrates the ROS2 integration without requiring the full simulation.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import sys

class SceneMonitor(Node):
    """Simple ROS2 node to monitor scene description messages."""
    
    def __init__(self):
        super().__init__('scene_monitor')
        
        # Subscribe to scene description topic
        self.subscription = self.create_subscription(
            MarkerArray,
            '/scene_description',
            self.scene_callback,
            10
        )
        
        self.get_logger().info('Scene monitor started. Listening to /scene_description...')
        self.get_logger().info('Start the simulation with: python ros2_main.py')
    
    def scene_callback(self, msg):
        """Handle incoming scene description messages."""
        num_objects = len(msg.markers)
        self.get_logger().info(f'Received scene description with {num_objects} objects:')
        
        for marker in msg.markers:
            # Extract object info from marker
            obj_name = f"Object_{marker.id}"  # Could be enhanced with actual name from marker text
            obj_type = self.marker_type_to_string(marker.type)
            
            pos = marker.pose.position
            scale = marker.scale
            color = marker.color
            
            self.get_logger().info(
                f'  - {obj_name}: {obj_type} at [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] '
                f'size=[{scale.x:.3f}, {scale.y:.3f}, {scale.z:.3f}] '
                f'color=({color.r:.2f}, {color.g:.2f}, {color.b:.2f}, {color.a:.2f})'
            )
    
    def marker_type_to_string(self, marker_type):
        """Convert marker type ID to human-readable string."""
        type_map = {
            1: 'CUBE',
            2: 'SPHERE', 
            3: 'CYLINDER',
            4: 'LINE_STRIP',
            5: 'LINE_LIST',
            6: 'CUBE_LIST',
            7: 'SPHERE_LIST',
            8: 'POINTS',
            9: 'TEXT_VIEW_FACING',
            10: 'MESH_RESOURCE',
            11: 'TRIANGLE_LIST'
        }
        return type_map.get(marker_type, f'UNKNOWN({marker_type})')

def main():
    try:
        rclpy.init()
        
        scene_monitor = SceneMonitor()
        
        print("Scene Monitor Started!")
        print("This node will display information about objects in the simulation scene.")
        print("To see output, run the simulation in another terminal:")
        print("  python ros2_main.py")
        print("Press Ctrl+C to exit.")
        
        rclpy.spin(scene_monitor)
        
    except KeyboardInterrupt:
        print("\nShutting down scene monitor...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'scene_monitor' in locals():
            scene_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()