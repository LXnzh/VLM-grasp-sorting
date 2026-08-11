#!/usr/bin/env python3
"""
Example script for ROS2 reset services.

This script demonstrates how to use the reset services programmatically.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time
import json
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class ResetClient(Node):
    def __init__(self):
        super().__init__('reset_service_tester')
        
        # Create service clients
        self.reset_client = self.create_client(Trigger, '/reset_sim')
        self.reset_custom_client = self.create_client(Trigger, '/reset_sim_custom')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for reset services...')
        self.reset_client.wait_for_service(timeout_sec=5.0)
        self.reset_custom_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Reset services are available!')

    def default_reset(self):
        """Test default reset service."""
        
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Default reset {"succeeded" if response.success else "failed"}: '
                f'{response.message}'
            )
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False

    def custom_reset(self, reset_spec):
        """Test custom reset service with given specification."""
        self.get_logger().info(f'Setting custom reset spec: {json.dumps(reset_spec, indent=2)}')
        
        # Set the parameter
        param = Parameter()
        param.name = 'reset_spec'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=json.dumps(reset_spec)
        )
        
        # Use parameter service to set it
        param_client = self.create_client(SetParameters, '/mujoco_ur10e_interface/set_parameters')
        
        if not param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Parameter service not available, trying direct call...')
        else:
            req = SetParameters.Request()
            req.parameters = [param]
            future = param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result():
                self.get_logger().info('Parameter set successfully')
        
        # Small delay to ensure parameter is set
        time.sleep(0.2)
        
        # Call the custom reset service
        self.get_logger().info('Calling custom reset service...')
        request = Trigger.Request()
        future = self.reset_custom_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Custom reset {"succeeded" if response.success else "failed"}: '
                f'{response.message}'
            )
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False


def main():
    rclpy.init()
    
    reset_client = ResetClient()
    
    try:
        # Test 1: Default reset
        print("\n" + "="*60)
        print("TEST 1: Default Reset")
        print("="*60)
        reset_client.default_reset()
        time.sleep(2)
        
        # Test 2: Custom reset with arm position only
        print("\n" + "="*60)
        print("TEST 2: Custom Reset - Arm Position Only")
        print("="*60)
        custom_spec_1 = {
            "robot": {
                "joint_positions": [0.0, -1.57, -1.57, 1.57, -1.57, 0.0]
            }
        }
        reset_client.custom_reset(custom_spec_1)
        time.sleep(2)
        
        # Test 3: Custom reset with object positions
        print("\n" + "="*60)
        print("TEST 3: Custom Reset - Object Positions")
        print("="*60)
        custom_spec_2 = {
            "objects": {
                "red_cube": {
                    "position": [-0.6, 0.0, 0.2],
                    "orientation": [1.0, 0.0, 0.0, 0.0]
                }
            }
        }
        reset_client.custom_reset(custom_spec_2)
        time.sleep(2)
        
        # Test 4: Full custom reset
        print("\n" + "="*60)
        print("TEST 4: Full Custom Reset")
        print("="*60)
        custom_spec_3 = {
            "robot": {
                "joint_positions": [-0.2345, -1.0715, -1.8688, -1.5812, 1.6339, 2.8947]
            },
            "objects": {
                "red_cube": {
                    "position": [-0.5, 0.2, 0.15],
                    "orientation": [1.0, 0.0, 0.0, 0.0]
                },
                "blue_sphere": {
                    "position": [-0.3, -0.3, 0.1]
                }
            }
        }
        reset_client.custom_reset(custom_spec_3)
        
        print("\n" + "="*60)
        print("All tests completed!")
        print("="*60)
        
    except KeyboardInterrupt:
        pass
    finally:
        reset_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
