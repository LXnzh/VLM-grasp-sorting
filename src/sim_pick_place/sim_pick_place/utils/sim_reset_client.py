import time
import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

class SimResetClient:
    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup | None = None):
        self.node = node
        self._cbg = callback_group or ReentrantCallbackGroup()

        # Create and keep clients (same callback group)
        self.reset_client = self.node.create_client(Trigger, '/reset_sim', callback_group=self._cbg)
        self.reset_custom_client = self.node.create_client(Trigger, '/reset_sim_custom', callback_group=self._cbg)
        self.param_client = self.node.create_client(SetParameters, '/mujoco_ur10e_interface/set_parameters', callback_group=self._cbg)

        # Wait for services (no spinning here; just the underlying discovery)
        self.node.get_logger().info('SimResetClient: Waiting for reset/param services...')
        for cli, name in [(self.reset_client, '/reset_sim'),
                          (self.reset_custom_client, '/reset_sim_custom'),
                          (self.param_client, '/mujoco_ur10e_interface/set_parameters')]:
            while rclpy.ok() and not cli.wait_for_service(timeout_sec=0.5):
                self.node.get_logger().info(f'  waiting for {name} ...')
        self.node.get_logger().info('SimResetClient: Services are available!')

    # ---- helpers ----
    def _wait_future(self, future, timeout_sec: float) -> bool:
        """Wait without spinning; executor in main thread does the spinning."""
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    # ---- API ----
    def default_reset(self) -> bool:
        req = Trigger.Request()
        fut = self.reset_client.call_async(req)
        if not self._wait_future(fut, 5.0):
            self.node.get_logger().error('Default reset: timed out')
            return False
        resp = fut.result()
        self.node.get_logger().info(
            f'Default reset {"succeeded" if resp.success else "failed"}: {resp.message}')
        return bool(resp.success)

    def custom_reset(self, reset_spec: dict) -> bool:
        self.node.get_logger().info(f'Setting custom reset spec:\n{json.dumps(reset_spec, indent=2)}')

        # set parameter first
        param = Parameter()
        param.name = 'reset_spec'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                     string_value=json.dumps(reset_spec))
        preq = SetParameters.Request()
        preq.parameters = [param]
        pfut = self.param_client.call_async(preq)
        if not self._wait_future(pfut, 2.0):
            self.node.get_logger().warn('Parameter service timed out; continuing anyway')
        else:
            self.node.get_logger().info('Parameter set successfully')

        # small settle delay (optional)
        time.sleep(0.2)

        # call custom reset
        self.node.get_logger().info('Calling custom reset service...')
        req = Trigger.Request()
        fut = self.reset_custom_client.call_async(req)
        if not self._wait_future(fut, 5.0):
            self.node.get_logger().error('Custom reset: timed out')
            return False
        resp = fut.result()
        self.node.get_logger().info(
            f'Custom reset {"succeeded" if resp.success else "failed"}: {resp.message}')
        return bool(resp.success)
