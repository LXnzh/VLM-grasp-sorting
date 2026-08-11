# Robotiq 2F URCap Adapter

This is a direct control interface to the Robotiq *2F gripper URCap* using a socket connection and string commands.
The used string interface is defined
[here](https://dof.robotiq.com/discussion/2420/control-robotiq-gripper-mounted-on-ur-robot-via-socket-communication-python).
For communicating with that interface we use work from SDU's `ur_rtde`
[repository](https://gitlab.com/sdurobotics/ur_rtde/blob/master/doc/_static/robotiq_gripper.py)
licensed under the MIT license.

> **Important**: Make sure the security setting on UR controller are not blocking incoming traffic on port 63352 (Settings -> Security -> General). 

The overall idea is as follows:

1) Mount the gripper to the UR robot of your choice and connect it via
Robotiq's recommended way. There seem to be two options: Directly over the
end-effector for URe versions, and over UR's control box for the other
versions.

2) Use Robotiq's URCap to interface the gripper from UR's Polyscope. This
URCap seems to be better maintained by Robotiq than ROS packages and should cover all of the
gripper's features. You should be able to get the URCap
[here](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button). The
[quick start
guide](https://blog.robotiq.com/hubfs/Support%20Documents/QSG/Quick_start_2Finger_e-Series_nocropmarks_EN.pdf)
should help you getting started quickly.

3) To allow the ROS2 package to properly control the gripper, some of the hardware specs need to specified via ROS parameters.
These include the grip width (in Meters; `max_gripper_width`, `min_gripper_width`), the speed (in Meters / s; `max_gripper_speed`, `min_gripper_speed`) and the effort (in Newton; `max_gripper_force` `min_gripper_force`).
The user also has to set the IP address (`robot_ip`) of the robot.

4) When starting the `robotiq_2f_urcap_adapter` node the gripper will activate, if not already activated, and auto-calibrate its movement range.

5) The gripper can then be controlled using ROS actions using the `robotiq_2f_urcap_adapter/gripper_command` action.

## Parameters
The node currently has the following parameters:

- **`robot_ip`**:
  The robot's IP address
  (*default: "192.168.0.104"*)
- **`robot_port`**:
  The port where the URCap interface is running on the robot. This should usually not be changed.
  (*default: 63352*)
- **`max_gripper_width`**:
  The maximum opening width of a gripper in meters. Change this if your have a 2F-140 gripper.
  (*default: 0.085*)
- **`min_gripper_width`**:
  The minimum opening width of a gripper in meters.
  (*default: 0.0*)
- **`max_gripper_speed`**:
  The maximum speed of a gripper in meters/sec. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested speeds higher than this will be rejected.
  (*default: 0.15*)
- **`min_gripper_speed`**:
  The minimum speed of a gripper in meters/sec. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested speeds lower than this will be rejected.
  (*default: 0.02*)
- **`max_gripper_force`**:
  The maximum force of a gripper in Newtons. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested forces higher than this will be rejected.
  (*default: 235*)
- **`min_gripper_force`**:
  The minimum force of a gripper in Newtons. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested forces lower than this will be rejected.
  (*default: 20*)
- **`action_server_name`**:
  The name of the action server.
  (*default: "robotiq_2f_urcap_adapter/gripper_command"*)

## Usage
Start the node:

```
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102
```

Close the gripper:
```
ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command control_msgs/GripperCommand '{ command: { position: 0.8, max_effort: 140 }}'
```

Open the gripper (assuming it's a 2F-85):
```
ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command control_msgs/GripperCommand '{ command: { position: 0.0, max_effort: 140 }}'
```

## Action Client Example (Python)

A simple example of an action client (Python) to control the gripper is provided in the `examples` folder. To run it, first start the adapter node and then run the example script:

```bash
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102

ros2 run robotiq_2f_urcap_adapter gripper_action_client.py
```



## Mock Gripper

To test this adapter you can start a mock server which emulates the gripper. 

Please start therefore first the server:

```
ros2 run robotiq_2f_urcap_adapter robotiq_2f_mock_server.py
```

Then start the adapter:

```
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=127.0.0.1
```

You can also start both command with configured parameters with this launch file:

```
ros2 launch robotiq_2f_urcap_adapter robotiq_2f85_urcap_adapter+mock_launch.py
```



## Trouble shooting

If the gripper does not get activated (open-close movement) at startup check the log, if the following error appears the reason is that the urcap returns the string "ACT ?" which means the gripper can not be communicated to.

```bash
Traceback (most recent call last):
[robotiq_2f_adapter_node.py-1]   File "/home/ws/install/robotiq_2f_urcap_adapter/lib/robotiq_2f_urcap_adapter/robotiq_2f_adapter_node.py", line 821, in <module>
[robotiq_2f_adapter_node.py-1]     main()
[robotiq_2f_adapter_node.py-1]   File "/home/ws/install/robotiq_2f_urcap_adapter/lib/robotiq_2f_urcap_adapter/robotiq_2f_adapter_node.py", line 805, in main
[robotiq_2f_adapter_node.py-1]     gripper_control_adapter = Robotiq2fAdapterNode()
[robotiq_2f_adapter_node.py-1]   File "/home/ws/install/robotiq_2f_urcap_adapter/lib/robotiq_2f_urcap_adapter/robotiq_2f_adapter_node.py", line 247, in __init__
[robotiq_2f_adapter_node.py-1]     self.gripper_adapter.activate(auto_calibrate=auto_calibrate)
[robotiq_2f_adapter_node.py-1]   File "/home/ws/install/robotiq_2f_urcap_adapter/local/lib/python3.10/dist-packages/robotiq_2f_urcap_adapter_socket/robotiq_2f_socket_adapter.py", line 242, in activate
[robotiq_2f_adapter_node.py-1]     if not self.is_active:
[robotiq_2f_adapter_node.py-1]   File "/home/ws/install/robotiq_2f_urcap_adapter/local/lib/python3.10/dist-packages/robotiq_2f_urcap_adapter_socket/robotiq_2f_socket_adapter.py", line 269, in is_active
[robotiq_2f_adapter_node.py-1]     return GripperStatus(status) == GripperStatus.ACTIVE
[robotiq_2f_adapter_node.py-1]   File "/usr/lib/python3.10/enum.py", line 385, in __call__
[robotiq_2f_adapter_node.py-1]     return cls.__new__(cls, value)
[robotiq_2f_adapter_node.py-1]   File "/usr/lib/python3.10/enum.py", line 710, in __new__
[robotiq_2f_adapter_node.py-1]     raise ve_exc
[robotiq_2f_adapter_node.py-1] ValueError: -1 is not a valid GripperStatus
```

**Solution:**

Please verify in the teach pendant that the gripper is configured at ID1 (of 4). Otherwise the ROS2 adapter cannot work.

![image](docs/urcap_gripper_id.png)