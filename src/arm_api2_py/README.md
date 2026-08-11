# ArmApi2Client

The `ArmApi2Client` class provides an interface to control a robotic arm using ROS 2 actions and services. This client allows you to set velocity and acceleration scaling factors, change the arm's state, control the gripper, and move the arm to specified poses or joint positions.


> **Hint:** The functions in the `ArmApi2Client` class are blocking, so you should call them in a separate thread to avoid blocking the main thread.

## Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository url>
```

Build the package:

```bash
cd ~/ros2_ws
colcon build
```

## Usage

### Initialization
To use the ArmApi2Client, you need to initialize it with a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from arm_api2_py.arm_api2_client import ArmApi2Client

rclpy.init()
node = Node('arm_api2_client_node')
client = ArmApi2Client(node)
```

### Get current EE pose
You can get the current end effector pose using the get_current_ee_pose method:

```python
pose_stamped = client.get_current_ee_pose()
```

This depends on which end effector link is set, see section [Setting End Effector Link](#setting-end-effector-link).

### Setting Velocity and Acceleration Scaling Factors
You can set the velocity and acceleration scaling factors using the set_vel_acc method:

```python
client.set_vel_acc(0.5, 0.5)
```

### Setting End Effector Link
You can set the end effector link name using the set_eelink method:

```python
client.set_eelink('tcp')
```

### Changing Arm State
You can change the arm's state using the change_state_to method. The available states are "JOINT_TRAJ_CTL", "CART_TRAJ_CTL", and "SERVO_CTL":

```python
client.change_state_to('JOINT_TRAJ_CTL')
```

Alternatively, you can use the convenience methods:

```python
client.change_state_to_joint_ctl()
client.change_state_to_cartesian_ctl()
client.change_state_to_servo_ctl()
```

### Controlling the Gripper
You can open and close the gripper using the gripper_open and gripper_close methods:

```python
client.gripper_open()
client.gripper_close()
```

It will return besides the `success` status the following data: current gripper `position`, `effort`, `is_stalled`, `reached_goal`.

### Moving the Arm to a Pose
You can move the arm to a specified pose using the move_to_pose method:

```python
from geometry_msgs.msg import PoseStamped

goal_pose = PoseStamped()
# Set the desired pose values
client.move_to_pose(goal_pose)
```

### Moving the Arm to Joint Positions
You can move the arm to specified joint positions using the move_to_joint method:

```python
from sensor_msgs.msg import JointState

joint_positions = JointState()
# Set the desired joint positions
client.move_to_joint(joint_positions)
```

### Moving the Arm to a Sequence of Poses
You can move the arm to a sequence of poses using the move_to_pose_path method:

```python
goal_path = [PoseStamped(), PoseStamped()]
# Set the desired poses
client.move_to_pose_path(goal_path)
```

### Velocity control via MoveIt Servo
You can send velocity command to the robot, either in cartesian space or joint space.

You need to change to servo_ctl mode first, see section [Changing Arm State](#changing-arm-state).

```python
client.change_state_to_servo_ctl()
```

#### Cartesian velocity control


```python
from geometry_msgs.msg import TwistStamped

cartesian_velocity = TwistStamped()
cartesian_velocity.header.frame_id = "tcp"  # e.g. tcp or base_link
cartesian_velocity.twist.linear.x = 0.1
cartesian_velocity.twist.linear.y = 0.0
cartesian_velocity.twist.linear.z = 0.0
cartesian_velocity.twist.angular.x = 0.0
cartesian_velocity.twist.angular.y = 0.0
cartesian_velocity.twist.angular.z = 0.0

client.send_twist_cmd(cartesian_velocity)
```

#### Joint velocity control

```python

names =["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
velocities = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]

client.send_joint_vel_cmd(names, velocities)
```


### Switch ROS2 controllers

You can switch between different ROS2 controllers using the `switch_controller` method (using the `/controller_manager/switch_controller`).

This function is automatically called when changing the arm state, but you can also call it directly.

```python
from control_msgs.srv import SwitchController

client.switch_controller(start_controllers=['joint_group_position_controller'], stop_controllers=['forward_position_controller'])
```



