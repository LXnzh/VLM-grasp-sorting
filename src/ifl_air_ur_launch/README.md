# ifl_air_ur_launch

This ROS2 package contains the multiple custom launch files to run multiple nodes with one command.

## UR driver with calibration

To launch the **ur_robot_driver** with correct calibration file for a specifc robot please run one of the follow commands:

UR10e Robot 1 (name: ex-ur10-1, ip: 192.168.1.102): `ros2 launch ifl_air_ur_launch ex-ur10-1.launch.py`

UR10e Robot 2 (name: ex-ur10-2, ip: 192.168.1.???): `ros2 launch ifl_air_ur_launch ex-ur10-2.launch.py`

## Usecase specific launch files

### UR10e + ORBBEC Femto Mega in hand

To launch the ORBBEC Femto Mega Camera mounted (measured in CAD) on the UR10. RVIZ will open already preconfigured to show colored point cloud. You need to start the driver for robot and gripper beforehand, see command below.

```bash
ros2 launch ifl_air_ur_launch orbbec_femto_mega_net.launch.py
```

### Big Cell (Robotertisch 1520x1800 + UR10e + Orbbec Femto Mega + Robotiq 2F-85)

#### driver

To launch the driver for UR10e (ex-ur10-1) and Robotiq 2f-85 gripper run:

```bash
ros2 launch ifl_air_ur_launch cell_big_ur_orbbec_robotiq.launch.py
```

For simulation (when robot is running as simulator, see AIR-Wiki):

```bash
ros2 launch ifl_air_ur_launch cell_big_ur_orbbec_robotiq_sim.launch.py
```

Don't forget to start the "External Control" program on the teachpanel of UR
after running the command above.

#### moveit

To launch MoveIt with RVIZ to control UR with robotiq gripper:

```bash
ros2 launch ifl_air_ur_launch moveit_cell_big_ur_orbbec_robotiq.launch.py
```


## Trouble shooting

### UR does not move if movement was planned with Moveit

Check if those two ROS2 actions are available and have a client and server:

```bash
$ ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory -t
Action: /scaled_joint_trajectory_controller/follow_joint_trajectory
Action clients: 1
    /moveit_simple_controller_manager [control_msgs/action/FollowJointTrajectory]
Action servers: 1
    /scaled_joint_trajectory_controller [control_msgs/action/FollowJointTrajectory]
```

```bash
$ ros2 action info /robotiq_2f_urcap_adapter/gripper_command -t
Action: /robotiq_2f_urcap_adapter/gripper_command
Action clients: 1
    /moveit_simple_controller_manager [control_msgs/action/GripperCommand]
Action servers: 1
    /robotiq_2f_urcap_adapter [control_msgs/action/GripperCommand]
```

### Error: Can't accept new action goals. Controller is not running

Check that the External control program is running on the robot.
You may have to restart it if you have restarted the ur_driver.
