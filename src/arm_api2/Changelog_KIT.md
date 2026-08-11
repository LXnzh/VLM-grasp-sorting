# Changes added by KIT (Edgar Welte)

# Sep 2024

- add frame_id to published current pose
- add service to set max_vel and max_acc scaling factor
- add action servers instead of subscribers to receive commands (pose, joint_state, or pose path)
- add example script to send pose to action server
- add LIN, EST and PRM Planner for pose action

# Oct 2024

- add keyboard control node to control the robot via keyboard in servo mode

# Nov 2024
- integrate xbox joy ctl


# Jan 2025

- refactor code for planning
- add gripper functionality incl. action client for robotiq_2f
- add service for setting end effector link in moveit2 interface
- add servo watchdog node to monitor twist/jog commands and publish zero velocity if inactive
- add cumotion as planner

# Feb 2025
- set proper planner priority: 1. cumotion, if failed then PRM, EST, LIN (each 3 times)
- add planonly mode