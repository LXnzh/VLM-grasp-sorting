# ifl_air_cell_small_ur_orbbec_robotiq_moveit_config

This package included the configuration for moveit2 for the small robot table (1520x900) with UR10e, in-hand ORBBEC femto mega and Robotiq 2F-85 gripper.

It was generated with the [MoveIt Setup Assistent](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) based on this URDF file from *ifl_air_description* package: [cell_small_ur_orbbec_robotiq2f85.urdf.xacro](https://gitlab.kit.edu/kit/ifl/gruppen/air/ros2/ifl_air_description/-/blob/main/urdf/cell_small_ur_orbbec_robotiq2f85.urdf.xacro). 

To start the setup assistant and adjust the settings of this config run the following command:

```
ros2 launch ifl_air_cell_small_ur_orbbec_robotiq_moveit_config setup_assistant.launch.py
```

The following changes were made manually after generating the config with the setup assistant:

- In file: **config/cell_small_ur_orbbec_robotiq2f85.urdf.xacro**: Rename "initial_positions_file" to "initial_positions_file2" to prevent name conflicts.
- In file: **package.xml**: comment out the line `<exec_depend>warehouse_ros_mongo</exec_depend>` because it is not available in ROS2.
- In file: **config/moveit_controllers.yaml**: Add the following line to the _robotiq_2f_urcap_adapter_ section: `max_effort: 140.0`
- Add planner config files, mostly copied from [moveit2 repo on GitHub](https://github.com/moveit/moveit2/tree/main/moveit_configs_utils/default_configs) and adjusted by comparing it with panda examples: ompl_planning.yaml, chomp_planning.yaml, stomp_planning.yaml, and pilz_industrial_motion_planner_planning.yaml