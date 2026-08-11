# IFL_AIR_DESCRIPTION

This package contains the description files for robot configurations of IRL AIR group projects. Concrete there are the following descriptions:

- [UR robot with Robotiq 2f-85 gripper](urdf/ur.urdf.xacro)
- [UR robot with in-hand orbbec camera and robotiq 2f-85 gripper](urdf/ur_orbbec_robotiq2f85.urdf.xacro)
- [UR robot mounted on big robottable with in-hand orbbec camera and robotiq 2f-85 gripper](urdf/cell_big_ur_orbbec_robotiq2f85.urdf.xacro)
- [UR robot mounted on the small mobile cell with in-hand orbbec camera and robotiq 2f-85 gripper](urdf/cell_small_ur_mobile_orbbec_robotiq2f85.urdf.xacro)

# Launch files

Run the following command to show UR10e robot with Robotiq 2f-85 gripper in RVIZ:

```
ros2 launch ifl_air_description view.launch.py ur_type:=ur10e
```

Run the following command to show UR10e robot with in-hand orbbec camera and robotiq 2f-85 gripper:

```
ros2 launch ifl_air_description view.launch.py ur_type:=ur10e description_file:=ur_orbbec_robotiq2f85.urdf.xacro
```

Run the following command to show UR10e robot mounted on big robottable with in-hand orbbec camera and robotiq 2f-85 gripper:

```
ros2 launch ifl_air_description view.launch.py ur_type:=ur10e description_file:=cell_big_ur_orbbec_robotiq2f85.urdf.xacro
```

Run the following command to show the UR10e robot in the small mobile cell setup (uses the `cell_small_ur_mobile_orbbec_robotiq2f85.urdf.xacro` description by default):

```
ros2 launch ifl_air_description view_cell_small.launch.py ur_type:=ur10e
```

# Hints about creating a moveit configuration

The MoveIt Configuration Assistant can be used to generate a moveit configuration for a given URDF file.
A tutorial describing the process can be found here: https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html

The following points need to be considered as well:

- controller name for robot arm need to be called `scaled_joint_trajectory_controller`
- controller name for the robotiq gripper need to be called `robotiq_2f_urcap_adapter`
- after generating the config files the following changes need to be applied directly on the files:
  - in file `config/xxx.urdf.xacro` the xacro argument `initial_positions_file` need to be renamed to something else (e.g. `initial_positions_file2`) and the corresponding line need to be moved after all include statements. Also the package_name in the find brackets need to be changed to name of the generated package.
  - in file `moveit_controllers.xaml` the parameter `max_effort: 140.0` need to be added to the controller `robotiq_2f_urcap_adapter`


# Configuring finger length of gripper Robotiq F2-85


The gripper can have different fingers installed. There for also the URDF description fo the gripper need to be adjusted.Currently the following `finger_type`s can be configured when using the `ur_orbbec_robotiq` macro:

- `"default"` -> original fingers
- `"custom_75"` -> 75 mm long fingers
- `"custom_160"` -> 160 mm long fingers


For example for the small cell this can be changing in file[urdf/cell_small_ur_orbbec_robotiq2f85.urdf.xacro](urdf/cell_small_ur_orbbec_robotiq2f85.urdf.xacro).

```xml
  <xacro:ur_orbbec_robotiq prefix="" parent="cell_origin" finger_type="custom_75">
    <origin xyz="0.3 0.2 0" rpy="0 0 ${-pi/2}"/>
  </xacro:ur_orbbec_robotiq>
```
