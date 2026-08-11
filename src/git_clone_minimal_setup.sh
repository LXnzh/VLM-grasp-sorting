#!/bin/bash

# General repos, will not change fequently
git clone --depth 1 --branch v2.0.7 https://github.com/orbbec/OrbbecSDK_ROS2.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/robotiq_2f_urcap_adapter.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/ros2_robotiq_gripper.git
git clone --branch ros2 https://github.com/tylerjw/serial.git

# Working repos, these will change frequently make sure to keep them updated
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/moveit_configs/ifl_air_cell_small_ur_orbbec_robotiq_moveit_config.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/ifl_air_description.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/ifl_air_ur_launch.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/arm_api2_msgs.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/arm_api2.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/arm_api2_py.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/ros2_aruco.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/vr_teleoperation_ros2.git
git clone git@gitlab.kit.edu:kit/ifl/gruppen/air/ros2/ros2_record_demo_pkg.git 


