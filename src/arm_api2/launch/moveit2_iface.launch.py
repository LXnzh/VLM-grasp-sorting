######################################################################
 # BSD 3-Clause License
 #
 # Copyright (c) 2024, Crobotic Solutions d.o.o.
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #
 # * Redistributions of source code must retain the above copyright notice, this
 #   list of conditions and the following disclaimer.
 #
 # * Redistributions in binary form must reproduce the above copyright notice,
 #   this list of conditions and the following disclaimer in the documentation
 #   and/or other materials provided with the distribution.
 #
 # * Neither the name of the copyright holder nor the names of its
 #   contributors may be used to endorse or promote products derived from
 #   this software without specific prior written permission.Need document name?RTICULAR PURPOSE
 # ARE
 # DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 # FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 # DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 # SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 # OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
######################################################################

 #      Title       : moveit2_iface.launch.py
 #      Project     : arm_api2
 #      Created     : 05/10/2024
 #      Author      : Filip Zoric
 #
 #      Description : Launch file for moveit2_iface
 #

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.substitutions import LaunchConfiguration

import yaml
import os

# TODO: Make this changeable without ERROR for wrong param type
use_sim_time = True
use_servo = True
dt = 0.1

def launch_setup(context, *args, **kwargs):

    launch_nodes_ = []
    arg_robot_name      = context.perform_substitution(LaunchConfiguration('robot_name'))
    arg_launch_joy      = context.perform_substitution(LaunchConfiguration('launch_joy', default=False))
    arg_launch_servo_watchdog = context.perform_substitution(LaunchConfiguration('launch_servo_watchdog', default=True))
    print("arg_launch_joy: ", arg_launch_joy)   

    # TODO: Swap between sim and real arg depending on the robot type
    robot_yaml = "{0}/{1}_sim.yaml".format(arg_robot_name, arg_robot_name)
    servo_yaml = "{0}/{1}_servo_sim.yaml".format(arg_robot_name, arg_robot_name)
    kinematics_yaml = "config/{0}/{1}_kinematics.yaml".format(arg_robot_name, arg_robot_name)
    
    # Arm params (ctl, servo) --> sent just as path
    # 3 different ways of loading and using yaml files, DISGUSTING [FIX ASAP]
    config_path = os.path.join(
        get_package_share_directory('arm_api2'),
        "config",
        robot_yaml
    )

    # Servo params created with the help of ParameterBuilder
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_api2") 
        .yaml(f"config/{servo_yaml}")
        .to_dict()
    }
    
    # Load kinematic params
    kinematic_params = load_yaml("arm_api2", kinematics_yaml)

    launch_move_group = Node(
        package='arm_api2',
        executable='moveit2_iface',
        #prefix=['gdbserver localhost:3000'], # Used for debugging
        parameters=[{"use_sim_time": use_sim_time},
                    {"enable_servo": use_servo},
                    {"dt": dt},
                    {"config_path": config_path},
                    kinematic_params,
                    servo_params,]
    )

    launch_nodes_.append(launch_move_group)
    
    if str(arg_launch_joy).lower() == "true": # To avoid pkg not found 
        # https://index.ros.org/p/joy/ --> joy node as joystick (Create subscriber that takes cmd_vel)
        joy_node = Node(
            package='joy_linux', 
            executable="joy_linux_node", 
            output="screen", 
            arguments={'device_name':"js0"}.items()
        )
        launch_nodes_.append(joy_node)
    
    if str(arg_launch_servo_watchdog).lower() == "true":
        launch_servo_watchdog = Node(
            package='arm_api2',
            executable='servo_watchdog.py',
            output='screen'
        )
        launch_nodes_.append(launch_servo_watchdog)

    return launch_nodes_

#https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/
def generate_launch_description(): 

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(name='robot_name',
                              default_value='kinova',
                              description='robot name')
    )
    # TODO: THIS IS NOT CONVERTED TO FALSE WHEN SETUP! FIX IT!
    declared_arguments.append(
        DeclareLaunchArgument(name='launch_joy', 
                              default_value='false', 
                              description='launch joystick')
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(name='launch_servo_watchdog', 
                              default_value='true', 
                              description='launch servo_watchdog node')
    )

    declared_arguments.append(
        DeclareLaunchArgument(name='dt', 
                              default_value='0.1', 
                              description='time step')
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)

def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None