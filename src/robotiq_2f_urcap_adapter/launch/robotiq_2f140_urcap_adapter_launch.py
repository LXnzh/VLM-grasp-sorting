# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_ip_launch_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.0.104"
    )
    robot_port_launch_arg = DeclareLaunchArgument(
        "robot_port", default_value="63352"
    )

    robotiq_2f_urcap_adapter_node = Node(
            package='robotiq_2f_urcap_adapter',
            namespace='robotiq_2f140_urcap_adapter',
            executable='robotiq_2f_adapter_node.py',
            name='robotiq_2f85_urcap_adapter',
            parameters=[{
                "robot_ip": LaunchConfiguration('robot_ip'),
                "robot_port": LaunchConfiguration('robot_port'),
                "max_gripper_width": 0.14,
                "min_gripper_width": 0.0,
                "max_gripper_speed": 0.25,
                "min_gripper_speed": 0.03,
                "max_gripper_force": 125.0,
                "min_gripper_force": 10.0,
            }]
        )
    return LaunchDescription([
        robot_ip_launch_arg,
        robot_port_launch_arg,
        robotiq_2f_urcap_adapter_node
    ])
