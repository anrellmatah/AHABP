#!/usr/bin/env python

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.-b  

# This launch file will run nodes: MicroXRCEAgent, v4l2, PX4-ROS comms, Scott's OpenCV code,
# and AHABP node.

__author__ = "Anyell Mata"
__contact__ = "anyellmata@gmail.com"

import rclpy #Humble docs
from rclpy.node import Node #Humble docs - py_pubsub
from std_msgs.msg import String #Humble docs - py_pubsub

from launch import LaunchDescription #Ark Electronics
from launch.actions import ExecuteProcess #Ark Electronics
from launch_ros.actions import Node #Ark Electronics
from ament_index_python.packages import get_package_share_directory #Ark Electronics

import os

print('Hello from ahabp_v2.launch.py')
print('This .launch.py file is only meant to launch the nodes after reboot.')

def generate_launch_description():
    package_dir = get_package_share_directory('ahabp_pkg')

    microxrce_agent=ExecuteProcess(
        cmd=[['MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5']],
        shell=True
    ),
    veh_gps_pos=ExecuteProcess(
        cmd=[['ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps']],
        shell=True
    ),
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')

    print('Getting into launch description.')
    return LaunchDescription([ # Return the LaunchDescription object, which now contains all nodes to launch.
# Put ahabp_node python files after this to keep organized.
        microxrce_agent,
	    veh_gps_pos,
        Node(
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node',
            name='ahabp_node',
            prefix='gnome-terminal --',  # This will launch the node in a new terminal.
        ),
        Node(
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node_opencv',
            output='screen',
            name='ahabp_node_opencv',
            prefix='gnome-terminal --',
        ),
        Node(
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node_tracking',
            name='ahabp_node_tracking',
            prefix='gnome-terminal --',
        ),
        Node(
            package='px4_ros_com',
            namespace='px4_ros_com',
            executable='vehicle_gps_position_listener',
            name='vehicle_gps_position_listener',
            prefix='gnome-terminal --',
        ),
        Node(
            package='px4_ros_com',
            namespace='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            prefix='gnome-terminal --',
        ),
        Node(
            package='px4_ros_com',
            executable='sensor_combined_listener',
            output='screen',
            shell=True,
        )
        ])

        #microxrce_agent,
	    #veh_gps_pos,
	    #ahabp_pkg_node,
	    #ahabp_pkg_opencv_node,
	    #ahabp_pkg_tracking_node,
	    #vehicle_gps_position_listener_node,
	    #offboard_control_node,
	    #sensor_combined_listener_node
