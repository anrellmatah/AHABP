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

def generate_launch_description():
    package_dir = get_package_share_directory('ahabp_pkg')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    microXRCE_Agent_node = ExecuteProcess(
        cmd=[['MicroXRCEAgent serial -b 57600 -D /dev/ttyAMA0 -v 6']],
        shell=True
    )
    sensor_combined_listener_node = Node(
        package='px4_ros_com',
        executable='sensor_combined_listener',
        output='screen',
        shell=True,
    )
#    v4l2_node = ExecuteProcess(
#        cmd=[['ros2 run v4l2_camera v4l2_camera_node -ros-args -p image_size:="[640,480]"']],
#        shell=True
#    )

    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
#        Node(
#            package='px4_offboard',
#            namespace='px4_offboard',
#            executable='visualizer',
#            name='visualizer'
#        ),
        Node( # v4l2 node - home/ahabp_v2_ws/src/v4l2_camera/src
            package='v4l2_camera',
            namespace='v4l2_camera',
            executable='v4l2_camera_node',  #.cpp
            name='v4l2_camera_node',
            prefix='gnome-terminal --', # This will launch the node in a new terminal.
        ),
        Node(
            package='px4_ros_com',
            namespace='px4_ros_com',
            executable='vehicle_gps_position_listener',
            name='vehicle_gps_position_listener',
            prefix='gnome-terminal --',
        ),
        Node(
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node_opencv',
            name='ahabp_node_opencv',
            prefix='gnome-terminal --',
        ),
#        Node(   # MicroXRCE Agent
#            package='Micro-XRCE-DDS-Agent',
#            namespace='Micro-XRCE-DDS-Agent',
#            executable='velocity_control',
#            name='velocity'
#        ),
        Node(
            package='px4_ros_com',
            namespace='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            prefix='gnome-terminal --',
        ),
    ])


