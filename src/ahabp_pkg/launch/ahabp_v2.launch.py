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

# https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst#id39

import rclpy # 
from rclpy.node import Node #Humble docs - py_pubsub
from std_msgs.msg import String #Humble docs - py_pubsub
from launch import LaunchDescription, action #Ark Electronics
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction # This action will execute a process given its path and arguments, and optionally other things like working directory or environment variables.
from launch_ros.actions import Node #Ark Electronics
from ament_index_python.packages import get_package_share_directory #Ark Electronics
import os
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown

print('#### Hello from ahabp_v2.launch.py ####')
print('This .launch.py file is only meant to launch the nodes after reboot.')

def generate_launch_description():
    package_dir = get_package_share_directory('ahabp_pkg')

    # microxrce_agent=ExecuteProcess(
    #     cmd=[['MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5']],
    #     shell=True
    # ),

    # microxrce_agent=ExecuteProcess(
    #     cmd=['MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5'],
    #     #required=True,
    #     shell=True
    # ),

    # veh_gps_pos=ExecuteProcess(
    #     cmd=[['ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps']],
    #     #required=True,
    #     shell=True
    # ),
    #bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')

    print('Getting into launch description...')
    return LaunchDescription([ # Return the LaunchDescription object, which now contains all nodes to launch. Have to run in this return or else does not work.
        #microxrce_agent,
	    #veh_gps_pos,

        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=microxrce_agent,
        #         on_start=[
        #             LogInfo(msg='MicroXRCEAgent started, communicating with PixRacer Pro')
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=veh_gps_pos,
        #         on_start=[
        #             LogInfo(msg='MicroXRCEAgent started, communicating with PixRacer Pro')
        #         ]
        #     )
        # ),
        # ExecuteProcess(
        #     cmd=['ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps'],
        #     output='screen',
        # ),
        Node( # This node does nothing. Just for testing
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node',
            name='ahabp_node',
            prefix='gnome-terminal --',  # This will launch the node in a new terminal.
        ),
        Node( # This node activates the offboard mode
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node_offboard',
            name='ahabp_node_offboard',
            prefix='gnome-terminal --',
        ),
        # Node( # This node activates the offboard mode
        #     package='px4_ros_com',
        #     namespace='px4_ros_com',
        #     executable='offboard_control',
        #     name='offboard_control',
        #     prefix='gnome-terminal --',
        # ),
        Node( # This node launches Scott's code using opencv
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node_opencv',
            output='screen',
            name='ahabp_node_opencv',
            prefix='gnome-terminal --',
        ),
        Node( # This node uses Dr. Das' code for tracking
            package='ahabp_pkg',
            namespace='ahabp_pkg',
            executable='ahabp_node_tracking',
            name='ahabp_node_tracking',
            prefix='gnome-terminal --',
        ),
        Node( # This code uses the example script that subscribes to the gps data
            package='px4_ros_com',
            namespace='px4_ros_com',
            executable='vehicle_gps_position_listener',
            name='vehicle_gps_position_listener',
            prefix='gnome-terminal --',
        ),
        Node( # This node runs the example that listens to 'sensor_combined' uorb topic
            package='px4_ros_com',
            executable='sensor_combined_listener',
            output='screen',
            shell=True,
        )
        ])

