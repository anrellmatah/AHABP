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
from ament_index_python.packages import get_package_share_directory #Ark Electronics
import os

from launch import LaunchDescription, action #Ark Electronics
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction # This action will execute a process given its path and arguments, and optionally other things like working directory or environment variables.
from launch.conditions import IfCondition
from launch_ros.actions import Node #Ark Electronics
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, LocalSubstitution, PythonExpression

print('##### Hello from ahabp_v2.launch.py #####')

def generate_launch_description():
    package_dir = get_package_share_directory('ahabp_pkg')

    cmd_uxrce=ExecuteProcess( # pre_command = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial -b 921600 -D /dev/ttyAMA0 -v 5'], # cmd=['your_command', '--arguments'],
        output='screen' # output='screen'
    )

    cmd_veh_gps_pos=ExecuteProcess(
        cmd=['ros2', 'topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps'],
        output='screen'
    )

    node_heartbeat=Node( # This node starts the necessary offboard heartbeat for the pixracer pro at 4 Hz
        package='ahabp_pkg',
        namespace='ahabp_pkg',
        executable='ahabp_node_heartbeat',
        name='ahabp_node_heartbeat',
        prefix='gnome-terminal --',  # This will launch the node in a new terminal.
    )

    node_offboard=Node( # This node activates the offboard mode
        package='ahabp_pkg',
        namespace='ahabp_pkg',
        executable='ahabp_node_offboard',
        name='ahabp_node_offboard',
        prefix='gnome-terminal --',
    )

    node_opencv=Node( # This node launches Scott's code using opencv
        package='ahabp_pkg',
        namespace='ahabp_pkg',
        executable='ahabp_node_opencv',
        output='screen',
        name='ahabp_node_opencv',
        prefix='gnome-terminal --',
    )

    node_tracking=Node( # This node uses Dr. Das' code for tracking
        package='ahabp_pkg',
        namespace='ahabp_pkg',
        executable='ahabp_node_tracking',
        name='ahabp_node_tracking',
        prefix='gnome-terminal --',
    )

    node_listen_gps=Node( # This code uses the example script that subscribes to the gps data
        package='px4_ros_com',
        namespace='px4_ros_com',
        executable='vehicle_gps_position_listener',
        name='vehicle_gps_position_listener',
        prefix='gnome-terminal --',
    )

    node_listen_sens=Node( # This node runs the example that listens to 'sensor_combined' uorb topic
        package='px4_ros_com',
        executable='sensor_combined_listener',
        output='screen',
        shell=True,
    )

    print('Getting into launch description...')
    return LaunchDescription([ # Return the LaunchDescription object, which now contains all nodes to launch. Have to run in this return or else does not work.
        cmd_uxrce,
        node_heartbeat,
        RegisterEventHandler( # The OnProcessStart event handler is used to register a callback function that is executed when the node_listen_gps node starts. It logs a message to the console and executes the cmd_veh_gps_pos action when the node starts.
            OnProcessStart(
                target_action=node_listen_gps,
                on_start=[
                    LogInfo(msg='Gps node started, echoing gps topic...'),
                    cmd_veh_gps_pos
                ]
            )
        ),
        node_offboard,
        node_opencv,
        node_tracking,
        node_listen_gps,
        node_listen_sens
    ])

