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
# limitations under the License.

__author__ = "Anyell Mata"
__contact__ = "anyellmata@gmail.com"

import rclpy #Humble docs
from rclpy.node import Node #Humble docs - py_pubsub
from std_msgs.msg import String, Header #Humble docs - py_pubsub

from launch import LaunchDescription #Ark Electronics
from launch.actions import ExecuteProcess #Ark Electronics
from launch_ros.actions import Node #Ark Electronics
from ament_index_python.packages import get_package_share_directory #Ark Electronics

import os

import rclpy
from rclpy.node import Node #Node class
from std_msgs.msg import String #imports the built-in string message type that the node uses to structure the data that it passes on the topic.
# Recall that dependencies have to be added to package.xml file in ~...ws/src folder
import sys
import select
import time

import cv2 as cv # Scott

from datetime import datetime, timezone # Dr.Das
import math # Dr.Das

print('Hello from ahabp_dev_buffer_file.')

def generate_launch_description():
   print('Here at launch description.')
   package_dir = get_package_share_directory('ahabp_pkg')

   microXRCE_Agent_node = ExecuteProcess(
       cmd=[['MicroXRCEAgent serial -b 57600 -D /dev/ttyAMA0']],
       shell=True
   )
   sensor_combined_listener_node = Node(
       package='px4_ros_com',
       executable='sensor_combined_listener',
       output='screen',
       shell=True,
   )
   # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
   return LaunchDescription([
       # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
       Node(
           package='px4_offboard',
           namespace='px4_offboard',
           executable='visualizer',
           name='visualizer'
       ),
       Node(
           package='px4_offboard',
           namespace='px4_offboard',
           executable='processes',
           name='processes',
           prefix='gnome-terminal --'
       ),
       Node(
           package='px4_offboard',
           namespace='px4_offboard',
           executable='control',
           name='control',
           prefix='gnome-terminal --',
       ),
       Node(
           package='px4_offboard',
           namespace='px4_offboard',
           executable='velocity_control',
           name='velocity'
       ),
       Node(
           namespace='v42l_start',
           cli_args=os.system('ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]"'), # Send the v4l2 command to start the pi cam.
           name='v4l2_',
           prefix='gnome-terminal --',
       ),
       microXRCE_Agent_node,
       sensor_combined_listener_node
   ])



class MinimalPublisher(Node):


   def __init__(self):
       super().__init__('minimal_publisher') #calls the Node class’s constructor and gives it your node name, in this case minimal_publisher.
       self.publisher_ = self.create_publisher(String, 'topic', 10) #declares that the node publishes messages of type.
       timer_period = 0.5  # seconds
       self.timer = self.create_timer(timer_period, self.timer_callback)
       self.i = 0


   def timer_callback(self): #timer is created with a callback to execute every 0.5 seconds. self.i is a counter used in the callback.
       msg = String()
       msg.data = 'Hello World: %d' % self.i
       self.publisher_.publish(msg)
       self.get_logger().info('Publishing: "%s"' % msg.data)
       self.i += 1


def main(args=None): #First the rclpy library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.
   print('Here at start of main.')
   rclpy.init(args=args)


   minimal_publisher = MinimalPublisher()


   rclpy.spin(minimal_publisher)


   # Destroy the node explicitly
   # (optional - otherwise it will be done automatically
   # when the garbage collector destroys the node object)
   minimal_publisher.destroy_node()
   rclpy.shutdown()
   print('Here at end of main.')


if __name__ == '__main__':
   main()
