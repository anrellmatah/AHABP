#!/usr/bin/env python3

'''
# *** BEFORE RUNNING SCRIPT ***
$ colcon build --event-handlers console_direct+ --executor sequential --continue-on-error
$ ros2 run v4l2_camera v4l2_camera_node -ros-args -p image_size:="[640,480]"
'''

'''
# *** AFTER RUNNING SCRIPT ***
$ ros2_graph /...
'''
# Import necessary libraries
import os
import subprocess
import sys
import stat
import datetime
import time
import rclpy
import math

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget, MountControl, CommandCode
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
from rclpy.clock import ROSClock
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

class YawControlNode(Node):
    def __init__(self):
        super().__init__('yaw_control_node')

        qos_profile = QoSProfile(reliability = ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.VOLATILE,
                history = HistoryPolicy.KEEP_LAST,
                depth = 1
                )
## Publisher
        self.command_code_pub = self.create_publisher(CommandCode, '/mavros/mount_control/command', 10)

        self.actuator_target_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.mount_control_pub = self.create_publisher(MountControl, '/mavros/mount_control/command', qos_profile)

## Subscriber
        self.subscription = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_profile_sensor_data)


### Initialize internal variables
        self.offboard_setpoint_counter = 0
        self.local_timestamp = 0 
        self.attitude_setpoint = AttitudeTarget()
        self.attitude_setpoint.type_mask = 3
#        self.command_code = CommandCode()
        self.des_yaw = math.radians(0.0)  # Desired yaw angle in radians
        self.des_thrust = 0.3

        self.yaw_rate = 0.2
        self.yaw = 0.0
        self.K_p = 0.6  # Proportional gain
        self.K_d = 0.0
        self.create_timer(0.1, self.publish_command_10Hz) #10 Hz
        #self.create_timer(0.1, self.publish_command_100Hz) #100 Hz
   
    def publish_command_10Hz(self):#  -> None:
        print('In timer callback function: ', self.offboard_setpoint_counter)

## mount command
        command = MountControl()
        now = self.get_clock().now()
        command.header = Header()
        command.header.stamp = now.to_msg()
        command.mode = 2
        self.mount_control_pub.publish(command)

        print('After mount command')

## arm command
        self.command_code = CommandCode()
        # self.command_code = 400
        # self.command_code.Arm = 1.0
        # self.command_code.Force = 21196.0
        # self.command_code_pub.publish(self.command_code)
        #command_code = CommandCode()


        self.command_code(
            CommandCode.COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=21196.0
            )
        self.command_code_pub.publish(self.command_code)

        # self.command_code.publish(
        #     CommandCode.COMPONENT_ARM_DISARM, 
        #     arm=1.0,
        #     force=21196.0
        # )
        #self.command_code_pub.publish(command_code)
    
## yaw error P controller
        diff_yaw = self.des_yaw - self.yaw
        self.attitude_setpoint.body_rate.z = diff_yaw

        diff_yaw_thrust = math.fabs(diff_yaw/math.pi)
        self.des_thrust = self.K_p*diff_yaw_thrust + self.K_d*math.fabs(self.yaw_rate)
        #self.IGNORE_ROLL_RATE = 1
        #self.IGNORE_PITCH_RATE = 2
        self.attitude_setpoint.thrust = self.des_thrust
        self.actuator_target_pub.publish(self.attitude_setpoint)
        print(diff_yaw, self.yaw)

        self.offboard_setpoint_counter += 1

    # def arm(self):
    #     self.get_logger().info('Sending arm')
    #     # self.publish_vehicle_command(
    #     #     CommandCode.COMPONENT_ARM_DISARM, 
    #     #     param1=1.0,
    #     #     param2=21196.0
    #     #     )
    #     self.command_code_pub.publish()

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        euler = self.quaternion_to_euler(orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w)

        euler = orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w
        self.yaw = euler[2]
        self.yaw_rate = msg.angular_velocity.z

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    # Output File Declarations
    date = datetime.datetime.now()                  # Create date string for output files
    timestr = time.strftime("%m%d%-y-%H%M")         # Create time string for output files - to keep things organized

    # Global Declarations
    pi = 3.14159265358979323846
    camera_angle = 0

    # Identifies current file path and constructs a file path string for the output file
    filePath = os.getcwd()                          # It should be the current folder location
    print("The workspace file path is: ", filePath) # "/home/anyell/ahabp_v2_ws"
    imagesPath = os.path.join(filePath, 'images')
    print("Images file path is: ", imagesPath)

    # File to save log information as a csv file. Should not overwrite as long as it doesn't take longer than a minute.
    test_dir_name = 'test_data'                     # "test_data" is the folder the test data will be at.
    dataPath = os.path.join(filePath, test_dir_name)# Makes path
    if not os.path.exists(dataPath):                # Checks if the directory already exists
        os.makedirs(dataPath)                       # Makes the directory if it does not exist
    print("Data file path is: ", dataPath)          # Outputs to terminal the path name; "$ pwd"
    data_file = dataPath + '/' + timestr + '_data.csv' # Makes 
    file = os.open(data_file, os.O_CREAT, 0o775)         # Creates the file and folder if it doesn't exist already.

    rclpy.init(args=args)
    yaw_control_node = YawControlNode()
 #   yaw_control_node.set_offboard_mode()
#    yaw_control_node.arm_payload()
    rclpy.spin(yaw_control_node)
    yaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
