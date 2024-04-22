#!/usr/bin/env python3

################################################################
# Title: AHABP Node Tracking 3
# Date: 4/8/2024
# Author: Anyell Mata
# Description: This code will incorporate Scott's code and manual Proportional controller commands.
################################################################

'''
# *** BEFORE RUNNING SCRIPT ***
$ colcon build --event-handlers console_direct+ --executor sequential --continue-on-error
$ MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5
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
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleAttitudeSetpoint, VehicleControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus # These are compatible .msg files. Check 'px4_msgs/msg'
from numpy import nan
import cv2 as cv
import time

print('##### Hi from ahabp_node_tracking_3.py #####')

class Tracking(Node): # Node. --> self.

    print('Inside class')
    def __init__(self):
        super().__init__('tracking_pub')  # This is the name of the node. It will appear as a oval in rqt's node graph.

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10  # Adjust the queue size as needed
        )

        # Create publisher for actuator motors
        self.offboard_control_mode_publisher = self.create_publisher( # create publisher for offboard mode
            OffboardControlMode, # px4_msg uORB message. Check 'pX4_msgs/msg'
            '/fmu/in/offboard_control_mode', # topic type. Check 'dds_topics.yaml'
            qos_profile # QoS
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, 
            '/fmu/in/vehicle_attitude_setpoint', 
            qos_profile
        )
        self.vehicle_control_mode_publisher = self.create_publisher(
            VehicleControlMode, 
            '/fmu/in/config_control_setpoints', 
            qos_profile
        )
        self.vehicle_trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )

        # Create subscribers
        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, # px4_msg uORB message. Check 'pX4_msgs/msg'
        #     '/fmu/out/vehicle_local_position', # topic type. Check 'dds_topics.yaml'
        #     self.vehicle_local_position_callback, # Callback function
        #     qos_profile # QoS
        # )
        # self.vehicle_status_subscriber = self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status', 
        #     self.vehicle_status_callback, 
        #     qos_profile
        # )

    # def vehicle_local_position_callback(self, vehicle_local_position):
    #     """Callback function for vehicle_local_position topic subscriber."""
    #     self.vehicle_local_position = vehicle_local_position

    # def vehicle_status_callback(self, vehicle_status):
    #     """Callback function for vehicle_status topic subscriber."""
    #     self.vehicle_status = vehicle_status

        # Initialize internal variables
        self.offboard_setpoint_counter = 0
        self.local_timestamp = 0 
        self.des_yaw = math.radians(0.0)  # Desired yaw angle in radians. 0.0 should be North
        self.K_p = 0.6  # Proportional gain

        self.timer = self.create_timer(.1, self.timer_callback_10Hz) #10Hz
        #self.timer = self.create_timer(.01, self.timer_callback_100Hz) #100Hz
        

#### Individual command functions ####

    def engage_offboard_mode(self): ## This command works
        self.get_logger().info("Sending engage_offboard_mode")
        # self.publish_vehicle_command(
        #     VehicleCommand.VEHICLE_CMD_DO_SET_MODE, # https://raw.githubusercontent.com/PX4/px4_msgs/main/msg/VehicleCommand.msg
        #     param1=194.0, # 216, 194, 220 - https://mavlink.io/en/messages/common.html#MAV_MODE
        #     )
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0, 
            param2=6.0
            )

    def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.get_logger().info("Sending pub_act_test#1")
        self.publish_vehicle_command( # Test motor #1
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, # 310 - https://raw.githubusercontent.com/PX4/px4_msgs/main/msg/VehicleCommand.msg
            param1=0.0, # Output value [min:-1, max:1]
            param2=3.0, # Timeout [seconds]
            param5=1.0, # https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_FUNCTION
            )
        self.get_logger().info("Sending pub_act_test#3")
        self.publish_vehicle_command( # Test motor #3
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, 
            param1=0.0,
            param2=3.0,
            param5=3.0,
            )

    def pub_act_test_off(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.get_logger().info("Sending pub_act_test_off")
        self.publish_vehicle_command( # Test motor #1
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, 
            param1=-1.0,
            param2=1.0,
            param5=1.0,
            )
        self.publish_vehicle_command( # Test motor #3
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, 
            param1=-1.0,
            param2=1.0,
            param5=3.0,
            )
        
    def pub_veh_att_set(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.get_logger().info("Sending pub_veh_att_set")
        self.publish_vehicle_attitude_setpoint_command(
            yaw_body            = 0.0,
            #yaw_sp_move_rate    = 0.1,
            #q_d                 = [0.97015, 0.0, 0.0, 0.0],
            #thrust_body         = [0.1, 0.1, 0.1],
            )

    def arm(self): ## This commands works
        self.get_logger().info('Sending arm')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0
            )

    def pub_veh_ctl_mode(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.get_logger().info("Sending pub_veh_ctl_mode")
        self.publish_vehicle_control_mode_command()

    def pub_traj_set(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.get_logger().info("Sending pub_traj_set")
        self.publish_trajectory_setpoint_command()

# #### Truman's imu stuff
#     def vehicle_attitude_callback(self, msg):
#         # Extract yaw angle from IMU data (assuming quaternion orientation)
#         orientation_q = msg.orientation
#         euler = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
#         yaw = euler[2]  # Yaw angle in radians

#         # Calculate yaw error
#         yaw_error = self.des_yaw - yaw
#         diff_yaw_thrust = math.fabs(yaw_error/math.pi)

#         # Calculate yaw rate command using proportional control
#         yaw_rate_cmd = self.K_p * yaw_error

#         # Publish attitude target with only yaw rate command
#         attitude_target = AttitudeTarget()
#         attitude_target.type_mask = 1  # Ignore roll and pitch rates
#         attitude_target.thrust = self.K_p*diff_yaw_thrust  # Set a thrust
#         attitude_target.body_rate.z = yaw_rate_cmd  # Set the yaw rate command

#         self.publisher.publish(attitude_target)

#### The message compilers ####
# These functions build the complete long command that the respective topics listen too. 
# They'll either make the command or revert to a default command. I am not sure if this is necessary
# but saw it in many examples.

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        msg.thrust_and_torque = True
        msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        self.get_logger().info('publish_offboard_control_heartbeat_signal published:')
        print(msg)

    def publish_vehicle_command(self, command, **params) -> None: # This defaults to nothing except for determining the targets which is redundant.
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 0
        msg.source_system = 0
        msg.source_component = 0
        msg.confirmation = 0
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('publish_vehicle_command published:')
        print(msg)

    def publish_vehicle_attitude_setpoint_command(self, **params) -> None:
        msg = VehicleAttitudeSetpoint()
        msg.roll_body           = params.get("roll_body", 0.0) ##
        msg.pitch_body          = params.get("pitch_body", 0.0) ##
        msg.yaw_body            = params.get("yaw_body", 0.0) ##
        #msg.yaw_sp_move_rate    = params.get("yaw_sp_move_rate", 0.01)
        #msg.q_d                 = params.get("q_d", [0.97015, 0.0, 0.0, 0.0])
        #msg.thrust_body         = params.get("thrust_body", [0.0, 0.0, 0.0])
        msg.reset_integral      = params.get("reset_integral", True)
        msg.fw_control_yaw_wheel = params.get("fw_control_yaw_wheel", False)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_attitude_setpoint_publisher.publish(msg)
        self.get_logger().info('publish_vehicle_attitude_setpoint_command published:')
        print(msg)

    def publish_vehicle_control_mode_command(self, **params) -> None:
        msg = VehicleControlMode()
        msg.flag_armed                          = params.get("flag_armed", True)

        msg.flag_multicopter_position_control_enabled= params.get("flag_multicopter_position_control_enabled", True)
        
        msg.flag_control_manual_enabled         = params.get("flag_control_manual_enabled", False) # true if manual input is mixed in
        msg.flag_control_auto_enabled			= params.get("flag_control_auto_enabled", False) # true if onboard autopilot should act
        msg.flag_control_offboard_enabled		= params.get("flag_control_offboard_enabled", True) # true if offboard control should be used
        msg.flag_control_position_enabled		= params.get("flag_control_position_enabled", False) # true if position is controlled
        msg.flag_control_velocity_enabled		= params.get("flag_control_velocity_enabled", False) # true if horizontal velocity (implies direction) is controlled
        msg.flag_control_altitude_enabled		= params.get("flag_control_altitude_enabled", False) # true if altitude is controlled
        msg.flag_control_climb_rate_enabled		= params.get("flag_control_climb_rate_enabled", False) # true if climb rate is controlled
        msg.flag_control_acceleration_enabled   = params.get("flag_control_acceleration_enabled", False) # true if acceleration is controlled
        msg.flag_control_attitude_enabled		= params.get("flag_control_attitude_enabled", True) # true if attitude stabilization is mixed in
        msg.flag_control_rates_enabled          = params.get("flag_control_rates_enabled", True) # true if rates are stabilized
        #msg.flag_control_allocation_enabled		= params.get("flag_control_allocation_enabled", True) # true if control allocation is enabled
        msg.flag_control_termination_enabled    = params.get("flag_control_termination_enabled", False) # true if flight termination is enabled

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_control_mode_publisher.publish(msg)
        self.get_logger().info('publish_vehicle_control_mode_command published: ')
        print(msg)

    def publish_trajectory_setpoint_command(self, **params) -> None:
        msg = TrajectorySetpoint()
        # Trajectory setpoint in NED frame
        # Input to PID position controller.
        # Needs to be kinematically consistent and feasible for smooth flight.
        # setting a value to NaN means the state should not be controlled

        # NED local world frame
        msg.position    = params.get("position", [nan, nan, nan]) # in meters
        msg.velocity    = params.get("velocity", [0.0, 0.0, 0.62585]) # in meters/second
        msg.acceleration= params.get("acceleration", [0.0, 0.0, 0.0]) # in meters/second^2
        msg.jerk        = params.get("jerk", [nan, nan, 0.0]) # in meters/second^3 (for logging only)
        msg.yaw         = params.get("yaw", nan) # euler angle of desired attitude in radians -PI..+PI
        msg.yawspeed    = params.get("yawspeed", 0.0) # angular velocity around NED frame z-axis in radians/second

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info('vehicle_trajectory_setpoint_publisher published: ')
        print(msg)

    # def publish_position_setpoint(self, x: float, y: float, z: float):
    #     msg = TrajectorySetpoint()
    #     msg.position = [x, y, z]
    #     msg.yaw = 1.57079  # (90 degree)
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

#### quat function ####
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

#### Callback functions ####
    def timer_callback_10Hz(self) -> None:
        print('In timer callback function: ', self.offboard_setpoint_counter)

        self.publish_offboard_control_heartbeat_signal() # Has to be first because it sets up offboard control

        if self.offboard_setpoint_counter == 1: # Perform the actuator test
            self.pub_act_test()

        if self.offboard_setpoint_counter < 20: 
            self.engage_offboard_mode()

        if self.offboard_setpoint_counter == 20: # Send an offboard mode
            self.arm()
        
        if self.offboard_setpoint_counter == 21:
            self.pub_veh_ctl_mode()
            self.pub_act_test_off()
            self.pub_traj_set()
        
        self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    # print('In main()')

    # # Output File Declarations
    # date = datetime.datetime.now()                  # Create date string for output files
    # timestr = time.strftime("%m%d%-y-%H%M")         # Create time string for output files - to keep things organized

    # # Global Declarations
    # pi = 3.14159265358979323846
    # camera_angle = 0

    # # Identifies current file path and constructs a file path string for the output file
    # filePath = os.getcwd()                          # It should be the current folder location
    # print("The workspace file path is: ", filePath) # "/home/anyell/ahabp_v2_ws"
    # imagesPath = os.path.join(filePath, 'images')
    # print("Images file path is: ", imagesPath)

    # # File to save log information as a csv file. Should not overwrite as long as it doesn't take longer than a minute.
    # test_dir_name = 'test_data'                     # "test_data" is the folder the test data will be at.
    # dataPath = os.path.join(filePath, test_dir_name)# Makes path
    # if not os.path.exists(dataPath):                # Checks if the directory already exists
    #     os.makedirs(dataPath)                       # Makes the directory if it does not exist
    # print("Data file path is: ", dataPath)          # Outputs to terminal the path name; "$ pwd"
    # data_file = dataPath + '/' + timestr + '_data.csv' # Makes 
    # file = os.open(data_file, os.O_CREAT, 0o775)         # Creates the file and folder if it doesn't exist already.

    # print("past data file.")
    # # file.write(f"Data log for {date}\n")
    # # file.write(f"Time,Latitude,Longitude,Altitude,Zenith,Azimuth,Heading,Camera,Yaw,Pitch\n")
    # # file.close()

    # # capture = cv.VideoCapture(0)
    # # cx = 320
    # # cy = 240

    # # picture = 1
    # # i = 0

    print('Before init()')
    rclpy.init(args=args) # Starts
    print('Passed inits')
    tracking = Tracking() # Create a publisher for the Actuator_Test message. Stays the majority in this function.
    print('Before spin()')
    rclpy.spin(tracking) # Keep the node alive until Ctrl+C is pressed
    print('Before destroy_node()')
    tracking.destroy_node() # Kills all the nodes
    rclpy.shutdown() # End

if __name__ == '__main__':
    try:
        print('Before main is called...')
        main()
    except Exception as e:
        print('Print exception...')
        print(e)
