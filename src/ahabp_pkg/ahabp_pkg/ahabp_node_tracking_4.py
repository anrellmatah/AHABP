#!/usr/bin/env python3

################################################################
# Title: AHABP Node Tracking V4
# Date: 4/21/2024
# Author: Anyell Mata
# Description: This code will incorporate Scott's OpenCV code and manual Proportional controller commands.
################################################################

'''
# *** BEFORE RUNNING SCRIPT ***
$ colcon build --event-handlers console_direct+ --executor sequential --continue-on-error
$ MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 4
$ ros2 run v4l2_camera v4l2_camera_node -ros-args -p image_size:="[640,480]"
'''

'''
$ /bin/python3 /home/anyell/ahabp_v2_ws/src/ahabp_pkg/ahabp_pkg/ahabp_node_tracking_4.py

# *** AFTER RUNNING SCRIPT ***
$ ros2_graph /...
'''

# Import necessary libraries
import os
import subprocess
import sys
import stat
import datetime
import math
import logging
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleAttitudeSetpoint, VehicleControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus # These are compatible .msg files. Check 'px4_msgs/msg'
from numpy import nan
import cv2 as cv
import numpy as np

print('##### Hi from ahabp_node_tracking_4.py #####')

logger = logging.getLogger(__name__)            # Start logger for the debugging prints

class Tracking(Node): # Node. --> self.
    def __init__(self):
        super().__init__('tracking_node') # This is the name of the node. It will appear as a oval in rqt's node graph.

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
            qos_profile # Infer QoS profile from earlier
        )
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.vehicle_control_mode_publisher = self.create_publisher(VehicleControlMode, '/fmu/in/config_control_setpoints', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Create subscribers
        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, # px4_msg uORB message. Check 'pX4_msgs/msg'
        #     '/fmu/out/vehicle_local_position', # topic type. Check 'dds_topics.yaml'
        #     self.vehicle_local_position_callback, # Callback function
        #     qos_profile # QoS
        # )
        # self.vehicle_local_position_subscriber  # prevent unused variable warning

#        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        #   self.vehicle_status_subscriber # prevent unused variable warning


        # Initialize internal class variables
        self.counter = 0
        self.local_timestamp = 0 
        self.des_yaw = math.radians(0.0)  # Desired yaw angle in radians. 0.0 should be North
        self.K_p = 0.6  # Proportional gain

        # Create timer callbacks
        self.timer = self.create_timer(.1, self.timer_callback_10Hz) #10Hz
        #self.timer = self.create_timer(.01, self.timer_callback_100Hz) #100Hz

#### Subscriber functions ####
    # def vehicle_local_position_callback(self, msg):
    #     logger.info('Received VehicleLocalPosition:\n'
    #                 f'x: {msg.x}\n'
    #                 f'y: {msg.y}\n'
    #                 f'z: {msg.z}'
    #                 f'heading: {msg.heading}'
    #                 f'ref_alt: {msg.ref_alt}'
    #                 f'heading_good_for_control: {msg.heading_good_for_control}')
    
    # def vehicle_status_callback(self, msg):
    #     logger.info('Received VehicleStatus:\n'
    #                 f'timestamp: {msg.timestamp}\n'
    #                 f'takeoff_time: {msg.takeoff_time}\n'
    #                 f'arming_state: {msg.arming_state}\n'
    #                 f'failsafe: {msg.failsafe}\n'
    #                 f'system_type: {msg.system_type}\n'
    #                 f'system_id: {msg.system_id}\n'
    #                 f'component_id: {msg.component_id}')

    def vehicle_status_callback(self, msg):
        #print(f'Received VehicleStatus: \n{msg}\n')
        logger.info("Received VehicleStatus: ")
        logger.info(msg)

#### Individual command functions ####

    def gimbal_neutral(self):
        logger.info("Sending gimbal_neutral...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW , # 100 - https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
            param1 = 180.0, # [-180, 180]Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).
            param2 = 0.0, # [-180, 180] Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).
            #param3 = 1.0, # Pitch rate (positive to pitch up). [deg/s]
            #param4 = 1.0, # Yaw rate (positive to yaw to the right). [deg/s]
            param5 = 2.0, # Gimbal manager flags
            param7 = 0.0 # Gimbal device ID
            )

    def gimbal_manager_configure(self):
        logger.info("Sending gimbal_manager_configure...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, # 1001 - https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
            param1 = -2.0, # Sysid primary control
            param2 = -2.0, # Compid primary control
            param3 = 0.0, # Ssid secondary control
            param4 = 0.0, # Compid secondary control
            param7 = 0.0 # Gimbal device ID
            )
        
    def mount_pitch_stabilize(self):
        logger.info("Sending mount_pitch_stabilize")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONFIGURE, # 204
            param1=2.0, # Mount operation mode (see MAV_MOUNT_MODE enum)
            param2=0.0, # Stabilize roll? (1 = yes, 0 = no)
            param3=1.0, # Stabilize pitch? (1 = yes, 0 = no)
            param4=0.0 # Stabilize yaw? (1 = yes, 0 = no)
            )

    def switch_offboard_mode(self):
        logger.debug("Sending switch_offboard_mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, # 176
            param1 = 81.0, # param1=194.0, # 216, 194, 220 - https://mavlink.io/en/messages/common.html#MAV_MODE
            param2 = 6.0
            )

    def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        logger.debug("Sending pub_act_test#1...")
        self.publish_vehicle_command( # Test motor #1
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, # 310 - https://raw.githubusercontent.com/PX4/px4_msgs/main/msg/VehicleCommand.msg
            param1 = 0.0, # Output value [min:-1, max:1]
            param2 = 1.0, # Timeout [seconds]
            param5 = 1.0, # https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_FUNCTION
            )
        logger.debug("Sending pub_act_test#3...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = 0.0, param2 = 1.0, param5 = 3.0,)
        logger.debug("Sending pub_act_test#6...") # Test Gimbal Pitch
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = 1.0, param2 = 1.0, param5 = 1421.0,)
        
    def pub_veh_att_set(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        logger.debug("Sending pub_veh_att_set...")
        self.publish_vehicle_attitude_setpoint_command(
            yaw_body            = 0.0,
            #yaw_sp_move_rate    = 0.1,
            #q_d                 = [0.97015, 0.0, 0.0, 0.0],
            #thrust_body         = [0.1, 0.1, 0.1],
            )

    def arm(self):
        logger.info('Sending arm...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1.0)
    
    def disarm(self):
        logger.info('Sending disarm...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 0.0)

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

    def publish_offboard_control_mode(self):
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
        logger.info('publish_offboard_control_mode published:')
        logger.debug(msg)

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
        msg.target_system = 1           # System which should execute the command
        msg.target_component = 0        # Component which should execute the command, 0 for all components
        msg.source_system = 0           # System sending the command
        msg.source_component = 0        # Component / mode executor sending the command
        msg.confirmation = 0            # 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        logger.info('publish_vehicle_command published:')
        logger.debug(msg)

    def publish_vehicle_attitude_setpoint_command(self, **params) -> None:
        msg = VehicleAttitudeSetpoint()
        msg.roll_body                           = params.get("roll_body", 0.0) ##
        msg.pitch_body                          = params.get("pitch_body", 0.0) ##
        msg.yaw_body                            = params.get("yaw_body", 0.0) ##
        msg.yaw_sp_move_rate                    = params.get("yaw_sp_move_rate", 0.01)
        msg.q_d                                 = params.get("q_d", [0.0, 0.0, 0.0, 0.0])
        msg.thrust_body                         = params.get("thrust_body", [0.0, 0.0, 0.0])
        msg.reset_integral                      = params.get("reset_integral", True)
        msg.fw_control_yaw_wheel                = params.get("fw_control_yaw_wheel", False)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_attitude_setpoint_publisher.publish(msg)
        logger.info('publish_vehicle_attitude_setpoint_command published:')
        logger.debug(msg)

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
        self.trajectory_setpoint_publisher.publish(msg)
        logger.info('trajectory_setpoint_publisher published: ')
        logger.debug(msg)

    # def publish_position_setpoint(self, x: float, y: float, z: float):
    #     msg = TrajectorySetpoint()
    #     msg.position = [x, y, z]
    #     msg.yaw = 1.57079  # (90 degree)
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     logger.debug(f"Publishing position setpoints {[x, y, z]}")

#### quaternion converter function ####
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
# These are where the 
    def timer_callback_10Hz(self) -> None:
        print('## timer_callback_10Hz: ', self.counter)     # 10 setpoints for every second
        self.publish_offboard_control_mode()    # Has to be first because it sets up offboard control

        if self.counter == 1: 
            self.switch_offboard_mode()                     # Switches to offboard mode in the first counter. Only needs to be called once.

        if self.counter < 20:
            self.pub_act_test()                             # Perform the actuator and servo test in the first few seconds

#        if self.counter > 30 and self.counter < 100:
        if self.counter == 30:            
            print('## at counter 30')
            self.gimbal_manager_configure()
            self.gimbal_neutral()
            
            #self.mount_pitch_stabilize()                    # This works alone.

        if self.counter == 40:
            self.arm()                                      # Force arm the payload. Will disarm from auto preflight disarming. Only needs to be called once.
            # self.pub_veh_ctl_mode()
            # self.pub_act_test_off()
            # self.pub_traj_set()

        if self.counter == 100 or self.counter == 120:
            self.disarm()                                   # Perform the actuator test in the first two seconds. Only needs to be called once. Don't spam the disarm.
        
        self.counter += 1

#### Main function ####
def main(args=None) -> None:
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO) # Set the format and level for logging. Will output to terminal. level=logging.DEBUG 
    logger.debug('In main()')                       # Print this debug statement when level is activated in basicConfig()

    # Output File Declarations
    date = datetime.datetime.now()                  # Create date string for output files
    timestr = time.strftime("%m%d%-y-%H%M")         # Create time string for output files - to keep things organized
    picture = 1

    # Global Declarations
    pi = math.pi
    camera_angle = 0
    mag_offset = 0 # magnetometer offset in degrees

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

    #dataFile = open(data_file, os.O_CREAT, 0o777)  # Creates the file and folder if it doesn't exist already.

    dataFile = open(data_file,'w+')                 # Write command that creates the T3P file. WILL OVERWRITE previous t3p file if it exists.
    os.chmod(data_file, 0o777)                      # Change permissions to be open to everyone
    dataFile.write("Time,Latitude,Longitude,Altitude,Zenith,Azimuth,Heading,Camera,Yaw,Pitch")
    dataFile.close()                                # Close file after writing or else corrupted file

    cx = 320                                        # Half of X-axis pixels for tracking image
    cy = 240                                        # Half of Y-axis pixels

    cap = cv.VideoCapture(0) # To capture a video, you need to create a VideoCapture object
    #cap.open()
    if cap.isOpened():                              # Checks if camera is open and prints statements accordingly
        logger.info("Opening camera...")
    else:
        logger.error("CANNOT OPEN CAMERA")
    ret, frame = cap.read()                         # Capture frame-by-frame

## Getting into PX4 tracking function
    logger.debug('Before init()...')
    rclpy.init(args=args) # Starts
    logger.debug('Before tracking declare...')
    tracking = Tracking() # Create a publisher for the Actuator_Test message. Stays the majority in this function.
    logger.debug('Before spin()...')

    rclpy.spin(tracking) # Keep the node alive until Ctrl+C is pressed

    logger.debug('Before close()...')
    dataFile.close()
    logger.debug('Before release()...')
    cap.release() # When everything done, release the capture
    cv.destroyAllWindows()
    logger.debug('Before destroy_node()...')
    tracking.destroy_node() # Kills all the nodes
    rclpy.shutdown() # End

if __name__ == '__main__':
    try:
        print('Before main is called...')
        main()
    except Exception as e:
        print('Print exception...')
        print(e)
