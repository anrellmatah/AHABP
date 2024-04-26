#!/bin/python3

################################################################
# Title: AHABP Node Tracking V10
# Date: 4/26/2024
# Author: Anyell Mata
# Description: This code will incorporate Scott's OpenCV code and manual Proportional controller commands.
################################################################

'''
*** BEFORE RUNNING SCRIPT ***
$ colcon build --event-handlers console_direct+ --executor sequential --continue-on-error

$ sudo chmod 777 /dev/video0
$ ros2 run v4l2_camera v4l2_camera_node -ros-args -p image_size:="[640,480]"

$ MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 4
'''
# Need to cancel v4l2 camera command - the script just needs it started

'''
*** AFTER RUNNING SCRIPT ***
$ ros2_graph /...
'''

# Import necessary libraries
import os
import subprocess
import sys
import stat
import math
import logging
import time
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from px4_msgs.msg import ( # These are compatible .msg files. Check 'px4_msgs/msg'
                        VehicleControlMode, ManualControlSetpoint, OffboardControlMode, GotoSetpoint, TrajectorySetpoint, VehicleAttitudeSetpoint, VehicleRatesSetpoint, VehicleCommand, VehicleThrustSetpoint, # Publishers: RPi4 --> PixRacer
                        ManualControlSetpoint, SensorCombined, VehicleAttitude, VehicleControlMode, VehicleGlobalPosition, VehicleStatus,   # Subscribers: PixRacer --> RPi4                        
                        )
from numpy import nan
import cv2 as cv
import numpy as np

print('##### Hi from ahabp_node_tracking_8.py #####')

logger = logging.getLogger(__name__)            # Start logger for the debugging prints

# This should close the video captures
cap = cv.VideoCapture(0)                        # To capture a video, you need to create a VideoCapture object
if cap.isOpened():                              # Checks if camera is open and prints statements accordingly
    logger.debug("Opening camera...")
else:
    logger.error("CANNOT OPEN CAMERA")
ret, frame = cap.read()                         # Capture frame-by-frame

def timeString():
    timestr = time.strftime("%m_%d_%-y-%H_%M_%S")# Create time string for output files - to keep things organized
    return timestr      # Should be a String variable

class Tracking(Node):
    def __init__(self, imagesPath, dataPath):
        super().__init__('tracking_node') # This is the name of the node. It will appear as a oval in rqt's node graph.

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10)  # Adjust the queue size as needed

        qos_profile_sub = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1)

        qos_profile_pub = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1)

        # Create publisher
        self.vehicle_control_mode_publisher = self.create_publisher( # create publisher for offboard mode
                VehicleControlMode, # px4_msg uORB message. Check 'pX4_msgs/msg'
                '/fmu/in/config_control_setpoints', # topic type. Check 'dds_topics.yaml'
                qos_profile) # Quality of State profile
        self.manual_control_setpoint_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile) 
        self.goto_setpoint_publisher = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile_pub)
        self.vehicle_rates_setpoint_publisher = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_thrust_setpoint_publisher = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)

        # Create subscribers
        self.manual_control_setpoint_subscriber = self.create_subscription(
            ManualControlSetpoint, # px4_msg uORB message. Check 'pX4_msgs/msg'
            '/fmu/out/manual_control_setpoint', # topic type. Check 'dds_topics.yaml'
            self.manual_control_input_callback, # Callback function
            qos_profile_sub) # Quality of State profile
        self.manual_control_setpoint = ManualControlSetpoint() # Needed for timer callback function
        self.sensor_combined_subscriber = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, qos_profile_sub)
        self.sensor_combined = SensorCombined()
        self.vehicle_attitude_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile_sub) 
        self.vehicle_attitude = VehicleAttitude() 
        self.vehicle_control_mode_subscriber = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile_sub)
        self.vehicle_control_mode = VehicleControlMode()     
        self.vehicle_global_position_subscriber = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile_sub)
        self.vehicle_global_position = VehicleGlobalPosition()           
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_sub)
        self.vehicle_status = VehicleStatus()

        # Initialize internal class variables
        self.imagesPath = imagesPath
        self.dataPath = dataPath
        self.counter = 0
        self.picture = 1
        self.local_timestamp = 0

        # Create timer callbacks
        self.timer = self.create_timer(.1, self.timer_callback_10Hz) #10Hz

#### Subscriber functions ####
    def manual_control_input_callback(self, msg):
        logger.debug(f'Received manual_control_input_callback: {msg}')
        #msg_parsed = (f'taliass ')
        self.manual_control_input = msg
    def sensor_combined_callback(self, msg):
        logger.debug(f'Received sensor_combined_callback: {msg}')
        msg_parsed = (f'timestamp: {msg.timestamp} gyro_rad: {msg.gyro_rad} accelerometer_m_s2: {msg.accelerometer_m_s2} ')
        self.sensor_combined = msg_parsed
    # Mavros equivalent IMU function
    def vehicle_attitude_callback(self, msg): # High Rate
        self.des_yaw = math.radians(0.0)  # [radians] Desired yaw angle in radians. 0.0 should be North -or- the starting orientation determined when the vehicle is armed.
        self.K_p = 0.8 # Proportional gain

        logger.debug(f' Received vehicle_attitude_callback: {msg}')
        msg_parsed = (f'q: {msg.q} delta_q_reset: {msg.delta_q_reset} ')
        logger.debug(f' msg_parsed: {msg_parsed}')

        # Extract yaw angle from IMU data (assuming quaternion orientation)
        orientation_q = msg.q # The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)
        logger.debug(f' orientation_q: {orientation_q}') # Quaternion rotation from the FRD body frame to the NED earth frame
        curr_roll, curr_pitch, curr_yaw = self.quaternion_to_euler( orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]) # Input is q(w, x, y, z)
        logger.debug(f' roll: {curr_roll} pitch: {curr_pitch} yaw: {curr_yaw} ## [radians]')

        # 1) Calculate yaw error
        logger.debug(f' self.des_yaw: {self.des_yaw}')
        yaw_error = self.des_yaw - curr_yaw # [radians]
        logger.debug(f' yaw_error: {yaw_error}') # [radians]

        # Calculate yaw speed move rate command using proportional control
        logger.debug(f' self.K_p: {self.K_p}')
        self.yaw_sp_move_rate = self.K_p * yaw_error # [radians]
        logger.debug(f' self.yaw_sp_move_rate: {self.yaw_sp_move_rate}')

        # I'm unsure what this does
        # diff_yaw_thrust = yaw_error/math.pi # math.fabs(yaw_error/math.pi)
        # logger.info(f'diff_yaw_thrust: {diff_yaw_thrust}')

        # Taken from Dr. Das code - https://github.com/darknight-007/dreams-pod-high-altitude/blob/main/test-offboard-control-pitch-yaw-tracking.py#L75
        dir_yaw = abs(curr_yaw) - 0.1 # [degrees]
        self.des_quaternion = quaternion_from_euler(0, 0, dir_yaw) # Outputs as q(x, y, z, w) [normalized] [-1.0,1.0]
        logger.debug(f' self.des_quaternion: {self.des_quaternion}')

    # def vehicle_attitude_callback(self, msg): # WORKS - High Rate
    #     logger.debug(f'Received vehicle_attitude_callback: {msg}')
    #     msg_parsed = (f'q: {msg.q} delta_q_reset: {msg.delta_q_reset} ')
    #     self.vehicle_attitude = msg_parsed

    def vehicle_control_mode_callback(self, msg): # NO WORK
        logger.debug(f'Received vehicle_control_mode_callback: {msg}')
        msg_parsed = (f'timestamp: {msg.timestamp} flag_armed: {msg.flag_armed} flag_control_offboard_enabled: {msg.flag_control_offboard_enabled} flag_control_attitude_enabled: {msg.flag_control_attitude_enabled} flag_control_position_enabled: {msg.flag_control_position_enabled} flag_control_altitude_enabled: {msg.flag_control_altitude_enabled} flag_control_termination_enabled: {msg.flag_control_termination_enabled} ')
        self.vehicle_control_mode = msg_parsed
    def vehicle_global_position_callback(self, msg): # WORKS - High Rate
        logger.debug(f'Received vehicle_global_position_callback: {msg}') 
        msg_parsed = (f'timestamp: {msg.timestamp} lat: {msg.lat} lon: {msg.lon} alt: {msg.alt} eph: {msg.eph} epv: {msg.epv} ')
        self.vehicle_global_position = msg_parsed
    def vehicle_status_callback(self, msg):
        logger.debug(f'Received vehicle_status_callback: {msg}') # NO WORKS
        msg_parsed = (f'timestamp: {msg.timestamp} armed_time: {msg.armed_time}') # takeoff_time: {msg.takeoff_time} arming_state: {msg.arming_state} failsafe: {msg.failsafe} system_type: {msg.system_type} system_id: {msg.system_id} component_id: {msg.component_id} safety_off: {msg.safety_off} power_input_valid: {msg.power_input_valid} pre_flight_checks_pass: {msg.pre_flight_checks_pass} ')
        self.vehicle_status = msg_parsed

#### Individual command functions ####
    def gimbal_manager_pitchyaw(self):
        logger.debug("Sending gimbal_neutral...")
        msg = VehicleCommand()
        msg.command = 1000       # 1000 - https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        msg.param1 = 0.0     # [-180, 180]Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).
        msg.param2 = 22.0       # [-180, 180] Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).
        msg.param3 = 5.0        # Pitch rate (positive to pitch up). [deg/s]
        msg.param4 = 5.0       # Yaw rate (positive to yaw to the right). [deg/s]
        msg.param5 = 2.0        # Gimbal manager flags - [1,2,4,8,16,32,64,128,256,512]
        msg.param7 = 0.0        # Gimbal device ID 
        msg.target_system = 1           # System which should execute the command
        msg.target_component = 0        # Component which should execute the command, 0 for all components
        msg.source_system = 0           # System sending the command
        msg.source_component = 0        # Component / mode executor sending the command
        msg.confirmation = 0            # 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        logger.debug('publish_vehicle_command published:')
        logger.debug(msg)

    def gimbal_manager_configure(self):
        logger.debug("Sending gimbal_manager_configure...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, # 1001 - https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
            param1 = 0.0, # Sysid primary control
            param2 = 0.0, # Compid primary control
            param3 = 0.0, # Ssid secondary control
            param4 = 0.0, # Compid secondary control
            param7 = 0.0) # Gimbal device ID
        
    def mount_pitch_stabilize(self):
        logger.debug("Sending mount_pitch_stabilize...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONFIGURE, # 204
            param1=2.0, # [0 <-> 6]Mount operation mode (see MAV_MOUNT_MODE enum) # https://mavlink.io/en/messages/common.html#MAV_MOUNT_MODE
            param2=0.0, # Stabilize roll? (1 = yes, 0 = no)
            param3=1.0, # Stabilize pitch? (1 = yes, 0 = no)
            param4=1.0) # Stabilize yaw? (1 = yes, 0 = no)

    def switch_offboard_mode(self):
        logger.debug("Sending switch_offboard_mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, # 176
            param1 = 81.0, # param1=194.0, # 216, 194, 220 - https://mavlink.io/en/messages/common.html#MAV_MODE
            param2 = 6.0)

    def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        logger.debug("Sending pub_act_test#1...")
        self.publish_vehicle_command( # Test motor #1
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, # 310 - https://raw.githubusercontent.com/PX4/px4_msgs/main/msg/VehicleCommand.msg
            param1 = 0.0, # Output value [min:-1, max:1]
            param2 = 1.0, # Timeout [seconds]
            param5 = 1.0) # https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_FUNCTION
        
        logger.debug("Sending pub_act_test#3...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = 0.0, param2 = 1.0, param5 = 3.0,)
        logger.debug("Sending pub_act_test#6...") # Test Gimbal Pitch
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = -1.0, param2 = 1.0, param5 = 1421.0,)

    def arm(self):
        logger.debug('Sending arm...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1.0)
    
    def disarm(self):
        logger.debug('Sending disarm...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 0.0)

    def publish_vehicle_command(self, command, **params) -> None: # This defaults to nothing except for determining the targets which is redundant.
        msg = VehicleCommand() # The most consistent, robust message - https://github.com/PX4/px4_msgs/blob/main/msg/VehicleCommand.msg
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
        logger.debug('publish_vehicle_command published:')
        logger.debug(msg)

    def publish_offboard_control_mode(self): # This publish function enables the various subjects for offboard mode
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
        logger.debug('publish_offboard_control_mode published:')
        logger.debug(msg)

    def motor_tracking_test(self, yaw, pitch, targeted, thresh):
        if (self.counter % 10) == 0:                    # Every 1 second, save threshold and targeted images
            print('## Modulo output')
            cv.imwrite(os.path.join(self.imagesPath, "thresh_" + str(self.picture) + "_" + timeString() + ".jpg"), thresh)
            cv.imwrite(os.path.join(self.imagesPath, "targeted_" + str(self.picture) + "_" + timeString() + ".jpg"), targeted)
            self.picture += 1
            logger.debug(f"Saved picture {self.picture}")
        
        if (pitch/10) > 0:
            print("## pitch DOWN ")
        elif (pitch/10) < 0:
            print("## pitch UP ")

        # Spin corresponding motor
        if yaw > 0:
            print("## yaw RIGHT ")
            print(yaw)
            logger.debug("## Actuating motor #1...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = 0.0, param2 = 0.5, param5 = 1.0,) # Use Motor 1
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = -1.0, param2 = 0.5, param5 = 3.0,) # Disarm Motor 3
        
        elif yaw <= 0:
            print("## yaw LEFT ")
            print(yaw)
            logger.debug("## Actuating motor #3...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = 0.0, param2 = 0.5, param5 = 3.0,) # Use Motor 3
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = -1.0, param2 = 0.5, param5 = 1.0,) # Disarm Motor 1

    def man_con_set(self):
        msg = ManualControlSetpoint()
        msg.valid = True
        msg.data_source = 0     # uint8 SOURCE_UNKNOWN   = 0
        msg.roll = 0.0          # move right,   positive roll rotation,  right side down
        msg.pitch = 0.0         # move forward, negative pitch rotation, nose down
        msg.yaw = 0.0           # positive yaw rotation,   clockwise when seen top down
        msg.throttle = 0.3      # move up,      positive thrust,         -1 is minimum available 0% or -100% +1 is 100% thrust
        self.manual_control_setpoint_publisher.publish(msg)
        logger.info('manual_control_setpoint_publisher published:')
        logger.debug(msg)

    def veh_att_set(self):
        # Publish attitude target with only yaw rate command
        msg = VehicleAttitudeSetpoint()
        msg.roll_body             = 0.0 # math.nan # body angle in NED frame (can be NaN for FW)
        msg.pitch_body            = 0.0 # math.nan # body angle in NED frame (can be NaN for FW)
        msg.yaw_body              = (self.des_quaternion[2])*(math.pi) # Range is [-PI, PI]
        msg.yaw_sp_move_rate      = self.yaw_sp_move_rate  # Set the yaw rate command. [rad/s]. A typical range is [-2.0, 2.0].
        msg.q_d                   = [self.des_quaternion[3], self.des_quaternion[0], self.des_quaternion[1], self.des_quaternion[2]] # q(w, x, y, z) # Desired quaternion for quaternion control. Range is [-1.0, 1.0]
        msg.thrust_body           = [0.0, 0.0, -0.2] # Normalized thrust command in body NED frame [-1,1]. Make sure third parameter is negative. It is receiving
        msg.reset_integral        = False
        msg.fw_control_yaw_wheel  = False
        self.vehicle_attitude_setpoint_publisher.publish(msg)
        logger.info('publish_vehicle_attitude_setpoint_command published:')
        logger.debug(msg)

    def traj_set(self):
        msg = TrajectorySetpoint()
        # Trajectory setpoint in NED frame
        # Input to PID position controller.
        # Needs to be kinematically consistent and feasible for smooth flight.
        # setting a value to NaN means the state should not be controlled
        msg.position = [math.nan, math.nan, math.nan] # [meters]
        msg.velocity = [math.nan, math.nan, math.nan] # in meters/second
        msg.acceleration = [math.nan, math.nan, math.nan] # in meters/second^2
        msg.jerk = [math.nan, math.nan, 0.00000] # in meters/second^3 (for logging only)
        msg.yaw = 0.00 # euler angle of desired attitude in radians -PI..+PI
        msg.yawspeed = 0.1 # angular velocity around NED frame z-axis in radians/second
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        logger.debug('trajectory_setpoint_publisher published: ')
        logger.debug(msg)

    def veh_con_mode(self):
        msg = VehicleControlMode()
        msg.flag_armed = True # synonym for actuator_armed.armed
        msg.flag_multicopter_position_control_enabled = True 
        msg.flag_control_manual_enabled = True # true if manual input is mixed in
        msg.flag_control_offboard_enabled = True 
        msg.flag_control_offboard_enabled = True		# true if offboard control should be used
        msg.flag_control_position_enabled = True		# true if position is controlled
        msg.flag_control_velocity_enabled = True		# true if horizontal velocity (implies direction) is controlled
        msg.flag_control_altitude_enabled = True		# true if altitude is controlled
        msg.flag_control_attitude_enabled		# true if attitude stabilization is mixed in
        msg.flag_control_rates_enabled			# true if rates are stabilized
        msg.source_id = 0                       # Mode ID (nav_state)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_control_mode_publisher.publish(msg)
        logger.info('vehicle_control_mode_publisher published:')
        logger.debug(msg)

    def go_to_set(self):
        '''
        # Position and (optional) heading setpoints with corresponding speed constraints
        # Setpoints are intended as inputs to position and heading smoothers, respectively
        # Setpoints do not need to be kinematically consistent
        # Optional heading setpoints may be specified as controlled by the respective flag
        # Unset optional setpoints are not controlled
        # Unset optional constraints default to vehicle specifications
        '''
        msg = GotoSetpoint()
        msg.position = [0.0,0.0,0.0]                # [m] NED local world frame
        msg.flag_control_heading = True             # true if heading is to be controlled
        msg.heading = 0.0                           # (optional) [rad] [-pi,pi] from North
        msg.flag_set_max_horizontal_speed = False   # true if setting a non-default horizontal speed limit
        msg.max_horizontal_speed = 0.0              # (optional) [m/s] maximum speed (absolute) in the NE-plane
        msg.flag_set_max_vertical_speed = False     # true if setting a non-default vertical speed limit
        msg.max_vertical_speed = 0.0                # (optional) [m/s] maximum speed (absolute) in the D-axis
        msg.flag_set_max_heading_rate = False       # true if setting a non-default heading rate limit
        msg.max_heading_rate = 0.0                  # (optional) [rad/s] maximum heading rate (absolute)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.goto_setpoint_publisher.publish(msg)
        logger.info('goto_setpoint_publisher published:')
        logger.debug(msg)

    def veh_rate_set(self):
        msg = VehicleRatesSetpoint()
        # body angular rates in NED frame
        msg.roll = 0.0                      # [rad/s] roll rate setpoint
        msg.pitch = 0.0                     # [rad/s] pitch rate setpoint
        msg.yaw = 0.5                       # [rad/s] yaw rate setpoint
        # For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
        # For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
        msg.thrust_body = [0.0,0.0,-0.3]    # Normalized thrust command in body NED frame [-1,1]
        msg.reset_integral = True           # Reset roll/pitch/yaw integrals (navigation logic change)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_rates_setpoint_publisher.publish(msg)
        logger.info('vehicle_rates_setpoint_publisher published:')
        logger.debug(msg)

    def veh_thrust_set(self):
        msg = VehicleThrustSetpoint()
        msg.xyz = [0.0, 0.0, 0.3]          # thrust setpoint along X, Y, Z body axis [-1, 1]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_thrust_setpoint_publisher.publish(msg)
        logger.info('vehicle_thrust_setpoint_publisher published:')
        logger.debug(msg)
    
    def veh_cmd_do_set_roi(self):
        '''
        uint16 VEHICLE_CMD_DO_SET_ROI = 201			# Sets the region of interest (ROI) for a sensor set or the vehicle itself. 
        This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. 
        |Region of interest mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty|
        x the location of the fixed ROI (see MAV_FRAME)| y| z|
        '''
        logger.debug("Sending veh_cmd_do_set_roi...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ROI, # 201
            param1=0.0, # Region of interest mode. (see MAV_ROI enum) [0.0, 4.0]
            param2=0.0, # MISSION index/ target ID. (see MAV_ROI enum)
            param3=0.0, # ROI index (allows a vehicle to manage multiple ROI's)
            param4=0.0, # Empty
            param5=0.0, # x the location of the fixed ROI (see MAV_FRAME)
            param6=0.0, # y
            param7=0.0) # z
        
    # def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
    #     logger.debug("Sending pub_act_test#1...")
    #     self.publish_vehicle_command( # Test motor #1
    #         VehicleCommand.GIMBAL_MANAGER_SET_PITCHYAW, # 287
    #         param1 = 0.0, # Output value [min:-1, max:1]
    #         param2 = 1.0, # Timeout [seconds]
    #         param5 = 1.0) # https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_FUNCTION


#### quaternion converter function ####
    def quaternion_to_euler(self, w, x, y, z):
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
        return roll, pitch, yaw # In radians

    #### OpenCV functions ####
    def target(self, frame, minimum=250, cx=320, cy=240):
        ''' This function targets the centroid of a frame and outputs
            vector in x and y of the error between center of frame and the centroid
            
            Raspberry Pi Camera v2 the image size is:
            480 rows (vertical)
            640 columns (horizontal)
        '''
        # copy and convert image to grayscale then process
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # Converts to grayscale
        thresholding, thresh = cv.threshold(gray, minimum, 255, cv.THRESH_BINARY) 
        (minVal, maxVal, minLoc, maxLoc) = cv.minMaxLoc(gray)
        M = cv.moments(thresh)

        # calculate centroid
        targx = int(M["m10"] / (M["m00"]+1))
        targy = int(M["m01"] / (M["m00"]+1))

        # calculate direction vector from center to centroid
        yaw_target = targx - cx
        pitch_target = targy - cy

        # # calculate rolling average of last 5
        # yaw_target_smooth = rolling_average(centroids_x, yaw_target)
        # pitch_target_smooth = rolling_average(centroids_y, pitch_target)

        # place vector arrow and label
        cv.putText(frame, "target", (targx - 45, targy + 45), cv.FONT_HERSHEY_SIMPLEX, 1, (25, 25, 255), 2)
        cv.circle(frame, [targx, targy], 25, (25, 25, 255), 2)
        cv.arrowedLine(frame, [cx, cy], [targx, targy], (25, 25, 255), 2) # center --> target

        return yaw_target, pitch_target, frame, thresh

#### Callback functions ####
    def timer_callback_10Hz(self): #-> None:
        print('## timer_callback_10Hz: ', self.counter)     # 10 setpoints for every second
        self.publish_offboard_control_mode()                # Has to be first because it enables parts for offboard control
 
        # Subscriber parsed message prints
        #logger.debug(f'## self.vehicle_attitude: {self.vehicle_attitude}') # WORKS
        #logger.debug(f'## self.vehicle_status: {self.vehicle_status}') # NOT WORKING
        #logger.debug(f'## self.vehicle_global_position: {self.vehicle_global_position}') # WORKS
        #logger.debug(f'## self.vehicle_control_mode: {self.vehicle_control_mode}') # WORKS
        #logger.debug(f'## self.sensor_combined: {self.sensor_combined}') # WORKS
        #logger.debug(f'## self.manual_control_input_callback: {self.manual_control_input_callback}')

        istrue, frame = cap.read()
        original = frame.copy()
        
        # Error calculations
        #yaw_ephem, pitch_ephem, latitude, longitude, altitude = ephem_update() # 'ephem' error is based on calculation with GPS/heading
        yaw, pitch, targeted, thresh = self.target(frame) # 'target' error is based on what's in the camera frame

        if self.counter > 2 or self.counter < 10:
            self.switch_offboard_mode()                     # Switches to offboard mode in the first counter. Only needs to be called once.

        # if self.counter < 10:
        #    self.pub_act_test()                              # Perform the actuator and servo test in the first few seconds

        # if self.counter == 10 or self.counter == 11:
        #     #3
        #     self.gimbal_manager_configure()                 # Needed to setup control permissions over gimbal
        #     self.gimbal_manager_pitchyaw()                           # Stabilizes pitch using the servo but can't turn off the motors

        if self.counter > 10: #and self.counter < 110:        # Testing ground for stabilization will run here
            print('## at counter 10 < x')
            
            #1
            self.veh_att_set()                             #

            #2
            #self.traj_set()                                # Does not seem to exercise feedback
            
            #4
            #self.mount_pitch_stabilize()                    # Regardless what param1 is, the servo will stabilize pitch and motors will spin to about 1150.
        
            #5
            #self.man_con_set()                              # 

            #6
            #self.go_to_set()

            #7
            #self.veh_rate_set()

            #8
            #self.veh_thrust_set()

            #9
            #self.veh_cmd_do_set_roi()
  

        if self.counter == 30 or self.counter == 40:
            #print('## arm placeholder')
            self.arm()                                      # Arm the payload. Can disarm from auto preflight disarming. Only needs to be called once.
        if self.counter == 130 or self.counter == 140:
            self.disarm()                                   # Only needs to be called once. DON'T SPAM the disarm.

        if (self.counter % 100) == 0:                       # Every 10 seconds, save the images
            print('## Modulo output')
            cv.imwrite(os.path.join(self.imagesPath, "raw_" + str(self.picture) + "_" + timeString() + ".jpg"), original)
            cv.imwrite(os.path.join(self.imagesPath, "thresh_" + str(self.picture) + "_" + timeString() + ".jpg"), thresh)
            cv.imwrite(os.path.join(self.imagesPath, "targeted_" + str(self.picture) + "_" + timeString() + ".jpg"), targeted)
            self.picture += 1
            logger.debug(f"Saved picture {self.picture}")

        self.counter += 1

#### Main function ####
def main(args=None) -> None:
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO) # Set the format and level for logging. Will output to terminal. level=logging.DEBUG 
    logger.debug('In main()')                               # Print this debug statement when level is activated in basicConfig()

    # Global Declarations
    pi = math.pi
    camera_angle = 0
    mag_offset = 0 # magnetometer offset in degrees

    # Identifies current file path and constructs a file path string for the output file
    wsPath = os.getcwd()                            # It should be the current folder location; like "$ pwd"
    print("The workspace path is: ", wsPath)        # It should be "/home/anyell/ahabp_v2_ws"

    images_path = 'images/' + timeString()
    imagesPath = os.path.join(wsPath, images_path)  # Makes images path
    if not os.path.exists(imagesPath):              # Checks if the directory already exists
        os.makedirs(imagesPath)                     # Makes the directory if it does not exist
    print("Images directory path is: ", imagesPath) # It should be "/home/anyell/ahabp_v2_ws/images"

    # File to save log information as a csv file. Should not overwrite.
    dataPath = os.path.join(wsPath, 'test_data')    # Makes path name for test data directory
    if not os.path.exists(dataPath):                # Checks if the directory already exists
        os.makedirs(dataPath)                       # Makes the directory if it does not exist
    print("Data directory path is: ", dataPath)     # Outputs to terminal the path name; "$ pwd"
    
    data_file = dataPath + '/' + timeString() + '.csv' # Creates path for data file based on time of start
    dataFile = open(data_file,'w+')                 # Write command that creates the .csv file. WILL OVERWRITE previous csv file if it exists.
    os.chmod(data_file, 0o777)                      # Change permissions to be open to everyone
    print("Data file path is: ", data_file)         # Outputs to terminal the path name
    
    dataFile.write("Time,Latitude,Longitude,Altitude,Zenith,Azimuth,Heading,Camera,Yaw,Pitch")
    dataFile.close()                                # Close file after writing or else corrupted file

## Getting into PX4 tracking function
    logger.debug('Before init()...')
    rclpy.init(args=args) # Starts
    logger.debug('Before tracking declare...')
    tracking = Tracking(imagesPath, dataPath) # Create a publisher for the Actuator_Test message. Stays the majority in this function.
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
