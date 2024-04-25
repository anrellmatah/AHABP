#!/bin/python3

################################################################
# Title: AHABP Node Tracking V7
# Date: 4/24/2024
# Author: Anyell Mata
# Description: This code will incorporate Scott's OpenCV code and manual Proportional controller commands.
################################################################

'''
# *** BEFORE RUNNING SCRIPT ***
$ colcon build --event-handlers console_direct+ --executor sequential --continue-on-error
$ MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 4
'''
# No need to run v4l2 camera command - the script does it already
# $ ros2 run v4l2_camera v4l2_camera_node -ros-args -p image_size:="[640,480]"

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
import math
import logging
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from px4_msgs.msg import ( # These are compatible .msg files. Check 'px4_msgs/msg'
                        OffboardControlMode, VehicleCommand, VehicleAttitudeSetpoint,VehicleControlMode, TrajectorySetpoint, VehicleRatesSetpoint, # Publishers: RPi4 --> PixRacer
                        VehicleAttitude, VehicleStatus, VehicleGlobalPosition, SensorCombined # Subscribers: PixRacer --> RPi4                        
                        )
from numpy import nan
import cv2 as cv
import numpy as np

print('##### Hi from ahabp_node_tracking_7.py #####')

logger = logging.getLogger(__name__)            # Start logger for the debugging prints

# This should close the video captures
cap = cv.VideoCapture(0) # To capture a video, you need to create a VideoCapture object
if cap.isOpened():                              # Checks if camera is open and prints statements accordingly
    logger.info("Opening camera...")
else:
    logger.error("CANNOT OPEN CAMERA")
ret, frame = cap.read()                         # Capture frame-by-frame

def timeString():
    timestr = time.strftime("%m_%d_%-y-%H_%M_%S")         # Create time string for output files - to keep things organized
    return timestr      # Should be a String variable

#### OpenCV functions ####
def target(frame, minimum=250, cx=320, cy=240):
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

    if (pitch_target/10) > 0:
        # if angle >= 5:
        #     angle -= 1
        # servo.ChangeDutyCycle(2 + (angle/18))
        print(" pitch DOWN ")
    elif (pitch_target/10) < 0:
        # if angle <= 95:
        #     angle += 1
        # servo.ChangeDutyCycle(2 + (angle/18))
        print(" pitch UP ")

    return yaw_target, pitch_target, frame, thresh

class Tracking(Node): # Node. --> self.
    def __init__(self, imagesPath, dataPath):
        super().__init__('tracking_node') # This is the name of the node. It will appear as a oval in rqt's node graph.

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 5)  # Adjust the queue size as needed

        qos_profile_sub = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1)

        # Create publisher
        self.offboard_control_mode_publisher = self.create_publisher( # create publisher for offboard mode
            OffboardControlMode, # px4_msg uORB message. Check 'pX4_msgs/msg'
            '/fmu/in/offboard_control_mode', # topic type. Check 'dds_topics.yaml'
            qos_profile) # Quality of State profile
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.vehicle_control_mode_publisher = self.create_publisher(VehicleControlMode, '/fmu/in/config_control_setpoints', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile) ###
        self.vehicle_rates_setpoint_publisher = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)

        # Create subscribers
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, # px4_msg uORB message. Check 'pX4_msgs/msg'
            '/fmu/out/vehicle_attitude', # topic type. Check 'dds_topics.yaml'
            self.vehicle_attitude_callback, # Callback function
            qos_profile_sub) # Quality of State profile
        self.vehicle_attitude = VehicleAttitude() # Needed for timer callback function
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_sub)
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position_subscriber = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile_sub)
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_control_mode_subscriber = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile_sub)
        self.vehicle_control_mode = VehicleControlMode()
        self.sensor_combined_subscriber = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, qos_profile_sub)
        self.sensor_combined = SensorCombined()

        # Initialize internal class variables
        self.imagesPath = imagesPath
        self.dataPath = dataPath
        self.counter = 0
        self.picture = 1
        self.local_timestamp = 0 
        self.des_yaw = math.radians(0.0)  # Desired yaw angle in radians. 0.0 should be North -or- the starting orientation determined when the vehicle is armed.
        self.K_p = 0.6 # Proportional gain

        # Create timer callbacks
        self.timer = self.create_timer(.1, self.timer_callback_10Hz) #10Hz
        #self.timer = self.create_timer(.01, self.timer_callback_100Hz) #100Hz

#### Subscriber functions ####
    def vehicle_attitude_callback(self, msg): # WORKS - High Rate
        logger.debug(f'Received vehicle_attitude_callback: {msg}')
        msg_parsed = (f'q: {msg.q} delta_q_reset: {msg.delta_q_reset} ')
        self.vehicle_attitude = msg_parsed
    def vehicle_status_callback(self, msg):
        logger.debug(f'Received vehicle_status_callback: {msg}') # NO WORKS
        msg_parsed = (f'timestamp: {msg.timestamp} armed_time: {msg.armed_time}') # takeoff_time: {msg.takeoff_time} arming_state: {msg.arming_state} failsafe: {msg.failsafe} system_type: {msg.system_type} system_id: {msg.system_id} component_id: {msg.component_id} safety_off: {msg.safety_off} power_input_valid: {msg.power_input_valid} pre_flight_checks_pass: {msg.pre_flight_checks_pass} ')
        self.vehicle_status = msg_parsed
    def vehicle_global_position_callback(self, msg): # WORKS - High Rate
        logger.debug(f'Received vehicle_global_position_callback: {msg}') 
        msg_parsed = (f'timestamp: {msg.timestamp} lat: {msg.lat} lon: {msg.lon} alt: {msg.alt} eph: {msg.eph} epv: {msg.epv} ')
        self.vehicle_global_position = msg_parsed
    def vehicle_control_mode_callback(self, msg): # NO WORKS
        logger.debug(f'Received vehicle_control_mode_callback: {msg}')
        msg_parsed = (f'timestamp: {msg.timestamp} flag_armed: {msg.flag_armed} flag_control_offboard_enabled: {msg.flag_control_offboard_enabled} flag_control_attitude_enabled: {msg.flag_control_attitude_enabled} flag_control_position_enabled: {msg.flag_control_position_enabled} flag_control_altitude_enabled: {msg.flag_control_altitude_enabled} flag_control_termination_enabled: {msg.flag_control_termination_enabled} ')
        self.vehicle_control_mode = msg_parsed
    def sensor_combined_callback(self, msg):
        logger.debug(f'Received sensor_combined_callback: {msg}')
        msg_parsed = (f'timestamp: {msg.timestamp} gyro_rad: {msg.gyro_rad} accelerometer_m_s2: {msg.accelerometer_m_s2} ')
        self.sensor_combined = msg_parsed

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
        logger.info("Sending mount_pitch_stabilize...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONFIGURE, # 204
            param1=2.0, # Mount operation mode (see MAV_MOUNT_MODE enum)
            param2=0.0, # Stabilize roll? (1 = yes, 0 = no)
            param3=1.0, # Stabilize pitch? (1 = yes, 0 = no)
            param4=0.0 # Stabilize yaw? (1 = yes, 0 = no)
            )

    def switch_offboard_mode(self):
        logger.info("Sending switch_offboard_mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, # 176
            param1 = 81.0, # param1=194.0, # 216, 194, 220 - https://mavlink.io/en/messages/common.html#MAV_MODE
            param2 = 6.0
            )

    def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        logger.info("Sending pub_act_test#1...")
        self.publish_vehicle_command( # Test motor #1
            VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, # 310 - https://raw.githubusercontent.com/PX4/px4_msgs/main/msg/VehicleCommand.msg
            param1 = 0.0, # Output value [min:-1, max:1]
            param2 = 1.0, # Timeout [seconds]
            param5 = 1.0, # https://mavlink.io/en/messages/common.html#ACTUATOR_OUTPUT_FUNCTION
            )
        logger.info("Sending pub_act_test#3...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = 0.0, param2 = 1.0, param5 = 3.0,)
        logger.info("Sending pub_act_test#6...") # Test Gimbal Pitch
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, param1 = -1.0, param2 = 1.0, param5 = 1421.0,)

    def arm(self):
        logger.info('Sending arm...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1.0)
    
    def disarm(self):
        logger.info('Sending disarm...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 0.0)

#### Truman's imu stuff
    # def vehicle_attitude_callback(self, msg):
    #     # Extract yaw angle from IMU data (assuming quaternion orientation)
    #     orientation_q = msg.orientation
    #     euler = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    #     yaw = euler[2]  # Yaw angle in radians

    #     # Calculate yaw error
    #     yaw_error = self.des_yaw - yaw
    #     diff_yaw_thrust = math.fabs(yaw_error/math.pi)

    #     # Calculate yaw rate command using proportional control
    #     yaw_rate_cmd = self.K_p * yaw_error

    #     # Publish attitude target with only yaw rate command
    #     attitude_target = AttitudeTarget()
    #     attitude_target.type_mask = 1  # Ignore roll and pitch rates
    #     attitude_target.thrust = self.K_p*diff_yaw_thrust  # Set a thrust
    #     attitude_target.body_rate.z = yaw_rate_cmd  # Set the yaw rate command

    #     self.publisher.publish(attitude_target)

#### The message compilers ####
# These functions build the complete long command that the respective topics listen too. 
# They'll either make the command or revert to a default command. I am not sure if defaulting 
# is necessary but saw it in many examples.
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
        logger.debug('publish_offboard_control_mode published:')
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
        logger.debug('publish_vehicle_command published:')
        logger.debug(msg)

    def publish_vehicle_attitude_setpoint_command(self, **params) -> None:
        msg = VehicleAttitudeSetpoint()

        msg.yaw_body                            = params.get("yaw_body", 0.0) ##
        #msg.yaw_sp_move_rate                    = params.get("yaw_sp_move_rate", 0.01)
        msg.q_d                                 = params.get("q_d", [1.0, 0.0, 0.0, 0.0])
        msg.thrust_body                         = params.get("thrust_body", [0.0, 0.0, 0.0])

        self.vehicle_attitude_setpoint_publisher.publish(msg)
        logger.debug('publish_vehicle_attitude_setpoint_command published:')
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
        logger.debug('trajectory_setpoint_publisher published: ')
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
    def timer_callback_10Hz(self): #-> None:
        print('## timer_callback_10Hz: ', self.counter)     # 10 setpoints for every second
        self.publish_offboard_control_mode()                # Has to be first because it sets up offboard control - like a heartbeat

## Subscriber parsed message prints
        #logger.info(f'## self.vehicle_attitude: {self.vehicle_attitude}') # Prints parsed message
        #logger.info(f'## self.vehicle_status: {self.vehicle_status}') # DOES NOT Prints parsed message
        #logger.info(f'## self.vehicle_global_position: {self.vehicle_global_position}') # Prints parsed message
        #logger.info(f'## self.vehicle_control_mode: {self.vehicle_control_mode}') # Prints parsed message
        #logger.info(f'## self.sensor_combined: {self.sensor_combined}') # Prints parsed message
        
        istrue, frame = cap.read()
        logger.debug('## istrue: ', istrue)
        logger.debug('## frame: ', frame)
        original = frame.copy()
        logger.debug('## original: ', original)
        
        # error calculations
        # 'ephem' error is based on calculation with GPS/heading
        # 'target' error is based on what's in the camera frame
        #yaw_ephem, pitch_ephem, latitude, longitude, altitude = ephem_update()   
        yaw, pitch, targeted, thresh = target(frame)
        logger.debug('## yaw_target: ', yaw)

        if self.counter == 1: 
            self.switch_offboard_mode()                     # Switches to offboard mode in the first counter. Only needs to be called once.

        if self.counter < 10:
            self.pub_act_test()                             # Perform the actuator and servo test in the first few seconds

        if self.counter > 20 and self.counter < 100:
#        if self.counter == 30:            
#            print('## at counter 30')
            self.gimbal_manager_configure()
            self.gimbal_neutral()
            self.mount_pitch_stabilize()                    # This works alone.

        if self.counter == 40 or self.counter == 50:
            self.arm()                                      # Arm the payload. Will disarm from auto preflight disarming. Only needs to be called once.
            # self.pub_veh_ctl_mode()
            # self.pub_act_test_off()
            # self.pub_traj_set()

        if self.counter == 100 or self.counter == 120:
            print('## disarm')
            #self.disarm()                                   # Only needs to be called once. DON'T SPAM the disarm.
        
        if (self.counter % 100) == 0:                       # Every 10 seconds, save the images
            print('## Modulo output')

            cv.imwrite(os.path.join(self.imagesPath, "raw_" + str(self.picture) + "_" + timeString() + ".jpg"), original)
            cv.imwrite(os.path.join(self.imagesPath, "thresh_" + str(self.picture) + "_" + timeString() + ".jpg"), thresh)
            cv.imwrite(os.path.join(self.imagesPath, "targeted_" + str(self.picture) + "_" + timeString() + ".jpg"), targeted)
            self.picture += 1
            print(f"Saved picture {self.picture}")

        self.counter += 1

#### Main function ####
def main(args=None) -> None:
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO) # Set the format and level for logging. Will output to terminal. level=logging.DEBUG 
    logger.debug('In main()')                       # Print this debug statement when level is activated in basicConfig()

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

    cx = 320                                        # Half of X-axis pixels for tracking image
    cy = 240                                        # Half of Y-axis pixels

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
