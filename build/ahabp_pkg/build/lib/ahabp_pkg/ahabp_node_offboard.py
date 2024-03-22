#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint # These are types of topics
import cv2 as cv
from cv_bridge import CvBridge

print('####Hi from ahabp_node_offboard.py')

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('Offboard_attitude_controller')
        self.br = CvBridge()
        # Configure QoS profile for publishing and subscribing
        
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Adjust the depth as needed
        )

        # Create publishers for offboard control mode, trajectory, vehicle command
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def timer_callback(self) -> None: # Callback function for the timer
        self.publish_offboard_control_heartbeat_signal() # Needed from PX4 to maintain offboard mode

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        elif self.vehicle_local_position.z <= self.takeoff_height:
            self.land()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control) # Basically expands to an instantiation and invocation of the Single-Threaded Executor, which is the simplest Executor
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
