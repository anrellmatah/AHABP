#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint, ActuatorTest  # These are compatible .msg files. Check 'px4_msgs/msg'
import time

print('##### Hi from ahabp_node_offboard_2.py #####')

class OffboardModePublisher(Node):
    print('In publisher node...')
    def __init__(self):
        super().__init__('offboard_mode_publisher') # This is the name of the node. It will appear as a oval in rqt's node graph.

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1  # Adjust the queue size as needed
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher( # create publisher for offboard mode
            OffboardControlMode, # px4_msg uORB message. Check 'pX4_msgs/msg'
            '/fmu/in/offboard_control_mode', # topic type. Check 'dds_topics.yaml'
            qos_profile # QoS
        )

        # Create publsiher for actuator motors
        self.actuator_test_publisher = self.create_publisher( # Create publisher for actuator test
            ActuatorTest, # Compatible px4_msgs message file. Check 'pX4_msgs/msg'
            '/fmu/in/actuator_test', # Topic type. Check 'dds_topics.yaml'
            qos_profile # QoS
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, # px4_msg uORB message. Check 'pX4_msgs/msg'
            '/fmu/out/vehicle_local_position', # topic type. Check 'dds_topics.yaml'
            self.vehicle_local_position_callback, # Callback function
            qos_profile # QoS
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile
        )

        # Initialize internal variables
        self.offboard_setpoint_counter = 0
        self.local_timestamp = 0 
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0 # in meters [m]

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback) # 4 Hz
    

##### The function definition portion #####
    # Callback function for vehicle_local_position topic subscriber.
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    # Callback function for vehicle_status topic subscriber.
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self): ## This commands works
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0
            )
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=0.0
            )
        self.get_logger().info('Disarm command sent')
    
    def engage_offboard_mode(self): ## This command works
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0, 
            param2=6.0
            )
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def pub_test_mot1(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, 
            param1=1.0,
            param3=1.0,
            )
        self.get_logger().info("Testing motor1")

    def pub_act_test(self):
        self.publish_actuator_test(
            ActuatorTest.ACTION_DO_CONTROL,
            value=0.3,
            timeout_ms=100,
            function=103,
            action=1,
            )
        self.get_logger().info("Sending pub_act_test")

###### Message compilers

    def publish_actuator_test(self, command, **params) -> None:
        msg = ActuatorTest()
        msg.action = command
        msg.value       = params.get("value", 0.3) ##
        msg.timeout_ms  = params.get("timeout_ms", 1000) ##
        msg.function    = params.get("function", 103) ##
        msg.action      = params.get("action", 1) ##
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) ##
        self.actuator_test_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg._direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
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
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

# This function is where the script is planned
    # Callback function for the timer
    def timer_callback(self) -> None: # Needed for publishing rate
        print('In timer callback function: ', self.offboard_setpoint_counter)
        self.publish_offboard_control_heartbeat_signal()
        if self.offboard_setpoint_counter == 4:
            self.engage_offboard_mode()
            #time.sleep(2)
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print('Publishing setpoint...')
            self.publish_position_setpoint(
                0.0, 
                0.0, 
                self.takeoff_height)

        elif self.vehicle_local_position.z <= self.takeoff_height: # If current local position is less than the takeoff height then proceed
            print('Landing...')
            self.land()
            print('Exiting...')
            exit(0)

        if self.offboard_setpoint_counter < 5:
            print('Setpoint countering...')
            self.offboard_setpoint_counter += 1
    
        self.pub_act_test()
    print('Leaving publisher node...')

def main(args=None) -> None:
    rclpy.init(args=args) # Starts
    offboard_control = OffboardModePublisher() # Delcare the class
    rclpy.spin(offboard_control) # Basically expands to an instantiation and invocation of the Single-Threaded Executor, which is the simplest Executor
    offboard_control.destroy_node() # Kills all the nodes
    rclpy.shutdown() # Ends

if __name__ == '__main__':
    try:
        print('Before main is called...')
        main()
    except Exception as e:
        print('Print exception...')
        print(e)