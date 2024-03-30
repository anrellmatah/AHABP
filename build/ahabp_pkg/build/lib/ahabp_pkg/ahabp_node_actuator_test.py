#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus, VehicleAttitudeSetpoint, ActuatorMotors, ActuatorTest, ActuatorOutputs # These are compatible .msg files. Check 'px4_msgs/msg'
import time

print('##### Hi from ahabp_node_actuator_test.py #####')

class Actuator_Test(Node): # Node. --> self.
    def __init__(self):
        super().__init__('actuator_test_pub')  # This is the name of the node. It will appear as a oval in rqt's node graph.

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

        self.actuator_test_publisher = self.create_publisher( # Create publisher for actuator test
            ActuatorTest, # Compatible px4_msgs message file. Check 'pX4_msgs/msg'
            '/fmu/in/actuator_test', # Topic type. Check 'dds_topics.yaml'
            qos_profile # QoS
        )

        self.actuator_motors_publisher = self.create_publisher( # Create publisher for actuator test
            ActuatorMotors, # Compatible px4_msgs message file. Check 'pX4_msgs/msg'
            '/fmu/in/actuator_motors', # Topic type. Check 'dds_topics.yaml'
            qos_profile # QoS
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )
        # Create subscribers

        # Initialize internal variables
        self.offboard_setpoint_counter = 0
        self.local_timestamp = 0 
        self.takeoff_height = -100.0 # in meters [m]

        self.timer = self.create_timer(.1, self.timer_callback)

    # def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
    #     self.actuator_mode = ActuatorMotors()
    #     self.actuator_mode.ACTUATOR_FUNCTION_MOTOR1 = 103
    #     self.actuator_mode.NUM_CONTROLS = 12
    #     self.actuator_mode.control = 0.0
    #     self.actuator_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.actuator_test_pub.publish(self.actuator_mode) # Publish the command message
    #     self.get_logger().info('###Actuator test published') # This prints into the logger which can be seen in the terminal.


#### Individual command functions ####

    def pub_test_mot1(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST, 
            param3=1.0,
            param4=0.0,
            )
        self.get_logger().info("Testing motors")

        # self.actuator_mode = ActuatorMotors()
        # self.actuator_mode.ACTUATOR_FUNCTION_MOTOR1 = 103
        # self.actuator_mode.NUM_CONTROLS = 12
        # self.actuator_mode.control = 0.0
        # self.actuator_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # self.actuator_test_pub.publish(self.actuator_mode) # Publish the command message
        # self.get_logger().info('###Actuator test published') # This prints into the logger which can be seen in the terminal.
    
    def pub_act_test(self):
        self.publish_actuator_test(
            ActuatorTest.ACTION_DO_CONTROL,
            reversible_flags=0,
            timeout_ms=1000,
            function=103,
            action=1,
            )
        self.get_logger().info("Sending pub_act_test")

    def pub_act_mot(self):
        self.publish_actuator_motors(
            #ActuatorTest.ACTUATOR_FUNCTION_MOTOR1,
            reversible_flags=0,
            )
        self.get_logger().info("Sending pub_act_mot")

#### The message compilers ####

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        msg.thrust_and_torque = True
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_actuator_test(self, command, **params) -> None:
        msg = ActuatorTest()
        msg.value       = params.get("value", 0.1) ##
        msg.timeout_ms  = params.get("timeout_ms", 100) ##
        msg.function    = params.get("function", 103) ##
        msg.action      = params.get("action", True) ##
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) ##
        self.actuator_test_publisher.publish(msg)

    def publish_actuator_motors(self, **params) -> None:
        msg = ActuatorMotors()
        #msg.command = command
        #msg.timestamp_sample    = params.get("timestamp_sample", 0) # the timestamp the data this control response is based on was sampled
        msg.reversible_flags    = params.get("reversible_flags", 0)
        msg.control             = params.get("control", [0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_motors_publisher.publish(msg)

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
        msg.confirmation = 0
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info('Motor test command published')

    # Callback function for the timer
    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()
        self.pub_act_mot()
        self.pub_test_mot1()
        self.pub_act_test()
    print('Leaving timer callback...')

def main(args=None) -> None:
    rclpy.init(args=args) # Starts
    actuator_test = Actuator_Test() # Create a publisher for the Actuator_Test message. Stays the majority in this function.
    rclpy.spin(actuator_test) # Keep the node alive until Ctrl+C is pressed
    actuator_test.destroy_node() # Kills all the nodes
    rclpy.shutdown() # End

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)


