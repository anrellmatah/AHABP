#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus, VehicleAttitudeSetpoint, ActuatorMotors, ActuatorTest # These are compatible .msg files. Check 'px4_msgs/msg'
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

        self.timer = self.create_timer(.25, self.timer_callback)

    # def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
    #     self.actuator_mode = ActuatorMotors()
    #     self.actuator_mode.ACTUATOR_FUNCTION_MOTOR1 = 103
    #     self.actuator_mode.NUM_CONTROLS = 12
    #     self.actuator_mode.control = 0.0
    #     self.actuator_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.actuator_test_pub.publish(self.actuator_mode) # Publish the command message
    #     self.get_logger().info('###Actuator test published') # This prints into the logger which can be seen in the terminal.

    def pub_test_mot1(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, 
            param1=1.0,
            param3=1.0,
            )
        self.get_logger().info("Testing motor1")

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
            value=0.3,
            timeout_ms=1000,
            function=103,
            action=1,
            )
        self.get_logger().info("Sending pub_act_test")

    def publish_actuator_test(self, command, **params) -> None:
        msg = ActuatorTest()
        msg.value       = params.get("value", 0.3) ##
        msg.timeout_ms  = params.get("timeout_ms", 1000) ##
        msg.function    = params.get("function", 103) ##
        msg.action      = params.get("action", 1) ##
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) ##
        self.actuator_test_publisher.publish(msg)

    # def publish_offboard_control_heartbeat_signal(self):
    #     msg = OffboardControlMode()
    #     msg.position = True
    #     msg.velocity = False
    #     msg.acceleration = False
    #     msg.attitude = False
    #     msg.body_rate = False
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.offboard_control_mode_publisher.publish(msg)

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
        #self.pub_test_mot1()
        self.pub_act_test()
    print('Leaving timer callback...')

def main(args=None) -> None:
    rclpy.init(args=args) # Starts
    actuator_test = Actuator_Test() # Create a publisher for the Actuator_Test message. Stays the majority in this function.
    rclpy.spin(actuator_test) # Keep the node alive until Ctrl+C is pressed
    actuator_test.destroy_node() # Kills all the nodes
    rclpy.shutdown() # End

if __name__ == '__main__':
    main()

