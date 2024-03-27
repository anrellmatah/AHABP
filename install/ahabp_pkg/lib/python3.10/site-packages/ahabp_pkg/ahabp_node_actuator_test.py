#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint, ActuatorMotors # These are types of topics. Check 'dds_topics.yaml'
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

        self.actuator_test_pub = self.create_publisher( # Create publisher for actuator test
        ActuatorMotors, # Compatible px4_msgs message file. Check 'pX4_msgs/msg'
        '/fmu/in/actuator_motors', # Topic type. Check 'dds_topics.yaml'
        qos_profile # QoS
        )

        self.timer = self.create_timer(1, self.pub_act_test) # Calls the offboard publisher every .5 seconds (2 Hz)

    def pub_act_test(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/ActuatorMotors.msg
        self.actuator_mode = ActuatorMotors()
        self.actuator_mode.ACTUATOR_FUNCTION_MOTOR1 = 103
        self.actuator_mode.NUM_CONTROLS = 12
        self.actuator_mode.control = 0.0
        self.actuator_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.actuator_test_pub.publish(self.actuator_mode) # Publish the command message
        self.get_logger().info('###Actuator test published') # This prints into the logger which can be seen in the terminal.

def main(args=None) -> None:
    rclpy.init(args=args) # Starts
    actuator_test = Actuator_Test() # Create a publisher for the Actuator_Test message. Stays the majority in this function.
    rclpy.spin(actuator_test) # Keep the node alive until Ctrl+C is pressed
    actuator_test.destroy_node() # Kills all the nodes
    rclpy.shutdown() # End

if __name__ == '__main__':
    main()

