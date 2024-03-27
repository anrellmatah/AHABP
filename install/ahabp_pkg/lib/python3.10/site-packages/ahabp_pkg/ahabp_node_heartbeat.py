#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitudeSetpoint # These are types of topics. Check 'dds_topics.yaml'
import cv2 as cv
from cv_bridge import CvBridge
import time

print('##### Hi from ahabp_node_heartbeat.py #####')
# This script is a good example for publishing nodes.

class OffboardHeartbeatPublisher(Node): # Node. --> self.
    def __init__(self):
        super().__init__('offboard_heartbeat_publisher')  # This is the name of the node. It will appear as a oval in rqt's node graph.

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10  # Adjust the queue size as needed
        )

        self.offboard_mode_pub = self.create_publisher( # Create publisher for offboard mode
        OffboardControlMode, # Compatible px4_msgs message file. Check 'pX4_msgs/msg'
        '/fmu/in/offboard_control_mode', # Topic type. Check 'dds_topics.yaml'
        qos_profile # QoS
        )
        self.timer = self.create_timer(0.25, self.publish_offboard_mode) # Calls the offboard publisher every .25 seconds (4 Hz)

    def publish_offboard_mode(self): # Gotta follow its format: https://github.com/PX4/px4_msgs/blob/main/msg/OffboardControlMode.msg
        self.offboard_mode = OffboardControlMode()
        # Set the desired offboard control mode
        self.offboard_mode.position = True  # Not using position control
        self.offboard_mode.velocity = False    # Not using velocity control
        self.offboard_mode.acceleration = False  # Not using acceleration control
        self.offboard_mode.attitude = False   # Not using attitude control
        self.offboard_mode.body_rate = False  # Not using body rate control
        self.offboard_mode.thrust_and_torque = False
        self.offboard_mode.direct_actuator = False
        self.offboard_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(self.offboard_mode) # Publish the offboard mode message
        self.get_logger().info('Offboard heartbeat published') # This prints into the logger which can be seen in the terminal.

def main(args=None) -> None:
    rclpy.init(args=args) # Starts
    offboard_heartbeat_publisher = OffboardHeartbeatPublisher() # Create a publisher for the OffboardControlMode message. Stays the majority in this function.
    rclpy.spin(offboard_heartbeat_publisher) # Keep the node alive until Ctrl+C is pressed
    offboard_heartbeat_publisher.destroy_node() # Kills all the nodes
    rclpy.shutdown() # End

if __name__ == '__main__':
    main()
