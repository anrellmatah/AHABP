import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Imu
import math
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

#TODO: Put in the QoS stuff


class YawControlNode(Node):
    def __init__(self):
        super().__init__('yaw_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile( # Relevant: https://answers.ros.org/question/332207/cant-receive-data-in-python-node/
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.TRANSIENT_LOCAL,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1  # Adjust the queue size as needed
        )

        self.publisher = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', qos_profile)

        self.subscription = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_profile)
        self.des_yaw = 0.0  # Desired yaw angle in radians
        self.K_p = 0.6  # Proportional gain

    def imu_callback(self, msg):
        # Extract yaw angle from IMU data (assuming quaternion orientation)
        orientation_q = msg.orientation
        euler = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        yaw = euler[2]  # Yaw angle in radians

        # Calculate yaw error
        yaw_error = self.des_yaw - yaw

        # Calculate yaw rate command using proportional control
        yaw_rate_cmd = self.K_p * yaw_error

        # Publish attitude target with only yaw rate command
        attitude_target = AttitudeTarget()
        attitude_target.type_mask = 1  # Ignore roll and pitch rates
        attitude_target.thrust = 0.5  # Set a constant thrust
        attitude_target.body_rate.z = yaw_rate_cmd  # Set the yaw rate command
        self.publisher.publish(attitude_target)

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

def main(args=None):
    rclpy.init(args=args)
    yaw_control_node = YawControlNode()
    rclpy.spin(yaw_control_node)
    yaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
