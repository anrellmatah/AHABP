import os
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

print('#### Hi from ahabp_node.py ####')

def main():
    print('befor uxrce_cmd')
#    uxrce_cmd = os.system('MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5')
#    uxrce_cmd = subprocess.Popen('MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5')
    #uxrce_cmd = subprocess.call('MicroXRCEAgent serial -b 921600 -D /dev/ttyAMA0 -v 5')
    print('after uxrce_cmd')

    print('before gps_msg_cmd')
#    gps_msg_cmd = os.system('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
#    gps_msg_cmd = subprocess.Popen('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
    #gps_msg_cmd = subprocess.call('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
    print('after gps_msg_cmd')

# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     #rclpy.init(args=args)
#     #offboard_control = OffboardControl()
#     #rclpy.spin(offboard_control)
#     #offboard_control.destroy_node()
#     rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
