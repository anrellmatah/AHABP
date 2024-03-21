import os
import subprocess

print('####Hi from ahabp_node.py')

def main():
    print('befor uxrce_cmd')
#    uxrce_cmd = os.system('MicroXRCEAgent serial -b 57600 -D /dev/ttyAMA0 -v 5')
#    uxrce_cmd = subprocess.Popen('MicroXRCEAgent serial -b 57600 -D /dev/ttyAMA0 -v 5')
    uxrce_cmd = subprocess.call('MicroXRCEAgent serial -b 57600 -D /dev/ttyAMA0 -v 5')
    print('after uxrce_cmd')

    print('before gps_msg_cmd')
#    gps_msg_cmd = os.system('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
#    gps_msg_cmd = subprocess.Popen('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
    gps_msg_cmd = subprocess.call('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
    print('after gps_msg_cmd')

if __name__ == '__main__':
    main()
