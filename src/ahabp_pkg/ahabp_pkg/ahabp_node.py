import os
import subprocess

print('Hi from ahabp_pkg.')

def main():
    print('Hi from ahabp_pkg.')
    result = os.system('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')
    print('ran vehicle_gps publisher.')

if __name__ == '__main__':
    main()
