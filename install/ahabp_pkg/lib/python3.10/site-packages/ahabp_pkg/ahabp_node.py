import os
import subprocess

def main():
    print('Hi from ahabp_pkg.')
    result = os.system('ros2 topic pub /fmu/out/vehicle_gps_position px4_msgs/msg/SensorGps')

if __name__ == '__main__':
    main()
