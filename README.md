# AHABP
Autonomous High-Altitude Balloon Payload

This github repository is for EEE 488/489 Senior Design I and II for electrical engineering.

## Okay, so what files do we mess with?
Files within the package of a src folder of a workspace folder
- “~<workspace name>_ws/src/<package name>/”
Python files that create nodes
“setup.py”
Primarily stuff found under the src directory
“.../ahabp_v2_ws/src/ahabp_pkg/package.xml”
“.../ahabp_v2_ws/src/ahabp_pkg/setup.py”
“.../ahabp_v2_ws/src/ahabp_pkg/ahabp_pkg/”
This is where the node/executables go in
“.../ahabp_v2_ws/src/ahabp_pkg/ahabp_pkg/”

## Common Mavlink Console commands:
- uorb top
- listener actuator_outputs
- uorb status actuator
- uorb status vehicle
- listener offboard_control_mode

Run motors 1 & 3 at 1150 PWM
- actuator_test set -m 1 -v 0 -t 2
- actuator_test set -m 3 -v 0 -t 2
- actuator_test iterate-motors
- actuator_test iterate-motors
- actuator_test set -f 101 -v 0 -t 5
- actuator_test set -f 103 -v 0 -t 5
- actuator_test set -f 421 -v 0 -t 5

Force arm
- commander arm -f

Uxrce
- uxrce_dds_client status

## Before running the launch file:
- cd
- cd ahabp_v2_ws/
- colcon build _--or--_ colcon build --event-handlers console_direct+ --executor sequential
- source install/setup.bash

## To make a new node within the launch folder:
1. Go to “home/ahabp_v2_ws/src/ahabp_pkg/ahabp_pkg/” directory 
2. Create new python file (preferable starts with ‘ahabp_node_…’)
3. Edit “home/ahabp_v2_ws/src/ahabp_pkg/setup.py”
4. Add an entry point after console scripts - follow the format: <node name> = <package name>.<node name>:main
5. Ex: “ahabp_node = ahabp_pkg.ahabp_node:main”
6. Save
7. Go to home and “colcon build”
8. Run “ros2 pkg executables <package name>” or find your node in “/home/anyell/ahabp_v2_ws/install/ahabp_pkg/lib/ahabp_pkg”
