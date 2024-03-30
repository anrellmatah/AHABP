# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/anyell/ahabp_v2_ws/src/mavlink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anyell/ahabp_v2_ws/build/mavlink

# Utility rule file for paparazzi.xml-v2.0.

# Include any custom commands dependencies for this target.
include CMakeFiles/paparazzi.xml-v2.0.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/paparazzi.xml-v2.0.dir/progress.make

CMakeFiles/paparazzi.xml-v2.0: paparazzi-v2.0-cxx-stamp

paparazzi-v2.0-cxx-stamp: /home/anyell/ahabp_v2_ws/src/mavlink/message_definitions/v1.0/paparazzi.xml
paparazzi-v2.0-cxx-stamp: /home/anyell/ahabp_v2_ws/src/mavlink/message_definitions/v1.0/common.xml
paparazzi-v2.0-cxx-stamp: /home/anyell/ahabp_v2_ws/src/mavlink/pymavlink/tools/mavgen.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/anyell/ahabp_v2_ws/build/mavlink/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating paparazzi-v2.0-cxx-stamp"
	/usr/bin/env PYTHONPATH="/home/anyell/ahabp_v2_ws/src/mavlink:/home/anyell/ahabp_v2_ws/install/ahabp_pkg/lib/python3.10/site-packages:/home/anyell/ahabp_v2_ws/install/py_pubsub/lib/python3.10/site-packages:/home/anyell/ahabp_v2_ws/install/px4_ros_com/local/lib/python3.10/dist-packages:/home/anyell/ahabp_v2_ws/install/px4_msgs/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages" /usr/bin/python3.10 /home/anyell/ahabp_v2_ws/src/mavlink/pymavlink/tools/mavgen.py --lang=C++11 --wire-protocol=2.0 --output=include/v2.0 /home/anyell/ahabp_v2_ws/src/mavlink/message_definitions/v1.0/paparazzi.xml
	touch paparazzi-v2.0-cxx-stamp

paparazzi.xml-v2.0: CMakeFiles/paparazzi.xml-v2.0
paparazzi.xml-v2.0: paparazzi-v2.0-cxx-stamp
paparazzi.xml-v2.0: CMakeFiles/paparazzi.xml-v2.0.dir/build.make
.PHONY : paparazzi.xml-v2.0

# Rule to build all files generated by this target.
CMakeFiles/paparazzi.xml-v2.0.dir/build: paparazzi.xml-v2.0
.PHONY : CMakeFiles/paparazzi.xml-v2.0.dir/build

CMakeFiles/paparazzi.xml-v2.0.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/paparazzi.xml-v2.0.dir/cmake_clean.cmake
.PHONY : CMakeFiles/paparazzi.xml-v2.0.dir/clean

CMakeFiles/paparazzi.xml-v2.0.dir/depend:
	cd /home/anyell/ahabp_v2_ws/build/mavlink && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anyell/ahabp_v2_ws/src/mavlink /home/anyell/ahabp_v2_ws/src/mavlink /home/anyell/ahabp_v2_ws/build/mavlink /home/anyell/ahabp_v2_ws/build/mavlink /home/anyell/ahabp_v2_ws/build/mavlink/CMakeFiles/paparazzi.xml-v2.0.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/paparazzi.xml-v2.0.dir/depend
