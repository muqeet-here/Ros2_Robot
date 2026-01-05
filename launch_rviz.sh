#!/bin/bash

# Fix snap library conflict by unsetting snap paths
unset GTK_PATH
unset LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

# Launch RViz with config
rviz2 -d ~/robot_ws/install/robot_package/share/robot_package/rviz/robot.rviz
