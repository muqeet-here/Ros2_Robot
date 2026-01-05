#!/bin/bash

# Launch Control GUI Node
# This opens the GUI for controlling the robot
# Run this on laptop or RPi

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

# Set ROS_DOMAIN_ID if needed for network communication
# export ROS_DOMAIN_ID=42

echo "===================================================="
echo "Launching Control GUI Node"
echo "===================================================="
echo ""
echo "This will open a GUI window to control the robot"
echo "Make sure the Robot Node is running on the RPi"
echo ""
echo "===================================================="
echo ""

ros2 launch robot_package control_gui.launch.py
