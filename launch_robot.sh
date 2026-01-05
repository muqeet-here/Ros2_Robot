#!/bin/bash

# Launch Robot Node
# This starts the ESP32 communication and odometry node
# Run this on the Raspberry Pi (on the robot)

source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

# Set ROS_DOMAIN_ID if needed for network communication
# export ROS_DOMAIN_ID=42

echo "===================================================="
echo "Launching Robot Node"
echo "===================================================="
echo ""
echo "Starting ESP32 communication and odometry..."
echo "Serial port: /dev/ttyUSB0"
echo "Baudrate: 115200"
echo ""
echo "===================================================="
echo ""

ros2 launch robot_package esp32_communication.launch.py
