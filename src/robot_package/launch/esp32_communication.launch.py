from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """
    Launch file for RPi - Runs Robot Node and robot state publisher
    Control Node should run separately or on a remote machine
    """
    # Get package share directory
    pkg_share = FindPackageShare('robot_package').find('robot_package')
    
    # Path to URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    return LaunchDescription([
        # Robot Node (ESP32 communication, odometry)
        Node(
            package='robot_package',
            executable='robot_node.py',
            name='robot_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False
            }]
        )
    ])
