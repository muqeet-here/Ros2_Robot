from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Control Node - GUI for controlling the robot
    Can run on laptop or RPi
    """
    return LaunchDescription([
        # Control Node (GUI for robot control)
        Node(
            package='robot_package',
            executable='control_node.py',
            name='control_node',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
