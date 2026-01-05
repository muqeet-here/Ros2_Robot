from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('robot_package').find('robot_package')
    
    # Paths to files
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Declare launch argument for enabling RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',  # Disabled by default due to snap conflicts
        description='Start RViz visualization'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        
        # ESP32 Communication Node
        Node(
            package='robot_package',
            executable='esp32_communication_node.py',
            name='esp32_communication_node',
            output='screen'
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
        ),
        
        # RViz (run separately with: rviz2 -d <path>)
        # Disabled by default due to snap conflicts
        # To enable: use_rviz:=true
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(
                LaunchConfiguration('use_rviz')
            )
        )
    ])
