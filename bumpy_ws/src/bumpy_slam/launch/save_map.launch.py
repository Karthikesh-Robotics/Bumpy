from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('bumpy_slam')
    
    # Create launch configuration variables
    map_name = LaunchConfiguration('map_name')
    map_dir = LaunchConfiguration('map_dir')
    
    # Declare launch arguments
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='amr_map',
        description='Name of the map to save'
    )
    
    declare_map_dir_cmd = DeclareLaunchArgument(
        'map_dir',
        default_value=PathJoinSubstitution([pkg_share, 'maps']),
        description='Directory to save the map'
    )
    
    # Map saver node
    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65
        }],
        arguments=[
            '-f', [map_dir, '/', map_name]
        ]
    )
    
    # Create and return the launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_map_dir_cmd)
    ld.add_action(map_saver_node)
    
    return ld