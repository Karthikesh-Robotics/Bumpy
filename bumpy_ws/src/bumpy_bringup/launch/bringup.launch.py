#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Paths
    bumpy_description_pkg = FindPackageShare('bumpy_description')
    bumpy_firmware_pkg = FindPackageShare('bumpy_firmware')
    bumpy_sensors_pkg = FindPackageShare('bumpy_sensors')
    
    # Include robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bumpy_description_pkg, 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bumpy_firmware_pkg, 'launch', 'motor_control.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # OLED Node
    oled_node = Node(
        package='bumpy_sensors',
        executable='oled_node',
        name='oled_node',
        output='screen'
    )
    
    # Camera Node
    camera_node = Node(
        package='bumpy_sensors',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )
    imu_node = Node(
        package='bumpy_sensors',
        executable='imu_node',
        name='imu_node',
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add robot description launch
    ld.add_action(robot_description_launch)
    ld.add_action(motor_control_launch)

    # Add sensor nodes
    ld.add_action(oled_node)
    ld.add_action(camera_node)
    
    return ld