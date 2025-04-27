from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import xacro
def generate_launch_description():
    
    # Package directories
    description_pkg_share = get_package_share_directory('bumpy_description')
    
    # URDF file path
    xacro_file = os.path.join(description_pkg_share, 'urdf', 'bumpy.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )
    # Robot state publisher parameters


# YLidar X2 node
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[{
            'port': "/dev/ttyUSB0",
            'frame_id': "lidar_link",
            'ignore_array': '',
            'baudrate': 115200,  # X2 uses 115200 instead of 128000
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 3,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'reversion': True,  
            'inverted': True,
            'auto_reconnect': True,
            'isSingleChannel': True,  # X2 is not single channel
            'intensity': False,
            'support_motor_dtr': True,  # X2 typically supports motor DTR
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 12.0,  # X2 has shorter range than X4 Pro
            'range_min': 0.1,
            'frequency': 10.0,  # X2 typically runs at 8Hz
            'invalid_range_is_inf': False
        }]
    )
    
    # Static transform publisher for LiDAR
    # Note: This is a fallback in case the URDF doesn't load properly
    lidar_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_transform_publisher',
        arguments=['0.16', '0', '0.05', '0', '0', '0', 'base_footprint', 'lidar_link']
    )
    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'use_sim_time': False}]
)

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumpy_description"), "config", "ekf.yaml")],
    )

    return LaunchDescription([
        
        robot_state_publisher_node,

        lidar_node,
        lidar_transform_node,
        joint_state_publisher_node,
        #robot_localization
        
    ])

