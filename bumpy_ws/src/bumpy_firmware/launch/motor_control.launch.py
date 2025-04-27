from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'speed_multiplier',
            default_value='0.1',
            description='Overall speed multiplier (0.0-1.0)'
        ),
        
        DeclareLaunchArgument(
            'rotation_multiplier',
            default_value='0.2',
            description='Rotation speed multiplier (0.0-1.0)'
        ),
        
        DeclareLaunchArgument(
            'linear_scale',
            default_value='0.4',
            description='Additional scaling for linear movement (0.0-1.0)'
        ),
        
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.4',
            description='Maximum linear velocity from cmd_vel (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='0.3',
            description='Maximum angular velocity from cmd_vel (rad/s)'
        ),
        
        # Motor control node
        Node(
            package='bumpy_firmware',
            executable='control',
            name='motor_control',
            output='screen',
        ),
        Node(
            package="bumpy_firmware",
            executable="tick",
            name="ticks_publisher"
        ),
        Node(
            package="bumpy_firmware",
            executable="diff",
            name="diff"
        )
    ])