from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_cpp = LaunchConfiguration('use_cpp')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_cpp',
            default_value='false',
            description='Use C++ implementation instead of Python'
        ),
        
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard_cpp' if use_cpp.perform(None) == 'true' else 'teleop_keyboard',
            name='teleop_keyboard',
            output='screen'
        )
    ]) 