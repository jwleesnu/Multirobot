from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_teleop_cpp',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen'
        )
    ]) 