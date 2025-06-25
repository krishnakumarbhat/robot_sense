from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sense',
            executable='d435i_reader',
            name='d435i_reader',
            output='screen'
        ),
    ])