from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='r1_upper',
            executable='can_motor',
            name='can_motor'
        ),
        Node(
            package='r1_upper',
            executable='new_manipulator',
            name='new_manipulator',
            output='log'
        )
    ])
