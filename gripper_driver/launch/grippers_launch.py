#launch_grippers.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_driver',
            executable='service',
            name='gripper_driver',
        ),
    ])
