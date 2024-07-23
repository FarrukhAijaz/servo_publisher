# servo_publisher/launch/servo_publisher_launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['sudo', 'pigpiod'],
            output='screen'
        ),
        Node(
            package='servo_publisher',
            executable='rotation',
            name='servo_publisher_node',
            output='screen'
        )
    ])

