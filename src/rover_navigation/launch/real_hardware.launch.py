#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    hardware_bridge = Node(
        package='rover_navigation',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
        parameters=[
            {
                'max_wheel_speed': 4.0
            }
        ]
    )

    return LaunchDescription([
        hardware_bridge
    ])
