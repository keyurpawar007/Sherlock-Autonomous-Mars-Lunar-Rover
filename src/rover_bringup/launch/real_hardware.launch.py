#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Physical Hardware Launch Entry Point.
    Enables physical UART serial communication, real sensors, and hardware watchdogs.
    """
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rover_bringup'),
                    'launch',
                    'full_autonomy.launch.py'
                ])
            ]),
            launch_arguments={
                'use_hardware': 'true',
                'serial_port': '/dev/ttyUSB0',
                'baudrate': '115200'
            }.items()
        )
    ])
