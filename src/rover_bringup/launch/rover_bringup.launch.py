#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Top-level launch file for the autonomous rover.
    """
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Set to true when running on physical Jetson + STM32 hardware'
    )

    return LaunchDescription([
        use_hardware_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rover_bringup'),
                    'launch',
                    'full_autonomy.launch.py'
                ])
            ]),
            launch_arguments={
                'use_hardware': LaunchConfiguration('use_hardware'),
                'serial_port': '/dev/ttyUSB0',
                'baudrate': '115200'
            }.items()
        )
    ])
