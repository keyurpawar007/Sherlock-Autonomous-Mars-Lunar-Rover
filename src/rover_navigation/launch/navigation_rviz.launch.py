#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'rover_navigation'
    )

    navigation_launch = os.path.join(
        pkg_share,
        'launch',
        'navigation.launch.py'
    )

    rviz_config = os.path.join(
        pkg_share,
        'config',
        'navigation.rviz'
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_launch
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    return LaunchDescription([
        navigation,
        rviz
    ])
