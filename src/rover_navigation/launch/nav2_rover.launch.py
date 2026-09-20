#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    rover_share = get_package_share_directory('rover_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(
        rover_share,
        'config',
        'nav2_params.yaml'
    )

    nav2_launch = os.path.join(
        nav2_share,
        'launch',
        'navigation_launch.py'
    )

    # Make Nav2 +X point toward physical rover front (-Y base_footprint)
    nav_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='nav_base_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '-1.57079632679',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'nav_base_link'
        ],
        output='screen'
    )

    cmd_adapter = Node(
        package='rover_navigation',
        executable='nav2_cmd_adapter',
        name='nav2_cmd_adapter',
        output='screen'
    )

    odom_adapter = Node(
        package='rover_navigation',
        executable='nav2_odom_adapter',
        name='nav2_odom_adapter',
        output='screen'
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav_base_tf,
        cmd_adapter,
        odom_adapter,
        navigation
    ])
