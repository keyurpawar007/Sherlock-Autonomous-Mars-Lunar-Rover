#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('rover_navigation'),
        'config',
        'navigation.yaml'
    )

    return LaunchDescription([

        Node(
            package='rover_navigation',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[config],
            remappings=[
                ('/global_path', '/global_path_raw')
            ],
        ),

        Node(
            package='rover_navigation',
            executable='path_smoother',
            name='path_smoother',
            output='screen',
        ),

        Node(
            package='rover_navigation',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen',
            parameters=[config],
        ),

        Node(
            package='rover_navigation',
            executable='goal_manager',
            name='goal_manager',
            output='screen',
            parameters=[
                config,
                {'goal_tolerance': 0.10}
            ],
        ),

        Node(
            package='rover_navigation',
            executable='direct_path_follower',
            name='direct_path_follower',
            output='screen',
        ),
    ])
