#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    global_planner = Node(
        package='rover_navigation',
        executable='global_planner',
        name='global_planner',
        output='screen',
    )

    local_planner = Node(
        package='rover_navigation',
        executable='local_planner',
        name='local_planner',
        output='screen',
    )

    obstacle_avoidance = Node(
        package='rover_navigation',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
    )

    path_follower = Node(
        package='rover_navigation',
        executable='path_follower',
        name='path_follower',
        output='screen',
        remappings=[
            ('/rover_velocity_controller/commands',
             '/nav_wheel_commands')
        ],
    )

    goal_manager = Node(
        package='rover_navigation',
        executable='goal_manager',
        name='goal_manager',
        output='screen',
    )



    return LaunchDescription([
        global_planner,
        local_planner,
        obstacle_avoidance,
        path_follower,
        goal_manager,
    ])
