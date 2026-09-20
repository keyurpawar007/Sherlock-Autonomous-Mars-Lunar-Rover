#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    rover_description_share = get_package_share_directory(
        'rover_description'
    )

    rover_mapping_share = get_package_share_directory(
        'rover_mapping'
    )

    # ---------------------------------------------------------
    # Existing Gazebo launch
    # Gazebo + rover + controllers + robot_state_publisher + RViz
    # ---------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_description_share,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # ---------------------------------------------------------
    # Wheel odometry
    # Start shortly after Gazebo/controllers
    # ---------------------------------------------------------

    wheel_odometry = TimerAction(
        period=3.0,
        actions=[
            Node(
            condition=IfCondition('false'),
                package='rover_description',
                executable='wheel_odometry.py',
                name='wheel_odometry',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': False
                    }
                ]
            )
        ]
    )

    # ---------------------------------------------------------
    # Mapping stack
    # Start after rover + odometry have had time to initialize
    # ---------------------------------------------------------

    mapping_launch = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        rover_mapping_share,
                        'launch',
                        'mapping.launch.py'
                    )
                )
            )
        ]
    )


    # ---------------------------------------------------------
    # RViz - automatically load saved rover visualization
    # ---------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory('rover_navigation'),
        'config',
        'navigation.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    rviz_start = TimerAction(
        period=22.0,
        actions=[rviz_node]
    )


    # Simulation uses Gazebo ground-truth odometry.
    # Keep map and odom aligned with a timeless static transform.
    map_to_odom_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map', 'odom'
        ],
        output='screen',
    )

    # Simulation drivetrain + odometry source
    sim_drive_adapter = Node(
        package='rover_navigation',
        executable='sim_drive_adapter',
        name='sim_drive_adapter',
        output='screen',
        remappings=[
            ('/rover_velocity_controller/commands',
             '/nav_wheel_commands')
        ],
    )


    # Convert Gazebo LaserScan timestamps to system time
    scan_retimestamp = Node(
        package='rover_mapping',
        executable='scan_retimestamp',
        name='scan_retimestamp',
        output='screen',
    )


    joint_state_retimestamp = Node(
        package='rover_navigation',
        executable='joint_state_retimestamp',
        name='joint_state_retimestamp',
        output='screen',
    )


    # Publish odom -> base_footprint TF from /odom
    odom_tf_bridge = Node(
        package='rover_navigation',
        executable='odom_tf_bridge',
        name='odom_tf_bridge',
        output='screen',
    )



    return LaunchDescription([
        joint_state_retimestamp,
        sim_drive_adapter,
        map_to_odom_static,
        odom_tf_bridge,
        scan_retimestamp,
        rviz_start,
        gazebo_launch,
        wheel_odometry,
        mapping_launch,
    ])
