#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    sdf_model_path = os.path.join(pkg_share, 'urdf', 'GOLDEN_WORKING_ROVER.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(sdf_model_path).read() if os.path.exists(sdf_model_path) else ''}]
        ),

        # Gazebo Launch Include (if ros_gz_sim or gazebo_ros is available)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]) if os.path.exists(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')) else PythonLaunchDescriptionSource([]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ) if os.path.exists(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')) else Node(
            package='rover_description',
            executable='full_bringup.launch.py',
            name='sim_placeholder',
            output='screen'
        )
    ])
