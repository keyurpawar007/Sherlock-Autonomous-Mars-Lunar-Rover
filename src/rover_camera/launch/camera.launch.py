#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_camera',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        # Note: In production, include realsense2_camera launch here:
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('realsense2_camera'),
        #             'launch', 'rs_launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'pointcloud.enable': 'true',
        #         'align_depth.enable': 'true',
        #     }.items()
        # )
    ])
