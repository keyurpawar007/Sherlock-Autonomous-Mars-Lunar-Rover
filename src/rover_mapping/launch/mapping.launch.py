import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('rover_mapping')

    slam_config = os.path.join(
        package_dir,
        'config',
        'slam_toolbox.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': False
            }
        ]
    )

    slam_monitor_node = Node(
        package='rover_mapping',
        executable='slam_node',
        name='rover_slam_monitor',
        output='screen',
        parameters=[
            {
                'use_sim_time': False
            }
        ]
    )

    costmap_manager_node = Node(
        package='rover_mapping',
        executable='costmap_manager',
        name='costmap_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'map_topic': '/map',
                'costmap_topic': '/costmap',
                'inflation_radius': 0.75,
                'occupied_threshold': 65,
            }
        ]
    )


    map_manager_node = Node(
        package='rover_mapping',
        executable='map_manager',
        name='map_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'map_topic': '/map',
                'loaded_map_topic': '/loaded_map',
                'map_directory': '/home/janhavi/ros2_ws/maps',
                'map_name': 'rover_map',
            }
        ]
    )
    return LaunchDescription([
        slam_toolbox_node,
        slam_monitor_node,
	costmap_manager_node,
	map_manager_node
    ])
