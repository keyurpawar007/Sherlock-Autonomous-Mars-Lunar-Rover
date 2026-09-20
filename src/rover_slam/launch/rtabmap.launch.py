import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import UnlessCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_mode = LaunchConfiguration('localization')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Run RTAB-Map in localization mode'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    rover_slam_share = FindPackageShare('rover_slam')
    config_path = PathJoinSubstitution([rover_slam_share, 'config', 'slam.yaml'])
    
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[config_path, {
            'use_sim_time': use_sim_time,
            'localization': localization_mode,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/odom'),
        ],
    )
    
    rgbd_sync_node = Node(
        package='rtabmap_ros',
        executable='rgbd_sync',
        name='rgbd_sync',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'queue_size': 10,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgbd_image', '/rtabmap/rgbd_image'),
        ],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([rover_slam_share, 'rviz', 'rtabmap.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_rviz),
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_localization,
        declare_use_rviz,
        rgbd_sync_node,
        rtabmap_node,
        rviz_node,
    ])
