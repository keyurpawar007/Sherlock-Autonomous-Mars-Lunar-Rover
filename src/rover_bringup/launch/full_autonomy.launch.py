#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rover_bringup')
    default_config_path = os.path.join(bringup_dir, 'config', 'rover_system.yaml')

    # Launch Arguments
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Set to true when running on physical Jetson + STM32 hardware'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for STM32 UART motor controller'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='UART Baudrate'
    )

    use_hardware = LaunchConfiguration('use_hardware')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')

    # Invert hardware flag for mock setting (if hardware is true -> use_mock is false)
    use_mock = PythonExpression(['"false" if "', use_hardware, '" == "true" else "true"'])

    return LaunchDescription([
        use_hardware_arg,
        serial_port_arg,
        baudrate_arg,

        # 1. Kinematics & Drive Control
        Node(
            package='rover_kinematics',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[default_config_path]
        ),
        Node(
            package='rover_kinematics',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[default_config_path]
        ),

        # 2. STM32 Hardware Bridge & Watchdog
        Node(
            package='rover_stm_bridge',
            executable='uart_bridge',
            name='uart_bridge',
            output='screen',
            parameters=[{
                'port': serial_port,
                'baudrate': baudrate,
                'use_mock_hardware': use_mock
            }]
        ),
        Node(
            package='rover_stm_bridge',
            executable='watchdog',
            name='watchdog',
            output='screen',
            parameters=[default_config_path]
        ),

        # 3. Sensor Publishers
        Node(
            package='rover_sensors',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{'use_mock': use_mock}]
        ),
        Node(
            package='rover_sensors',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{'use_mock': use_mock}]
        ),

        # 4. Localization Monitor
        Node(
            package='rover_localization',
            executable='localization_monitor',
            name='localization_monitor',
            output='screen',
            parameters=[default_config_path]
        ),

        # 5. Modular Perception Engine
        Node(
            package='rover_perception',
            executable='obstacle_node',
            name='obstacle_node',
            output='screen',
            parameters=[default_config_path]
        ),
        Node(
            package='rover_perception',
            executable='cliff_detector',
            name='cliff_detector',
            output='screen',
            parameters=[default_config_path]
        ),
        Node(
            package='rover_perception',
            executable='red_zone_detector',
            name='red_zone_detector',
            output='screen',
            parameters=[default_config_path]
        ),
        Node(
            package='rover_perception',
            executable='terrain_detector',
            name='terrain_detector',
            output='screen',
            parameters=[default_config_path]
        ),
        Node(
            package='rover_perception',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[default_config_path]
        ),

        # 6. Navigation Stack Goal Manager
        Node(
            package='rover_navigation',
            executable='goal_manager',
            name='goal_manager',
            output='screen',
            parameters=[default_config_path]
        ),

        # 7. Parallel Safety Manager
        Node(
            package='rover_safety',
            executable='safety_manager',
            name='safety_manager',
            output='screen',
            parameters=[default_config_path]
        ),

        # 8. Top-Level Mission State Machine
        Node(
            package='rover_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[default_config_path]
        ),
    ])
