"""
gazebo.launch.py

Spawns the 6-wheel rocker-bogie rover into Gazebo Classic 11 with a live
ros2_control interface (rover_velocity_controller, JointGroupVelocityController
over all 6 wheel joints), replacing the old broken libgazebo_ros_diff_drive.so
plugin that only drove one wheel pair.

Workflow (unchanged from your established one):
  - robot_state_publisher consumes urdf/rover.urdf (tree structure, has the
    <ros2_control> hardware-interface block used by gazebo_ros2_control)
  - spawn_entity.py consumes urdf/rover.sdf (has the hand-added loop-closing
    joints for the trapezoid linkages, which URDF cannot express, plus the
    gazebo_ros2_control plugin tag -- see rover.sdf for why it's inserted
    there rather than carried over from URDF)

Because rover.sdf is a static file (not xacro), it can't resolve
find-pkg-share at parse time. The plugin block in rover.sdf contains a
placeholder, __CONTROLLERS_YAML_PATH__, which this launch file substitutes
with the real installed path to controllers.yaml before spawning, writing
the result to a temp file.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _render_sdf_and_spawn(context, *args, **kwargs):
    pkg_path = get_package_share_directory('rover_description')

    sdf_template_path = os.path.join(pkg_path, 'urdf', 'rover.sdf')
    controllers_yaml_path = os.path.join(pkg_path, 'config', 'controllers.yaml')

    with open(sdf_template_path, 'r') as f:
        sdf_content = f.read()

    sdf_content = sdf_content.replace('__CONTROLLERS_YAML_PATH__', controllers_yaml_path)

    # Write the rendered SDF to a temp file for spawn_entity.py to consume.
    # Not writing back into the install/source tree -- keeps rover.sdf
    # portable across machines/CI rather than baking in an absolute path.
    tmp_sdf = tempfile.NamedTemporaryFile(
        mode='w', suffix='_rover.sdf', delete=False
    )
    tmp_sdf.write(sdf_content)
    tmp_sdf.close()

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover',
            '-file', tmp_sdf.name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    rover_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_velocity_controller'],
        output='screen'
    )

    # spawn_entity.py is a one-shot script that exits once the model is
    # spawned. Chain the controller spawners off that exit event instead of
    # a fixed TimerAction delay -- avoids racing controller_manager startup
    # against variable Gazebo load time.
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner, rover_velocity_controller_spawner],
        )
    )

    return [spawn_entity_node, delayed_controller_spawners]


def generate_launch_description():
    pkg_path = get_package_share_directory('rover_description')

    urdf_file = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'rover_world.world')

    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.dirname(pkg_path) + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen',
        remappings=[
            ('/joint_states', '/joint_states_synced')
        ],
    )

    rviz_node = Node(
            condition=IfCondition('false'),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        robot_state_publisher_node,
        OpaqueFunction(function=_render_sdf_and_spawn),
        rviz_node,
    ])
