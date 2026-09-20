from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Janhavi',
    maintainer_email='janhavi@example.com',

    description='Custom navigation stack for the six-wheel rover.',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'global_planner = rover_navigation.global_planner:main',
            'local_planner = rover_navigation.local_planner:main',
            'obstacle_avoidance = rover_navigation.obstacle_avoidance:main',
            'path_follower = rover_navigation.path_follower:main',
            'goal_manager = rover_navigation.goal_manager:main',
            'direct_path_follower = rover_navigation.direct_path_follower:main',
            'path_smoother = rover_navigation.path_smoother:main',
            'nav2_cmd_adapter = rover_navigation.nav2_cmd_adapter:main',
            'nav2_odom_adapter = rover_navigation.nav2_odom_adapter:main',
            'hardware_bridge = rover_navigation.hardware_bridge:main',
            'sim_drive_adapter = rover_navigation.sim_drive_adapter:main',
            'odom_tf_bridge = rover_navigation.odom_tf_bridge:main',
            'sim_level_stabilizer = rover_navigation.sim_level_stabilizer:main',
            'joint_state_retimestamp = rover_navigation.joint_state_retimestamp:main',
        ],
    },
)
