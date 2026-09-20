import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover_team',
    maintainer_email='rover@todo.todo',
    description='Modular 3D perception engine (Obstacle, Cliff, Red Zone, Terrain, Object, ArUco Docking, YOLOv8)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'obstacle_node = rover_perception.obstacle_node:main',
            'cliff_detector = rover_perception.cliff_detector:main',
            'red_zone_detector = rover_perception.red_zone_detector:main',
            'terrain_detector = rover_perception.terrain_detector:main',
            'object_detector = rover_perception.object_detector:main',
            'aruco_docking_node = rover_perception.aruco_docking_node:main',
            'yolo_target_detector = rover_perception.yolo_target_detector:main',
            'toolbox_node = rover_perception.toolbox_node:main',
            'toolbox_goal_bridge = rover_perception.toolbox_goal_bridge:main',
        ],
    },
)
