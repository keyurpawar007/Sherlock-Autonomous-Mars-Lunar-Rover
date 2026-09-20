from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'rover_mapping'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

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
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Janhavi',

    maintainer_email='you@example.com',

    description=(
        'SLAM, map management and costmap utilities '
        'for the six-wheel rover'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'scan_retimestamp = rover_mapping.scan_retimestamp:main',
            'slam_node = rover_mapping.slam_node:main',
            'costmap_manager = rover_mapping.costmap_manager:main',
            'map_manager = rover_mapping.map_manager:main',
        ],
    },
)
