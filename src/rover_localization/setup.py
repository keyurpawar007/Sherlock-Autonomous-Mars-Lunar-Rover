import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover_team',
    maintainer_email='rover@todo.todo',
    description='Localization monitor and EKF configuration',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'localization_monitor = rover_localization.localization_monitor:main',
        ],
    },
)
