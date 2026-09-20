from setuptools import find_packages, setup

package_name = 'rover_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover_team',
    maintainer_email='rover@todo.todo',
    description='Differential drive kinematics and closed-loop PID control for rover',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'kinematics_node = rover_kinematics.kinematics_node:main',
            'pid_controller = rover_kinematics.pid_controller:main',
        ],
    },
)
