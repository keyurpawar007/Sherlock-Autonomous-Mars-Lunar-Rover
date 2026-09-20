from setuptools import find_packages, setup

package_name = 'rover_stm_bridge'

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
    description='CAN Bus (can0 @ 500kbps) and UART Bridge interface with motor controllers',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'can_bridge = rover_stm_bridge.can_bridge:main',
            'uart_bridge = rover_stm_bridge.uart_bridge:main',
            'watchdog = rover_stm_bridge.watchdog:main',
        ],
    },
)
