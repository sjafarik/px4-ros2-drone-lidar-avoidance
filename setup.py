from setuptools import setup
from glob import glob
import os

package_name = 'drone_lidar_avoidance_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
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
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saeed Jafari Kang',
    maintainer_email='sjafarik@mtu.edu',
    description='ROS 2 + PX4 LiDAR-based obstacle avoidance drone project in Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = drone_lidar_avoidance_py.offboard_control:main',
            'mission_manager = drone_lidar_avoidance_py.mission_manager:main',
            'lidar_processor = drone_lidar_avoidance_py.lidar_processor:main',
            'obstacle_detector = drone_lidar_avoidance_py.obstacle_detector:main',
            'avoidance_manager = drone_lidar_avoidance_py.avoidance_manager:main',
        ],
    },
)