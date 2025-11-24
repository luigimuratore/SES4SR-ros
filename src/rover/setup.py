from setuptools import setup
import os
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gigi',
    maintainer_email='your@email.com',
    description='Rover control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = rover.controller:main',
            'arduino_bridge = rover.arduino_bridge:main',
            'odometry_publisher = rover.odometry_publisher:main',
            'motor_driver = rover.motor_driver:main',
            'lidar_test = rover.lidar_test:main',
            'imu_node = rover.imu_node:main',            
        ],
    },
)
