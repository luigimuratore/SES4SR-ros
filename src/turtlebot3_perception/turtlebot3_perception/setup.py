from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Ambrosio',
    maintainer_email='marco.ambrosio@polito.it',
    description='Enable perception on Turtlebot3 through RealSense D435 and Oak D PRO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection2landmark = turtlebot3_perception.detection2landmark:main',
            'laserscan2lines = turtlebot3_perception.laserscan2lines_node:main',
            'landmark_simulator = turtlebot3_perception.landmark_simulator:main',
        ],
    },
)
