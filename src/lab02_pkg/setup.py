from setuptools import find_packages, setup
import os           
from glob import glob 

package_name = 'lab02_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),         # Install ament resource index marker for package discovery
        ('share/' + package_name, ['package.xml']),         # Install package.xml metadata file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))), # Install all Python launch files from the local launch/ directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu-gigi',
    maintainer_email='gigiomuratore@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = lab02_pkg.controller:main',
        ],
    },
)