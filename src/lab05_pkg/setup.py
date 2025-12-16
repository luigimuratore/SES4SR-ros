from setuptools import find_packages, setup

package_name = 'lab05_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='gigiomuratore@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_1 = lab05_pkg.task_1:main',
            'task_2 = lab05_pkg.task_2:main',
            'task_2_metrics = lab05_pkg.task_2_metrics:main',
            'task_3 = lab05_pkg.task_3:main',
            'task_3_original = lab05_pkg.task_3_original:main',
        ],
    },
)
