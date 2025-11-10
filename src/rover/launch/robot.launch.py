from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld19_launch_dir = os.path.join(
        get_package_share_directory('ld19_lidar'),
        'launch'
    )
    
    return LaunchDescription([
        # Include LiDAR launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ld19_launch_dir, 'lidar.launch.py')
            )
        ),
        
        # Static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # Arduino bridge
        Node(
            package='rover',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud_rate': 115200
            }],
            output='screen'
        ),
        
        # Controller (INACTIVE for manual testing)
        Node(
            package='rover',
            executable='controller',
            name='controller',
            parameters=[{
                'max_speed': 0.20,
                'max_turn_rate': 1.5,
                'is_active': False,  # Inactive - won't send commands
                'use_odometry': False
            }],
            output='screen'
        ),
    ])