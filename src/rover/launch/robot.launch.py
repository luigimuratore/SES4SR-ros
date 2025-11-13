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
        # Include LiDAR launch file (includes base_link -> laser transform)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ld19_launch_dir, 'lidar.launch.py')
            )
        ),
        
        # Arduino bridge
        Node(
            package='rover',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200
            }],
            output='screen'
        ),
        
        # Odometry publisher
        Node(
            package='rover',
            executable='odometry_publisher',
            name='odometry_publisher',
            parameters=[{
                'wheel_radius': 0.065,      # meters (adjust to your wheel size)
                'wheel_base': 0.30,         # meters (distance between left/right wheels)
                'ticks_per_rev': 1600       # encoder ticks per wheel revolution (adjust based on your encoder)
            }],
            output='screen'
        ),
        
        # Controller (INACTIVE by default for safety)
        Node(
            package='rover',
            executable='controller',
            name='controller',
            parameters=[{
                'max_speed': 0.20,
                'max_turn_rate': 1.0,
                'obstacle_threshold': 0.3,
                'is_active': True  # Set to True when ready to run autonomous navigation
            }],
            output='screen'
        ),
    ])