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
        # Include LiDAR launch file with custom port parameter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ld19_launch_dir, 'lidar.launch.py')
            ),
            launch_arguments={
                'port': '/dev/ttyUSB1',      # Override port here!
                'frame_id': 'laser',
                'topic_name': 'scan',
            }.items()
        ),

        # ODOM node (publishes to /odom_encoder and /odometry/fused)
        Node(
            package='rover',
            executable='odom_node',
            name='odom_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',     # Arduino
                'baud_rate': 115200,
                'wheel_radius': 0.065,
                'wheel_base': 0.30,
                'ticks_per_rev': 1600,
                'min_velocity_threshold': 0.005,   # 5mm/s
                'min_encoder_ticks': 2,            # Must move at least 2 ticks
                'min_gyro_threshold': 0.05,        # 2.9°/s - ignore below this
            }],
            output='screen'
        ),
                
        # Controller (subscribes to /odometry/fused)
        Node(
            package='rover',
            executable='controller',
            name='controller',
            parameters=[{
                'max_speed': 0.14,
                'max_turn_rate': 0.85,
                'obstacle_threshold': 0.45,
                'is_active': True,
                'turn_tolerance': 0.05,       # 2.9°
                'min_turn_rate': 0.8,     # Prevents stalling
            }],
            output='screen'
        ),
    ])