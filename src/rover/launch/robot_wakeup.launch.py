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

        # ODOM node (encoder + IMU)
        Node(
            package='rover',
            executable='odom_node',
            name='odom_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'wheel_radius': 0.065,
                'wheel_base': 0.30,
                'ticks_per_rev': 1600
            }],
            output='screen'
        ),
        
        # EKF for sensor fusion (encoder + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 50.0,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                'print_diagnostics': False,  # Less verbose
                
                # Encoder odometry - ONLY linear velocity
                'odom0': '/odom_encoder',
                'odom0_config': [
                    False, False, False,
                    False, False, False,
                    True,  False, False,
                    False, False, False,
                    False, False, False
                ],
                'odom0_differential': False,
                'odom0_relative': False,
                'odom0_queue_size': 10,
                
                # IMU - orientation and angular velocity
                'imu0': '/imu/data_raw',
                'imu0_config': [
                    False, False, False,
                    False, False, True,   # YAW
                    False, False, False,
                    False, False, True,   # VYAW
                    False, False, False
                ],
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_queue_size': 10,
                'imu0_remove_gravitational_acceleration': True,
            }]
        ),
        
        # NO CONTROLLER - Manual control only via /cmd_vel
    ])