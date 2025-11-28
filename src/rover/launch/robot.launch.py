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
        
        # Encoder node
        Node(
            package='rover',
            executable='encoder_node',
            name='encoder_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200
            }],
            output='screen'
        ),
        
        # IMU node
        Node(
            package='rover',
            executable='imu_node',
            name='imu_node',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud_rate': 115200
            }],
            output='screen'
        ),
        
        # EKF for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                # Encoder odometry (position + velocity)
                'odom0': '/odom_encoder',
                'odom0_config': [False, False, False,  # x, y, z
                                 False, False, False,  # roll, pitch, yaw
                                 True,  True,  False,  # vx, vy, vz
                                 False, False, True,   # vroll, vpitch, vyaw
                                 False, False, False], # ax, ay, az
                'odom0_differential': False,
                'odom0_relative': False,
                
                # IMU (angular velocity + linear acceleration)
                'imu0': '/imu/data_raw',
                'imu0_config': [False, False, False,  # x, y, z
                                False, False, False,  # roll, pitch, yaw
                                False, False, False,  # vx, vy, vz
                                False, False, True,   # vroll, vpitch, vyaw
                                True,  True,  False], # ax, ay, az
                'imu0_differential': False,
                'imu0_relative': True,
            }]
        ),
                
        # Controller
        Node(
            package='rover',
            executable='controller',
            name='controller',
            parameters=[{
                'max_speed': 0.20,
                'max_turn_rate': 1.0,
                'obstacle_threshold': 0.3,
                'is_active': True
            }],
            output='screen'
        ),
    ])