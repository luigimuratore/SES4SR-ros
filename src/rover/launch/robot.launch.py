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
        
        # Odometry publisher
        Node(
            package='rover',
            executable='odometry_publisher',
            name='odometry_publisher',
            parameters=[{
                'wheel_radius': 0.065,
                'wheel_base': 0.30,
                'ticks_per_rev': 1600
            }],
            output='screen',
            remappings=[
                ('/odom', '/odom_encoder')  # Rename to avoid conflict with EKF output
            ]
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
        
        # Robot localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                # Fuse encoder odometry (x, y, yaw, vx, vy, vyaw)
                'odom0': '/odom_encoder',
                'odom0_config': [False, False, False,  # x, y, z position
                                False, False, False,   # roll, pitch, yaw orientation
                                True,  True,  False,   # vx, vy, vz velocity
                                False, False, True,    # vroll, vpitch, vyaw
                                False, False, False],  # ax, ay, az acceleration
                
                # Fuse IMU (orientation and angular velocity)
                'imu0': '/imu/data_raw',
                'imu0_config': [False, False, False,   # x, y, z position
                               False, False, True,     # roll, pitch, yaw orientation
                               False, False, False,    # vx, vy, vz velocity
                               False, False, True,     # vroll, vpitch, vyaw
                               False, False, False],   # ax, ay, az acceleration
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