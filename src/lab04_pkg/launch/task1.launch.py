from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='lab04_pkg',
            executable='task_1',
            name='task_1',
            output='screen',
            parameters=[{
                'prediction_rate': 20.0,
                'initial_x': 0.0,
                'initial_y': 0.0,
                'initial_theta': 0.0,
                'process_noise_v': 0.1,
                'process_noise_omega': 0.05,
                'measurement_noise_range': 0.1,
                'measurement_noise_bearing': 0.05,
            }]
        )
    ])