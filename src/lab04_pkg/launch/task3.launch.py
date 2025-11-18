from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    task_3 = Node(
        package='lab04_pkg',
        executable='task_3',          
        name='task_3',
        output='screen',
        parameters=[{
            'prediction_rate': 20.0,
            'initial_x': 0.0, 
            'initial_y': 0.77, 
            'initial_theta': 0.0,
            'process_noise_v': 0.1, 
            'process_noise_omega': 0.05,
            'measurement_noise_range': 0.1,
            'measurement_noise_bearing': 0.05,
        }]
    )

    return LaunchDescription([task_3])
