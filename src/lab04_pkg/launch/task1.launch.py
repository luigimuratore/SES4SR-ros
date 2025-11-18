from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    task_1 = Node(
        package='lab04_pkg',
        executable='task_1',          
        name='task_1',
        output='screen',
        parameters=[{
            'prediction_rate': 20.0,
            'initial_x': -2.5, # real position x of the robot at start -2.0
            'initial_y': -0.0, # real position y of the robot at start -0.5
            'initial_theta': 0.0,
            'process_noise_v': 0.1,
            'process_noise_omega': 0.05,
            'measurement_noise_range': 0.1,
            'measurement_noise_bearing': 0.05,
        }]
    )

    return LaunchDescription([task_1])
