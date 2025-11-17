from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch EKF localization node for Lab 4 Task 1.
    """

    ekf_node = Node(
        package='lab04_pkg',
        executable='task1',          
        name='ekf_localization',
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

    return LaunchDescription([ekf_node])
