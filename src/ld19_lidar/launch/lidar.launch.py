import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ld19_lidar",
                executable="ld19_node",
                name="ld19_node",
                output="screen",
                parameters=[
                    {"port": "/dev/ttyUSB1"},  # Changed from /dev/ld19_lidar
                    {"frame_id": "laser"},
                    {"topic_name": "scan"},
                ],
            ),

                    # Static transform: base_link -> laser
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser_tf',
                arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
            ),
        ]
    )
