from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab02_pkg',
            namespace='controller1',
            executable='controller',
            name='controller',
            parameters=[PathJoinSubstitution([
                FindPackageShare('lab02_pkg'), 'params', 'params.yaml'
            ])] 
        ),
    ])