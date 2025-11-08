from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_node = Node(
        package='lab03_pkg',
        namespace='controller1',
        executable='controller',
        name='controller',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lab03_pkg'), 'params', 'params.yaml'
        ])] 
    )

    return LaunchDescription([
        controller_node       # launch controller node
    ])