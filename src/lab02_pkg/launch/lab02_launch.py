from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription              
from launch.launch_description_sources import PythonLaunchDescriptionSource 


def generate_launch_description():

    # 1. Azione per includere l'altro file launch
    gazebo_node = IncludeLaunchDescription( # #
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 
                'launch',
                'lab02.launch.py' # import external launch file
            ])
        ])
    )

    # Our controller node
    controller_node = Node(
        package='lab02_pkg',
        namespace='controller1',
        executable='controller',
        name='controller',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lab02_pkg'), 'params', 'params.yaml'
        ])] 
    )

    return LaunchDescription([
        gazebo_node,            # start gazebo
        controller_node       # start controller
    ])