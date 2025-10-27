from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription               # <-- Aggiunto
from launch.launch_description_sources import PythonLaunchDescriptionSource # <-- Aggiunto


def generate_launch_description():

    # 1. Azione per includere l'altro file launch
    include_other_launch = IncludeLaunchDescription( # #
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 
                'launch',
                'lab02.launch.py' # <-- Nome dell'altro launch file
            ])
        ])
    )

    # Azione per il tuo nodo controller (invariata)
    controller_node = Node(
        package='lab02_pkg',
        namespace='controller1',
        executable='controller',
        name='controller',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lab02_pkg'), 'params', 'params.yaml'
        ])] 
    )

    # Ritorna la lista con entrambe le azioni
    return LaunchDescription([
        include_other_launch, # Avvia prima il launch di Gazebo/mondo
        controller_node       # E poi avvia il tuo controller
    ])