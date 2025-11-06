from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_rover = get_package_share_directory('rover')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Use the maze world from turtlebot3_gazebo
    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'maze.world')
    xacro_path = os.path.join(pkg_rover, 'urdf', 'rover.urdf.xacro')

    # 🧩 Process the XACRO file into URDF XML
    doc = xacro.process_file(xacro_path)
    robot_description = doc.toxml()

    # ✅ Save URDF for debugging (optional)
    urdf_path = os.path.join(pkg_rover, 'urdf', 'rover_expanded.urdf')
    with open(urdf_path, 'w') as f:
        f.write(robot_description)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rover',
                       '-topic', 'robot_description',
                       '-x', '0.0',
                       '-y', '0.0',
                       '-z', '0.1'],  # Spawn slightly above ground
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': True,
                         'robot_description': robot_description}],
        ),
    ])
