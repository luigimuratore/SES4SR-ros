from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = FindPackageShare("turtlebot3_ignition")
    world = PathJoinSubstitution([pkg_share, "worlds", "maze.world"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "world.launch.py"]),
                ),
                launch_arguments={"world": world, "x_pose": "0.0", "y_pose": "0.0"}.items(),
            )
        ]
    )
