from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

    pkg_share = FindPackageShare("turtlebot3_ignition")
    world = PathJoinSubstitution([pkg_share, "worlds", "turtlebot3_world.world"])

    landmark_simulator = Node(
        package="turtlebot3_perception",
        executable="landmark_simulator",
        output="screen",
        parameters=[
            {
                "frequency_hz": 5.0,
                "field_of_view_deg": 70.0,
                "max_range": 3.0,
                "min_range": 0.3,
                "range_stddev": 0.01,
                "bearing_stddev_deg": 1.0,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "world.launch.py"]),
                ),
                launch_arguments={"world": world, "x_pose": "-2.0", "y_pose": "-0.5"}.items(),
            ),
            landmark_simulator,
        ]
    )