from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    ld = LaunchDescription()

    CAMERA_MODEL = os.environ["CAMERA_MODEL"]

    match CAMERA_MODEL:
        case "realsense":
            # fmt: off
            ld.add_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare("turtlebot3_perception"), "launch", "rs_launch_composable.launch.py",])
                    ),
                    launch_arguments={
                        "rgb_camera.color_profile": "1280x720x6"
                    }.items(),
                )
            )
            ld.add_action(
                Node(
                    name="camera_tf_static",
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=[
                        "--x", "-0.059", "--y", "0.0", "--z", "0.240",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "base_footprint", "--child-frame-id", "camera_link",
                    ],
                    output="screen"
                )
            )
            # fmt: on
        case "oakd":
            # fmt: off
            params_file = PathJoinSubstitution([FindPackageShare("turtlebot3_perception"), "config", "oakd_config.yaml"])
            ld.add_action(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"])
                    ),
                    launch_arguments={
                        "rs_compat": "true",
                        "parent_frame": "base_footprint",
                        "cam_pos_x": "-0.062",
                        "cam_pos_y": "0.0",
                        "cam_pos_z": "0.245",
                        "cam_roll": "0.0",
                        "cam_pitch": "0.0",
                        "cam_yaw": "0.0",
                        "params_file": params_file,
                        "rgb_camera.color_profile": "1280,720,6",
                    }.items(),
                )
            )
            # fmt: on

    return ld
