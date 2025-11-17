from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory

import os
import yaml

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def generate_launch_description():

    package_name = "turtlebot3_perception"

    params = os.path.join(
        get_package_share_directory(package_name), "config", "apriltag.yaml"
    )
    params = yaml_to_dict(params)

    load_composable_nodes = LoadComposableNodes(
        target_container="/camera/camera_container",
        composable_node_descriptions=[
            ComposableNode(
                namespace="camera",
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                remappings=[
                    ("image_rect", "camera/color/image_raw"),
                    ("camera_info", "camera/color/camera_info"),
                ],
                parameters=[params["camera"]["apriltag"]],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    landmark_node = Node(
        namespace="camera",
        package="turtlebot3_perception",
        executable="detection2landmark",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_base_frame": "base_link"}]
    )

    return LaunchDescription([load_composable_nodes, landmark_node])
