#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot3_ignition_dir = get_package_share_directory("turtlebot3_ignition")
    launch_file_dir = os.path.join(turtlebot3_ignition_dir, "launch")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    default_world = os.path.join(turtlebot3_ignition_dir, "worlds", "empty_world.world")
    world = LaunchConfiguration("world")
    declare_arg_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="""
            SDF world file. Can be one of the worlds included in the turtlebot3_ignition package or a custom world file. 
            Eg.: world:=turtlebot3_ignition/worlds/turtlebot3_world.world OR world:=/path/to/custom/world.world
        """,
    )

    declare_arg_x_pose = DeclareLaunchArgument(
        "x_pose",
        default_value="-2.0",
        description="Initial x position of the robot in the world frame",
    )

    declare_arg_y_pose = DeclareLaunchArgument(
        "y_pose",
        default_value="-0.5",
        description="Initial y position of the robot in the world frame",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v4 ", world], "on_exit_shutdown": "true"}.items(),
    )

    gui_config = os.path.join(turtlebot3_ignition_dir, "config", "gui", "gui.config")
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-g -v4 --gui-config ", gui_config]}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")),
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_arg_world)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
