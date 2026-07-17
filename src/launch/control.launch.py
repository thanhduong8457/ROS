#!/usr/bin/env python3
"""Launch the delta robot visualization and operator control panel."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("my_delta_robot")
    display_launch = os.path.join(package_share, "launch", "display.launch.py")

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(display_launch)),
            Node(
                package="my_delta_robot",
                executable="gui_user_interface_node.py",
                name="gui_user_interface_node",
                output="screen",
            ),
        ]
    )
