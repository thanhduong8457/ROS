#!/usr/bin/env python3
"""Compatibility wrapper for visualization without the operator GUI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("my_delta_robot")
    bringup_launch = os.path.join(package_share, "launch", "bringup.launch.py")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    "gui": "false",
                    "rviz": "true",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
