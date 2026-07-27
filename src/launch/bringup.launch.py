#!/usr/bin/env python3
"""Canonical launch entry point for the delta-robot runtime."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "my_delta_robot"
    package_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_share, "urdf", "delta_robot.urdf")
    rviz_config_path = os.path.join(package_share, "rviz", "my_config.rviz")

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    if not os.path.isfile(rviz_config_path):
        raise FileNotFoundError(f"RViz configuration not found: {rviz_config_path}")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    max_velocity_mm_s = ParameterValue(
        LaunchConfiguration("max_velocity_mm_s"), value_type=float
    )
    max_acceleration_mm_s2 = ParameterValue(
        LaunchConfiguration("max_acceleration_mm_s2"), value_type=float
    )
    draw_offset_mm = ParameterValue(
        LaunchConfiguration("draw_offset_mm"), value_type=float
    )
    circle_radius_mm = ParameterValue(
        LaunchConfiguration("circle_radius_mm"), value_type=float
    )
    circle_center_x_mm = ParameterValue(
        LaunchConfiguration("circle_center_x_mm"), value_type=float
    )
    circle_center_y_mm = ParameterValue(
        LaunchConfiguration("circle_center_y_mm"), value_type=float
    )
    circle_base_z_mm = ParameterValue(
        LaunchConfiguration("circle_base_z_mm"), value_type=float
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Tkinter operator control panel",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the package configuration",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "max_velocity_mm_s",
                default_value="5000.0",
                description="Maximum Cartesian velocity in millimetres per second",
            ),
            DeclareLaunchArgument(
                "max_acceleration_mm_s2",
                default_value="100.0",
                description=(
                    "Maximum Cartesian acceleration in millimetres per second squared"
                ),
            ),
            DeclareLaunchArgument(
                "draw_offset_mm",
                default_value="20.0",
                description="Drawing path offset in millimetres",
            ),
            DeclareLaunchArgument(
                "circle_radius_mm",
                default_value="25.0",
                description="Circle drawing radius in millimetres",
            ),
            DeclareLaunchArgument(
                "circle_center_x_mm",
                default_value="-100.0",
                description="Circle center X coordinate in millimetres",
            ),
            DeclareLaunchArgument(
                "circle_center_y_mm",
                default_value="-100.0",
                description="Circle center Y coordinate in millimetres",
            ),
            DeclareLaunchArgument(
                "circle_base_z_mm",
                default_value="-453.0",
                description="Circle base Z coordinate in millimetres",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_link",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "base_link",
                ],
                output="log",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 50.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package=package_name,
                executable="main_node",
                name="main_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "max_velocity_mm_s": max_velocity_mm_s,
                        "max_acceleration_mm_s2": max_acceleration_mm_s2,
                    }
                ],
            ),
            Node(
                package=package_name,
                executable="draw_node",
                name="draw_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "draw_offset_mm": draw_offset_mm,
                        "circle_radius_mm": circle_radius_mm,
                        "circle_center_x_mm": circle_center_x_mm,
                        "circle_center_y_mm": circle_center_y_mm,
                        "circle_base_z_mm": circle_base_z_mm,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(rviz),
            ),
            Node(
                package=package_name,
                executable="gui_user_interface_node.py",
                name="gui_user_interface_node",
                output="screen",
                condition=IfCondition(gui),
            ),
        ]
    )
