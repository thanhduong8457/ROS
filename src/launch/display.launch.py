#!/usr/bin/env python3
"""Launch RViz, robot_state_publisher, and motion nodes for the delta robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'my_delta_robot'
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'delta_robot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'my_config.rviz')

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF file not found: {urdf_path}')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        urdf_content = urdf_file.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),

        # Root frame for RViz Fixed Frame and TF tree
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='log',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
                'publish_frequency': 50.0,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='my_delta_robot',
            executable='main_node',
            name='main_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='my_delta_robot',
            executable='draw_node',
            name='draw_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Start RViz after TF and joint_states have time to initialize
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ],
        ),
    ])
