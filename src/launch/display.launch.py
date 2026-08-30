#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'my_delta_robot'
    
    # Use installed share paths (urdf/ and rviz/ are installed by CMakeLists.txt)
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'delta_robot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'my_config.rviz')

    # Ensure the URDF file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # Read URDF content
    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        # main_node publishes initial joint_states; initial_pose_publisher omitted to avoid duplicate /joint_states
        # draw_node
        Node(
            package='my_delta_robot',
            executable='draw_node',
            name='draw_node'
        ),
        # main_node
        Node(
            package='my_delta_robot',
            executable='main_node',
            name='main_node'
        )
    ])
