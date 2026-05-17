#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_delta_robot'

    return LaunchDescription([
        # Initial Pose Publisher
        Node(
            package='my_delta_robot',
            executable='initial_pose_publisher.py',
            name='initial_pose_publisher',
            output='screen'
        )
    ])
