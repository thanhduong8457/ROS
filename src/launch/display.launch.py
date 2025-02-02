from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'my_delta_robot'
    
    # Get URDF file path
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        '../../../../src/urdf',
        'delta_robot.urdf'  # Change to delta_robot.urdf.xacro if needed
    )
    
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
            arguments=['-d', os.path.join(get_package_share_directory(package_name), 
                '../../../../src/rviz', 
                'my_config.rviz'
            )]
        ),
        # Initial Pose Publisher
        Node(
            package='my_delta_robot',
            executable='initial_pose_publisher',
            name='initial_pose_publisher'
        ),
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
