#!/usr/bin/env python3
"""Optional standalone node to publish initial joint_states once.

When using display.launch.py, main_node publishes the initial pose on /joint_states.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'base_brazo1', 'base_brazo2', 'base_brazo3',
    'codo1_a', 'codo1_b', 'codo2_a', 'codo2_b', 'codo3_a', 'codo3_b',
    'act_x', 'act_y', 'act_z',
]

INITIAL_POSITIONS = [
    0.000266315, 0.000266533, 0.000266315,
    0.9382, 7.38599e-08, 0.9382, -1.32324e-11, 0.9382, -7.38454e-08,
    0.0, 0.0, -0.375,
]


def main():
    rclpy.init()
    node = Node('initial_pose_publisher')
    pub = node.create_publisher(JointState, 'joint_states', 10)

    rclpy.spin_once(node, timeout_sec=0.5)

    msg = JointState()
    msg.name = JOINT_NAMES
    msg.position = INITIAL_POSITIONS
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Published initial joint_states')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
