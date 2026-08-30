#!/usr/bin/env python3
"""Optional standalone node to publish initial joint_states once. When using display.launch.py, main_node publishes initial pose instead."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def make_initial_joint_state():
    joint_state = JointState()
    joint_state.name = [
        'base_brazo1', 'base_brazo2', 'base_brazo3',
        'codo1_a', 'codo1_b', 'codo2_a', 'codo2_b', 'codo3_a', 'codo3_b',
        'act_x', 'act_y', 'act_z', 'gripper'
    ]
    joint_state.position = [
        0.000266315, 0.000266533, 0.000266315,
        0.9382, 7.38599e-08, 0.9382, -1.32324e-11, 0.9382, -7.38454e-08,
        6.15308e-12, 3.96538e-08, -0.375, 0.0
    ]
    return joint_state


def main():
    rclpy.init()
    node = Node('initial_pose_publisher')
    pub = node.create_publisher(JointState, 'joint_states', 10)
    # Give time for subscribers to connect
    rclpy.spin_once(node, timeout_sec=0.5)
    joint_state = make_initial_joint_state()
    joint_state.header.stamp = node.get_clock().now().to_msg()
    pub.publish(joint_state)
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
