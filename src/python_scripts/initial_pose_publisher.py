#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.2, self.publish_initial_pose)  # Publish once

    def publish_initial_pose(self):
        self.get_logger().info("publish_initial_pose")
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'base_brazo1',
            'base_brazo2',
            'base_brazo3',
            'codo1_a',
            'codo1_b',
            'codo2_a',
            'codo2_b',
            'codo3_a',
            'codo3_b',
            'act_x',
            'act_y',
            'act_z',
            'gripper'
        ]  # Joint names

        joint_state.position = [
            0.000266315, 
            0.000266533, 
            0.000266315,
            0.9382, 
            7.38599e-08, 
            0.9382,
            -1.32324e-11, 
            0.9382, 
            -7.38454e-08,
            6.15308e-12, 
            3.96538e-08, 
            -0.375,
            0.0
        ]  # Initial angles (radians)

        self.publisher.publish(joint_state)
        self.timer.cancel()  # Stop after publishing once

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
