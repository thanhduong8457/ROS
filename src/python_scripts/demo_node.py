#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.counter = 0
        self.get_logger().info("Hello ROS2")
        self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info("Hello " + str(self.counter))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode("py_test")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    