#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("test_node_node")  # Note: Different from file name
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"hello: {self.counter_}")
        self.counter_ += 1


def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)

    # create a node
    node = MyNode()

    # keep the node alive until you kill it
    rclpy.spin(node)

    # shutdown ROS2 communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
