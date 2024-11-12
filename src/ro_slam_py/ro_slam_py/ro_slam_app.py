#!/usr/bin/env python

import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from ro_slam_py.ro_slam_node import ROSlamNode

def main():
    # Initialize ROS 2 context and node
    rclpy.init(args=sys.argv)
    ro_slam_node = ROSlamNode()

    executor = SingleThreadedExecutor()
    executor.add_node(ro_slam_node)

    # Run the node
    try:
        executor.spin()
    except KeyboardInterrupt:
        ro_slam_node.cleanup()
        ro_slam_node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()
