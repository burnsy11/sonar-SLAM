#!/usr/bin/env python3

# python imports
import rclpy

# pull in the dead reckoning code
from bruce_slam.utils.io import *
from bruce_slam.dead_reckoning import DeadReckoningNode


if __name__ == "__main__":
    rclpy.init()

    node = DeadReckoningNode('dead_reckoning')
    node.init_node()

    args, _ = common_parser().parse_known_args()
    if not args.file:
        loginfo("Start online localization...")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    else:
        loginfo("Start offline localization...")
        offline(args)
        node.destroy_node()
        rclpy.shutdown()
