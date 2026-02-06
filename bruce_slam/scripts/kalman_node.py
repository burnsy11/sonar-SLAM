#!/usr/bin/env python3
import rclpy
from bruce_slam.utils.io import *
from bruce_slam.kalman import KalmanNode

if __name__ == "__main__":
    rclpy.init()

    node = KalmanNode('kalman_filter')
    node.init_node()

    args, _ = common_parser().parse_known_args()
    if not args.file:
        loginfo("Start online kalman filter...")
        rclpy.spin(node)
    else:
        loginfo("Start offline kalman filter...")
        offline(args)

    node.destroy_node()
    rclpy.shutdown()
