#!/usr/bin/env python3
import rclpy
from bruce_slam.utils.io import *
from bruce_slam.gyro import GyroFilter

if __name__ == "__main__":
    rclpy.init()

    node = GyroFilter('gyro_filter')
    node.init_node()

    args, _ = common_parser().parse_known_args()
    if not args.file:
        loginfo("Start gyro_fusion...")
        rclpy.spin(node)
    else:
        loginfo("Start offline gyro_fusion...")
        # offline mode not implemented for gyro

    node.destroy_node()
    rclpy.shutdown()
