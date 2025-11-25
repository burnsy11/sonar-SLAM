#!/usr/bin/env python3
import rclpy
from bruce_slam.utils.io import *
from bruce_slam.feature_extraction import FeatureExtraction

if __name__ == "__main__":
    rclpy.init()

    #call class constructor
    node = FeatureExtraction('feature_extraction')

    #get args
    parser = common_parser()
    args, _ = parser.parse_known_args()

    # spin the node to handle callbacks
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
