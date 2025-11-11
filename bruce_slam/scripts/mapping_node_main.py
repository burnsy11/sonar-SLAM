#!/usr/bin/env python3
import rclpy
from bruce_slam.utils.io import *
from bruce_slam.mapping import MappingNode

def offline(args):
    from localization_node import LocalizationNode
    from rosgraph_msgs.msg import Clock
    from bruce_slam.utils import io

    io.offline = True

    loc_node = LocalizationNode('localization_offline')
    loc_node.init_node("/bruce/localization/")
    clock_pub = node.create_publisher(Clock, "/clock", 100)
    
    for topic, msg in read_bag(args.file, args.start, args.duration, progress=True):
        while rclpy.ok():
            if callback_lock_event.wait(1.0):
                break
        if not rclpy.ok():
            break

        if topic == IMU_TOPIC:
            loc_node.imu_sub.callback(msg)
        elif topic == DVL_TOPIC:
            loc_node.dvl_sub.callback(msg)
        elif topic == DEPTH_TOPIC:
            loc_node.depth_sub.callback(msg)
        elif topic == SONAR_TOPIC:
            node.sonar_sub.callback(msg)
        
        clock_msg = Clock()
        clock_msg.clock = msg.header.stamp
        clock_pub.publish(clock_msg)


if __name__ == "__main__":
    rclpy.init()

    node = MappingNode('mapping')
    node.init_node()

    args, _ = common_parser().parse_known_args()
    if not args.file:
        loginfo("Start online mapping...")
        rclpy.spin(node)
    else:
        loginfo("Start offline mapping...")
        offline(args)

    node.destroy_node()
    rclpy.shutdown()
