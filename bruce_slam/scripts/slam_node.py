#!/usr/bin/env python3

import rclpy
from bruce_slam.utils.io import *
from bruce_slam.slam_ros import SLAMNode
from bruce_slam.utils.topics import *

def offline(args)->None:
    """run the SLAM system offline

    Args:
        args (Any): the arguments to run the system
    """

    # pull in the extra imports required
    from rosgraph_msgs.msg import Clock
    from dead_reckoning_node import DeadReckoningNode
    from feature_extraction_node import FeatureExtraction
    from mapping_node import MappingNode
    from bruce_slam.utils import io

    # set some params
    io.offline = True
    node.save_fig = False
    node.save_data = False

    # instaciate the nodes required
    dead_reckoning_node = DeadReckoningNode('dead_reckoning_offline')
    dead_reckoning_node.init_node(SLAM_NS + "localization/")
    feature_extraction_node = FeatureExtraction('feature_extraction_offline')
    feature_extraction_node.init_node(SLAM_NS + "feature_extraction/")
    """mp_node = MappingNode()
    mp_node.init_node(SLAM_NS + "mapping/")"""
    clock_pub = node.create_publisher(Clock, "/clock", 100)

    # loop over the entire rosbag
    for topic, msg in read_bag(args.file, args.start, args.duration, progress=True):
        while rclpy.ok():
            if callback_lock_event.wait(1.0):
                break

        if not rclpy.ok():
            break

        if topic == IMU_TOPIC or topic == IMU_TOPIC_MK_II:
            dead_reckoning_node.imu_sub.callback(msg)
        elif topic == DVL_TOPIC:
            dead_reckoning_node.dvl_sub.callback(msg)
        elif topic == DEPTH_TOPIC:
            dead_reckoning_node.depth_sub.callback(msg)
        elif topic == SONAR_TOPIC or SONAR_TOPIC_UNCOMPRESSED:
            feature_extraction_node.sonar_sub.callback(msg)


        # use the IMU to drive the clock
        if topic == IMU_TOPIC or topic == IMU_TOPIC_MK_II:
            clock_msg = Clock()
            clock_msg.clock = msg.header.stamp
            clock_pub.publish(clock_msg)

            # Publish map to world so we can visualize all in a z-down frame in rviz.
            # NOTE: TF broadcasting in ROS2 needs proper transform message construction
            # node.tf.sendTransform((0, 0, 0), [1, 0, 0, 0], msg.header.stamp, "map", "world")
    

if __name__ == "__main__":

    #init the node
    rclpy.init()

    #call the class constructor
    node = SLAMNode('slam')
    node.init_node()

    #parse and start
    args, _ = common_parser().parse_known_args()

    if not args.file:
        loginfo("Start online slam...")
        rclpy.spin(node)
    else:
        loginfo("Start offline slam...")
        offline(args)

    node.destroy_node()
    rclpy.shutdown()
