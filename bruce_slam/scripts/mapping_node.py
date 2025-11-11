# NOTE: This file was auto-converted from ROS1 to ROS2
# Manual review and testing required for:
# - Parameter declarations (declare_parameter before get_parameter)
# - Time conversions may need adjustment
# - Transform broadcasting may need geometry_msgs imports
# - Message filter callbacks may need adjustment

#!/usr/bin/env python
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from bruce_msgs.srv import GetOccupancyMap, GetOccupancyMapResponse

from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.io import *
from bruce_slam.mapping import Mapping


class MappingNode(Node, Mapping):
    def __init__(self, node_name='mapping'):
        Node.__init__(self, node_name)
        Mapping.__init__(self)
        set_global_logger(self.get_logger())

        self.lock = threading.RLock()
        self.use_slam_traj = True

    def init_node(self, ns="~"):
        self.use_slam_traj = self.get_parameter(ns + "use_slam_traj", True).value  # CHECK: declare_parameter needed first

        self.x0, self.y0 = self.get_parameter(ns + "origin").value  # CHECK: declare_parameter needed first
        self.width, self.height = self.get_parameter(ns + "size").value  # CHECK: declare_parameter needed first
        self.resolution = self.get_parameter(ns + "resolution").value  # CHECK: declare_parameter needed first
        self.inc = self.get_parameter(ns + "inc").value  # CHECK: declare_parameter needed first

        self.pub_occupancy1 = self.get_parameter(ns + "pub_occupancy1").value  # CHECK: declare_parameter needed first
        self.hit_prob = self.get_parameter(ns + "hit_prob").value  # CHECK: declare_parameter needed first
        self.miss_prob = self.get_parameter(ns + "miss_prob").value  # CHECK: declare_parameter needed first
        self.inflation_angle = self.get_parameter(ns + "inflation_angle").value  # CHECK: declare_parameter needed first
        self.inflation_radius = self.get_parameter(ns + "inflation_range").value  # CHECK: declare_parameter needed first

        self.pub_occupancy2 = self.get_parameter(ns + "pub_occupancy2").value  # CHECK: declare_parameter needed first
        self.inflation_radius = self.get_parameter(ns + "inflation_radius").value  # CHECK: declare_parameter needed first
        self.outlier_filter_radius = self.get_parameter(ns + "outlier_filter_radius").value  # CHECK: declare_parameter needed first
        self.outlier_filter_min_points = self.get_parameter(
            ns + "outlier_filter_min_points"
        ).value  # CHECK: declare_parameter needed first

        self.pub_intensity = self.get_parameter(ns + "pub_intensity").value  # CHECK: declare_parameter needed first

        # Only update keyframe that has significant movement
        self.min_translation = self.get_parameter(ns + "min_translation").value  # CHECK: declare_parameter needed first
        self.min_rotation = self.get_parameter(ns + "min_rotation").value  # CHECK: declare_parameter needed first

        self.sonar_sub = Subscriber(self, SONAR_TOPIC, Ping)
        if self.use_slam_traj:
            self.traj_sub = Subscriber(SLAM_TRAJ_TOPIC, PointCloud2)
        else:
            self.traj_sub = Subscriber(LOCALIZATION_TRAJ_TOPIC, PointCloud2)
        # Method 1
        if self.pub_occupancy1:
            self.feature_sub = Subscriber(SONAR_FEATURE_TOPIC, PointCloud2)
        # Method 2
        if self.pub_occupancy2:
            self.feature_sub = Subscriber(SLAM_CLOUD_TOPIC, PointCloud2)

        # The time stamps for trajectory and ping have to be exactly the same
        # A big queue_size is required to assure no keyframe is missed especially
        # for offline playing.
        self.ts = TimeSynchronizer(
            [self.traj_sub, self.sonar_sub, self.feature_sub], 100
        )
        self.ts.registerCallback(self.tpf_callback)

        self.intensity_map_pub = self.create_publisher(OccupancyGrid, MAPPING_INTENSITY_TOPIC, 1)
        self.occupancy_map_pub = self.create_publisher(OccupancyGrid, MAPPING_OCCUPANCY_TOPIC, 1)

        self.get_map_srv = self.create_service(GetOccupancyMap, ns + "get_map", self.get_map)

        self.configure()
        loginfo("Mapping node is initialized")

    def get_map(self, req):
        resp = GetOccupancyMapResponse()
        with self.lock:
            occ_msg = self.get_occupancy_grid(req.frames, req.resolution)
            resp.occ = occ_msg

        return resp

    @add_lock
    def tpf_callback(self, traj_msg, ping, feature_msg):
        self.lock.acquire()
        with CodeTimer("Mapping - add keyframe"):
            traj = r2n(traj_msg)
            pose = pose322(n2g(traj[-1, :6], "Pose3"))
            points = r2n(feature_msg)
            self.add_keyframe(len(traj) - 1, pose, ping, points)

        with CodeTimer("Mapping - update keyframe"):
            for x in range(len(traj) - 1):
                pose = pose322(n2g(traj[x, :6], "Pose3"))
                self.update_pose(x, pose)

        if self.pub_intensity:
            with CodeTimer("Mapping - publish intensity map"):
                intensity_msg = self.get_intensity_grid()
                intensity_msg.header.stamp = ping.header.stamp
                if not self.use_slam_traj:
                    intensity_msg.header.frame_id = "odom"
                self.intensity_map_pub.publish(intensity_msg)

        if self.pub_occupancy1:
            with CodeTimer("Mapping - publish occupancy map"):
                occupancy_msg = self.get_occupancy_grid1()
                occupancy_msg.header.stamp = ping.header.stamp
                if not self.use_slam_traj:
                    occupancy_msg.header.frame_id = "odom"
                self.occupancy_map_pub.publish(occupancy_msg)

        if self.pub_occupancy2:
            with CodeTimer("Mapping - publish occupancy map"):
                occupancy_msg = self.get_occupancy_grid2()
                occupancy_msg.header.stamp = ping.header.stamp
                if not self.use_slam_traj:
                    occupancy_msg.header.frame_id = "odom"
                self.occupancy_map_pub.publish(occupancy_msg)

        # # Why doesn't this work?
        # # Delete unnecessary ping cached in time synchronizer since we use a big queue.
        # q = self.ts.queues[1]
        # self.ts.queues[1] = {
        #     t: m for t, m in q.items() if t >= ping.header.stamp
        # }
        self.lock.release()
        if self.save_fig:
            self.save_submaps()

    def save_submaps(self):
        submaps = []
        for keyframe in self.keyframes:
            submap = (
                g2n(keyframe.pose),
                keyframe.r,
                keyframe.c,
                keyframe.l,
                keyframe.i,
                keyframe.cimg,
                keyframe.limg,
            )
            submaps.append(submap)
        np.savez(
            "step-{}-submaps.npz".format(len(self.keyframes) - 1),
            submaps=submaps,
            map_size=(self.x0, self.y0, self.width, self.height, self.resolution),
        )


def offline(args):
    from localization_node import LocalizationNode
    from rosgraph_msgs.msg import Clock
    from bruce_slam.utils import io

    io.offline = True

    loc_node = LocalizationNode()
    loc_node.init_node("/bruce/localization/")
    clock_pub = self.create_publisher(Clock, "/clock", 100)
    for topic, msg in read_bag(args.file, args.start, args.duration, progress=True):
        while not not rclpy.ok():
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
        clock_pub.publish(Clock(msg.header.stamp))


if __name__ == "__main__":
    # ROS2: Node initialization moved to class constructor

    node = MappingNode()
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
