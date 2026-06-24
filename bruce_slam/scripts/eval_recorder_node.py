#!/usr/bin/env python3

import math
import pickle
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from bruce_slam.utils.conversions import r2n
from bruce_slam.utils.io import set_global_logger
from bruce_slam.utils.topics import SONAR_FEATURE_TOPIC


class EvalRecorderNode(Node):
    """Record synced raw frontend features and GT pose for offline evaluation."""

    def __init__(self) -> None:
        super().__init__("eval_recorder")
        set_global_logger(self.get_logger())

        self.declare_parameter("output_path", "evaluation/recorded_raw_cfar.pkl")
        self.declare_parameter("queue_size", 20)
        self.declare_parameter("sync_slop", 0.5)
        self.declare_parameter("log_every_n", 100)

        self.output_path = Path(self.get_parameter("output_path").value)
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.records = []

        self.feature_sub = Subscriber(self, PointCloud2, SONAR_FEATURE_TOPIC)
        self.gt_sub = Subscriber(self, PoseStamped, "/oceansim/robot/gt_pose")
        self.sync = ApproximateTimeSynchronizer(
            [self.feature_sub, self.gt_sub],
            self.queue_size,
            self.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.callback)

        self.get_logger().info(
            f"Recording synced features and GT pose to {self.output_path}"
        )

    def callback(self, feature_msg: PointCloud2, gt_msg: PoseStamped) -> None:
        points_bl = np.asarray(r2n(feature_msg), dtype=np.float32)
        if points_bl.size == 0:
            points_bl = np.zeros((0, 3), dtype=np.float32)
        else:
            points_bl = points_bl.reshape((-1, points_bl.shape[-1]))

        # The frontend publishes a single NaN placeholder when frames are skipped.
        if points_bl.size and np.isnan(points_bl).any():
            return

        p = gt_msg.pose.position
        q = gt_msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if len(points_bl):
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            rotation = np.array(
                [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]],
                dtype=np.float32,
            )
            translation = np.array([p.x, p.y], dtype=np.float32)
            points_map = (rotation @ points_bl[:, :2].T).T + translation
        else:
            points_map = np.zeros((0, 2), dtype=np.float32)

        stamp = (
            feature_msg.header.stamp.sec
            + feature_msg.header.stamp.nanosec * 1e-9
        )

        self.records.append(
            {
                "stamp": stamp,
                "gt_pose": np.array([p.x, p.y, yaw], dtype=np.float32),
                "points_base_link": points_bl[:, :2].copy(),
                "points_map": points_map.copy(),
                "n_points": int(points_map.shape[0]),
            }
        )

        if self.log_every_n > 0 and len(self.records) % self.log_every_n == 0:
            self.get_logger().info(f"Recorded {len(self.records)} synced frames")

    def save(self) -> None:
        with self.output_path.open("wb") as handle:
            pickle.dump(self.records, handle, protocol=pickle.HIGHEST_PROTOCOL)
        self.get_logger().info(
            f"Saved {len(self.records)} synced frames to {self.output_path}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvalRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
