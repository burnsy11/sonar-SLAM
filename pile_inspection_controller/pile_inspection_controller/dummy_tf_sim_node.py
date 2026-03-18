import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class DummyTfSimNode(Node):
    def __init__(self) -> None:
        super().__init__('dummy_tf_sim')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.poi_frame = self.declare_parameter('poi_frame', 'poi').value
        self.cmd_topic = self.declare_parameter('cmd_topic', 'cmd_vel').value

        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 50.0).value)
        self.cmd_timeout_s = float(self.declare_parameter('cmd_timeout_s', 0.5).value)

        self.poi_x = float(self.declare_parameter('poi_x', 2.0).value)
        self.poi_y = float(self.declare_parameter('poi_y', 0.0).value)
        self.poi_z = float(self.declare_parameter('poi_z', 0.0).value)

        self.x = float(self.declare_parameter('initial_x', 0.0).value)
        self.y = float(self.declare_parameter('initial_y', 0.0).value)
        self.z = float(self.declare_parameter('initial_z', 0.0).value)
        self.yaw = math.radians(float(self.declare_parameter('initial_yaw_deg', 0.0).value))

        self.latest_cmd = Twist()
        self.last_cmd_time_s: Optional[float] = None
        self.last_loop_time_s = self._now_s()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_topic, self._cmd_cb, 10)

        dt = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(dt, self._on_timer)

        self.get_logger().info('dummy_tf_sim started')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _yaw_to_quat(yaw: float):
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time_s = self._now_s()

    def _active_cmd(self, now_s: float) -> Twist:
        if self.last_cmd_time_s is None:
            return Twist()
        if now_s - self.last_cmd_time_s > self.cmd_timeout_s:
            return Twist()
        return self.latest_cmd

    def _integrate(self, cmd: Twist, dt: float) -> None:
        # Convert commanded body-frame velocities to map-frame motion.
        vx_b = cmd.linear.x
        vy_b = cmd.linear.y
        vz_b = cmd.linear.z
        wz = cmd.angular.z

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        vx_m = cos_yaw * vx_b - sin_yaw * vy_b
        vy_m = sin_yaw * vx_b + cos_yaw * vy_b

        self.x += vx_m * dt
        self.y += vy_m * dt
        self.z += vz_b * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

    def _publish_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()

        tf_map_base = TransformStamped()
        tf_map_base.header.stamp = stamp
        tf_map_base.header.frame_id = self.map_frame
        tf_map_base.child_frame_id = self.base_frame
        tf_map_base.transform.translation.x = self.x
        tf_map_base.transform.translation.y = self.y
        tf_map_base.transform.translation.z = self.z
        qx, qy, qz, qw = self._yaw_to_quat(self.yaw)
        tf_map_base.transform.rotation.x = qx
        tf_map_base.transform.rotation.y = qy
        tf_map_base.transform.rotation.z = qz
        tf_map_base.transform.rotation.w = qw

        tf_map_poi = TransformStamped()
        tf_map_poi.header.stamp = stamp
        tf_map_poi.header.frame_id = self.map_frame
        tf_map_poi.child_frame_id = self.poi_frame
        tf_map_poi.transform.translation.x = self.poi_x
        tf_map_poi.transform.translation.y = self.poi_y
        tf_map_poi.transform.translation.z = self.poi_z
        tf_map_poi.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([tf_map_base, tf_map_poi])

    def _on_timer(self) -> None:
        now_s = self._now_s()
        dt = max(now_s - self.last_loop_time_s, 1e-4)
        self.last_loop_time_s = now_s

        cmd = self._active_cmd(now_s)
        self._integrate(cmd, dt)
        self._publish_transforms()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyTfSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
