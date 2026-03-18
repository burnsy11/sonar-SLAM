import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Phase(Enum):
    APPROACH = 'approach'
    DESCEND = 'descend'
    ASCEND = 'ascend'
    RETURN_HOME = 'return_home'
    DONE = 'done'


@dataclass
class PdState:
    prev_error: float = 0.0
    prev_time_s: Optional[float] = None


class PileInspectionController(Node):
    def __init__(self) -> None:
        super().__init__('pile_inspection_controller')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.poi_frame = self.declare_parameter('poi_frame', 'poi').value
        self.cmd_topic = self.declare_parameter('cmd_topic', 'cmd_vel').value

        self.control_rate_hz = float(self.declare_parameter('control_rate_hz', 20.0).value)

        self.x_target_to_poi = float(self.declare_parameter('x_target_to_poi', 0.4).value)
        self.y_target_to_poi = float(self.declare_parameter('y_target_to_poi', 0.0).value)
        self.initial_yaw_deg = float(self.declare_parameter('initial_yaw_deg', 0.0).value)
        self.yaw_step_deg = float(self.declare_parameter('yaw_step_deg', 10.0).value)

        self.bottom_z_m = float(self.declare_parameter('bottom_z_m', -7.0).value)
        self.top_z_m = float(self.declare_parameter('top_z_m', 0.0).value)
        self.vertical_speed_mps = abs(float(self.declare_parameter('vertical_speed_mps', 0.5).value))

        self.kp_x = float(self.declare_parameter('kp_x', 1.0).value)
        self.kd_x = float(self.declare_parameter('kd_x', 0.2).value)
        self.kp_y = float(self.declare_parameter('kp_y', 1.0).value)
        self.kd_y = float(self.declare_parameter('kd_y', 0.2).value)
        self.kp_yaw = float(self.declare_parameter('kp_yaw', 1.2).value)
        self.kd_yaw = float(self.declare_parameter('kd_yaw', 0.25).value)
        self.kp_home_z = float(self.declare_parameter('kp_home_z', 1.0).value)

        self.max_vx = float(self.declare_parameter('max_vx', 0.8).value)
        self.max_vy = float(self.declare_parameter('max_vy', 0.8).value)
        self.max_vz = float(self.declare_parameter('max_vz', 0.6).value)
        self.max_wz = float(self.declare_parameter('max_wz', 1.2).value)

        self.xy_tolerance_m = float(self.declare_parameter('xy_tolerance_m', 0.05).value)
        self.z_tolerance_m = float(self.declare_parameter('z_tolerance_m', 0.05).value)
        self.yaw_tolerance_rad = math.radians(float(self.declare_parameter('yaw_tolerance_deg', 2.0).value))

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.phase = Phase.APPROACH
        self.target_yaw_deg = self.initial_yaw_deg

        self.pd_x = PdState()
        self.pd_y = PdState()
        self.pd_yaw = PdState()

        dt = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info('pile_inspection_controller started')
        self.get_logger().info(
            f'Initial state: phase={self.phase.value}, '
            f'target_yaw={self.target_yaw_deg:.1f} deg, '
            f'xy_target_to_poi=({self.x_target_to_poi:.2f}, {self.y_target_to_poi:.2f}), '
            f'z_range=[{self.bottom_z_m:.2f}, {self.top_z_m:.2f}]'
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _pd(self, error: float, state: PdState, kp: float, kd: float, now_s: float) -> float:
        derivative = 0.0
        if state.prev_time_s is not None:
            dt = max(now_s - state.prev_time_s, 1e-4)
            derivative = (error - state.prev_error) / dt

        state.prev_error = error
        state.prev_time_s = now_s
        return kp * error + kd * derivative

    def _reset_pd(self) -> None:
        self.pd_x = PdState()
        self.pd_y = PdState()
        self.pd_yaw = PdState()

    def _lookup(self, target_frame: str, source_frame: str):
        return self.tf_buffer.lookup_transform(target_frame, source_frame, Time())

    def _increment_target_yaw(self) -> None:
        self.target_yaw_deg += self.yaw_step_deg
        self.get_logger().info(f'Updated yaw target to {self.target_yaw_deg:.1f} deg')

    def _set_phase(self, new_phase: Phase, reason: str) -> None:
        if self.phase == new_phase:
            return
        old_phase = self.phase
        self.phase = new_phase
        self.get_logger().info(f'Phase transition: {old_phase.value} -> {new_phase.value} ({reason})')

    def _log_tracking_status(self, map_z: float, err_x: float, err_y: float, err_yaw: float) -> None:
        self.get_logger().info(
            f'Tracking [{self.phase.value}]: z={map_z:.2f} m, err_x={err_x:.3f} m, '
            f'err_y={err_y:.3f} m, err_yaw={math.degrees(err_yaw):.1f} deg, '
            f'target_yaw={self.target_yaw_deg:.1f} deg',
            throttle_duration_sec=1.0,
        )

    def _log_return_home_status(self, map_z: float, err_x: float, err_y: float, err_yaw: float, err_z: float) -> None:
        self.get_logger().info(
            f'Return-home: z={map_z:.2f} m, err_x={err_x:.3f} m, err_y={err_y:.3f} m, '
            f'err_z={err_z:.3f} m, err_yaw={math.degrees(err_yaw):.1f} deg',
            throttle_duration_sec=1.0,
        )

    def _target_yaw_rad(self) -> float:
        return math.radians(self.target_yaw_deg)

    def _finished_full_rotation(self) -> bool:
        return self.target_yaw_deg >= 360.0

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def _control_loop(self) -> None:
        now_s = self._now_s()
        cmd = Twist()

        try:
            tf_map_base = self._lookup(self.map_frame, self.base_frame)
            tf_base_poi = self._lookup(self.base_frame, self.poi_frame)
        except TransformException as exc:
            self.get_logger().warn(f'TF unavailable: {exc}', throttle_duration_sec=2.0)
            self._publish_zero()
            return

        map_z = tf_map_base.transform.translation.z
        q = tf_map_base.transform.rotation
        yaw_map_base = self._yaw_from_quat(q.x, q.y, q.z, q.w)

        if self.phase in (Phase.APPROACH, Phase.DESCEND, Phase.ASCEND):
            # Control sign is chosen so positive error commands motion that reduces
            # poi position in base_link toward the target offset.
            err_x = tf_base_poi.transform.translation.x - self.x_target_to_poi
            err_y = tf_base_poi.transform.translation.y - self.y_target_to_poi
            err_yaw = self._normalize_angle(self._target_yaw_rad() - yaw_map_base)

            self._log_tracking_status(map_z, err_x, err_y, err_yaw)

            cmd.linear.x = self._clamp(self._pd(err_x, self.pd_x, self.kp_x, self.kd_x, now_s), -self.max_vx, self.max_vx)
            cmd.linear.y = self._clamp(self._pd(err_y, self.pd_y, self.kp_y, self.kd_y, now_s), -self.max_vy, self.max_vy)
            cmd.angular.z = self._clamp(self._pd(err_yaw, self.pd_yaw, self.kp_yaw, self.kd_yaw, now_s), -self.max_wz, self.max_wz)

            if self.phase == Phase.APPROACH:
                cmd.linear.z = 0.0
                if (
                    abs(err_x) <= self.xy_tolerance_m
                    and abs(err_y) <= self.xy_tolerance_m
                    and abs(err_yaw) <= self.yaw_tolerance_rad
                ):
                    self.get_logger().info(
                        'Approach complete: 2D + yaw controllers are within tolerance. Starting descend.'
                    )
                    self._set_phase(Phase.DESCEND, 'approach objective reached')
                    self._reset_pd()

            elif self.phase == Phase.DESCEND:
                cmd.linear.z = -self.vertical_speed_mps
                if map_z <= self.bottom_z_m + self.z_tolerance_m:
                    cmd.linear.z = 0.0
                    self.get_logger().info(
                        f'Reached bottom threshold: z={map_z:.2f} m '
                        f'(target={self.bottom_z_m:.2f} m). Switching to ascend.'
                    )
                    # self._increment_target_yaw()
                    self._set_phase(Phase.ASCEND, 'bottom reached')
                    self._reset_pd()
            elif self.phase == Phase.ASCEND:
                cmd.linear.z = self.vertical_speed_mps
                if map_z >= self.top_z_m - self.z_tolerance_m:
                    cmd.linear.z = 0.0
                    self.get_logger().info(
                        f'Reached top threshold: z={map_z:.2f} m '
                        f'(target={self.top_z_m:.2f} m). Evaluating next scan step.'
                    )
                    self._increment_target_yaw()
                    if self._finished_full_rotation():
                        self.get_logger().info('Full 360 yaw sweep complete. Returning home pose.')
                        self._set_phase(Phase.RETURN_HOME, 'full yaw sweep complete')
                    else:
                        self._set_phase(Phase.APPROACH, 'continue yaw sweep')
                    self._reset_pd()

        elif self.phase == Phase.RETURN_HOME:
            try:
                tf_base_map = self._lookup(self.base_frame, self.map_frame)
            except TransformException as exc:
                self.get_logger().warn(f'TF unavailable for return-home: {exc}', throttle_duration_sec=2.0)
                self._publish_zero()
                return

            self.target_yaw_deg = 0.0

            err_x = tf_base_map.transform.translation.x
            err_y = tf_base_map.transform.translation.y
            err_yaw = self._normalize_angle(0.0 - yaw_map_base)
            err_z = self.top_z_m - map_z

            self._log_return_home_status(map_z, err_x, err_y, err_yaw, err_z)

            cmd.linear.x = self._clamp(self._pd(err_x, self.pd_x, self.kp_x, self.kd_x, now_s), -self.max_vx, self.max_vx)
            cmd.linear.y = self._clamp(self._pd(err_y, self.pd_y, self.kp_y, self.kd_y, now_s), -self.max_vy, self.max_vy)
            cmd.angular.z = self._clamp(self._pd(err_yaw, self.pd_yaw, self.kp_yaw, self.kd_yaw, now_s), -self.max_wz, self.max_wz)
            cmd.linear.z = self._clamp(self.kp_home_z * err_z, -self.max_vz, self.max_vz)

            if (
                abs(err_x) <= self.xy_tolerance_m
                and abs(err_y) <= self.xy_tolerance_m
                and abs(err_yaw) <= self.yaw_tolerance_rad
                and abs(err_z) <= self.z_tolerance_m
            ):
                self._set_phase(Phase.DONE, 'home pose reached within tolerance')
                self._publish_zero()
                self.get_logger().info('Inspection pattern complete. Holding position.')
                return

        elif self.phase == Phase.DONE:
            self._publish_zero()
            return

        cmd.linear.z = self._clamp(cmd.linear.z, -self.max_vz, self.max_vz)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PileInspectionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
