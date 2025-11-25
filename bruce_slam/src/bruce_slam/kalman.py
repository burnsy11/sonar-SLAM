# NOTE: This file was auto-converted from ROS1 to ROS2
# Manual review and testing required for:
# - Parameter declarations (declare_parameter before get_parameter)
# - Time conversions may need adjustment
# - Transform broadcasting may need geometry_msgs imports
# - Message filter callbacks may need adjustment

import math
from typing import Optional, Sequence

import gtsam
import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from dvl_msgs.msg import DVL
# from bar30_depth.msg import Depth  # Pressure sensor integration is disabled but kept for reference.
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

try:
	from tf_transformations import euler_from_quaternion  # type: ignore
except Exception:  # pragma: no cover - keeps node running without tf_transformations
	def euler_from_quaternion(quat: Sequence[float]):
		if hasattr(quat, "x"):
			x, y, z, w = quat.x, quat.y, quat.z, quat.w
		else:
			x, y, z, w = quat
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
		return roll_x, pitch_y, yaw_z

# bruce imports
from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import g2r
from bruce_slam.utils.io import loginfo, set_global_logger


DEFAULT_H_DVL = [
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
]
DEFAULT_H_IMU = [
	[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
	[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]


class KalmanNode(Node):
	"""Kalman filter that fuses DVL and IMU measurements."""

	def __init__(self, node_name: str = "kalman") -> None:
		super().__init__(node_name)
		set_global_logger(self.get_logger())

		self.state_vector = np.zeros((12, 1))
		self.cov_matrix = np.eye(12)
		self.pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 0.0, 0.0))
		self.imu_yaw0 = None
		self.imu_offset = 0.0
		self.dt_imu = 0.01
		self.dvl_max_velocity = 0.5

		self.R_dvl = np.eye(3)
		self.R_imu = np.eye(3)
		self.H_dvl = np.array(DEFAULT_H_DVL)
		self.H_imu = np.array(DEFAULT_H_IMU)
		self.A_imu = np.eye(12)
		self.Q = np.eye(12) * 1e-3

		self.odom_pub_kalman = None
		self.tf1: Optional[TransformBroadcaster] = None
		self.imu_sub = None
		self.dvl_sub = None
		# self.depth_sub = None  # Pressure sensor currently disabled.

	def init_node(self, ns: str = "~") -> None:
		"""Initialise parameters, publishers, and subscribers."""

		self._declare_parameters(ns)
		self._load_parameters(ns)

		dvl_qos = QoSProfile(depth=10)
		dvl_qos.reliability = ReliabilityPolicy.BEST_EFFORT
		dvl_qos.history = HistoryPolicy.KEEP_LAST

		imu_qos = QoSProfile(depth=50)
		imu_qos.reliability = ReliabilityPolicy.RELIABLE
		imu_qos.history = HistoryPolicy.KEEP_LAST

		self.dvl_sub = self.create_subscription(DVL, DVL_TOPIC, self.dvl_callback, dvl_qos)

		imu_version = int(self.get_parameter(ns + "imu_version").value)
		imu_topic = IMU_TOPIC_MK_II if imu_version == 2 else IMU_TOPIC
		self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, imu_qos)

		# self.depth_sub = self.create_subscription(Depth, DEPTH_TOPIC, self.pressure_callback, 10)

		self.odom_pub_kalman = self.create_publisher(Odometry, LOCALIZATION_ODOM_TOPIC, 10)
		self.tf1 = TransformBroadcaster(self)

		loginfo("Kalman Node initialised (IMU + DVL).")

	# def pressure_callback(self, depth_msg: Depth) -> None:
	#     """Handle the Kalman Filter using the Depth measurements."""
	#     depth = np.array([[depth_msg.depth], [0.0], [0.0]])
	#     self.state_vector, self.cov_matrix = self.kalman_correct(
	#         self.state_vector, self.cov_matrix, depth, self.H_depth, self.R_depth
	#     )

	def dvl_callback(self, dvl_msg: DVL) -> None:
		dvl_measurement = np.array(
			[[dvl_msg.velocity.x], [dvl_msg.velocity.y], [dvl_msg.velocity.z]]
		)

		if np.any(np.abs(dvl_measurement) > self.dvl_max_velocity):
			return

		self.state_vector, self.cov_matrix = self.kalman_correct(
			self.state_vector, self.cov_matrix, dvl_measurement, self.H_dvl, self.R_dvl
		)

	def imu_callback(self, imu_msg: Imu) -> None:
		predicted_x, predicted_P = self.kalman_predict(self.state_vector, self.cov_matrix, self.A_imu)

		roll_x, pitch_y, yaw_z = euler_from_quaternion(
			(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
		)
		euler_angle = np.array([[self.imu_offset + roll_x], [pitch_y], [yaw_z]])

		if self.imu_yaw0 is None:
			self.imu_yaw0 = yaw_z
		euler_angle[2] -= self.imu_yaw0

		self.state_vector, self.cov_matrix = self.kalman_correct(
			predicted_x, predicted_P, euler_angle, self.H_imu, self.R_imu
		)

		trans_x = self.state_vector[6][0] * self.dt_imu
		trans_y = self.state_vector[7][0] * self.dt_imu
		local_point = gtsam.Point2(trans_x, trans_y)

		R = gtsam.Rot3.Ypr(
			self.state_vector[5][0], self.state_vector[4][0], self.state_vector[3][0]
		)
		pose2 = gtsam.Pose2(self.pose.x(), self.pose.y(), self.pose.rotation().yaw())
		point = pose2.transformFrom(local_point)
		self.pose = gtsam.Pose3(R, gtsam.Point3(point[0], point[1], 0.0))

		self.send_odometry(imu_msg.header.stamp)

	def kalman_predict(self, previous_x: np.ndarray, previous_P: np.ndarray, A: np.ndarray):
		predicted_P = A @ previous_P @ A.T + self.Q
		predicted_x = A @ previous_x
		return predicted_x, predicted_P

	def kalman_correct(
		self,
		predicted_x: np.ndarray,
		predicted_P: np.ndarray,
		z: np.ndarray,
		H: np.ndarray,
		R: np.ndarray,
	):
		S = H @ predicted_P @ H.T + R
		K = predicted_P @ H.T @ np.linalg.inv(S)
		corrected_x = predicted_x + K @ (z - H @ predicted_x)
		corrected_P = predicted_P - K @ H @ predicted_P
		return corrected_x, corrected_P

	def send_odometry(self, stamp) -> None:
		header = Header()
		header.stamp = stamp
		header.frame_id = "odom"

		odom_msg = Odometry()
		odom_msg.header = header
		odom_msg.pose.pose = g2r(self.pose)
		odom_msg.child_frame_id = "base_link"
		odom_msg.twist.twist.linear.x = 0.0
		odom_msg.twist.twist.linear.y = 0.0
		odom_msg.twist.twist.linear.z = 0.0
		odom_msg.twist.twist.angular.x = 0.0
		odom_msg.twist.twist.angular.y = 0.0
		odom_msg.twist.twist.angular.z = 0.0
		self.odom_pub_kalman.publish(odom_msg)

		if self.tf1 is None:
			return

		ts = TransformStamped()
		ts.header = header
		ts.child_frame_id = "base_link"
		ts.transform.translation.x = odom_msg.pose.pose.position.x
		ts.transform.translation.y = odom_msg.pose.pose.position.y
		ts.transform.translation.z = odom_msg.pose.pose.position.z
		ts.transform.rotation = odom_msg.pose.pose.orientation
		self.tf1.sendTransform(ts)

	def _declare_parameters(self, ns: str) -> None:
		self.declare_parameter(ns + "state_vector", [0.0] * 12)
		self.declare_parameter(ns + "cov_matrix", np.eye(12).flatten().tolist())
		self.declare_parameter(ns + "R_dvl", np.eye(3).flatten().tolist())
		self.declare_parameter(ns + "dt_dvl", 0.1)
		self.declare_parameter(ns + "H_dvl", sum(DEFAULT_H_DVL, []))
		self.declare_parameter(ns + "R_imu", np.eye(3).flatten().tolist())
		self.declare_parameter(ns + "dt_imu", 0.01)
		self.declare_parameter(ns + "H_imu", sum(DEFAULT_H_IMU, []))
		self.declare_parameter(ns + "Q", (np.eye(12) * 1e-3).flatten().tolist())
		self.declare_parameter(ns + "A_imu", np.eye(12).flatten().tolist())
		self.declare_parameter(ns + "dvl_max_velocity", 0.5)
		self.declare_parameter(ns + "imu_offset", 0.0)
		self.declare_parameter(ns + "imu_version", 1)
		# self.declare_parameter(ns + "H_depth", [1.0, 0.0, 0.0])
		# self.declare_parameter(ns + "R_depth", [0.01])

	def _load_parameters(self, ns: str) -> None:
		self.state_vector = self._vector_param(ns + "state_vector", 12)
		cov = self._matrix_param(ns + "cov_matrix", 12, 12)
		self.cov_matrix = cov

		self.R_dvl = self._matrix_param(ns + "R_dvl", 3, 3)
		self.dt_dvl = float(self.get_parameter(ns + "dt_dvl").value)
		self.H_dvl = self._matrix_param(ns + "H_dvl", 3, 12)

		self.R_imu = self._matrix_param(ns + "R_imu", 3, 3)
		self.dt_imu = float(self.get_parameter(ns + "dt_imu").value)
		self.H_imu = self._matrix_param(ns + "H_imu", 3, 12)

		self.Q = self._matrix_param(ns + "Q", 12, 12)
		self.A_imu = self._matrix_param(ns + "A_imu", 12, 12)

		self.dvl_max_velocity = float(self.get_parameter(ns + "dvl_max_velocity").value)
		# self.imu_offset = float(self.get_parameter(ns + "imu_offset").value)
		self.imu_offset = np.pi / 2

	def _vector_param(self, name: str, length: int) -> np.ndarray:
		raw = np.array(self.get_parameter(name).value, dtype=float).reshape(-1)
		if raw.size != length:
			self.get_logger().warning(
				f"Parameter '{name}' expected {length} elements but received {raw.size}. Using resized vector."
			)
			vec = np.zeros(length)
			vec[: min(length, raw.size)] = raw[: min(length, raw.size)]
			return vec.reshape((length, 1))
		return raw.reshape((length, 1))

	def _matrix_param(self, name: str, rows: int, cols: int) -> np.ndarray:
		raw = np.array(self.get_parameter(name).value, dtype=float).reshape(-1)
		expected = rows * cols
		if raw.size != expected:
			self.get_logger().warning(
				f"Parameter '{name}' expected {expected} elements but received {raw.size}. Using resized matrix."
			)
			mat = np.zeros((rows * cols))
			mat[: min(expected, raw.size)] = raw[: min(expected, raw.size)]
			return mat.reshape((rows, cols))
		return raw.reshape((rows, cols))
