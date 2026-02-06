# NOTE: This file was auto-converted from ROS1 to ROS2
# Manual review and testing required for:
# - Parameter declarations (declare_parameter before get_parameter)
# - Time conversions may need adjustment
# - Transform broadcasting may need geometry_msgs imports
# - Message filter callbacks may need adjustment

# python imports
import tf2_ros
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import gtsam
import numpy as np

# ros-python imports
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Cache, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# import custom messages
from dvl_msgs.msg import DVL
# from bar30_depth.msg import Depth

# bruce imports
from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.io import *
from bruce_slam.utils.visualization import ros_colorline_trajectory

import math
from std_msgs.msg import String, Float32, Header
try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except Exception:
    # Minimal fallback implementations to avoid hard dependency on tf_transformations
    print("tf_transformations not found, using minimal implementations.")
    def euler_from_quaternion(q):
        # Accept either tuple/list or message-like with x,y,z,w
        if hasattr(q, 'x'):
            x, y, z, w = q.x, q.y, q.z, q.w
        else:
            x, y, z, w = q
        # From: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
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

    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

class DeadReckoningNode(Node):
    """A class to support dead reckoning using DVL and IMU readings"""

    def __init__(self, node_name: str = "dead_reckoning") -> None:
        super().__init__(node_name)
        set_global_logger(self.get_logger())

        # Runtime state
        self.pose = None  # vehicle pose (gtsam.Pose3)
        self.prev_time: Time | None = None  # previous reading time (rclpy Time)
        self.prev_vel: np.ndarray | None = None  # previous reading velocity
        self.keyframes = []  # keyframe list

        # Force yaw at origin to be aligned with x axis
        self.imu_yaw0 = None
        self.imu_pose = [0, 0, 0, 0, 0, 0]
        self.imu_rot = None
        self.dvl_max_velocity = 0.5

        # Keyframe thresholds
        self.keyframe_duration = None
        self.keyframe_translation = None
        self.keyframe_rotation = None
        self.dvl_error_timer = 0.0

        # Place holder for multi-robot SLAM
        self.rov_id = ""

        # publishers/subscribers will be created in `init_node`
        self.dvl_sub = None
        self.imu_sub = None
        self.traj_pub = None
        self.odom_pub = None
        self.tf = None

    def init_node(self, ns="~")->None:
        """Init the node, fetch all paramaters from ROS

        Args:
            ns (str, optional): The namespace of the node. Defaults to "~".
        """
        # Parameters for Node (declare defaults to support running without external declarations)
        self.declare_parameter("imu_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("dvl_max_velocity", 0.5)
        self.declare_parameter("keyframe_duration", 0.25)
        self.declare_parameter("keyframe_translation", 0.2)
        self.declare_parameter("keyframe_rotation", 0.01)
        self.declare_parameter("imu_version", 1)

        # Get parameters
        self.imu_pose = self.get_parameter("imu_pose").value
        self.imu_pose = n2g(self.imu_pose, "Pose3")
        self.imu_rot = self.imu_pose.rotation()
        self.dvl_max_velocity = self.get_parameter("dvl_max_velocity").value
        self.keyframe_duration = self.get_parameter("keyframe_duration").value
        self.keyframe_translation = self.get_parameter("keyframe_translation").value
        self.keyframe_rotation = self.get_parameter("keyframe_rotation").value

        # Subscribers and caches
        # Build QoS profiles that match the publishers we expect
        dvl_qos = QoSProfile(depth=10)
        dvl_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        dvl_qos.history = HistoryPolicy.KEEP_LAST

        imu_qos = QoSProfile(depth=10)
        imu_qos.reliability = ReliabilityPolicy.RELIABLE
        imu_qos.history = HistoryPolicy.KEEP_LAST
        # For ROS2 message_filters, the Subscriber signature expects (node, msg_type, topic)
        # Prefer parameters that can override topic names; keep compatibility with testing bag topics
        self.dvl_sub = Subscriber(self, DVL, "/dvl/data", qos_profile=dvl_qos)
        # self.depth_sub = Subscriber(self, DEPTH_TOPIC, Depth)
        # self.depth_cache = Cache(self.depth_sub, 1)

        # select IMU topic based on version param or the testing ZED topic
        imu_version = self.get_parameter("imu_version").value
        if imu_version == 2:
            self.imu_sub = Subscriber(self, Imu, IMU_TOPIC_MK_II, qos_profile=imu_qos)
        else:
            # fall back to configured IMU_TOPIC or the ZED imu for testing
            try:
                self.imu_sub = Subscriber(self, Imu, IMU_TOPIC, qos_profile=imu_qos)
            except Exception:
                self.imu_sub = Subscriber(self, Imu, "/zed/zed_node/imu/data", qos_profile=imu_qos)

        # Use point cloud for visualization
        self.traj_pub = self.create_publisher(PointCloud2, "traj_dead_reck", 10)

        self.odom_pub = self.create_publisher(Odometry, LOCALIZATION_ODOM_TOPIC, 10)

        # define the message synchronizer (IMU + DVL)
        self.ts = ApproximateTimeSynchronizer([self.imu_sub, self.dvl_sub], 200, .1)
        self.ts.registerCallback(self.callback)

        self.tf = TransformBroadcaster(self)

        loginfo("Localization node is initialized")


    def callback(self, imu_msg:Imu, dvl_msg:DVL)->None:
        """Handle the dead reckoning using the VN100 and DVL only. Fuse and publish an odometry message.

        Args:
            imu_msg (Imu): the message from VN100
            dvl_msg (DVL): the message from the DVL
        """
        #get the previous depth message
        # depth_msg = self.depth_cache.getLast()
        #if there is no depth message, then skip this time step
        # if depth_msg is None:
        #     return
        depth = 0.0

        #check the delay between the depth message and the DVL
        # dd_delay = (depth_msg.header.stamp - dvl_msg.header.stamp).to_sec()
        #print(dd_delay)
        # if abs(dd_delay) > 1.0:
        #     logdebug("Missing depth message for {}".format(dd_delay))

        #convert the imu message from msg to gtsam rotation object
        rot = r2g(imu_msg.orientation)
        rot = rot.compose(self.imu_rot.inverse())

        #if we have no yaw yet, set this one as zero
        if self.imu_yaw0 is None:
            self.imu_yaw0 = rot.yaw()

        # Get a rotation matrix
        rot = gtsam.Rot3.Ypr(rot.yaw()-self.imu_yaw0, rot.pitch(), np.radians(90)+rot.roll())

        # parse the DVL message into an array of velocites
        vel = np.array([dvl_msg.velocity.x, dvl_msg.velocity.y, dvl_msg.velocity.z])

        # package the odom message and publish it
        self.send_odometry(vel,rot,dvl_msg.header.stamp,depth)





    def send_odometry(self,vel:np.array,rot:gtsam.Rot3,dvl_time_msg,depth:float)->None:
        """Package the odometry given all the DVL, rotation matrix, and depth

        Args:
            vel (np.array): a numpy array (1D) of the DVL velocities
            rot (gtsam.Rot3): the rotation matrix of the vehicle
            dvl_time_msg (msg): the time stamp for the DVL message
            depth (float): vehicle depth
        """
        dvl_time = Time.from_msg(dvl_time_msg)

        #if the DVL message has any velocity above the max threhold do some error handling
        if np.any(np.abs(vel) > self.dvl_max_velocity):
            if self.pose:

                self.dvl_error_timer += (dvl_time - self.prev_time).nanoseconds / 1e9
                if self.dvl_error_timer > 5.0:
                    logwarn(
                        "DVL velocity ({:.1f}, {:.1f}, {:.1f}) exceeds max velocity {:.1f} for {:.1f} secs.".format(
                            vel[0],
                            vel[1],
                            vel[2],
                            self.dvl_max_velocity,
                            self.dvl_error_timer,
                        )
                    )
                vel = self.prev_vel
            else:
                return
        else:
            self.dvl_error_timer = 0.0

        if self.pose:
            # figure out how far we moved in the body frame using the DVL message
            dt = (dvl_time - self.prev_time).nanoseconds / 1e9
            dv = (vel + self.prev_vel) * 0.5
            trans = dv * dt

            # get a rotation matrix with only roll and pitch
            rotation_flat = gtsam.Rot3.Ypr(0, rot.pitch(), rot.roll())

            # transform our movement to the global frame
            #trans[2] = -trans[2]
            #trans = trans.dot(rotation_flat.matrix())

            # propagate our movement forward using the GTSAM utilities
            local_point = gtsam.Point2(trans[0], trans[1])

            pose2 = gtsam.Pose2(
                self.pose.x(), self.pose.y(), self.pose.rotation().yaw()
            )
            point = pose2.transformFrom(local_point)

            self.pose = gtsam.Pose3(
                rot, gtsam.Point3(point[0], point[1], depth)
            )

        else:
            # init the pose
            self.pose = gtsam.Pose3(rot, gtsam.Point3(0, 0, depth))

        # log the this timesteps messages for next time
        self.prev_time = dvl_time
        self.prev_vel = vel

        new_keyframe = False
        if not self.keyframes:
            new_keyframe = True
        else:
            current_time_sec = self.prev_time.nanoseconds / 1e9
            duration = current_time_sec - self.keyframes[-1][0]
            if duration > self.keyframe_duration:
                odom = self.keyframes[-1][1].between(self.pose)
                odom = g2n(odom)
                translation = np.linalg.norm(odom[:3])
                rotation = abs(odom[-1])

                if (
                    translation > self.keyframe_translation
                    or rotation > self.keyframe_rotation
                ):
                    new_keyframe = True

        if new_keyframe:
            current_time_sec = self.prev_time.nanoseconds / 1e9
            self.keyframes.append((current_time_sec, self.pose))
        self.publish_pose(new_keyframe)


    def publish_pose(self, publish_traj:bool=False)->None:
        """Publish the pose

        Args:
            publish_traj (bool, optional): Are we publishing the whole set of keyframes?. Defaults to False.

        """
        if self.pose is None:
            return

        header = Header()
        header.stamp = self.prev_time.to_msg()
        header.frame_id = "odom"

        odom_msg = Odometry()
        odom_msg.header = header
        # pose in odom frame
        odom_msg.pose.pose = g2r(self.pose)
        # twist in local frame
        odom_msg.child_frame_id = "base_link"
        # Local planer behaves worse
        # odom_msg.twist.twist.linear.x = self.prev_vel[0]
        # odom_msg.twist.twist.linear.y = self.prev_vel[1]
        # odom_msg.twist.twist.linear.z = self.prev_vel[2]
        # odom_msg.twist.twist.angular.x = self.prev_omega[0]
        # odom_msg.twist.twist.angular.y = self.prev_omega[1]
        # odom_msg.twist.twist.angular.z = self.prev_omega[2]
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom_msg)

        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        ts = TransformStamped()
        ts.header = header
        ts.child_frame_id = "base_link"
        ts.header.frame_id = "odom"
        ts.transform.translation.x = p.x
        ts.transform.translation.y = p.y
        ts.transform.translation.z = p.z
        ts.transform.rotation = q
        self.tf.sendTransform(ts)
        if publish_traj:
            traj = np.array([g2n(pose) for _, pose in self.keyframes])
            traj_msg = ros_colorline_trajectory(traj)
            traj_msg.header = header
            self.traj_pub.publish(traj_msg)
