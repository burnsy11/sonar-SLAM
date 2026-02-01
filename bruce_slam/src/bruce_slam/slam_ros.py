# NOTE: This file was auto-converted from ROS1 to ROS2
# Manual review and testing required for:
# - Parameter declarations (declare_parameter before get_parameter)
# - Time conversions may need adjustment
# - Transform broadcasting may need geometry_msgs imports
# - Message filter callbacks may need adjustment

# python imports
import os
import threading
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from message_filters import  Subscriber
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs_py import point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory

# bruce imports
from bruce_slam.utils.io import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.visualization import *
from bruce_slam.slam import SLAM, Keyframe
from bruce_slam import pcl

# Sonar Oculus imports
from oculus_interfaces.msg import Ping


class SLAMNode(Node, SLAM):
    """This class takes the functionality from slam.py and implments it in the ros
    environment. 
    """
    
    def __init__(self, node_name='slam'):
        Node.__init__(self, node_name)
        SLAM.__init__(self)
        set_global_logger(self.get_logger())

        # the threading lock
        self.lock = threading.RLock()

    def init_node(self, ns="~")->None:
        """Configures the SLAM node

        Args:
            ns (str, optional): The namespace of the node. Defaults to "~".
        """

        # Declare parameters with defaults
        self.declare_parameter("keyframe_duration", 1.0)
        self.declare_parameter("keyframe_translation", 3.0)
        self.declare_parameter("keyframe_rotation", 0.5235987755982988)
        self.declare_parameter("enable_slam", True)
        self.declare_parameter("prior_sigmas", [0.1, 0.1, 0.01])
        self.declare_parameter("odom_sigmas", [0.2, 0.2, 0.02])
        self.declare_parameter("icp_odom_sigmas", [0.1, 0.1, 0.01])
        self.declare_parameter("point_resolution", 0.5)
        self.declare_parameter("ssm.enable", True)
        self.declare_parameter("ssm.min_points", 50)
        self.declare_parameter("ssm.max_translation", 3.0)
        self.declare_parameter("ssm.max_rotation", 0.5235987755982988)
        self.declare_parameter("ssm.target_frames", 3)
        self.declare_parameter("nssm.enable", True)
        self.declare_parameter("nssm.min_st_sep", 8)
        self.declare_parameter("nssm.min_points", 50)
        self.declare_parameter("nssm.max_translation", 10.0)
        self.declare_parameter("nssm.max_rotation", 1.0471975511965976)
        self.declare_parameter("nssm.source_frames", 5)
        self.declare_parameter("nssm.cov_samples", 30)
        self.declare_parameter("pcm_queue_size", 5)
        self.declare_parameter("min_pcm", 2)
        self.declare_parameter("icp_config", "config/icp.yaml")

        #keyframe paramters, how often to add them
        self.keyframe_duration = self.get_parameter("keyframe_duration").value  # Keep as float (seconds)
        self.keyframe_translation = self.get_parameter("keyframe_translation").value
        self.keyframe_rotation = self.get_parameter("keyframe_rotation").value

        #SLAM paramter, are we using SLAM or just dead reckoning
        self.enable_slam = self.get_parameter("enable_slam").value
        print("SLAM STATUS: ", self.enable_slam)

        #noise models
        self.prior_sigmas = self.get_parameter("prior_sigmas").value
        self.odom_sigmas = self.get_parameter("odom_sigmas").value
        self.icp_odom_sigmas = self.get_parameter("icp_odom_sigmas").value

        #resultion for map downsampling
        self.point_resolution = self.get_parameter("point_resolution").value

        #sequential scan matching parameters (SSM)
        self.ssm_params.enable = self.get_parameter("ssm.enable").value
        self.ssm_params.min_points = self.get_parameter("ssm.min_points").value
        self.ssm_params.max_translation = self.get_parameter("ssm.max_translation").value
        self.ssm_params.max_rotation = self.get_parameter("ssm.max_rotation").value
        self.ssm_params.target_frames = self.get_parameter("ssm.target_frames").value
        print("SSM: ", self.ssm_params.enable)

        #non sequential scan matching parameters (NSSM) aka loop closures
        self.nssm_params.enable = self.get_parameter("nssm.enable").value
        self.nssm_params.min_st_sep = self.get_parameter("nssm.min_st_sep").value
        self.nssm_params.min_points = self.get_parameter("nssm.min_points").value
        self.nssm_params.max_translation = self.get_parameter("nssm.max_translation").value
        self.nssm_params.max_rotation = self.get_parameter("nssm.max_rotation").value
        self.nssm_params.source_frames = self.get_parameter("nssm.source_frames").value
        self.nssm_params.cov_samples = self.get_parameter("nssm.cov_samples").value
        print("NSSM: ", self.nssm_params.enable)

        #pairwise consistency maximization parameters for loop closure 
        #outliar rejection
        self.pcm_queue_size = self.get_parameter("pcm_queue_size").value
        self.min_pcm = self.get_parameter("min_pcm").value

        #mak delay between an incoming point cloud and dead reckoning
        self.feature_odom_sync_max_delay = 0.5

        #define the subsrcibing topics
        self.feature_sub = Subscriber(self, PointCloud2, SONAR_FEATURE_TOPIC)
        self.odom_sub = Subscriber(self, Odometry, LOCALIZATION_ODOM_TOPIC)

        #define the sync policy
        self.time_sync = ApproximateTimeSynchronizer(
            [self.feature_sub, self.odom_sub], 20, 
            self.feature_odom_sync_max_delay, allow_headerless = False)

        #register the callback in the sync policy
        self.time_sync.registerCallback(self.SLAM_callback)

        #pose publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, SLAM_POSE_TOPIC, 10)

        #dead reckoning topic
        self.odom_pub = self.create_publisher(Odometry, SLAM_ODOM_TOPIC, 10)

        #SLAM trajectory topic
        self.traj_pub = self.create_publisher(PointCloud2, SLAM_TRAJ_TOPIC, 1)

        #constraints between poses
        self.constraint_pub = self.create_publisher(Marker, SLAM_CONSTRAINT_TOPIC, 1)

        #point cloud publisher topic
        self.cloud_pub = self.create_publisher(PointCloud2, SLAM_CLOUD_TOPIC, 1)

        #tf broadcaster to show pose
        self.tf = TransformBroadcaster(self)

        #cv bridge object
        self.CVbridge = CvBridge()

        #get the ICP configuration from the yaml file
        icp_config = self.get_parameter("icp_config").value
        # If icp_config is a relative path, resolve it relative to the package share directory
        if not os.path.isabs(icp_config):
            try:
                package_share = get_package_share_directory('bruce_slam')
                icp_config = os.path.join(package_share, icp_config)
            except Exception:
                pass  # Keep the original path if package lookup fails
        self.icp.loadFromYaml(icp_config)
        
        # define the robot ID this is not used here, extended in multi-robot SLAM
        self.rov_id = ""

        #call the configure function
        self.configure()
        loginfo("SLAM node is initialized")

    @add_lock
    def sonar_callback(self, ping:Ping)->None:
        """Subscribe once to configure Oculus property.
        Assume sonar configuration doesn't change much.

        Args:
            ping (Ping): The sonar message. 
        """
        
        self.oculus.configure(ping)
        self.sonar_sub.unregister()

    @add_lock
    def SLAM_callback(self, feature_msg:PointCloud2, odom_msg:Odometry)->None:
        """SLAM call back. Subscibes to the feature msg point cloud and odom msg
            Handles the whole SLAM system and publishes map, poses and constraints

        Args:
            feature_msg (PointCloud2): the incoming sonar point cloud
            odom_msg (Odometry): the incoming DVL/IMU state estimate
        """
        #aquire the lock 
        self.lock.acquire()

        #get rostime from the point cloud (convert to float seconds for Keyframe)
        stamp = feature_msg.header.stamp
        time = stamp.sec + stamp.nanosec * 1e-9

        #get the dead reckoning pose from the odom msg, GTSAM pose object
        dr_pose3 = r2g(odom_msg.pose.pose)

        #init a new key frame
        frame = Keyframe(False, time, dr_pose3)
        frame.stamp = stamp  # Store original ROS Time for publishing

        #convert the point cloud message to a numpy array of 2D
        points_gen = pc2.read_points(feature_msg, field_names=('x', 'y', 'z'), skip_nans=False)
        points_list = list(points_gen)
        if len(points_list) > 0:
            # Extract x, y, z from structured array
            points = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
            points = np.c_[points[:,0], points[:,1]]  # Use x, y for 2D SLAM
        else:
            points = np.zeros((0, 2), dtype=np.float32)

        # In case feature extraction is skipped in this frame
        if len(points) and np.isnan(points[0, 0]):
            frame.status = False
        else:
            frame.status = self.is_keyframe(frame)

        #set the frames twist
        frame.twist = odom_msg.twist.twist

        #update the keyframe with pose information from dead reckoning
        if self.keyframes:
            dr_odom = self.current_keyframe.dr_pose.between(frame.dr_pose)
            pose = self.current_keyframe.pose.compose(dr_odom)
            frame.update(pose)


        #check frame staus, are we actually adding a keyframe? This is determined based on distance 
        #traveled according to dead reckoning
        if frame.status:

            #add the point cloud to the frame
            frame.points = points

            #perform seqential scan matching
            #if this is the first frame do not
            if not self.keyframes:
                self.add_prior(frame)
            else:
                self.add_sequential_scan_matching(frame)

            #update the factor graph with the new frame
            self.update_factor_graph(frame)

            #if loop closures are enabled
            #nonsequential scan matching is True (a loop closure occured) update graph again
            if self.nssm_params.enable  and self.add_nonsequential_scan_matching():
                self.update_factor_graph()
            
        #update current time step and publish the topics
        self.current_frame = frame
        self.publish_all()
        self.lock.release()

    def publish_all(self)->None:
        """Publish to all ouput topics
            trajectory, contraints, point cloud and the full GTSAM instance
        """
        if not self.keyframes:
            return

        self.publish_pose()
        if self.current_frame.status:
            self.publish_trajectory()
            self.publish_constraint()
            self.publish_point_cloud()

    def publish_pose(self)->None:
        """Append dead reckoning from Localization to SLAM estimate to achieve realtime TF.
        """

        #define a pose with covariance message 
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.current_frame.stamp
        if self.rov_id == "":
            pose_msg.header.frame_id = "map"
        else:
            pose_msg.header.frame_id = self.rov_id + "_map"
        pose_msg.pose.pose = g2r(self.current_frame.pose3)

        cov = 1e-4 * np.identity(6, np.float32)
        # FIXME Use cov in current_frame
        cov[np.ix_((0, 1, 5), (0, 1, 5))] = self.current_keyframe.transf_cov
        pose_msg.pose.covariance = cov.ravel().tolist()
        self.pose_pub.publish(pose_msg)

        o2m = self.current_frame.pose3.compose(self.current_frame.dr_pose3.inverse())
        o2m = g2r(o2m)
        p = o2m.position
        q = o2m.orientation
        
        # ROS2 sendTransform requires a TransformStamped message
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = self.current_frame.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.tf.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.pose.pose = pose_msg.pose.pose
        if self.rov_id == "":
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = self.rov_id + "_base_link"
        odom_msg.twist.twist = self.current_frame.twist
        self.odom_pub.publish(odom_msg)

    def publish_constraint(self)->None:
        """Publish constraints between poses in the factor graph,
        either sequential or non-sequential.
        """

        #define a list of all the constraints
        links = []

        #iterate over all the keframes
        for x, kf in enumerate(self.keyframes[1:], 1):

            #append each SSM factor in green
            p1 = self.keyframes[x - 1].pose3.x(), self.keyframes[x - 1].pose3.y(), self.keyframes[x - 1].dr_pose3.z()
            p2 = self.keyframes[x].pose3.x(), self.keyframes[x].pose3.y(), self.keyframes[x].dr_pose3.z()
            links.append((p1, p2, "green"))

            #loop over all loop closures in this keyframe and append them in red
            for k, _ in self.keyframes[x].constraints:
                p0 = self.keyframes[k].pose3.x(), self.keyframes[k].pose3.y(), self.keyframes[k].dr_pose3.z()
                links.append((p0, p2, "red"))

        #if nothing, do nothing
        if links:

            #conver this list to a series of multi-colored lines and publish
            link_msg = ros_constraints(links)
            link_msg.header.stamp = self.current_keyframe.stamp
            if self.rov_id != "":
                link_msg.header.frame_id = self.rov_id + "_map"
            self.constraint_pub.publish(link_msg)


    def publish_trajectory(self)->None:
        """Publish 3D trajectory as point cloud in [x, y, z, roll, pitch, yaw, index] format.
        """

        #get all the poses from each keyframe
        poses = np.array([g2n(kf.pose3) for kf in self.keyframes])

        #convert to a ros color line
        traj_msg = ros_colorline_trajectory(poses)
        traj_msg.header.stamp = self.current_keyframe.stamp
        if self.rov_id == "":
            traj_msg.header.frame_id = "map"
        else:
            traj_msg.header.frame_id = self.rov_id + "_map"
        self.traj_pub.publish(traj_msg)

    def publish_point_cloud(self)->None:
        """Publish downsampled 3D point cloud with z = 0.
        The last column represents keyframe index at which the point is observed.
        """

        #define an empty array
        all_points = [np.zeros((0, 2), np.float32)]

        #list of keyframe ids
        all_keys = []

        #loop over all the keyframes, register 
        #the point cloud to the orign based on the SLAM estinmate
        for key in range(len(self.keyframes)):

            #parse the pose
            pose = self.keyframes[key].pose

            #get the resgistered point cloud
            transf_points = self.keyframes[key].transf_points

            #append
            all_points.append(transf_points)
            all_keys.append(key * np.ones((len(transf_points), 1)))

        all_points = np.concatenate(all_points)
        all_keys = np.concatenate(all_keys)

        #use PCL to downsample this point cloud
        sampled_points, sampled_keys = pcl.downsample(
            all_points, all_keys, self.point_resolution
        )

        #parse the downsampled cloud into the ros xyzi format
        sampled_xyzi = np.c_[sampled_points, np.zeros_like(sampled_keys), sampled_keys]
        
        #if there are no points return and do nothing
        if len(sampled_xyzi) == 0:
            return

        #convert the point cloud to a ros message and publish
        cloud_msg = n2r(sampled_xyzi, "PointCloudXYZI")
        cloud_msg.header.stamp = self.current_keyframe.stamp
        if self.rov_id == "":
            cloud_msg.header.frame_id = "map"
        else:
            cloud_msg.header.frame_id = self.rov_id + "_map"

        print("Publishing point cloud with ", len(sampled_xyzi), " points.")
        self.cloud_pub.publish(cloud_msg)
