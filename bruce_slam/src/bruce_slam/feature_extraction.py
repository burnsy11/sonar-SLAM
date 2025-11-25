# NOTE: This file was auto-converted from ROS1 to ROS2
# Manual review and testing required for:
# - Parameter declarations (declare_parameter before get_parameter)
# - Time conversions may need adjustment
# - Transform broadcasting may need geometry_msgs imports
# - Message filter callbacks may need adjustment

#!/usr/bin/env python
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
# ros_numpy replaced with sensor_msgs_py in ROS2

from bruce_slam.utils.io import *
from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.visualization import apply_custom_colormap
#from bruce_slam.feature import FeatureExtraction
from bruce_slam import pcl
import matplotlib.pyplot as plt
from oculus_interfaces.msg import Ping
from scipy.interpolate import interp1d

from .utils import *
from .sonar import *

from bruce_slam.CFAR import CFAR

#from bruce_slam.bruce_slam import sonar

class FeatureExtraction(Node):
    '''Class to handle extracting features from Sonar images using CFAR
    subsribes to the sonar driver and publishes a point cloud
    '''

    def __init__(self, node_name="featureextraction"):
        super().__init__(node_name)
        set_global_logger(self.get_logger())

        '''Class constructor, no args required all read from yaml file
        '''

        #oculus info
        self.oculus = OculusProperty()

        # default parameters for CFAR (defaults mirror config/feature.yaml)
        self.Ntc = 40
        self.Ngc = 10
        self.Pfa = 0.1
        self.rank = 10
        self.alg = "SOCA"
        self.detector = None
        self.threshold = 65
        self.cimg = None

        # default parameters for point cloud / filtering
        self.colormap = "RdBu_r"
        self.pub_rect = True
        self.resolution = 0.5
        self.outlier_filter_radius = 1.0
        self.outlier_filter_min_points = 5
        self.skip = 1

        # for offline visualization
        self.feature_img = None

        # for remapping from polar to cartisian
        self.res = None
        self.height = None
        self.rows = None
        self.width = None
        self.cols = None
        self.map_x = None
        self.map_y = None
        self.f_bearings = None
        self.to_rad = lambda bearing: bearing * np.pi / 18000
        self.REVERSE_Z = 1
        self.maxRange = None

        # which vehicle is being used
        self.compressed_images = False

        # place holder for the multi-robot system
        self.rov_id = ""

        # --- declare parameters (dot notation) with defaults from feature.yaml ---
        # CFAR
        self.declare_parameter('CFAR.Ntc', 40)
        self.declare_parameter('CFAR.Ngc', 10)
        self.declare_parameter('CFAR.Pfa', 0.1)
        self.declare_parameter('CFAR.rank', 10)
        self.declare_parameter('CFAR.alg', 'SOCA')

        # filter
        self.declare_parameter('filter.threshold', 65)
        self.declare_parameter('filter.resolution', 0.5)
        self.declare_parameter('filter.radius', 1.0)
        self.declare_parameter('filter.min_points', 5)
        self.declare_parameter('filter.skip', 1)

        # visualization
        self.declare_parameter('visualization.coordinates', 'cartesian')
        self.declare_parameter('visualization.radius', 2)
        self.declare_parameter('visualization.color', [0, 165, 255])

        # other
        self.declare_parameter('compressed_images', False)

        # --- read parameters into instance variables ---
        self.Ntc = self.get_parameter('CFAR.Ntc').value
        self.Ngc = self.get_parameter('CFAR.Ngc').value
        self.Pfa = self.get_parameter('CFAR.Pfa').value
        self.rank = self.get_parameter('CFAR.rank').value
        self.alg = self.get_parameter('CFAR.alg').value

        self.threshold = self.get_parameter('filter.threshold').value
        self.resolution = self.get_parameter('filter.resolution').value
        self.outlier_filter_radius = self.get_parameter('filter.radius').value
        self.outlier_filter_min_points = self.get_parameter('filter.min_points').value
        self.skip = self.get_parameter('filter.skip').value

        self.coordinates = self.get_parameter('visualization.coordinates').value
        self.radius = self.get_parameter('visualization.radius').value
        self.color = self.get_parameter('visualization.color').value

        self.compressed_images = self.get_parameter('compressed_images').value

        # cv bridge
        self.BridgeInstance = CvBridge()

        # sonar subscription and feature publishers
        self.sonar_sub = self.create_subscription(
            Ping,
            SONAR_TOPIC,
            self.sonar_callback,
            10,
        )

        # feature publish topic
        self.feature_pub = self.create_publisher(PointCloud2, SONAR_FEATURE_TOPIC, 10)

        # vis publish topic
        self.feature_img_pub = self.create_publisher(Image, SONAR_FEATURE_IMG_TOPIC, 10)

        # finalize detector
        
        # Log CFAR parameters
        self.get_logger().info(f"CFAR Parameters: Ntc={self.Ntc}, Ngc={self.Ngc}, Pfa={self.Pfa}, rank={self.rank}")

        self.detector = CFAR(self.Ntc, self.Ngc, self.Pfa, self.rank)

    def generate_map_xy(self, ping):
        '''Generate a mesh grid map for the sonar image, this enables converison to cartisian from the 
        source polar images

        ping: Ping message
        '''

        #get the parameters from the ping message
        _res = ping.range_resolution
        _height = ping.n_ranges * _res
        _rows = ping.n_ranges
        # Convert bearings from 100th of degree to radians
        bearings_deg = np.array(ping.bearings) * 0.01
        bearings_rad = bearings_deg * np.pi / 180.0
        _width = np.sin((bearings_rad[-1] - bearings_rad[0]) / 2) * _height * 2
        _cols = int(np.ceil(_width / _res))

        #check if the parameters have changed
        if self.res == _res and self.height == _height and self.rows == _rows and self.width == _width and self.cols == _cols:
            return

        #if they have changed do some work    
        self.res, self.height, self.rows, self.width, self.cols = _res, _height, _rows, _width, _cols

        #generate the mapping
        # Convert bearings from 100th of degree to radians
        bearings_deg = np.asarray(ping.bearings, dtype=np.float32) * 0.01
        bearings = bearings_deg * np.pi / 180.0
        f_bearings = interp1d(
            bearings,
            range(len(bearings)),
            kind='linear',
            bounds_error=False,
            fill_value=-1,
            assume_sorted=True)

        #build the meshgrid
        XX, YY = np.meshgrid(range(self.cols), range(self.rows))
        x = self.res * (self.rows - YY)
        y = self.res * (-self.cols / 2.0 + XX + 0.5)
        b = np.arctan2(y, x) * self.REVERSE_Z
        r = np.sqrt(np.square(x) + np.square(y))
        self.map_y = np.asarray(r / self.res, dtype=np.float32)
        self.map_x = np.asarray(f_bearings(b), dtype=np.float32)

    def publish_features(self, ping, points):
        '''Publish the feature message using the provided parameters in a Ping message
        ping: Ping message
        points: points to be converted to a ros point cloud, in cartisian meters
        '''

        #shift the axis
        points = np.c_[points[:,0],np.zeros(len(points)),  points[:,1]]

        #convert to a pointcloud
        feature_msg = n2r(points, "PointCloudXYZ")

        #give the feature message the same time stamp as the source sonar image
        #this is CRITICAL to good time sync downstream
        feature_msg.header.stamp = ping.header.stamp
        feature_msg.header.frame_id = "base_link"

        #publish the point cloud, to be used by SLAM
        self.feature_pub.publish(feature_msg)

    #@add_lock
    def sonar_callback(self, sonar_msg):
        '''Feature extraction callback
        sonar_msg: a Ping message, in polar coordinates
        '''

        if sonar_msg.ping_id % self.skip != 0:
            self.feature_img = None
            # Don't extract features in every frame.
            # But we still need empty point cloud for synchronization in SLAM node.
            nan = np.array([[np.nan, np.nan]])
            self.publish_features(sonar_msg, nan)
            return

        # Extract image data from ping_data
        # ping_data is a raw byte array in row-major format
        n_ranges = sonar_msg.n_ranges
        n_beams = sonar_msg.n_beams
        sample_size = sonar_msg.sample_size
        step = sonar_msg.step
        has_gains = sonar_msg.has_gains
        
        # Convert ping_data to numpy array
        ping_data = np.frombuffer(sonar_msg.ping_data, dtype=np.uint8)
        
        # If gains are present, each row starts with 4 bytes of gain data
        if has_gains:
            # Reconstruct image, removing gain data from each row
            img_data = []
            for i in range(n_ranges):
                row_start = i * step
                # Skip first 4 bytes (gain) and extract the actual image data
                row_data = ping_data[row_start + 4 : row_start + 4 + n_beams * sample_size]
                img_data.append(row_data)
            img = np.array(img_data, dtype=np.uint8).reshape(n_ranges, n_beams)
        else:
            # No gains, just reshape the data
            img = ping_data.reshape(n_ranges, n_beams).astype(np.uint8)

        #generate a mesh grid mapping from polar to cartisian
        self.generate_map_xy(sonar_msg)

        # Detect targets and check against threshold using CFAR (in polar coordinates)
        peaks = self.detector.detect(img, self.alg)
        peaks &= img > self.threshold

        # 1. Remap the intensity image (grayscale)
        vis_img = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR)

        # 2. Convert to BGR for overlay
        vis_img = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)

        # 3. Remap the binary peaks (USE INTER_NEAREST)
        cartesian_peaks = cv2.remap(peaks.astype(np.uint8), self.map_x, self.map_y, cv2.INTER_NEAREST)

        # 4. OVERLAY: Set all detected pixels to bright red
        vis_img[cartesian_peaks != 0] = [0, 0, 255]  # Bright Red

        # 5. Publish the combined image (background remains grayscale)
        img_msg = self.BridgeInstance.cv2_to_imgmsg(vis_img, encoding="bgr8")
        img_msg.header.stamp = sonar_msg.header.stamp
        img_msg.header.frame_id = "base_link"
        self.feature_img_pub.publish(img_msg)

        # 6. Now, use the `cartesian_peaks` you already calculated
        locs = np.c_[np.nonzero(cartesian_peaks)]




        # vis_img = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR)
        # vis_img = cv2.applyColorMap(vis_img, 2)
        # img_msg = self.BridgeInstance.cv2_to_imgmsg(vis_img, encoding="bgr8")
        # img_msg.header.stamp = sonar_msg.header.stamp
        # img_msg.header.frame_id = "base_link"
        # self.feature_img_pub.publish(img_msg)

        #convert to cartisian
        # peaks = cv2.remap(peaks, self.map_x, self.map_y, cv2.INTER_LINEAR)        
        # locs = np.c_[np.nonzero(peaks)]

        #convert from image coords to meters
        x = locs[:,1] - self.cols / 2.
        x = (-1 * ((x / float(self.cols / 2.)) * (self.width / 2.))) #+ self.width
        y = (-1*(locs[:,0] / float(self.rows)) * self.height) + self.height
        points = np.column_stack((y,x))

        # #filter the cloud using PCL
        # if len(points) and self.resolution > 0:
        #     points = pcl.downsample(points, self.resolution)

        # #remove some outliars
        # if self.outlier_filter_min_points > 1 and len(points) > 0:
        #     # points = pcl.density_filter(points, 5, self.min_density, 1000)
        #     points = pcl.remove_outlier(
        #         points, self.outlier_filter_radius, self.outlier_filter_min_points
        #     )

        #publish the feature message
        self.publish_features(sonar_msg, points)
