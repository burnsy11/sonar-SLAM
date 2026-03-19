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

        # --- clustering params (new) ---
        self.enable_clustering = True
        self.cluster_A_min = 4          # min blob area in pixels
        self.cluster_A_max = 99999999       # max blob area in pixels (near blobs can be big)
        self.cluster_morph = 5          # 0 disables morphology; otherwise odd kernel size (3,5,...)
        self.cluster_repr = "max"       # "max" or "centroid"

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
        # Store bearing range for coordinate conversion
        self.bearing_min = None
        self.bearing_max = None
        self.bearing_center = None

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

        # debug
        self.declare_parameter('debug.enable', True)
        self.declare_parameter('debug.every_n', 10)

        # clustering
        self.declare_parameter('clustering.enable', True)
        self.declare_parameter('clustering.A_min', 1)
        self.declare_parameter('clustering.A_max', 999999999)
        self.declare_parameter('clustering.morph_kernel', 0)
        self.declare_parameter('clustering.repr', 'max')  # 'max' or 'centroid' 

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
        self.debug_enable = self.get_parameter('debug.enable').value
        self.debug_every_n = int(self.get_parameter('debug.every_n').value)
        self.enable_clustering = self.get_parameter('clustering.enable').value
        self.cluster_A_min = self.get_parameter('clustering.A_min').value
        self.cluster_A_max = self.get_parameter('clustering.A_max').value
        self.cluster_morph = self.get_parameter('clustering.morph_kernel').value
        self.cluster_repr = self.get_parameter('clustering.repr').value

        # FIXME: Just for debugging, will remove later
        self.threshold = 0

        # cv bridge
        self.BridgeInstance = CvBridge()

        # sonar subscription and feature publishers
        self.sonar_sub = self.create_subscription(
            Ping,
            SONAR_TOPIC,
            self.sonar_callback,
            10,
        )

        # feature publish topic (for SLAM sync)
        self.feature_pub = self.create_publisher(PointCloud2, SONAR_FEATURE_TOPIC, 10)

        # vis publish topic
        self.feature_img_pub = self.create_publisher(Image, SONAR_FEATURE_IMG_TOPIC, 10)

        # polar peaks debug image publisher
        self.peaks_img_pub = self.create_publisher(Image, "sonar/peaks_polar", 10)

        # finalize detector
        
        # Log CFAR parameters
        # self.get_logger().info(
        #     f"CFAR Parameters: Ntc={self.Ntc}, Ngc={self.Ngc}, Pfa={self.Pfa}, rank={self.rank}"
        # )

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
        
        # Store bearing range for coordinate conversion
        self.bearing_min = bearings_rad[0]
        self.bearing_max = bearings_rad[-1]
        self.bearing_center = (self.bearing_min + self.bearing_max) / 2.0
        bearing_span = self.bearing_max - self.bearing_min
        
        # Calculate width based on actual bearing span
        _width = np.sin(bearing_span / 2) * _height * 2
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

        #build the meshgrid - account for bearing center offset
        XX, YY = np.meshgrid(range(self.cols), range(self.rows))
        # Range increases going up (row 0 = max range)
        r = self.res * (self.rows - YY)
        # Lateral position: map columns to bearing range centered on bearing_center
        y = self.res * (-self.cols / 2.0 + XX + 0.5)
        # Compute bearing for each pixel, offset by the center bearing
        b = np.arctan2(y, r) * self.REVERSE_Z + self.bearing_center
        
        self.map_y = np.asarray(np.sqrt(r**2 + y**2) / self.res, dtype=np.float32)
        self.map_x = np.asarray(f_bearings(b), dtype=np.float32)

    def publish_features(self, ping, points):
        '''Publish the feature message using the provided parameters in a Ping message
        ping: Ping message
        points: points to be converted to a ros point cloud, in cartisian meters
        '''

        # Map to ROS convention: x = forward, y = lateral, z = up
        # Sonar is an in-plane sensor, so we place points in the X-Y plane (z = 0)
        points = np.c_[points[:,0], points[:,1], np.zeros(len(points))]

        #convert to a pointcloud
        feature_msg = n2r(points, "PointCloudXYZ")

        #give the feature message the same time stamp as the source sonar image
        #this is CRITICAL to good time sync downstream
        feature_msg.header.stamp = ping.header.stamp
        feature_msg.header.frame_id = "base_link"

        # publish the point cloud, to be used by SLAM
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

        # Store original image dimensions for later scaling
        orig_height, orig_width = img.shape
        img_original = img.copy()  # Keep original for visualization
        
        # Downscale to 480p for faster feature extraction (maintain aspect ratio)
        target_size = 480
        max_dim = max(orig_height, orig_width)
        scale_factor = target_size / max_dim
        target_height = int(orig_height * scale_factor)
        target_width = int(orig_width * scale_factor)
        img = cv2.resize(img, (target_width, target_height), interpolation=cv2.INTER_AREA)

        # Quick debug: image stats (rate-limited)
        # if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
        #     try:
        #         self.get_logger().info(
        #             f"ping {sonar_msg.ping_id} img: shape={img.shape} "
        #             f"min={int(img.min())} max={int(img.max())} mean={float(img.mean()):.1f}"
        #         )
        #     except Exception:
        #         pass

        #generate a mesh grid mapping from polar to cartisian
        self.generate_map_xy(sonar_msg)

        # Detect targets and check against threshold using CFAR (in polar coordinates)
        points = None
        peaks = self.detector.detect(img, self.alg)
        peaks_before = int(np.count_nonzero(peaks))

        peaks &= img > self.threshold

        rows = np.nonzero(peaks)[0]

        if len(rows):
            closest_row = rows.min()
            closest_range = (closest_row + 0.5) * sonar_msg.range_resolution
            # self.get_logger().info(
            #     f"closest CFAR detection: row={closest_row} range~{closest_range:.2f} m"
            # )

        # if self.debug_enable and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
        #     if len(rows):
        #         self.get_logger().info(
        #             f"peaks row stats: min={rows.min()} p10={np.percentile(rows,10):.0f} "
        #             f"med={np.median(rows):.0f} p90={np.percentile(rows,90):.0f} max={rows.max()}"
        #         )
        #     else:
        #         self.get_logger().info("peaks row stats: empty")


        peaks_after = int(np.count_nonzero(peaks))

        # Debug: log CFAR counts and publish polar mask (rate-limited)
        if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
            try:
                # self.get_logger().info(
                #     f"ping {sonar_msg.ping_id} CFAR peaks: before_thr={peaks_before} after_thr={peaks_after} "
                #     f"thr={self.threshold} Pfa={self.Pfa} Ntc={self.Ntc} Ngc={self.Ngc} alg={self.alg}"
                # )
                polar_vis = (peaks.astype(np.uint8) * 255)
                polar_vis_bgr = cv2.cvtColor(polar_vis, cv2.COLOR_GRAY2BGR)
                msg = self.BridgeInstance.cv2_to_imgmsg(polar_vis_bgr, encoding="bgr8")
                msg.header.stamp = sonar_msg.header.stamp
                msg.header.frame_id = "base_link"
                self.peaks_img_pub.publish(msg)
            except Exception:
                pass

        A_min = int(self.cluster_A_min)
        A_max = int(self.cluster_A_max)
        # -------------------------------
        # Polar clustering: peaks -> blobs -> 1 detection per blob
        # -------------------------------
        if self.enable_clustering:
            mask = peaks.astype(np.uint8)

            # Compute scaling factors between downsampled and original images
            # row_scale = orig / target (multiply downsampled indices to get original indices)
            row_scale = float(orig_height) / float(target_height) if target_height > 0 else 1.0
            col_scale = float(orig_width) / float(target_width) if target_width > 0 else 1.0
            area_scale = row_scale * col_scale

            # Optional: small morphological opening to remove isolated white speckles
            # without affecting larger clusters, followed by median blur.
            if self.cluster_morph and self.cluster_morph >= 3:
                k = int(self.cluster_morph)
                # scale kernel sizes to downsampled image (keep odd >=1)
                k_scaled = max(1, int(round(k * (1.0 / area_scale**0.5))))
                if k_scaled % 2 == 0:
                    k_scaled += 1
                open_k = 3
                open_k_scaled = max(1, int(round(open_k * (1.0 / area_scale**0.5))))
                if open_k_scaled % 2 == 0:
                    open_k_scaled += 1
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k_scaled, open_k_scaled))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                # then apply median blur to smooth remaining speckle
                mask = cv2.medianBlur(mask, k_scaled)

            num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

            # after connectedComponentsWithStats
            comp_info = []
            for cid in range(1, num):
                area = stats[cid, cv2.CC_STAT_AREA]
                x, y, w, h = stats[cid, cv2.CC_STAT_LEFT], stats[cid, cv2.CC_STAT_TOP], stats[cid, cv2.CC_STAT_WIDTH], stats[cid, cv2.CC_STAT_HEIGHT]
                r_min = y
                r_max = y + h - 1
                comp_info.append((area, r_min, r_max, cid))
            comp_info.sort(reverse=True)

            # if self.debug_enable and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
            #     for area, rmin, rmax, cid in comp_info[:5]:
            #         self.get_logger().info(f"comp {cid}: area={area} r=[{rmin},{rmax}]")


            # Debug: connected components stats
            areas = stats[1:, cv2.CC_STAT_AREA] if num > 1 else np.array([])
            # if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
            #     self.get_logger().info(
            #         f"ping {sonar_msg.ping_id} CC: total={num-1} (excluding background) connectivity=8"
            #     )
            #     try:
            #         if len(areas):
            #             self.get_logger().info(
            #                 f"ping {sonar_msg.ping_id} CC: num={num-1} "
            #                 f"area[min/med/max]={int(areas.min())}/{int(np.median(areas))}/{int(areas.max())} "
            #                 f"A_min={A_min} A_max={A_max}"
            #             )
            #         else:
            #             self.get_logger().info(
            #                 f"ping {sonar_msg.ping_id} CC: num=0 (mask empty after morphology?)"
            #             )
            #     except Exception:
            #         self.get_logger().info(
            #             f"ping {sonar_msg.ping_id} CC: num={num-1} (error logging stats)"
            #         )
            #         pass
                

            det_rc = []  # list of (range_row, beam_col) detections in POLAR indices
            passed = 0
            # Adjust area thresholds for downsampled image
            A_min_eff = max(1, int(max(1, A_min) / area_scale))
            A_max_eff = max(1, int(max(1, A_max) / area_scale))

            for cid in range(1, num):  # skip background 0
                area = stats[cid, cv2.CC_STAT_AREA]
                if area < A_min_eff or area > A_max_eff:
                    continue
                passed += 1

                if self.cluster_repr == "centroid":
                    # centroids are (x=col, y=row)
                    c_col, c_row = centroids[cid]
                    r = int(round(c_row))
                    b = int(round(c_col))
                    det_rc.append((r, b))
                else:
                    # "max": choose strongest-intensity pixel inside the blob (more stable than centroid)
                    # limit search to bounding box for speed
                    x, y, w, h = stats[cid, cv2.CC_STAT_LEFT], stats[cid, cv2.CC_STAT_TOP], stats[cid, cv2.CC_STAT_WIDTH], stats[cid, cv2.CC_STAT_HEIGHT]
                    roi = img[y:y+h, x:x+w]
                    roi_labels = labels[y:y+h, x:x+w]
                    mask_loc = (roi_labels == cid)
                    if not mask_loc.any():
                        continue
                    rr, cc = np.where(mask_loc)
                    vals = roi[rr, cc]
                    j = int(np.argmax(vals))
                    det_rc.append((int(rr[j] + y), int(cc[j] + x)))

            det_rc = np.asarray(det_rc, dtype=np.int32)

            # Debug: log passed components
            # if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
            #     try:
            #         if det_rc.size:
            #             self.get_logger().info(
            #                 f"ping {sonar_msg.ping_id} CC passed={passed} det_idx: r[min/max]={det_rc[:,0].min()}/{det_rc[:,0].max()} "
            #                 f"b[min/max]={det_rc[:,1].min()}/{det_rc[:,1].max()} n_ranges={sonar_msg.n_ranges} n_beams={sonar_msg.n_beams}"
            #             )
            #         else:
            #             self.get_logger().info(
            #                 f"ping {sonar_msg.ping_id} CC passed={passed} det_idx: none"
            #             )
            #     except Exception:
            #         pass

            # Convert polar detections (row, col) -> XY points
            if len(det_rc) == 0:
                points = np.zeros((0, 2), dtype=np.float32)
            else:
                # Scale detection indices back to original image size
                det_rc = det_rc.astype(np.float32)
                det_rc[:, 0] = det_rc[:, 0] * row_scale  # row
                det_rc[:, 1] = det_rc[:, 1] * col_scale  # col
                det_rc = det_rc.astype(np.int32)

                r_idx = det_rc[:, 0]
                b_idx = det_rc[:, 1]

                dr = float(sonar_msg.range_resolution)

                # Row index increases downward: row 0 = far, bottom = near.
                range_m = (r_idx + 0.5) * dr

                # if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
                #     try:
                #         self.get_logger().info(
                #             f"range_m[min/max]={float(range_m.min()):.3f}/{float(range_m.max()):.3f} dr={dr}"
                #         )
                #     except Exception:
                #         pass

                bearings_rad = (np.asarray(sonar_msg.bearings, dtype=np.float32) * 0.01) * np.pi / 180.0
                # Beam index should correspond to original image column index (orig_width == n_beams)
                n_beams = int(sonar_msg.n_beams)
                b_idx_clipped = np.clip(b_idx.astype(np.int32), 0, n_beams - 1)
                bearing = bearings_rad[b_idx_clipped]

                # if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
                #     try:
                #         self.get_logger().info(
                #             f"bearing[min/max]={float(bearing.min()):.3f}/{float(bearing.max()):.3f} rad bearings_len={len(bearings_rad)}"
                #         )
                #     except Exception:
                #         pass

                # Keep your existing convention: subtract 90° to align with robot frame
                bearing_ros = bearing - np.pi / 2.0
                x_pts = range_m * np.cos(bearing_ros)
                y_pts = range_m * np.sin(bearing_ros)
                points = np.column_stack((x_pts, y_pts)).astype(np.float32)
        # 1. Remap the original intensity image (grayscale) for publication
        vis_img = cv2.remap(img_original, self.map_x, self.map_y, cv2.INTER_LINEAR)

        # 2. Convert to BGR for overlay
        vis_img = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)

        # 3. Remap the binary peaks (USE INTER_NEAREST) - need to upscale peaks back to original size first
        peaks_original = cv2.resize(peaks, (orig_width, orig_height), interpolation=cv2.INTER_NEAREST)
        cartesian_peaks = cv2.remap(peaks_original.astype(np.uint8), self.map_x, self.map_y, cv2.INTER_NEAREST)

        # 4. OVERLAY: Set all detected pixels to bright red
        vis_img[cartesian_peaks != 0] = [0, 0, 255]  # Bright Red

        # 5. Publish the combined image (background remains grayscale)
        img_msg = self.BridgeInstance.cv2_to_imgmsg(vis_img, encoding="bgr8")
        img_msg.header.stamp = sonar_msg.header.stamp
        img_msg.header.frame_id = "base_link"
        self.feature_img_pub.publish(img_msg)

        # 6. Convert detections to points.
        if points is None:
            locs = np.c_[np.nonzero(cartesian_peaks)]

            #convert from image coords to meters
            # Column index -> lateral offset from image center
            lateral = (locs[:,1] - self.cols / 2.0) * self.res
            # Row index -> range (row 0 = far, row max = close)
            range_m = (self.rows - locs[:,0]) * self.res

            # Convert to Cartesian (x=forward, y=left) accounting for bearing center
            bearing = np.arctan2(lateral, range_m) + self.bearing_center
            dist = np.sqrt(lateral**2 + range_m**2)

            # Sonar convention: bearing=0 is forward (+X), positive bearing is left (+Y)
            # Subtract 90° to align sonar frame with robot frame (sonar looks along +X)
            bearing_ros = bearing - np.pi / 2.0
            x_pts = dist * np.cos(bearing_ros)
            y_pts = dist * np.sin(bearing_ros)
            points = np.column_stack((x_pts, y_pts)).astype(np.float32)

        # Optional small downsampling after clustering/filtering
        # if points is not None and len(points) and self.resolution > 0:
        #     try:
        #         points = pcl.downsample(points, self.resolution)
        #     except Exception:
        #         pass

        # #filter the cloud using PCL
        # if len(points) and self.resolution > 0:
        #     points = pcl.downsample(points, self.resolution)

        # #remove some outliars
        # if self.outlier_filter_min_points > 1 and len(points) > 0:
        #     # points = pcl.density_filter(points, 5, self.min_density, 1000)
        #     points = pcl.remove_outlier(
        #         points, self.outlier_filter_radius, self.outlier_filter_min_points
        #     )

        # Debug: log the points we will publish (rate-limited)
        # if getattr(self, 'debug_enable', False) and (sonar_msg.ping_id % getattr(self, 'debug_every_n', 10) == 0):
        #     try:
        #         if points is not None and len(points):
        #             self.get_logger().info(
        #                 f"publishing points: N={len(points)} "
        #                 f"x[min/max]={float(points[:,0].min()):.2f}/{float(points[:,0].max()):.2f} "
        #                 f"y[min/max]={float(points[:,1].min()):.2f}/{float(points[:,1].max()):.2f}"
        #             )
        #         else:
        #             self.get_logger().info(f"publishing points: N=0")
        #     except Exception:
        #         pass

        #publish the feature message
        self.publish_features(sonar_msg, points)
