# from rti_dvl.msg import DVL # FIXEME: this needs to be changed to ours
from sensor_msgs.msg import Imu
# from bar30_depth.msg import Depth # FIXEME: this needs to be changed to ours
from oculus_interfaces.msg import Ping


"""
Topics for the bruce_slam project
"""
IMU_TOPIC = "/oceansim/robot/imu"
IMU_TOPIC_MK_II = "/vectornav/IMU"
DVL_TOPIC = "/dvl/data"
DEPTH_TOPIC = "/bar30/depth/raw"
SONAR_TOPIC = "/sonar/ping"
SONAR_TOPIC_UNCOMPRESSED = "/sonar_oculus_node/ping"
SONAR_VERTICAL_TOPIC = "/sonar_oculus_node/M1200d/ping"


SLAM_NS = "/bruce/slam/"

SONAR_FUSION_TOPIC = SLAM_NS + "sonar_fusion"
LOCALIZATION_ODOM_TOPIC = SLAM_NS + "localization/odom"
LOCALIZATION_TRAJ_TOPIC = SLAM_NS + "localization/traj"
SLAM_POSE_TOPIC = SLAM_NS + "slam/pose"
SLAM_ODOM_TOPIC = SLAM_NS + "slam/odom"
SLAM_TRAJ_TOPIC = SLAM_NS + "slam/traj"
SLAM_CLOUD_TOPIC = SLAM_NS + "slam/cloud"
SLAM_CONSTRAINT_TOPIC = SLAM_NS + "slam/constraint"
SLAM_ISAM2_TOPIC = SLAM_NS + "slam/isam2"
SLAM_PREDICT_SLAM_UPDATE_SERVICE = SLAM_NS + "slam/predict_slam_update"
MAPPING_INTENSITY_TOPIC = SLAM_NS + "mapping/intensity"
MAPPING_OCCUPANCY_TOPIC = SLAM_NS + "mapping/occupancy"
MAPPING_GET_MAP_SERVICE = SLAM_NS + "mapping/get_map"
SONAR_FEATURE_TOPIC = SLAM_NS + "feature_extraction/feature"
SONAR_FEATURE_VIZ_TOPIC = SLAM_NS + "feature_extraction/feature_viz"
SONAR_FEATURE_IMG_TOPIC = SLAM_NS + "feature_extraction/feature_img"
