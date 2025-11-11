# bruce_slam ROS2 Conversion

This package has been converted from ROS1 (Noetic) to ROS2 (Humble).

## Conversion Summary

### What Was Changed

#### Build System
- **package.xml**: Updated from format 2 to format 3 with ROS2 dependencies
- **CMakeLists.txt**: Converted from catkin to ament_cmake
- **setup.py**: Updated for ROS2 Python package structure

#### Python Code
All Python code has been converted from rospy API to rclpy API:

**API Conversions:**
- `rospy` → `rclpy` and `Node` class
- `rospy.init_node()` → `rclpy.init()` + Node constructor
- `rospy.get_param()` → `self.declare_parameter()` + `self.get_parameter()`
- `rospy.Publisher()` → `self.create_publisher()`
- `rospy.Subscriber()` → `self.create_subscription()`
- `rospy.Service()` → `self.create_service()`
- `rospy.Time.now()` → `self.get_clock().now()`
- `rospy.spin()` → `rclpy.spin(node)`
- `tf.TransformBroadcaster()` → `TransformBroadcaster(self)`
- `message_filters.Subscriber` now requires node parameter

**Time Handling:**
- `.to_sec()` → `.sec + .nanosec * 1e-9`
- `rospy.Duration()` → `rclpy.duration.Duration()`

**Message Filters:**
- `Subscriber` now takes node as first parameter

**Imports:**
- `import tf` → `import tf2_ros` + `from tf2_ros import TransformBroadcaster`
- `import cv_bridge` → `from cv_bridge import CvBridge`
- `sensor_msgs.point_cloud2` → `sensor_msgs_py.point_cloud2`
- `from tf.transformations` → `from tf_transformations`

#### Launch Files
- Converted XML launch files to Python launch files
- `launch/slam_launch.py` replaces `launch_ros1/slam.launch`

### What Was NOT Changed

✅ **No Logic Changes**: All algorithms remain identical
✅ **No Message Changes**: Using converted bruce_msgs package
✅ **No Parameter Names**: Configuration file structure unchanged
✅ **No Topics/Services**: Topic and service names unchanged

## Building

```bash
cd ~/ros2_ws/src
# Clone or copy bruce_slam here
cd ~/ros2_ws
colcon build --packages-select bruce_slam
source install/setup.bash
```

## Running

### Online Mode (default)
```bash
ros2 launch bruce_slam slam_launch.py
```

### With RViz
```bash
ros2 launch bruce_slam slam_launch.py rviz:=true
```

### Without SLAM (dead reckoning only)
```bash
ros2 launch bruce_slam slam_launch.py enable_slam:=false
```

### Using Kalman Filter
```bash
ros2 launch bruce_slam slam_launch.py kalman_dead_reckoning:=true
```

## Configuration

Configuration files remain in `config/` directory:
- `dead_reckoning.yaml`
- `feature.yaml`
- `gyro.yaml`
- `slam.yaml`
- `kalman.yaml`

Parameter names and structure are unchanged from ROS1.

## Dependencies

### ROS2 Packages
- rclpy
- sensor_msgs
- geometry_msgs
- nav_msgs
- std_msgs
- visualization_msgs
- tf2_ros
- tf2_geometry_msgs
- message_filters
- bruce_msgs (ROS2 version)

### External Dependencies  
- libpointmatcher
- PCL (Point Cloud Library)
- GTSAM
- pybind11
- cv_bridge

### Sensor Packages (if available)
- rti_dvl
- bar30_depth
- kvh_gyro
- sonar_oculus

## Known Issues & Limitations

1. **Offline Mode (rosbag playback)**: The offline bag reading functionality needs ROS2 rosbag2 API, which differs from ROS1. The code includes compatibility layers but may need testing.

2. **Parameter Declaration**: Some dynamic parameter accesses may need `declare_parameter()` calls added before `get_parameter()`.

3. **TF Broadcasting**: TF2 has different API for broadcasting transforms. Some transform publishing may need adjustment.

4. **Message Filter Timing**: ApproximateTimeSynchronizer behavior may differ slightly in ROS2.

5. **ros_numpy**: This package is not available in ROS2. Code uses `sensor_msgs_py` instead.

## Testing Status

- ⏳ Build testing requires ROS2 Humble environment with dependencies
- ⏳ Runtime testing requires full sensor suite or bag files
- ⏳ Integration testing with hardware pending

## Migration Notes for Users

If you have custom code that depends on bruce_slam:

1. Update your `package.xml` to depend on `bruce_slam` ROS2 version
2. Import from `bruce_slam.msg` remains the same (via bruce_msgs)
3. Topic/service names are unchanged
4. Configuration file structure is unchanged

## Backup Files

Original ROS1 files are backed up with `.ros1_backup` extension in case reference is needed.

## Support

For issues specific to the ROS2 conversion, please check:
1. All dependencies are installed for ROS2 Humble
2. bruce_msgs package is built and sourced
3. Configuration files are present in the install/share directory

## Version History

- **v0.0.1-ros2**: Initial ROS2 Humble conversion
  - Converted from ROS1 Noetic
  - All Python nodes converted to rclpy
  - Build system updated to ament_cmake
  - Launch files converted to Python
