# bruce_msgs ROS2 Conversion

This package has been converted from ROS1 (Noetic) to ROS2 (Humble).

## Message Data Structures

**Important**: All message data structures remain EXACTLY the same as in ROS1. No fields were added, removed, or modified.

### Messages

1. **ISAM2Update.msg**
   - `std_msgs/Header header`
   - `uint32 key`
   - `uint8[] isam2`
   - `uint8[] graph`
   - `uint8[] values`

2. **posehistory.msg**
   - `std_msgs/Header header`
   - `float64[] data`

3. **keyframe_image.msg**
   - `std_msgs/Header header`
   - `int64 keyframe`
   - `sonar_oculus/OculusPing img`
   - `sensor_msgs/PointCloud2 cloud`

### Services

1. **GetOccupancyMap.srv**
   - Request: `uint32[] frames`, `float32 resolution`
   - Response: `nav_msgs/OccupancyGrid occ`

2. **PredictSLAMUpdate.srv**
   - Request: `uint32 key`, `bool return_isam2_update`, `nav_msgs/Path[] paths`
   - Response: `bruce_msgs/ISAM2Update[] isam2_updates`, `nav_msgs/Path[] keyframes`

3. **QueryExplorationPath.srv**
   - Request: (empty)
   - Response: `nav_msgs/Path path`, `string type`

## Key Changes Made

### Build System
- Converted from `catkin` to `ament_cmake`
- Updated `CMakeLists.txt` to use `rosidl_generate_interfaces`
- Updated `package.xml` from format 2 to format 3

### Message Files
- Added explicit `std_msgs/` prefix to `Header` type (required in ROS2)
- This does NOT change the data structure - it's only a syntax requirement

### C++ Conversions Library (Currently Commented Out)
- Updated includes to use ROS2 message header paths (e.g., `bruce_msgs/msg/isam2_update.hpp`)
- Changed pointer type from raw pointer to `std::shared_ptr`
- Updated namespace references to `bruce_msgs::msg::`

## Building

To build this package in a ROS2 Humble workspace:

```bash
cd ~/ros2_ws/src
# Clone or copy bruce_msgs here
cd ~/ros2_ws
colcon build --packages-select bruce_msgs
source install/setup.bash
```

## Dependencies

Required ROS2 packages:
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `std_msgs`
- `rosidl_default_generators` (build)
- `rosidl_default_runtime` (runtime)

Optional (for C++ conversions library when enabled):
- GTSAM library

## Compatibility Notes

- The message definitions are semantically identical to ROS1
- Field types and ordering are preserved
- Binary serialization format follows ROS2 standards
- The `sonar_oculus` dependency in `keyframe_image.msg` must also be available as ROS2 package

## Status

✅ Message definitions converted and validated
✅ Service definitions converted and validated
✅ Package configuration updated for ROS2
✅ C++ conversions library updated (currently commented out)
⏳ Build testing requires ROS2 Humble environment
⏳ Runtime testing requires full ROS2 workspace with dependencies

## Next Steps

1. Test building in a ROS2 Humble environment
2. Verify message generation produces correct C++/Python bindings
3. Enable and test C++ conversions library if GTSAM support is needed
4. Update dependent packages (e.g., bruce_slam) to use ROS2 APIs
