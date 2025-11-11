cmake_minimum_required(VERSION 3.8)
project(bruce_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(oculus_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ISAM2Update.msg"
  "msg/KeyframeImage.msg"
  "msg/PoseHistory.msg"
  "srv/QueryExplorationPath.srv"
  "srv/PredictSLAMUpdate.srv"
  "srv/GetOccupancyMap.srv"
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    oculus_interfaces
)

# Optional: Uncomment if/when GTSAM support is needed
# find_package(GTSAM REQUIRED)
# set(GTSAM_LIBRARIES gtsam)

# Optional: Build conversions library if needed
# add_library(conversions src/bruce_msgs/conversions.cpp)
# target_include_directories(conversions PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(conversions
#   rclcpp
# )
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(conversions "${cpp_typesupport_target}")

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
