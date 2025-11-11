cmake_minimum_required(VERSION 3.8)
project(bruce_slam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bruce_msgs REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(PCL REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(
  ${libpointmatcher_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Build pybind11 modules
pybind11_add_module(pcl src/${PROJECT_NAME}/cpp/pcl.cpp)
target_link_libraries(pcl PRIVATE ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS pcl
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

pybind11_add_module(cfar src/${PROJECT_NAME}/cpp/cfar.cpp)
install(TARGETS cfar
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Install Python scripts
install(PROGRAMS
  scripts/slam_node.py
  scripts/mapping_node.py
  scripts/dead_reckoning_node.py
  scripts/feature_extraction_node.py
  scripts/gyro_node.py
  scripts/kalman_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
