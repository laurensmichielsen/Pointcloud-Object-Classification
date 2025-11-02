cmake_minimum_required(VERSION 3.8)
project(pre_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(pc_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(mlpack REQUIRED)

# Enable OpenMP
if(OpenMP_CXX_FOUND)
  message(STATUS "Found OpenMP")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Include directories (order matters — DBSCAN first to prioritize its dbscan.hpp)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/external/DBSCAN
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  include
)

# Define executable
add_executable(pointcloud_processing_node 
  src/pointcloud_processing_node.cpp 
  src/pointcloud_clustering.cpp 
  src/pointcloud_filter.cpp
  src/pointcloud_ground_removal.cpp
)

# Link dependencies
target_link_libraries(pointcloud_processing_node
  dbscan::dbscan
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  mlpack
  ${OpenMP_CXX_LIBRARIES}
  armadillo
)

ament_target_dependencies(pointcloud_processing_node
  rclcpp
  sensor_msgs
  cv_msgs
  pcl_conversions
  visualization_msgs
)

# Install target
install(TARGETS
  pointcloud_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
