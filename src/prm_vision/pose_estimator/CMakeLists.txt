cmake_minimum_required(VERSION 3.8)
project(pose_estimator CXX)
set(CMAKE_CXX_STANDARD 17) # for filesystem support
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV 4.6.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)  # GTest
find_package(Eigen3 REQUIRED) # Eigen3

# Include directories
include_directories(/usr/include)
include_directories(include)

# Build target
add_library(PoseEstimator STATIC src/PoseEstimator.cpp)
target_link_libraries(PoseEstimator ${OpenCV_LIBS} ValidityFilter Eigen3::Eigen)

add_library(ValidityFilter STATIC src/ValidityFilter.cpp)

add_executable(PoseEstimatorNode src/PoseEstimatorNode.cpp)
target_link_libraries(PoseEstimatorNode PoseEstimator ValidityFilter ${OpenCV_LIBS})

ament_target_dependencies(PoseEstimatorNode 
  rclcpp
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  vision_msgs
  Eigen3
)

ament_target_dependencies(PoseEstimator
  rclcpp
)

# GTest setup
ament_add_gtest(test_PoseEstimator test/test_PoseEstimator.cpp)
ament_add_gtest(test_ValidityFilter test/test_ValidityFilter.cpp)
target_include_directories(test_PoseEstimator PRIVATE include)
target_include_directories(test_ValidityFilter PRIVATE include)
target_link_libraries(test_PoseEstimator PoseEstimator)
target_link_libraries(test_ValidityFilter ValidityFilter)

# Install the node
install(TARGETS PoseEstimatorNode
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs cv_bridge vision_msgs Eigen3 tf2)
ament_package()