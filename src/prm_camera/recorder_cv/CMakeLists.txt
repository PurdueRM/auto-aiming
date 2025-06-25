cmake_minimum_required(VERSION 3.5)
project(recorder)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Set include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Create executable
add_executable(VideoRecorderNode src/VideoRecorderNode.cpp)

# Link libraries
ament_target_dependencies(VideoRecorderNode
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install executable
install(TARGETS VideoRecorderNode
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

ament_package()
