cmake_minimum_required(VERSION 3.5)
project(mv_publisher)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add SDK include & lib paths
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/x64
)

add_executable(MVCameraNode src/MVCameraNode.cpp)
ament_target_dependencies(MVCameraNode
  rclcpp
  sensor_msgs
  std_msgs
  camera_info_manager
  image_transport
  camera_calibration_parsers
)

# Link against MVSDK
target_link_libraries(MVCameraNode MVSDK)

install(
  TARGETS MVCameraNode
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs)
ament_package()
