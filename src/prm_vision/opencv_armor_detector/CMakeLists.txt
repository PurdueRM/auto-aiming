cmake_minimum_required(VERSION 3.8)
project(opencv_armor_detector CXX)
set(CMAKE_CXX_STANDARD 17) # for filesystem support
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.6.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED) # Filesystem search in tests
find_package(Eigen3 REQUIRED) # Eigen3
find_package(ament_cmake_gtest REQUIRED)  # GTest

# Include directories
include_directories(/usr/include)
include_directories(include)

# Build target
add_library(OpenCVArmorDetector STATIC src/OpenCVArmorDetector.cpp)
target_link_libraries(OpenCVArmorDetector ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(OpenCVArmorDetectorNode src/OpenCVArmorDetectorNode.cpp)
target_link_libraries(OpenCVArmorDetectorNode OpenCVArmorDetector ${OpenCV_LIBS})

ament_target_dependencies(OpenCVArmorDetectorNode 
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  vision_msgs
  Eigen3
  ament_index_cpp
)

ament_target_dependencies(OpenCVArmorDetector
  rclcpp
  ament_index_cpp
)

# GTest setup
ament_add_gtest(test_OpenCVArmorDetector test/test_OpenCVArmorDetector.cpp)
target_include_directories(test_OpenCVArmorDetector PRIVATE include)
target_link_libraries(test_OpenCVArmorDetector OpenCVArmorDetector)

# Install the node
install(TARGETS OpenCVArmorDetectorNode
  DESTINATION lib/${PROJECT_NAME})

# Install resources directory
install(DIRECTORY test/resources/
        DESTINATION share/${PROJECT_NAME}/resources)

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs)
ament_package()
